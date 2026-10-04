// SPDX-License-Identifier: GPL-2.0
/*
 * Flyme / Meizu vibrator compatibility layer for the Awinic AW8697 haptic
 * controller.
 *
 * Flyme's userspace (FlymeVibratorHelper plus Meizu's vibrator HAL) drives the
 * motor through a Meizu specific sysfs class:
 *
 *	/sys/class/meizu/motor/on_off		- trigger a predefined effect by id
 *	/sys/class/meizu/motor/rtp		- trigger an RTP preset (40001..40999)
 *	/sys/class/meizu/motor/set_rtp		- RTP parameters
 *	/sys/class/meizu/motor/set_mback	- mBack key feedback: "<wave> <strength>"
 *	/sys/class/meizu/motor/set_cspress	- pressure key feedback strength
 *	/sys/class/meizu/motor/waveform		- select the waveform number
 *	/sys/class/meizu/motor/freq		- drive frequency
 *	/sys/class/meizu/motor/proline		- production line test
 *	/sys/class/meizu/motor/haptic_audio	- audio driven vibration
 *	/sys/class/timed_output/vibrator/enable	- vibrate for N ms, 0 stops
 *
 * None of these exist on a Xiaomi SM8250 tree, which is why vibration is dead
 * after a Flyme port.  Meizu never shipped the kernel side, so this layer
 * re-implements the ABI on top of the AW8697 driver that is already there:
 * the Awinic driver exposes an effect oriented interface of its own
 * (effect_id / activate_mode / duration / activate) with the same RAM-plus-RTP
 * shape, so most of the work here is translation and a small effect table.
 *
 * The layer is registered from the AW8697 probe, so the haptic engine is
 * guaranteed to exist before any node is created.
 *
 * Copyright (c) 2026 fuzhi
 */

#define pr_fmt(fmt) "flyme-vibrator: " fmt

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/leds.h>

#include "aw8697_compat.h"

#define FLYME_VIBRATOR_DEV_NAME "motor"
#define FLYME_LED_NAME "vibrator"

/* Meizu effect ids in [FLYME_RTP_ID_BASE, FLYME_RTP_ID_MAX] are RTP presets. */
#define FLYME_RTP_ID_BASE 40001
#define FLYME_RTP_ID_MAX 40999

#define FLYME_GAIN_LIGHT AW8697_COMPAT_GAIN_LIGHT
#define FLYME_GAIN_MEDIUM AW8697_COMPAT_GAIN_MEDIUM
#define FLYME_GAIN_STRONG AW8697_COMPAT_GAIN_STRONG

#define FLYME_WAVE_MAX AW8697_COMPAT_WAVE_MAX

/*
 * Group writable: FlymeVibratorHelper runs as the Android system UID, not as
 * root, and Meizu's init.target.rc chowns the nodes to "system system".  0666
 * is not an option - the kernel rejects other-writable sysfs files in
 * VERIFY_OCTAL_PERMISSIONS().
 */
#define FLYME_DEVICE_ATTR_RW(_name)					\
	struct device_attribute dev_attr_##_name =			\
		__ATTR(_name, 0660, _name##_show, _name##_store)

/*
 * Meizu's firmware ships a waveform library; ids below FLYME_RTP_ID_BASE pick
 * one of those waveforms and play it once.  Meizu's tables are not public, so
 * the ids the Meizu vibrator HAL actually emits (see
 * android.hardware.vibrator@1.3-service.meizu) are mapped explicitly and every
 * other id falls back to a medium tap, which keeps notifications silent-free
 * instead of failing the write.
 */
struct flyme_effect {
	u32 id;
	const char *name;
	u8 wave;
	u8 loop;
	u8 gain;
	u32 duration_ms;
};

static const struct flyme_effect flyme_effects[] = {
	{ 21000, "tick", 1, 0, FLYME_GAIN_LIGHT, 8 },
	{ 22520, "pop", 2, 0, FLYME_GAIN_MEDIUM, 18 },
	{ 30900, "thud", 3, 0, FLYME_GAIN_STRONG, 28 },
	{ 31003, "double-click", 4, 0, FLYME_GAIN_MEDIUM, 35 },
	{ 31008, "click", 5, 0, FLYME_GAIN_MEDIUM, 12 },
};

static const struct flyme_effect flyme_effect_default = {
	.id = 0,
	.name = "default",
	.wave = 2,
	.loop = 0,
	.gain = FLYME_GAIN_MEDIUM,
	.duration_ms = 18,
};

struct flyme_vibrator {
	struct class *motor_class;
	struct device *motor_dev;
	struct class *timed_class;
	struct device *timed_dev;
	struct led_classdev led;

	struct mutex lock;
	u8 wave;	/* waveform used by timed_output/vibrator/enable */
	u8 mback_wave;
	u8 mback_strength;
	u8 cspress_strength;
	u32 freq;
	u32 rtp_id;
	u8 rtp_wave;
	u8 rtp_gain;
	u32 rtp_duration_ms;
	u32 proline;
	/* Last effect id written to on_off / effect_id, replayed by activate. */
	u32 last_effect;
	/* Raw gain from /sys/class/leds/vibrator/gain, 0 keeps the table value. */
	u8 led_gain;
};

static struct flyme_vibrator *flyme_vib;

static inline struct flyme_vibrator *flyme_vib_get(struct device *dev)
{
	return dev_get_drvdata(dev);
}

static u8 flyme_gain_from_strength(u8 strength)
{
	switch (strength) {
	case 0:
		return FLYME_GAIN_LIGHT;
	case 1:
		return FLYME_GAIN_MEDIUM;
	case 2:
		return FLYME_GAIN_STRONG;
	default:
		return FLYME_GAIN_MEDIUM;
	}
}

static u8 flyme_gain_of(struct flyme_vibrator *vib, u8 fallback)
{
	return vib->led_gain ? vib->led_gain : fallback;
}

static int flyme_play_effect(struct flyme_vibrator *vib, u32 id)
{
	const struct flyme_effect *e = &flyme_effect_default;
	u8 gain;
	int i;

	for (i = 0; i < ARRAY_SIZE(flyme_effects); i++) {
		if (flyme_effects[i].id == id) {
			e = &flyme_effects[i];
			break;
		}
	}

	gain = flyme_gain_of(vib, e->gain);

	pr_debug("effect %u -> %s (wave %u, gain 0x%02x, %u ms)\n", id, e->name,
		 e->wave, gain, e->duration_ms);

	return aw8697_compat_play_ram(e->wave, e->loop, gain, e->duration_ms);
}

/*
 * Meizu ids 40001..40999 are RTP presets.  Every RTP slot in this tree points
 * at the same firmware blob (aw8697_rtp_1.bin), so the waveform is picked from
 * the id instead: it keeps the different Flyme effects distinguishable and
 * does not depend on an RTP blob being present.
 */
static int flyme_play_rtp(struct flyme_vibrator *vib, u32 id)
{
	u8 wave;
	u8 gain;
	u32 duration_ms;

	if (vib->rtp_gain) {
		wave = vib->rtp_wave;
		gain = vib->rtp_gain;
		duration_ms = vib->rtp_duration_ms;
	} else {
		wave = 1 + ((id - FLYME_RTP_ID_BASE) % FLYME_WAVE_MAX);
		gain = flyme_gain_of(vib, FLYME_GAIN_MEDIUM);
		duration_ms = 20;
	}

	if (wave > FLYME_WAVE_MAX)
		wave = FLYME_WAVE_MAX;

	pr_debug("rtp %u -> wave %u (gain 0x%02x, %u ms)\n", id, wave, gain,
		 duration_ms);

	return aw8697_compat_play_ram(wave, 0, gain, duration_ms);
}

/* ------------------------------------------------------------------ *
 * /sys/class/meizu/motor
 * ------------------------------------------------------------------ */

static ssize_t on_off_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", vib->last_effect);
}

/*
 * 0 stops, 1 replays the effect last written, anything else is an effect id.
 * Meizu's own on_off node uses the same convention, and so do the LED class
 * effect_id (same handler) and activate nodes.
 */
static ssize_t on_off_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 id;
	int rc;

	rc = kstrtou32(buf, 0, &id);
	if (rc)
		return rc;

	if (!id) {
		aw8697_compat_stop();
		return count;
	}

	if (id == 1) {
		if (vib->last_effect)
			flyme_play_effect(vib, vib->last_effect);
		return count;
	}

	mutex_lock(&vib->lock);
	vib->last_effect = id;
	mutex_unlock(&vib->lock);

	if (id >= FLYME_RTP_ID_BASE && id <= FLYME_RTP_ID_MAX)
		flyme_play_rtp(vib, id);
	else
		flyme_play_effect(vib, id);

	return count;
}
static FLYME_DEVICE_ATTR_RW(on_off);

static ssize_t rtp_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", vib->rtp_id);
}

static ssize_t rtp_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 id;
	int rc;

	rc = kstrtou32(buf, 0, &id);
	if (rc)
		return rc;

	mutex_lock(&vib->lock);
	vib->rtp_id = id;
	mutex_unlock(&vib->lock);

	flyme_play_rtp(vib, id);

	return count;
}
static FLYME_DEVICE_ATTR_RW(rtp);

static ssize_t set_rtp_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u %u %u %u\n", vib->rtp_id,
			 vib->rtp_wave, vib->rtp_gain, vib->rtp_duration_ms);
}

/*
 * Optional RTP tuning: "<id> [wave] [gain] [duration_ms]".  When gain is set,
 * rtp writes use these values instead of the default per-id mapping.
 */
static ssize_t set_rtp_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 id = vib->rtp_id, wave = vib->rtp_wave;
	u32 gain = vib->rtp_gain, duration = vib->rtp_duration_ms;
	int rc;

	rc = sscanf(buf, "%u %u %u %u", &id, &wave, &gain, &duration);
	if (rc <= 0)
		return -EINVAL;

	mutex_lock(&vib->lock);
	if (rc > 0)
		vib->rtp_id = id;
	if (rc > 1)
		vib->rtp_wave = wave > FLYME_WAVE_MAX ? FLYME_WAVE_MAX : wave;
	if (rc > 2)
		vib->rtp_gain = gain;
	if (rc > 3)
		vib->rtp_duration_ms = duration;
	mutex_unlock(&vib->lock);

	return count;
}
static FLYME_DEVICE_ATTR_RW(set_rtp);

static ssize_t set_mback_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u %u\n", vib->mback_wave,
			 vib->mback_strength);
}

/*
 * Meizu's userspace writes "<wave> <strength>" here, e.g. "1 1" - the m2391
 * TWRP device tree uses exactly that for a single finite mBack tap.  A bare
 * value is accepted as well and plays that waveform at the stored strength.
 */
static ssize_t set_mback_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 wave = vib->mback_wave, strength = vib->mback_strength;
	int rc;

	rc = sscanf(buf, "%u %u", &wave, &strength);
	if (rc <= 0)
		return -EINVAL;

	mutex_lock(&vib->lock);
	if (rc > 0)
		vib->mback_wave = wave > FLYME_WAVE_MAX ? FLYME_WAVE_MAX : wave;
	if (rc > 1)
		vib->mback_strength = strength;
	mutex_unlock(&vib->lock);

	if (rc == 1) {
		aw8697_compat_play_ram(vib->mback_wave, 0,
				       flyme_gain_from_strength(
					       vib->mback_strength),
				       0);
	} else if (strength) {
		aw8697_compat_play_ram(vib->mback_wave, 0,
				       flyme_gain_from_strength(strength), 20);
	} else {
		aw8697_compat_stop();
	}

	return count;
}
static FLYME_DEVICE_ATTR_RW(set_mback);

static ssize_t set_cspress_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", vib->cspress_strength);
}

/* Pressure key feedback: store the strength and emit a light, short tap. */
static ssize_t set_cspress_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 strength;
	int rc;

	rc = kstrtou32(buf, 0, &strength);
	if (rc)
		return rc;
	if (strength > 2)
		strength = 2;

	mutex_lock(&vib->lock);
	vib->cspress_strength = strength;
	mutex_unlock(&vib->lock);

	aw8697_compat_play_ram(1, 0, flyme_gain_from_strength(strength), 10);

	return count;
}
static FLYME_DEVICE_ATTR_RW(set_cspress);

static ssize_t waveform_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", vib->wave);
}

static ssize_t waveform_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 wave;
	int rc;

	rc = kstrtou32(buf, 0, &wave);
	if (rc)
		return rc;

	mutex_lock(&vib->lock);
	vib->wave = wave > FLYME_WAVE_MAX ? FLYME_WAVE_MAX : wave;
	mutex_unlock(&vib->lock);

	return count;
}
static FLYME_DEVICE_ATTR_RW(waveform);

static ssize_t freq_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", vib->freq);
}

/*
 * Stored only: the AW8697 continuous-drive frequency is fixed by its f0
 * calibration and cannot be retuned per write.  The node exists so Flyme's
 * helper does not fail when it writes the user's preference.
 */
static ssize_t freq_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 freq;
	int rc;

	rc = kstrtou32(buf, 0, &freq);
	if (rc)
		return rc;

	mutex_lock(&vib->lock);
	vib->freq = freq;
	mutex_unlock(&vib->lock);

	return count;
}
static FLYME_DEVICE_ATTR_RW(freq);

static ssize_t proline_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", vib->proline);
}

/* Factory / production line test: non-zero buzzes, 0 stops. */
static ssize_t proline_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 on;
	int rc;

	rc = kstrtou32(buf, 0, &on);
	if (rc)
		return rc;

	mutex_lock(&vib->lock);
	vib->proline = on;
	mutex_unlock(&vib->lock);

	if (on)
		aw8697_compat_play_ram(vib->wave, 1,
				       flyme_gain_of(vib, FLYME_GAIN_MEDIUM),
				       0);
	else
		aw8697_compat_stop();

	return count;
}
static FLYME_DEVICE_ATTR_RW(proline);

static ssize_t haptic_audio_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0\n");
}

/*
 * Game vibration is driven from audio.  The Awinic driver implements the
 * engine internally (haptic_audio work) but this tree does not expose a
 * userspace attribute for it, so the value is accepted and ignored rather
 * than failing the write.
 */
static ssize_t haptic_audio_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	pr_debug("haptic_audio write ignored\n");

	return count;
}
static FLYME_DEVICE_ATTR_RW(haptic_audio);

static struct attribute *flyme_motor_attrs[] = {
	&dev_attr_on_off.attr,
	&dev_attr_rtp.attr,
	&dev_attr_set_rtp.attr,
	&dev_attr_set_mback.attr,
	&dev_attr_set_cspress.attr,
	&dev_attr_waveform.attr,
	&dev_attr_freq.attr,
	&dev_attr_proline.attr,
	&dev_attr_haptic_audio.attr,
	NULL,
};

static const struct attribute_group flyme_motor_group = {
	.attrs = flyme_motor_attrs,
};

/* ------------------------------------------------------------------ *
 * /sys/class/timed_output/vibrator
 * ------------------------------------------------------------------ */

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0\n");
}

/*
 * Classic Android timed vibrator: a value in milliseconds, 0 stops.  Meizu's
 * HAL writes the waveform number first, then the duration here.
 */
static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 duration_ms;
	int rc;

	rc = kstrtou32(buf, 0, &duration_ms);
	if (rc)
		return rc;

	if (!duration_ms) {
		aw8697_compat_stop();
		return count;
	}

	/* Loop the selected waveform for the requested time. */
	aw8697_compat_play_ram(vib->wave, 1,
			       flyme_gain_of(vib, FLYME_GAIN_MEDIUM),
			       duration_ms);

	return count;
}
static FLYME_DEVICE_ATTR_RW(enable);

static struct attribute *flyme_timed_attrs[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group flyme_timed_group = {
	.attrs = flyme_timed_attrs,
};

/* ------------------------------------------------------------------ *
 * /sys/class/leds/vibrator
 *
 * FlymeVibratorHelper keeps three more constants on the LED class:
 * CONTROL_PATH_EFFECTID, CONTROL_PATH_ACTIVATE and CONTROL_PATH_GAIN.
 * effect_id speaks the same id space as on_off, activate is the 0/1
 * start-stop of the last effect, and gain is the raw AW8697 gain value.
 * ------------------------------------------------------------------ */

static ssize_t activate_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", vib->last_effect ? 1 : 0);
}

static ssize_t activate_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 on;
	int rc;

	rc = kstrtou32(buf, 0, &on);
	if (rc)
		return rc;

	if (!on) {
		aw8697_compat_stop();
		return count;
	}

	if (vib->last_effect)
		flyme_play_effect(vib, vib->last_effect);

	return count;
}
static FLYME_DEVICE_ATTR_RW(activate);

static ssize_t led_gain_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", vib->led_gain);
}

/* Raw gain register value; it overrides the effect table for later plays. */
static ssize_t led_gain_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct flyme_vibrator *vib = flyme_vib_get(dev);
	u32 gain;
	int rc;

	rc = kstrtou32(buf, 0, &gain);
	if (rc)
		return rc;
	if (gain > 0xff)
		gain = 0xff;

	mutex_lock(&vib->lock);
	vib->led_gain = gain;
	mutex_unlock(&vib->lock);

	return count;
}
static FLYME_DEVICE_ATTR_RW(led_gain);

/* Same effect id space as on_off, so it reuses those handlers. */
static struct device_attribute dev_attr_led_effect_id =
	__ATTR(effect_id, 0660, on_off_show, on_off_store);

static struct attribute *flyme_led_attrs[] = {
	&dev_attr_led_effect_id.attr,
	&dev_attr_activate.attr,
	&dev_attr_led_gain.attr,
	NULL,
};

static const struct attribute_group flyme_led_group = {
	.attrs = flyme_led_attrs,
};

/* ------------------------------------------------------------------ *
 * registration, driven by the AW8697 probe
 * ------------------------------------------------------------------ */

int flyme_vibrator_register(void)
{
	struct flyme_vibrator *vib;
	int rc;

	if (flyme_vib)
		return 0;

	vib = kzalloc(sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	mutex_init(&vib->lock);
	vib->wave = 2;
	vib->mback_wave = 1;
	vib->mback_strength = 1;
	vib->cspress_strength = 1;
	vib->freq = 235;
	vib->rtp_wave = 2;
	vib->rtp_duration_ms = 20;

	vib->motor_class = class_create(THIS_MODULE, "meizu");
	if (IS_ERR(vib->motor_class)) {
		rc = PTR_ERR(vib->motor_class);
		goto err_free;
	}

	vib->motor_dev = device_create(vib->motor_class, NULL, MKDEV(0, 0), vib,
				       FLYME_VIBRATOR_DEV_NAME);
	if (IS_ERR(vib->motor_dev)) {
		rc = PTR_ERR(vib->motor_dev);
		goto err_motor_class;
	}

	rc = sysfs_create_group(&vib->motor_dev->kobj, &flyme_motor_group);
	if (rc)
		goto err_motor_dev;

	vib->timed_class = class_create(THIS_MODULE, "timed_output");
	if (IS_ERR(vib->timed_class)) {
		rc = PTR_ERR(vib->timed_class);
		goto err_motor_group;
	}

	vib->timed_dev = device_create(vib->timed_class, NULL, MKDEV(0, 0), vib,
				       "vibrator");
	if (IS_ERR(vib->timed_dev)) {
		rc = PTR_ERR(vib->timed_dev);
		goto err_timed_class;
	}

	rc = sysfs_create_group(&vib->timed_dev->kobj, &flyme_timed_group);
	if (rc)
		goto err_timed_dev;

	vib->led.name = FLYME_LED_NAME;
	vib->led.max_brightness = 0xff;
	rc = led_classdev_register(NULL, &vib->led);
	if (rc) {
		pr_err("failed to register LED class device: %d\n", rc);
		goto err_timed_group;
	}

	dev_set_drvdata(vib->led.dev, vib);
	rc = sysfs_create_group(&vib->led.dev->kobj, &flyme_led_group);
	if (rc)
		goto err_led;

	flyme_vib = vib;

	pr_info("Flyme vibrator compatibility layer ready\n");

	return 0;

err_led:
	led_classdev_unregister(&vib->led);
err_timed_group:
	sysfs_remove_group(&vib->timed_dev->kobj, &flyme_timed_group);
err_timed_dev:
	device_destroy(vib->timed_class, MKDEV(0, 0));
err_timed_class:
	class_destroy(vib->timed_class);
err_motor_group:
	sysfs_remove_group(&vib->motor_dev->kobj, &flyme_motor_group);
err_motor_dev:
	device_destroy(vib->motor_class, MKDEV(0, 0));
err_motor_class:
	class_destroy(vib->motor_class);
err_free:
	kfree(vib);

	pr_err("failed to create Meizu motor nodes: %d\n", rc);

	return rc;
}

void flyme_vibrator_unregister(void)
{
	struct flyme_vibrator *vib = flyme_vib;

	if (!vib)
		return;

	flyme_vib = NULL;

	aw8697_compat_stop();

	sysfs_remove_group(&vib->led.dev->kobj, &flyme_led_group);
	led_classdev_unregister(&vib->led);

	sysfs_remove_group(&vib->timed_dev->kobj, &flyme_timed_group);
	device_destroy(vib->timed_class, MKDEV(0, 0));
	class_destroy(vib->timed_class);

	sysfs_remove_group(&vib->motor_dev->kobj, &flyme_motor_group);
	device_destroy(vib->motor_class, MKDEV(0, 0));
	class_destroy(vib->motor_class);

	kfree(vib);
}
