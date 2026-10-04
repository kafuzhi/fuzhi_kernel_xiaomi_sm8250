/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Public surface of the Flyme / Meizu vibrator compatibility layer.
 *
 * Kept separate from aw8697.h on purpose: that header defines a file scope
 * table (pctl_names) which already breaks the build if a second translation
 * unit includes it, so the compatibility layer deliberately does not pull it
 * in.  Everything shared here only uses plain scalar types.
 *
 * Copyright (c) 2026 fuzhi
 */
#ifndef __AW8697_COMPAT_H__
#define __AW8697_COMPAT_H__

#include <linux/types.h>

/* AW8697 gain register values used for Meizu's strength levels. */
#define AW8697_COMPAT_GAIN_LIGHT 0x30
#define AW8697_COMPAT_GAIN_MEDIUM 0x50
#define AW8697_COMPAT_GAIN_STRONG 0x80

/* Highest waveform index in the AW8697 RAM sequencer (it holds 8 of them). */
#define AW8697_COMPAT_WAVE_MAX 7

#ifdef CONFIG_INPUT_AW8697_HAPTIC_FLYME_VIBRATOR
int aw8697_compat_play_ram(unsigned char wave, bool loop, unsigned char gain,
			   unsigned int duration_ms);
int aw8697_compat_stop(void);

int flyme_vibrator_register(void);
void flyme_vibrator_unregister(void);
#else
static inline int flyme_vibrator_register(void)
{
	return 0;
}

static inline void flyme_vibrator_unregister(void)
{
}
#endif

#endif /* __AW8697_COMPAT_H__ */
