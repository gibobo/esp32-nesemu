#ifndef __PALETTE_H__
#define __PALETTE_H__

#include "CompositeColorOutput.h"
#include "esp_attr.h"
#include <stdint.h>

#if defined(EMU_ATARI)
#include "palette_atari800.h"
#endif
#if defined(EMU_NES)
#include "palette_nofrendo.h"
#endif

// https://github.com/Roger-random/ESP_8_BIT_composite/blob/main/ESP_8_BIT_composite.cpp

#if defined(SUPPORT_NTSC)
const uint32_t *ntsc_palette() {
#if defined(EMU_ATARI)
  return atari_4_phase_ntsc;
#elif defined(EMU_NES)
  return nes_4_phase;
#endif
};
#endif

#if defined(SUPPORT_PAL)
const uint32_t *pal_palette() {
  return atari_4_phase_pal;
}
#endif

#endif /* __PALETTE_H__ */