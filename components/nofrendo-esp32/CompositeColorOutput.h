
/* Copyright (c) 2020, Peter Barrett
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#ifndef __COMPOSITE_COLOR_OUTPUT_H__
#define __COMPOSITE_COLOR_OUTPUT_H__

#include <stdint.h>

// #define EMU_ATARI
// #define EMU_NONE
// #define EMU_GBC
#define EMU_NES

// #if defined(CONFIG_HW_COMPOSITE_VIDEO_NTSC)
#define SUPPORT_NTSC
// #elif defined(CONFIG_HW_COMPOSITE_VIDEO_PAL)
// #define SUPPORT_PAL
// #endif

void video_init(void);
void sendFrameHalfResolution(const uint8_t **frame);

#endif /*__COMPOSITE_COLOR_OUTPUT_H__*/
