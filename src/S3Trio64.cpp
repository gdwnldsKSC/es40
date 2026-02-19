/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project & Others
 * Copyright (C) 2020-2025 by gdwnldsKSC
 * Copyright (C) 2014-2024 by Barry Rodewald, MAME project
 *
 * WWW    : https://github.com/gdwnldsKSC/es40
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Although this is not required, the author would appreciate being notified of,
 * and receiving any modifications you may make to the source code that might serve
 * the general public.
 */

 /**
  * \file
  * Contains the code for emulated S3 Trio 64 Video Card device.
  *
  * $Id$
  *
  * X-1.21       gdwnldsKSC                                      27-AUG-2025
  *      Real S3 BIOS boots now! Not 100% implemented but....
  *      Quite a lot of CRTC implementation and behavior added and fixed
  *
  * X-1.20       Camiel Vanderhoeven                             31-MAY-2008
  *      Changes to include parts of Poco.
  *
  * X-1.19       Camiel Vanderhoeven                             13-APR-2008
  *      Fixed Doxygen comment.
  *
  * X-1.18       Camiel Vanderhoeven                             25-MAR-2008
  *      Added comments on VGA registers.
  *
  * X-1.17       Camiel Vanderhoeven                             16-MAR-2008
  *      Fixed threading problems with SDL (I hope).
  *
  * X-1.16       Camiel Vanderhoeven                             14-MAR-2008
  *      Formatting.
  *
  * X-1.15       Camiel Vanderhoeven                             14-MAR-2008
  *   1. More meaningful exceptions replace throwing (int) 1.
  *   2. U64 macro replaces X64 macro.
  *
  * X-1.14       Camiel Vanderhoeven                             13-MAR-2008
  *      Create init(), start_threads() and stop_threads() functions.
  *
  * X-1.13       Camiel Vanderhoeven                             05-MAR-2008
  *      Multi-threading version.
  *
  * X-1.12       Brian Wheeler                                   27-FEB-2008
  *      Avoid compiler warnings.
  *
  * X-1.11       Fang Zhe                                        08-JAN-2008
  *      Endianess.
  *
  * X-1.10       Camiel Vanderhoeven                             02-JAN-2008
  *      Cleanup.
  *
  * X-1.9        Camiel Vanderhoeven                             30-DEC-2007
  *      Print file id on initialization.
  *
  * X-1.8        Camiel Vanderhoeven                             28-DEC-2007
  *      Throw exceptions rather than just exiting when errors occur.
  *
  * X-1.7        Camiel Vanderhoeven                             28-DEC-2007
  *      Keep the compiler happy.
  *
  * X-1.6        Camiel Vanderhoeven                             17-DEC-2007
  *      SaveState file format 2.1
  *
  * X-1.5        Brian Wheeler                                   10-DEC-2007
  *      Made refresh function name unique.
  *
  * X-1.4        Camiel Vanderhoeven                             10-DEC-2007
  *      Use new base class VGA.
  *
  * X-1.3        Camiel Vanderhoeven                             7-DEC-2007
  *      Code cleanup.
  *
  * X-1.2        Camiel Vanderhoeven/Brian Wheeler               6-DEC-2007
  *      Changed implementation (with thanks to the Bochs project!!)
  *
  * X-1.1        Camiel Vanderhoeven                             1-DEC-2007
  *      Initial version in CVS.
  **/
#include "StdAfx.h"
#include "S3Trio64.h"
#include "System.h"
#include "AliM1543C.h"
#include <algorithm>
#include <chrono>
#include "gui/gui.h"
#include "xtal.h"
#include "emu.h"

  // es40 specific
	// turn on or off debug output
#define DEBUG_VGA 1
#define DEBUG_VGA_NOISY 0
#define DEBUG_PCI 0
#define S3_LFB_TRACE 1
// end es40 specific

// begin MAME code

#define LOG_WARN      (1U << 1)
#define LOG_REGS      (1U << 2) // deprecated
#define LOG_DSW       (1U << 3) // Input sense at $3c2
#define LOG_CRTC      (1U << 4) // CRTC setups with monitor geometry

#define VERBOSE (LOG_GENERAL | LOG_CRTC | LOG_WARN | LOG_REGS)
  //#define LOG_OUTPUT_FUNC osd_printf_info
#include "logmacro.h"

// TODO: remove this enum
enum
{
	IBM8514_IDLE = 0,
	IBM8514_DRAWING_RECT,
	IBM8514_DRAWING_LINE,
	IBM8514_DRAWING_BITBLT,
	IBM8514_DRAWING_PATTERN,
	IBM8514_DRAWING_SSV_1,
	IBM8514_DRAWING_SSV_2,
	MACH8_DRAWING_SCAN
};

#define LOGWARN(...)           LOGMASKED(LOG_WARN, __VA_ARGS__)
#define LOGREGS(...)           LOGMASKED(LOG_REGS, __VA_ARGS__)
#define LOGDSW(...)            LOGMASKED(LOG_DSW, __VA_ARGS__)
#define LOGCRTC(...)           LOGMASKED(LOG_CRTC, __VA_ARGS__)


/***************************************************************************

	Local variables

***************************************************************************/

//#define TEXT_LINES (LINES_HELPER)
#define LINES ((vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1))
#define TEXT_LINES (vga.crtc.vert_disp_end+1)

#define GRAPHIC_MODE (vga.gc.alpha_dis) /* else text mode */

#define EGA_COLUMNS (vga.crtc.horz_disp_end+1)
#define EGA_LINE_LENGTH (vga.crtc.offset<<1)

#define VGA_COLUMNS (vga.crtc.horz_disp_end+1)
#define VGA_LINE_LENGTH (vga.crtc.offset<<3)

#define VGA_CH_WIDTH ((vga.sequencer.data[1]&1)?8:9)

#define TEXT_COLUMNS (vga.crtc.horz_disp_end+1)
#define TEXT_START_ADDRESS (vga.crtc.start_addr<<3)
#define TEXT_LINE_LENGTH (vga.crtc.offset<<1)

#define TEXT_COPY_9COLUMN(ch) (((ch & 0xe0) == 0xc0)&&(vga.attribute.data[0x10]&4))

// Special values for SVGA Trident - Mode Vesa 110h
#define TLINES (LINES)
#define TGA_COLUMNS (EGA_COLUMNS)
#define TGA_LINE_LENGTH (vga.crtc.offset<<3)

static unsigned old_iHeight = 0, old_iWidth = 0, old_MSL = 0;

static int s3_diag_update_counter = 0;
static int s3_diag_frame_counter = 0;

// MAME FUNCTIONS - not all present yet

//s3vision864_vga_device::s3vision864_vga_device(const machine_config& mconfig, const char* tag, device_t* owner, uint32_t clock)
//	: s3vision864_vga_device(mconfig, S3_VISION864_VGA, tag, owner, clock)

//s3vision864_vga_device::s3vision864_vga_device(const machine_config& mconfig, device_type type, const char* tag, device_t* owner, uint32_t clock)
//	: svga_device(mconfig, type, tag, owner, clock)

// void s3vision864_vga_device::device_add_mconfig(machine_config &config)

uint32_t CS3Trio64::latch_start_addr()
{
	if (s3.memory_config & 0x08)
	{
		// - SDD scrolling test expects a << 2 for 8bpp and no shift for anything else
		// - Slackware 3.x XF86_S3 expect a << 2 shift (to be confirmed)
		// - przonegd expect no shift (RGB16)
		return vga.crtc.start_addr_latch << (svga.rgb8_en ? 2 : 0);
	}
	return vga.crtc.start_addr_latch;
}

// void s3vision864_vga_device::device_start()

// void s3vision864_vga_device::device_reset()

u16 CS3Trio64::line_compare_mask()
{
	// TODO: pinpoint condition
	return svga.rgb8_en ? 0x7ff : 0x3ff;
}

uint16_t CS3Trio64::offset()
{
	if (s3.memory_config & 0x08)
		return vga.crtc.offset << 3;
	return CVGA::offset();
}

void CS3Trio64::s3_define_video_mode()
{
	int divisor = 1;
	int xtal = ((vga.miscellaneous_output & 0xc) ? XTAL(28'636'363) : XTAL(25'174'800)).value();
	double freq;

	if ((vga.miscellaneous_output & 0xc) == 0x0c)
	{
		// DCLK calculation
		freq = ((double)(s3.clk_pll_m + 2) / (double)((s3.clk_pll_n + 2) * (pow(2.0, s3.clk_pll_r)))) * 14.318; // clock between XIN and XOUT
		xtal = freq * 1000000;
	}

	if ((s3.ext_misc_ctrl_2) >> 4)
	{
		svga.rgb8_en = 0;
		svga.rgb15_en = 0;
		svga.rgb16_en = 0;
		svga.rgb32_en = 0;
		// FIXME: vision864 has only first 7 modes
		switch ((s3.ext_misc_ctrl_2) >> 4)
		{
			// 0001 Mode 8: 2x 8-bit 1 VCLK/2 pixels
		case 0x01: svga.rgb8_en = 1; break;
			// 0010 Mode 1: 15-bit 2 VCLK/pixel
		case 0x02: svga.rgb15_en = 1; break;
			// 0011 Mode 9: 15-bit 1 VCLK/pixel
		case 0x03: svga.rgb15_en = 1; divisor = 2; break;
			// 0100 Mode 2: 24-bit 3 VCLK/pixel
		case 0x04: svga.rgb24_en = 1; break;
			// 0101 Mode 10: 16-bit 1 VCLK/pixel
		case 0x05: svga.rgb16_en = 1; divisor = 2; break;
			// 0110 Mode 3: 16-bit 2 VCLK/pixel
		case 0x06: svga.rgb16_en = 1; break;
			// 0111 Mode 11: 24/32-bit 2 VCLK/pixel
		case 0x07: svga.rgb32_en = 1; divisor = 4; break;
		case 0x0d: svga.rgb32_en = 1; divisor = 1; break;
		default:
			popmessage("pc_vga_s3: PA16B-COLOR-MODE %02x\n", ((s3.ext_misc_ctrl_2) >> 4));
			break;
		}
	}
	else
	{
		// 0000: Mode 0 8-bit 1 VCLK/pixel
		svga.rgb8_en = (s3.memory_config & 8) >> 3;
		svga.rgb15_en = 0;
		svga.rgb16_en = 0;
		svga.rgb32_en = 0;
	}

#ifdef DEBUG_VGA_RENDER
	LOG("S3: define_video_mode: ext_misc_ctrl_2=%02x memory_config=%02x "
		"rgb8=%d rgb15=%d rgb16=%d rgb32=%d crtc_offset=%03x offset()=%d\n",
		s3.ext_misc_ctrl_2, s3.memory_config,
		svga.rgb8_en, svga.rgb15_en, svga.rgb16_en, svga.rgb32_en,
		vga.crtc.offset, offset());
#endif

	recompute_params_clock(divisor, xtal);
}

void CS3Trio64::refresh_pitch_offset()
{
	// bit 2 = bit 8 of offset register, but only if bits 4-5 of CR51 are 00h.
	vga.crtc.offset &= 0xff;
	if ((s3.cr51 & 0x30) == 0)
		vga.crtc.offset |= (s3.cr43 & 0x04) << 6;
	else
		vga.crtc.offset |= (s3.cr51 & 0x30) << 4;
}

void CS3Trio64::crtc_map(address_map& map)
{
	map(0x00, 0x00).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.horz_total & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			// doom (DOS) tries to write to protected regs
			LOGCRTC("CR00 H total %02x %s", data, vga.crtc.protect_enable ? "P?\n" : "-> ");
			if (vga.crtc.protect_enable)
				return;
			vga.crtc.horz_total = (vga.crtc.horz_total & ~0xff) | (data & 0xff);
			LOGCRTC("%04d\n", vga.crtc.horz_total);
			recompute_params();
			})
	);
	map(0x01, 0x01).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.horz_disp_end & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			LOGCRTC("CR01 H display end %02x %s", data, vga.crtc.protect_enable ? "P?\n" : "-> ");
			if (vga.crtc.protect_enable)
				return;
			vga.crtc.horz_disp_end = (vga.crtc.horz_disp_end & ~0xff) | (data & 0xff);
			LOGCRTC("%04d\n", vga.crtc.horz_disp_end);
			recompute_params();
			})
	);
	map(0x02, 0x02).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.horz_blank_start & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			LOGCRTC("CR02 H start blank %02x %s", data, vga.crtc.protect_enable ? "P?\n" : "-> ");
			if (vga.crtc.protect_enable)
				return;
			vga.crtc.horz_blank_start = (vga.crtc.horz_blank_start & ~0xff) | (data & 0xff);
			LOGCRTC("%04d\n", vga.crtc.horz_blank_start);
			})
	);
	map(0x03, 0x03).lrw8(
		NAME([this](offs_t offset) {
			u8 res = vga.crtc.horz_blank_end & 0x1f;
			res |= (vga.crtc.disp_enable_skew & 3) << 5;
			res |= (vga.crtc.evra & 1) << 7;
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			LOGCRTC("CR03 H blank end %02x %s", data, vga.crtc.protect_enable ? "P?\n" : "-> ");
			if (vga.crtc.protect_enable)
				return;
			vga.crtc.horz_blank_end &= ~0x1f;
			vga.crtc.horz_blank_end |= data & 0x1f;
			vga.crtc.disp_enable_skew = (data & 0x60) >> 5;
			vga.crtc.evra = BIT(data, 7);
			LOGCRTC("%04d evra %d display enable skew %01x\n", vga.crtc.horz_blank_end, vga.crtc.evra, vga.crtc.disp_enable_skew);
			})
	);
	map(0x04, 0x04).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.horz_retrace_start & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			LOGCRTC("CR04 H retrace start %02x %s", data, vga.crtc.protect_enable ? "P?\n" : "-> ");
			if (vga.crtc.protect_enable)
				return;
			vga.crtc.horz_retrace_start = (vga.crtc.horz_retrace_start & ~0xff) | (data & 0xff);
			LOGCRTC("%04d\n", vga.crtc.horz_retrace_start);
			})
	);
	map(0x05, 0x05).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.horz_blank_end & 0x20) << 2;
			res |= (vga.crtc.horz_retrace_skew & 3) << 5;
			res |= (vga.crtc.horz_retrace_end & 0x1f);
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			LOGCRTC("CR05 H blank end %02x %s", data, vga.crtc.protect_enable ? "P?\n" : "-> ");
			if (vga.crtc.protect_enable)
				return;
			vga.crtc.horz_blank_end &= ~0x20;
			vga.crtc.horz_blank_end |= ((data & 0x80) >> 2);
			vga.crtc.horz_retrace_skew = ((data & 0x60) >> 5);
			vga.crtc.horz_retrace_end = (vga.crtc.horz_retrace_end & ~0x1f) | (data & 0x1f);
			LOGCRTC("%04d retrace skew %01d retrace end %02d\n"
				, vga.crtc.horz_blank_end
				, vga.crtc.horz_retrace_skew
				, vga.crtc.horz_retrace_end
			);
			})
	);
	map(0x06, 0x06).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.vert_total & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			LOGCRTC("CR06 V total %02x %s", data, vga.crtc.protect_enable ? "P?\n" : "-> ");
			if (vga.crtc.protect_enable)
				return;
			vga.crtc.vert_total &= ~0xff;
			vga.crtc.vert_total |= data & 0xff;
			LOGCRTC("%04d\n", vga.crtc.vert_total);
			recompute_params();
			})
	);
	// Overflow Register
	map(0x07, 0x07).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.line_compare & 0x100) >> 4;
			res |= (vga.crtc.vert_retrace_start & 0x200) >> 2;
			res |= (vga.crtc.vert_disp_end & 0x200) >> 3;
			res |= (vga.crtc.vert_total & 0x200) >> 4;
			res |= (vga.crtc.vert_blank_start & 0x100) >> 5;
			res |= (vga.crtc.vert_retrace_start & 0x100) >> 6;
			res |= (vga.crtc.vert_disp_end & 0x100) >> 7;
			res |= (vga.crtc.vert_total & 0x100) >> 8;
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.line_compare &= ~0x100;
			vga.crtc.line_compare |= ((data & 0x10) << (8 - 4));
			LOGCRTC("CR07 Overflow %02x -> line compare %04d %s", data, vga.crtc.line_compare, vga.crtc.protect_enable ? "P?\n" : "");
			if (vga.crtc.protect_enable)
				return;
			vga.crtc.vert_total &= ~0x300;
			vga.crtc.vert_retrace_start &= ~0x300;
			vga.crtc.vert_disp_end &= ~0x300;
			vga.crtc.vert_blank_start &= ~0x100;
			vga.crtc.vert_retrace_start |= ((data & 0x80) << (9 - 7));
			vga.crtc.vert_disp_end |= ((data & 0x40) << (9 - 6));
			vga.crtc.vert_total |= ((data & 0x20) << (9 - 5));
			vga.crtc.vert_blank_start |= ((data & 0x08) << (8 - 3));
			vga.crtc.vert_retrace_start |= ((data & 0x04) << (8 - 2));
			vga.crtc.vert_disp_end |= ((data & 0x02) << (8 - 1));
			vga.crtc.vert_total |= ((data & 0x01) << (8 - 0));
			LOGCRTC("V total %04d V retrace start %04d V display end %04d V blank start %04d\n"
				, vga.crtc.vert_total
				, vga.crtc.vert_retrace_start
				, vga.crtc.vert_disp_end
				, vga.crtc.vert_blank_start
			);
			recompute_params();
			})
	);
	// Preset Row Scan Register
	map(0x08, 0x08).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.byte_panning & 3) << 5;
			res |= (vga.crtc.preset_row_scan & 0x1f);
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.byte_panning = (data & 0x60) >> 5;
			vga.crtc.preset_row_scan = (data & 0x1f);
			LOGCRTC("CR08 Preset Row Scan %02x -> %02d byte panning %d\n"
				, data
				, vga.crtc.preset_row_scan
				, vga.crtc.byte_panning
			);
			})
	);
	// Maximum Scan Line Register
	map(0x09, 0x09).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.maximum_scan_line - 1) & 0x1f;
			res |= (vga.crtc.scan_doubling & 1) << 7;
			res |= (vga.crtc.line_compare & 0x200) >> 3;
			res |= (vga.crtc.vert_blank_start & 0x200) >> 4;
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.line_compare &= ~0x200;
			vga.crtc.vert_blank_start &= ~0x200;
			vga.crtc.scan_doubling = ((data & 0x80) >> 7);
			vga.crtc.line_compare |= ((data & 0x40) << (9 - 6));
			vga.crtc.vert_blank_start |= ((data & 0x20) << (9 - 5));
			vga.crtc.maximum_scan_line = (data & 0x1f) + 1;
			LOGCRTC("CR09 Maximum Scan Line %02x -> %02d V blank start %04d line compare %04d scan doubling %d\n"
				, data
				, vga.crtc.maximum_scan_line
				, vga.crtc.vert_blank_start
				, vga.crtc.line_compare
				, vga.crtc.scan_doubling
			);
			})
	);
	map(0x0a, 0x0a).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.cursor_scan_start & 0x1f);
			res |= ((vga.crtc.cursor_enable & 1) ^ 1) << 5;
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.cursor_enable = ((data & 0x20) ^ 0x20) >> 5;
			vga.crtc.cursor_scan_start = data & 0x1f;
			state.vga_mem_updated = 1; // text mode: cursor shape change triggers update_text_mode()
			})
	);
	map(0x0b, 0x0b).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.cursor_skew & 3) << 5;
			res |= (vga.crtc.cursor_scan_end & 0x1f);
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.cursor_skew = (data & 0x60) >> 5;
			vga.crtc.cursor_scan_end = data & 0x1f;
			state.vga_mem_updated = 1; // text mode: cursor shape change triggers update_text_mode()
			})
	);
	map(0x0c, 0x0d).lrw8(
		NAME([this](offs_t offset) {
			return (vga.crtc.start_addr_latch >> ((offset & 1) ^ 1) * 8) & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.start_addr_latch &= ~(0xff << (((offset & 1) ^ 1) * 8));
			vga.crtc.start_addr_latch |= (data << (((offset & 1) ^ 1) * 8));
			state.vga_mem_updated = 1; // text mode: cursor shape change triggers update_text_mode()
			})
	);
	map(0x0e, 0x0f).lrw8(
		NAME([this](offs_t offset) {
			return (vga.crtc.cursor_addr >> ((offset & 1) ^ 1) * 8) & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.cursor_addr &= ~(0xff << (((offset & 1) ^ 1) * 8));
			vga.crtc.cursor_addr |= (data << (((offset & 1) ^ 1) * 8));
			state.vga_mem_updated = 1; // text mode: cursor shape change triggers update_text_mode()
			})
	);
	map(0x10, 0x10).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.vert_retrace_start & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.vert_retrace_start &= ~0xff;
			vga.crtc.vert_retrace_start |= data & 0xff;
			LOGCRTC("CR10 V retrace start %02x -> %04d\n", data, vga.crtc.vert_retrace_start);
			})
	);
	map(0x11, 0x11).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.protect_enable & 1) << 7;
			res |= (vga.crtc.bandwidth & 1) << 6;
			res |= (vga.crtc.vert_retrace_end & 0xf);
			res |= (vga.crtc.irq_clear & 1) << 4;
			res |= (vga.crtc.irq_disable & 1) << 5;
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.protect_enable = BIT(data, 7);
			vga.crtc.bandwidth = BIT(data, 6);
			// IRQ: Original VGA only supports this for PS/2, but clone cards may supports this on ISA too
			// see https://scalibq.wordpress.com/2022/12/06/the-myth-of-the-vertical-retrace-interrupt/
			vga.crtc.irq_disable = BIT(data, 5);
			vga.crtc.irq_clear = BIT(data, 4);
			vga.crtc.vert_retrace_end = (vga.crtc.vert_retrace_end & ~0xf) | (data & 0x0f);

			if (vga.crtc.irq_clear == 0)
			{
				vga.crtc.irq_latch = 0;
				m_vsync_cb(0);
			}

			LOGCRTC("CR11 V retrace end %02x -> %02d protect enable %d bandwidth %d irq %02x\n"
				, data
				, vga.crtc.vert_retrace_end
				, vga.crtc.protect_enable
				, vga.crtc.bandwidth
				, data & 0x30
			);
			})
	);
	map(0x12, 0x12).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.vert_disp_end & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.vert_disp_end &= ~0xff;
			vga.crtc.vert_disp_end |= data & 0xff;
			LOGCRTC("CR12 V display end %02x -> %04d\n", data, vga.crtc.vert_disp_end);
			recompute_params();
			})
	);
	map(0x13, 0x13).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.offset & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.offset &= ~0xff;
			vga.crtc.offset |= data & 0xff;
			})
	);
	map(0x14, 0x14).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.dw & 1) << 6;
			res |= (vga.crtc.div4 & 1) << 5;
			res |= (vga.crtc.underline_loc & 0x1f);
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.dw = (data & 0x40) >> 6;
			vga.crtc.div4 = (data & 0x20) >> 5;
			vga.crtc.underline_loc = (data & 0x1f);
			})
	);
	map(0x15, 0x15).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.vert_blank_start & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.vert_blank_start &= ~0xff;
			vga.crtc.vert_blank_start |= data & 0xff;
			LOGCRTC("CR15 V blank start %02x -> %04d\n", data, vga.crtc.vert_blank_start);
			})
	);
	map(0x16, 0x16).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.vert_blank_end & 0x7f;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.vert_blank_end = (vga.crtc.vert_blank_end & ~0x7f) | (data & 0x7f);
			LOGCRTC("CR16 V blank end %02x -> %04d\n", data, vga.crtc.vert_blank_end);
			})
	);
	map(0x17, 0x17).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.sync_en & 1) << 7;
			res |= (vga.crtc.word_mode & 1) << 6;
			res |= (vga.crtc.aw & 1) << 5;
			res |= (vga.crtc.div2 & 1) << 3;
			res |= (vga.crtc.sldiv & 1) << 2;
			res |= (vga.crtc.map14 & 1) << 1;
			res |= (vga.crtc.map13 & 1) << 0;
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.sync_en = BIT(data, 7);
			vga.crtc.word_mode = BIT(data, 6);
			vga.crtc.aw = BIT(data, 5);
			vga.crtc.div2 = BIT(data, 3);
			vga.crtc.sldiv = BIT(data, 2);
			vga.crtc.map14 = BIT(data, 1);
			vga.crtc.map13 = BIT(data, 0);
			LOGCRTC("CR17 Mode control %02x -> Sync Enable %d Word/Byte %d Address Wrap select %d\n"
				, data
				, vga.crtc.sync_en
				, vga.crtc.word_mode
				, vga.crtc.aw
			);
			LOGCRTC("\tDIV2 %d Scan Line Divide %d MAP14 %d MAP13 %d\n"
				, vga.crtc.div2
				, vga.crtc.sldiv
				, vga.crtc.map14
				, vga.crtc.map13
			);
			})
	);
	map(0x18, 0x18).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.line_compare & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.line_compare &= ~0xff;
			vga.crtc.line_compare |= data & 0xff;
			LOGCRTC("CR18 Line Compare %02x -> %04d\n", data, vga.crtc.line_compare);
			})
	);
	// TODO: (undocumented) CR22 Memory Data Latch Register (read only)
	// map(0x22, 0x22).lr8(
	// (undocumented) CR24 Attribute Controller Toggle Register (read only)
		// 0--- ---- index
		// 1--- ---- data
	map(0x24, 0x24).lr8(
		NAME([this](offs_t offset) {
			if (!machine().side_effects_disabled())
				LOG("CR24 read undocumented Attribute reg\n");
			return vga.attribute.state << 7;
			})
	);
	map(0x2d, 0x2d).lr8(
		NAME([this](offs_t offset) {
			return s3.id_high;
			})
	);
	map(0x2e, 0x2e).lr8(
		NAME([this](offs_t offset) {
			return s3.id_low;
			})
	);
	map(0x2f, 0x2f).lr8(
		NAME([this](offs_t offset) {
			return s3.revision;
			})
	);
	// CR30 Chip ID/REV register
	map(0x30, 0x30).lr8(
		NAME([this](offs_t offset) {
			return s3.id_cr30;
			})
	);
	// CR31 Memory Configuration Register
	map(0x31, 0x31).lrw8(
		NAME([this](offs_t offset) {
			return s3.memory_config;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.memory_config = data;
			vga.crtc.start_addr_latch &= ~0x30000;
			vga.crtc.start_addr_latch |= ((data & 0x30) << 12);
			s3_define_video_mode();
			})
	);
	// TODO: CR32, CR33 & CR34 (backward compatibility) - in our ES40 we do these ! :)
	// CR32: Backward Compatibility 1 (BKWD_1)
	map(0x32, 0x32).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr32;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr32 = data;
			})
	);
	// CR33: Backward Compatibility 2 (BKWD_2)
	map(0x33, 0x33).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr33;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr33 = data;
			// ES40 extension: CR33 bit5 forces 8-dot chars
			recompute_scanline_layout();
			state.vga_mem_updated = 1;
			})
	);
	// CR34: Backward Compatibility 3 (BKWD_3)
	map(0x34, 0x34).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr34;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr34 = data;
			})
	);
	map(0x35, 0x35).lrw8(
		NAME([this](offs_t offset) {
			return s3.crt_reg_lock;
			}),
		NAME([this](offs_t offset, u8 data) {
			// lock register
			if ((s3.reg_lock1 & 0xc) != 8 || ((s3.reg_lock1 & 0xc0) == 0))
				return;
			s3.crt_reg_lock = data;
			svga.bank_w = data & 0xf;
			svga.bank_r = svga.bank_w;
			})
	);
	// Configuration register 1
	map(0x36, 0x36).lrw8(
		NAME([this](offs_t offset) {
			// PCI (not really), Fast Page Mode DRAM
			return s3.strapping & 0x000000ff;
			}),
		NAME([this](offs_t offset, u8 data) {
			if (s3.reg_lock2 == 0xa5)
			{
				s3.strapping = (s3.strapping & 0xffffff00) | data;
				LOG("CR36: Strapping data = %08x\n", s3.strapping);
			}
			})
	);
	// Configuration register 2
	map(0x37, 0x37).lrw8(
		NAME([this](offs_t offset) {
			return (s3.strapping & 0x0000ff00) >> 8;  // enable chipset, 64k BIOS size, internal DCLK/MCLK
			}),
		NAME([this](offs_t offset, u8 data) {
			// TODO: monitor ID at 7-5 (PD15-13)
			if (s3.reg_lock2 == 0xa5)
			{
				s3.strapping = (s3.strapping & 0xffff00ff) | (data << 8);
				LOG("CR37: Strapping data = %08x\n", s3.strapping);
			}
			})
	);
	map(0x38, 0x38).lrw8(
		NAME([this](offs_t offset) {
			return s3.reg_lock1;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.reg_lock1 = data;
			})
	);
	map(0x39, 0x39).lrw8(
		NAME([this](offs_t offset) {
			return s3.reg_lock2;
			}),
		NAME([this](offs_t offset, u8 data) {
			// TODO: reg lock mechanism
			s3.reg_lock2 = data;
			})
	);
	// CR3A: Miscellaneous 1 Register (MISC_1)
	map(0x3a, 0x3a).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr3a;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr3a = data;
			})
	);
	// CR3B: Data Transfer Position (DT_EX-POS)
	map(0x3b, 0x3b).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr3b;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr3b = data;
			})
	);
	// CR3C: Interlace Retrace Start (IL_RTSTART)
	map(0x3c, 0x3c).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr3c;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr3c = data;
			})
	);
	// CR40: System Configuration Register (SYS_CNFG)
	map(0x40, 0x40).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr40;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr40 = data;
			// enable 8514/A registers (x2e8, x6e8, xae8, xee8)
			s3.enable_8514 = BIT(data, 0);
			})
	);
	// CR41 
	map(0x41, 0x41).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr41;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr41 = data;
			})
	);
	// CR42 Mode Control
	map(0x42, 0x42).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr42;
			}),
		NAME([this](offs_t offset, u8 data) {
			// bit 5 = interlace, bits 0-3 = dot clock (seems to be undocumented)
			s3.cr42 = data;
			s3_define_video_mode();
			})
	);
	map(0x43, 0x43).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr43;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr43 = data;  // bit 2 = bit 8 of offset register, but only if bits 4-5 of CR51 are 00h.
			refresh_pitch_offset();
			s3_define_video_mode();
			})
	);
	/*
	CR45 Hardware Graphics Cursor Mode
	bit    0  HWGC ENB. Hardware Graphics Cursor Enable. Set to enable the
			  HardWare Cursor in VGA and enhanced modes.
		   1  (911/24) Delay Timing for Pattern Data Fetch
		   2  (801/5,928) Hardware Cursor Horizontal Stretch 2. If set the cursor
			   pixels are stretched horizontally to two bytes and items 0 and 1 of
			   the fore/background stacks in 3d4h index 4Ah/4Bh are used.
		   3  (801/5,928) Hardware Cursor Horizontal Stretch 3. If set the cursor
			   pixels are stretched horizontally to three bytes and items 0,1 and
			   2 of the fore/background stacks in 3d4h index 4Ah/4Bh are used.
		 2-3  (805i,864/964) HWC-CSEL. Hardware Cursor Color Select.
				0: 4/8bit, 1: 15/16bt, 2: 24bit, 3: 32bit
			  Note: So far I've had better luck with: 0: 8/15/16bit, 1: 32bit??
		   4  (80x +) Hardware Cursor Right Storage. If set the cursor data is
			   stored in the last 256 bytes of 4 1Kyte lines (4bits/pixel) or the
			   last 512 bytes of 2 2Kbyte lines (8bits/pixel). Intended for
			   1280x1024 modes where there are no free lines at the bottom.
		   5  (928) Cursor Control Enable for Brooktree Bt485 DAC. If set and 3d4h
			   index 55h bit 5 is set the HC1 output becomes the ODF and the HC0
			   output becomes the CDE
			  (964) BT485 ODF Selection for Bt485A RAMDAC. If set pin 185 (RS3
			   /ODF) is the ODF output to a Bt485A compatible RamDAC (low for even
			   fields and high for odd fields), if clear pin185 is the RS3 output.
	 */
	map(0x45, 0x45).lrw8(
		NAME([this](offs_t offset) {
			const u8 res = s3.cursor_mode;
			if (!machine().side_effects_disabled())
			{
				s3.cursor_fg_ptr = 0;
				s3.cursor_bg_ptr = 0;
			}
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_mode = data;
			})
	);
	/*
	CR46/7 Hardware Graphics Cursor Origin-X
	bit 0-10  The HardWare Cursor X position. For 64k modes this value should be
			  twice the actual X co-ordinate.
	 */
	map(0x46, 0x46).lrw8(
		NAME([this](offs_t offset) {
			return (s3.cursor_x & 0xff00) >> 8;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_x = (s3.cursor_x & 0x00ff) | (data << 8);
			})
	);
	map(0x47, 0x47).lrw8(
		NAME([this](offs_t offset) {
			return s3.cursor_x & 0x00ff;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_x = (s3.cursor_x & 0xff00) | data;
			})
	);
	/*
	CR48/9 Hardware Graphics Cursor Origin-Y
	bit  0-9  (911/24) The HardWare Cursor Y position.
		0-10  (80x +) The HardWare Cursor Y position.
	Note: The position is activated when the high byte of the Y coordinate (index
		  48h) is written, so this byte should be written last (not 911/924 ?)
	 */
	map(0x48, 0x48).lrw8(
		NAME([this](offs_t offset) {
			return (s3.cursor_y & 0xff00) >> 8;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_y = (s3.cursor_y & 0x00ff) | (data << 8);
			})
	);
	map(0x49, 0x49).lrw8(
		NAME([this](offs_t offset) {
			return s3.cursor_y & 0x00ff;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_y = (s3.cursor_y & 0xff00) | data;
			})
	);
	/*
	CR4A Hardware Graphics Cursor Foreground Stack       (80x +)
	bit  0-7  The Foreground Cursor color. Three bytes (4 for the 864/964) are
			  stacked here. When the Cursor Mode register (3d4h index 45h) is read
			  the stackpointer is reset. When a byte is written the byte is
			  written into the current top of stack and the stackpointer is
			  increased. The first byte written (item 0) is allways used, the
			  other two(3) only when Hardware Cursor Horizontal Stretch (3d4h
			  index 45h bit 2-3) is enabled.
	 */
	map(0x4a, 0x4a).lrw8(
		NAME([this](offs_t offset) {
			const u8 res = s3.cursor_fg[s3.cursor_fg_ptr];
			if (!machine().side_effects_disabled())
			{
				s3.cursor_fg_ptr++;
				s3.cursor_fg_ptr %= 4;
			}
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_fg[s3.cursor_fg_ptr++] = data;
			s3.cursor_fg_ptr %= 4;
			})
	);
	/*
	CR4B Hardware Graphics Cursor Background Stack       (80x +)
	bit  0-7  The Background Cursor color. Three bytes (4 for the 864/964) are
			  stacked here. When the Cursor Mode register (3d4h index 45h) is read
			  the stackpointer is reset. When a byte is written the byte is
			  written into the current top of stack and the stackpointer is
			  increased. The first byte written (item 0) is allways used, the
			  other two(3) only when Hardware Cursor Horizontal Stretch (3d4h
			  index 45h bit 2-3) is enabled.
	 */
	map(0x4b, 0x4b).lrw8(
		NAME([this](offs_t offset) {
			const u8 res = s3.cursor_bg[s3.cursor_bg_ptr];
			if (!machine().side_effects_disabled())
			{
				s3.cursor_bg_ptr++;
				s3.cursor_bg_ptr %= 4;
			}
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_bg[s3.cursor_bg_ptr++] = data;
			s3.cursor_bg_ptr %= 4;
			})
	);
	/*
	CR4C/D Hardware Graphics Cursor Storage Start Address
	bit  0-9  (911,924) HCS_STADR. Hardware Graphics Cursor Storage Start Address
		0-11  (80x,928) HWGC_STA. Hardware Graphics Cursor Storage Start Address
		0-12  (864,964) HWGC_STA. Hardware Graphics Cursor Storage Start Address
			  Address of the HardWare Cursor Map in units of 1024 bytes (256 bytes
			  for planar modes). The cursor map is a 64x64 bitmap with 2 bits (A
			  and B) per pixel. The map is stored as one word (16 bits) of bit A,
			  followed by one word with the corresponding 16 B bits.
			  The bits are interpreted as:
				 A    B    MS-Windows:         X-11:
				 0    0    Background          Screen data
				 0    1    Foreground          Screen data
				 1    0    Screen data         Background
				 1    1    Inverted screen     Foreground
			  The Windows/X11 switch is only available for the 80x +.
			  (911/24) For 64k color modes the cursor is stored as one byte (8
				bits) of A bits, followed by the 8 B-bits, and each bit in the
				cursor should be doubled to provide a consistent cursor image.
			  (801/5,928) For Hi/True color modes use the Horizontal Stretch bits
				(3d4h index 45h bits 2 and 3).
	 */
	map(0x4c, 0x4c).lrw8(
		NAME([this](offs_t offset) {
			return (s3.cursor_start_addr & 0xff00) >> 8;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_start_addr = (s3.cursor_start_addr & 0x00ff) | (data << 8);
			//popmessage("HW Cursor Data Address %04x\n",s3.cursor_start_addr);
			})
	);
	map(0x4d, 0x4d).lrw8(
		NAME([this](offs_t offset) {
			return s3.cursor_start_addr & 0x00ff;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_start_addr = (s3.cursor_start_addr & 0xff00) | data;
			//popmessage("HW Cursor Data Address %04x\n",s3.cursor_start_addr);
			})
	);
	/*
	CR4E HGC Pattern Disp Start X-Pixel Position
	bit  0-5  Pattern Display Start X-Pixel Position.
	 */
	map(0x4e, 0x4e).lrw8(
		NAME([this](offs_t offset) {
			return s3.cursor_pattern_x;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_pattern_x = data;
			})
	);
	/*
	CR4F HGC Pattern Disp Start Y-Pixel Position
	bit  0-5  Pattern Display Start Y-Pixel Position.
	 */
	map(0x4f, 0x4f).lrw8(
		NAME([this](offs_t offset) {
			return s3.cursor_pattern_y;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cursor_pattern_y = data;
			})
	);
	// CR50
	map(0x50, 0x50).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr50;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr50 = data;
			})
	);
	map(0x51, 0x51).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.crtc.start_addr_latch & 0x0c0000) >> 18;
			res |= ((svga.bank_w & 0x30) >> 2);
			//          res   |= ((vga.crtc.offset & 0x0300) >> 4);
			res |= (s3.cr51 & 0x30);
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr51 = data;
			vga.crtc.start_addr_latch &= ~0xc0000;
			vga.crtc.start_addr_latch |= ((data & 0x3) << 18);
			svga.bank_w = (svga.bank_w & 0xcf) | ((data & 0x0c) << 2);
			svga.bank_r = svga.bank_w;
			refresh_pitch_offset();
			s3_define_video_mode();
			})
	);
	// Extended BIOS flag 1 register (EXT_BBFLG1) (CR52)
	map(0x52, 0x52).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr52;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr52 = data;
			})
	);
	map(0x53, 0x53).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr53;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr53 = data;
			})
	);
	// Extended Memory Control 2 Register (EX_MCTL_2) (CR54) 
	map(0x54, 0x54).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr54;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr54 = data;
			})
	);
	/*
	CR55 Extended Video DAC Control Register             (80x +)
	bit 0-1  DAC Register Select Bits. Passed to the RS2 and RS3 pins on the
			 RAMDAC, allowing access to all 8 or 16 registers on advanced RAMDACs.
			 If this field is 0, 3d4h index 43h bit 1 is active.
		  2  Enable General Input Port Read. If set DAC reads are disabled and the
			 STRD strobe for reading the General Input Port is enabled for reading
			 while DACRD is active, if clear DAC reads are enabled.
		  3  (928) Enable External SID Operation if set. If set video data is
			   passed directly from the VRAMs to the DAC rather than through the
			   VGA chip
		  4  Hardware Cursor MS/X11 Mode. If set the Hardware Cursor is in X11
			 mode, if clear in MS-Windows mode
		  5  (80x,928) Hardware Cursor External Operation Mode. If set the two
			  bits of cursor data ,is output on the HC[0-1] pins for the video DAC
			  The SENS pin becomes HC1 and the MID2 pin becomes HC0.
		  6  ??
		  7  (80x,928) Disable PA Output. If set PA[0-7] and VCLK are tristated.
			 (864/964) TOFF VCLK. Tri-State Off VCLK Output. VCLK output tri
			  -stated if set
	 */
	map(0x55, 0x55).lrw8(
		NAME([this](offs_t offset) {
			return s3.extended_dac_ctrl;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.extended_dac_ctrl = data;
			})
	);
	// External Sync Control 1 Register (EX_SYNC_1) (CR56)
	map(0x56, 0x56).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr56;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr56 = data & 0x1F; // bits 7-5 reserved
			})
	);
	// External Sync Control 2 Register (EX_SYNC_2) (CR57)
	map(0x57, 0x57).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr57;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr57 = data;
			})
	);
	// Linear Address Window Control Register (LAW_CTL) (CR58) - dosbox calls VGA_StartUpdateLFB() after storing the value
	map(0x58, 0x58).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr58;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr58 = data;
			on_crtc_linear_regs_changed();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			})
	);
	// Linear Address Window Position High
	map(0x59, 0x59).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr59;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr59 = data;
			on_crtc_linear_regs_changed();
			})
	);
	// Linear Address Window Position Low
	map(0x5a, 0x5a).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr5a;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr5a = data;
			on_crtc_linear_regs_changed();
			})
	);
	// undocumented on trio64?
	map(0x5b, 0x5b).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr5b;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr5b = data;
			})
	);
	// TODO: bits 7-4 (w/o?) for GPIO
	map(0x5c, 0x5c).lr8(
		NAME([this](offs_t offset) {
			u8 res = 0;
			// if VGA dot clock is set to 3 (misc reg bits 2-3), then selected dot clock is read, otherwise read VGA clock select
			if ((vga.miscellaneous_output & 0xc) == 0x0c)
				res = s3.cr42 & 0x0f;
			else
				res = (vga.miscellaneous_output & 0xc) >> 2;
			return res;
			})
	);
	// TODO: following two registers must be read-backable
	/*
	CR5D Extended Horizontal Overflow Register           (80x +)
	bit    0  Horizontal Total bit 8. Bit 8 of the Horizontal Total register (3d4h
			  index 0)
		   1  Horizontal Display End bit 8. Bit 8 of the Horizontal Display End
			  register (3d4h index 1)
		   2  Start Horizontal Blank bit 8. Bit 8 of the Horizontal Start Blanking
			  register (3d4h index 2).
		   3  (864,964) EHB+64. End Horizontal Blank +64. If set the /BLANK pulse
			   is extended by 64 DCLKs. Note: Is this bit 6 of 3d4h index 3 or
			   does it really extend by 64 ?
		   4  Start Horizontal Sync Position bit 8. Bit 8 of the Horizontal Start
			  Retrace register (3d4h index 4).
		   5  (864,964) EHS+32. End Horizontal Sync +32. If set the HSYNC pulse
			   is extended by 32 DCLKs. Note: Is this bit 5 of 3d4h index 5 or
			   does it really extend by 32 ?
		   6  (928,964) Data Transfer Position bit 8. Bit 8 of the Data Transfer
				Position register (3d4h index 3Bh)
		   7  (928,964) Bus-Grant Terminate Position bit 8. Bit 8 of the Bus Grant
				Termination register (3d4h index 5Fh).
	*/
	map(0x5d, 0x5d).lrw8(
		NAME([this](offs_t offset) {
			// Recompose CR5D from the extended CRTC fields
			u8 res = 0;
			res |= (vga.crtc.horz_total >> 8) & 0x01;           // bit 0
			res |= ((vga.crtc.horz_disp_end >> 7) & 0x02);      // bit 1
			res |= ((vga.crtc.horz_blank_start >> 6) & 0x04);    // bit 2 (from vga.crtc.horz_blank_start if needed)
			// bit 3: EHB+64 extension — stored in s3.cr5d
			res |= (s3.cr5d & 0x08);
			res |= ((vga.crtc.horz_retrace_start >> 4) & 0x10);  // bit 4
			// bit 5: EHS+32 extension — stored in s3.cr5d
			res |= (s3.cr5d & 0x20);
			// bits 6-7: DTP bit8, BGT bit8 — stored in s3.cr5d
			res |= (s3.cr5d & 0xC0);
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr5d = data;  // ES40: cache for readback of non-decomposed bits
			vga.crtc.horz_total = (vga.crtc.horz_total & 0xfeff) | ((data & 0x01) << 8);
			vga.crtc.horz_disp_end = (vga.crtc.horz_disp_end & 0xfeff) | ((data & 0x02) << 7);
			vga.crtc.horz_blank_start = (vga.crtc.horz_blank_start & 0xfeff) | ((data & 0x04) << 6);
			vga.crtc.horz_blank_end = (vga.crtc.horz_blank_end & 0xffbf) | ((data & 0x08) << 3);
			vga.crtc.horz_retrace_start = (vga.crtc.horz_retrace_start & 0xfeff) | ((data & 0x10) << 4);
			vga.crtc.horz_retrace_end = (vga.crtc.horz_retrace_end & 0xffdf) | (data & 0x20);
			s3_define_video_mode();
			// ES40 extension: recompute derived layout
			recompute_scanline_layout();
			})
	);

	/*
	CR5E: Extended Vertical Overflow Register             (80x +)
	bit    0  Vertical Total bit 10. Bit 10 of the Vertical Total register (3d4h
			  index 6). Bits 8 and 9 are in 3d4h index 7 bit 0 and 5.
		   1  Vertical Display End bit 10. Bit 10 of the Vertical Display End
			  register (3d4h index 12h). Bits 8 and 9 are in 3d4h index 7 bit 1
			  and 6
		   2  Start Vertical Blank bit 10. Bit 10 of the Vertical Start Blanking
			  register (3d4h index 15h). Bit 8 is in 3d4h index 7 bit 3 and bit 9
			  in 3d4h index 9 bit 5
		   4  Vertical Retrace Start bit 10. Bit 10 of the Vertical Start Retrace
			  register (3d4h index 10h). Bits 8 and 9 are in 3d4h index 7 bit 2
			  and 7.
		   6  Line Compare Position bit 10. Bit 10 of the Line Compare register
			  (3d4h index 18h). Bit 8 is in 3d4h index 7 bit 4 and bit 9 in 3d4h
			  index 9 bit 6.
	 */
	map(0x5e, 0x5e).lrw8(
		NAME([this](offs_t offset) {
			// Recompose CR5E from the extended CRTC fields
			u8 res = 0;
			res |= (vga.crtc.vert_total >> 10) & 0x01;           // bit 0
			res |= ((vga.crtc.vert_disp_end >> 9) & 0x02);       // bit 1
			res |= ((vga.crtc.vert_blank_start >> 8) & 0x04);    // bit 2
			// bit 3: reserved
			res |= ((vga.crtc.vert_retrace_start >> 6) & 0x10);  // bit 4
			// bit 5: reserved
			res |= ((vga.crtc.line_compare >> 4) & 0x40);        // bit 6
			// bit 7: reserved
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.vert_total = (vga.crtc.vert_total & 0xfbff) | ((data & 0x01) << 10);
			vga.crtc.vert_disp_end = (vga.crtc.vert_disp_end & 0xfbff) | ((data & 0x02) << 9);
			vga.crtc.vert_blank_start = (vga.crtc.vert_blank_start & 0xfbff) | ((data & 0x04) << 8);
			vga.crtc.vert_retrace_start = (vga.crtc.vert_retrace_start & 0xfbff) | ((data & 0x10) << 6);
			vga.crtc.line_compare = (vga.crtc.line_compare & 0xfbff) | ((data & 0x40) << 4);
			s3_define_video_mode();
			})
	);
	map(0x5f, 0x5f).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr5f;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr5f = data;
			})
	);
	// Extended Memory Control 3 Register (EXT-MCTL-3) (CR60) 
	map(0x60, 0x60).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr60;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr60 = data;
			})
	);
	// Extended Memory Control 4 Register (EXT-MCTL-4) (CR61)
	map(0x61, 0x61).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr61;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr61 = data;
			})
	);
	// undocumented?
	map(0x62, 0x62).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr62;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr62 = data;
			})
	);
	// External Sync Control 3 Register (EX-SYNC-3) (CR63) 
	map(0x63, 0x63).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr63;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr63 = data;
			})
	);
	// undocumented?
	map(0x64, 0x64).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr64;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr64 = data;
			})
	);
	// Extended Miscellaneous Control Register (EXT-MISC-CTL) (CR6S)
	map(0x65, 0x65).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr65;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr65 = data;
			})
	);
	// Extended Miscellaneous Control 1 Register (EXT-MISC-1) (CR66) - S3 BIOS writes 0 here - normal operation & PCI bus disconnect disabled
	map(0x66, 0x66).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr66;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr66 = data;
			if (data & 0x02) {
				accel_reset();
			}
			})
	);
	map(0x67, 0x67).lrw8(
		NAME([this](offs_t offset) {
			return s3.ext_misc_ctrl_2;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.ext_misc_ctrl_2 = data;
			s3_define_video_mode();
			})
	);
	map(0x68, 0x68).lrw8(
		NAME([this](offs_t offset) {  // Configuration register 3
			// no /CAS,/OE stretch time, 32-bit data bus size
			return (s3.strapping & 0x00ff0000) >> 16;
			}),
		NAME([this](offs_t offset, u8 data) {
			if (s3.reg_lock2 == 0xa5)
			{
				s3.strapping = (s3.strapping & 0xff00ffff) | (data << 16);
				LOG("CR68: Strapping data = %08x\n", s3.strapping);
			}
			})
	);
	map(0x69, 0x69).lrw8(
		NAME([this](offs_t offset) {
			return vga.crtc.start_addr_latch >> 16;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.crtc.start_addr_latch &= ~0x1f0000;
			vga.crtc.start_addr_latch |= ((data & 0x1f) << 16);
			s3_define_video_mode();
			})
	);
	// TODO: doesn't match number of bits
	map(0x6a, 0x6a).lrw8(
		NAME([this](offs_t offset) {
			return svga.bank_r & 0x7f;
			}),
		NAME([this](offs_t offset, u8 data) {
			svga.bank_w = data & 0x3f;
			svga.bank_r = svga.bank_w;
			})
	);
	// Extended BIOS Flag 3 Register (EBIOS-FLG3) (CR6B) - Bios scratchpad
	map(0x6b, 0x6b).lrw8(
		NAME([this](offs_t offset) {
			const u8 cr53 = s3.cr53;
			const u8 cr59 = m_crtc_map.read_byte(0x59);
			if (cr53 & 0x08) {
				// Trio64 (not Trio64V2): mask per 86Box for non-V chips 0xFE
				// (Trio64V would use &0xFC, but ES40 emulates Trio64.)
				return (u8)(cr59 & 0xFE);
			}
			else {
				return cr59;
			}
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr6b = data;
			})
	);
	// Extended BIOS Flag 4 Register (EBIOS-FLG3) (CR6C) - Bios scratchpad
	map(0x6c, 0x6c).lrw8(
		NAME([this](offs_t offset) {
			const u8 cr53 = s3.cr53;
			if (cr53 & 0x08) {
				// When NEWMMIO bit is set, readback is 00h. 
				return 0x00;
			}
			else {
				// Otherwise mirror the documented bit from CR5A (mask to 0x80)
				return (s3.cr5a & 0x80);
			}
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr6c = data;
			})
	);
	// undocumented
	map(0x6d, 0x6d).lrw8(
		NAME([this](offs_t offset) {
			return s3.cr6d;
			}),
		NAME([this](offs_t offset, u8 data) {
			s3.cr6d = data;
			})
	);
	// Configuration register 4 (Trio64V+)
	map(0x6f, 0x6f).lrw8(
		NAME([this](offs_t offset) {
			// LPB(?) mode, Serial port I/O at port 0xe8, Serial port I/O disabled (MMIO only), no WE delay
			return (s3.strapping & 0xff000000) >> 24;
			}),
		NAME([this](offs_t offset, u8 data) {
			if (s3.reg_lock2 == 0xa5)
			{
				s3.strapping = (s3.strapping & 0x00ffffff) | (data << 24);
				LOG("CR6F: Strapping data = %08x\n", s3.strapping);
			}
			})
	);
}

void CS3Trio64::sequencer_map(address_map& map)
{
	// TODO: legacy fallback trick
	map(0x00, 0xff).lr8(
		NAME([this](offs_t offset) {
			const u8 res = vga.sequencer.data[offset];
			if (!machine().side_effects_disabled())
				LOGREGS("Reading unmapped sequencer read register [%02x] -> %02x (SVGA?)\n", offset, res);
			return res;
			})
	);
	//  map(0x00, 0x00) Reset Register
	map(0x00, 0x00).lw8( // es40 deviation
		NAME([this](offs_t offset, u8 data) {
			if (seq_reset1() && ((data & 0x01) == 0))
			{
				vga.sequencer.data[3] = 0;
				vga.sequencer.char_sel.A = 0;
				vga.sequencer.char_sel.B = 0;
				vga.sequencer.char_sel.base[0] = 0x20000;
				vga.sequencer.char_sel.base[1] = 0x20000;
				bx_gui->lock();
				bx_gui->set_text_charmap(&vga.memory[0x20000]);
				bx_gui->unlock();
				state.vga_mem_updated = 1;
			}
			vga.sequencer.data[0] = data;
			})
	);
	//  map(0x01, 0x01) Clocking Mode Register
	// SR01: Clocking Mode Register
	map(0x01, 0x01).lw8( // es40 deviation
		NAME([this](offs_t offset, u8 data) {
			u8 newreg1 = data & 0x3f;
			if (m_crtc_map.read_byte(0x34) & 0x20) {
				newreg1 = (newreg1 & ~0x01) | (vga.sequencer.data[1] & 0x01);
			}
			vga.sequencer.data[1] = newreg1;
			})
	);

	map(0x02, 0x02).lw8(
		NAME([this](offs_t offset, u8 data) {
			vga.sequencer.map_mask = data & 0xf;
			})
	);
	// SR03: Character Map Select
	map(0x03, 0x03).lw8(
		NAME([this](offs_t offset, u8 data) {
			/* --2- 84-- character select A
			   ---2 --84 character select B */
			vga.sequencer.char_sel.A = (((data & 0xc) >> 2) << 1) | ((data & 0x20) >> 5);
			vga.sequencer.char_sel.B = (((data & 0x3) >> 0) << 1) | ((data & 0x10) >> 4);
			// optimization for screen update inner loop
			vga.sequencer.char_sel.base[0] = 0x20000 + (vga.sequencer.char_sel.B * 0x2000);
			vga.sequencer.char_sel.base[1] = 0x20000 + (vga.sequencer.char_sel.A * 0x2000);
			//if(data)
			//	popmessage("Char SEL checker (%02x %02x)\n",vga.sequencer.char_sel.A,vga.sequencer.char_sel.B);
			})
	);
	// Sequencer Memory Mode Register
//  map(0x04, 0x04)
	map(0x04, 0x04).lw8(
		NAME([this](offs_t offset, u8 data) {
			vga.sequencer.data[4] = data;
			})
	);

	// (undocumented) Sequencer Horizontal Character Counter Reset
	// Any write strobe to this register will lock the character generator until another write to other regs happens.
	//  map(0x07, 0x07)
	// TODO: SR8 (unlocks SRD)
	// SR08: PLL Unlock
	map(0x08, 0x08).lrw8(
		NAME([this](offs_t offset) { return vga.sequencer.data[0x08]; }),
		NAME([this](offs_t offset, u8 data) {
			vga.sequencer.data[0x08] = data;
			})
	);
	// SR09: Extended (reserved)
	map(0x09, 0x09).lrw8(
		NAME([this](offs_t offset) { return vga.sequencer.data[0x09]; }),
		NAME([this](offs_t offset, u8 data) {
			vga.sequencer.data[0x09] = data;
			})
	);
	// SR0A
	map(0x0a, 0x0a).lrw8(
		NAME([this](offs_t offset) { return vga.sequencer.data[0x0a]; }),
		NAME([this](offs_t offset, u8 data) {
			vga.sequencer.data[0x0a] = data;
			})
	);
	// SR0B
	map(0x0b, 0x0b).lrw8(
		NAME([this](offs_t offset) { return vga.sequencer.data[0x0b]; }),
		NAME([this](offs_t offset, u8 data) {
			vga.sequencer.data[0x0b] = data;
			})
	);
	// SR0D
	map(0x0d, 0x0d).lrw8(
		NAME([this](offs_t offset) { return vga.sequencer.data[0x0d]; }),
		NAME([this](offs_t offset, u8 data) {
			vga.sequencer.data[0x0d] = data;
			})
	);
	// Memory CLK PLL
	map(0x10, 0x10).lrw8(
		NAME([this](offs_t offset) { return s3.sr10; }),
		NAME([this](offs_t offset, u8 data) { s3.sr10 = data; })
	);
	map(0x11, 0x11).lrw8(
		NAME([this](offs_t offset) { return s3.sr11; }),
		NAME([this](offs_t offset, u8 data) { s3.sr11 = data; })
	);
	// Video CLK PLL
	map(0x12, 0x12).lrw8(
		NAME([this](offs_t offset) { return s3.sr12; }),
		NAME([this](offs_t offset, u8 data) { s3.sr12 = data; })
	);
	map(0x13, 0x13).lrw8(
		NAME([this](offs_t offset) { return s3.sr13; }),
		NAME([this](offs_t offset, u8 data) { s3.sr13 = data; })
	);
	// SR14: CLKSYN Control 1
	map(0x14, 0x14).lrw8(
		NAME([this](offs_t offset) { return vga.sequencer.data[0x14]; }),
		NAME([this](offs_t offset, u8 data) {
			vga.sequencer.data[0x14] = data;
			})
	);
	map(0x15, 0x15).lrw8(
		NAME([this](offs_t offset) { return s3.sr15; }),
		NAME([this](offs_t offset, u8 data) {
			// load DCLK frequency (would normally have a small variable delay)
			if (data & 0x02)
			{
				s3.clk_pll_n = s3.sr12 & 0x1f;
				s3.clk_pll_r = (s3.sr12 & 0x60) >> 5;
				s3.clk_pll_m = s3.sr13 & 0x7f;
				s3_define_video_mode();
			}
			// immediate DCLK/MCLK load
			if (data & 0x20)
			{
				s3.clk_pll_n = s3.sr12 & 0x1f;
				s3.clk_pll_r = (s3.sr12 & 0x60) >> 5;
				s3.clk_pll_m = s3.sr13 & 0x7f;
				s3_define_video_mode();
			}
			s3.sr15 = data;
			})
	);
	map(0x17, 0x17).lr8(
		NAME([this](offs_t offset) {
			// CLKSYN test register
			const u8 res = s3.sr17;
			// who knows what it should return, docs only say it defaults to 0, and is reserved for testing of the clock synthesiser
			if (!machine().side_effects_disabled())
				s3.sr17--;
			return res;
			})
	);
	// SR18: RAMDAC/CLKSYN Control
	map(0x18, 0x18).lrw8(
		NAME([this](offs_t offset) { return s3.sr18; }),
		NAME([this](offs_t offset, u8 data) {
			s3.sr18 = data;
			vga.sequencer.data[0x18] = data;
			})
	);
}

uint8_t CS3Trio64::mem_r(uint32_t offset)
{
	if (svga.rgb8_en || svga.rgb15_en || svga.rgb16_en || svga.rgb32_en)
	{
		int data;
		if (offset & 0x10000)
			return 0;
		data = 0;
		if (vga.sequencer.data[4] & 0x8)
		{
			if ((offset + (svga.bank_r * 0x10000)) < vga.svga_intf.vram_size)
				data = vga.memory[(offset + (svga.bank_r * 0x10000))];
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				if (vga.sequencer.map_mask & 1 << i)
				{
					if ((offset * 4 + i + (svga.bank_r * 0x10000)) < vga.svga_intf.vram_size)
						data |= vga.memory[(offset * 4 + i + (svga.bank_r * 0x10000))];
				}
			}
		}
		return (uint8_t)data;
	}

	// Standard VGA fallback — full read-mode/latch pipeline via base class
	if ((offset + (svga.bank_r * 0x10000)) < vga.svga_intf.vram_size)
		return CVGA::mem_r(offset);
	return 0xff;
}

void CS3Trio64::mem_w(offs_t offset, uint8_t data)
{
	ibm8514a_device* dev = get_8514();
	// bit 4 of CR53 enables memory-mapped I/O
	// 0xA0000-0xA7fff maps to port 0xE2E8 (pixel transfer)
	if (s3.cr53 & 0x10)
	{
		if (offset < 0x8000)
		{
			// pass through to the pixel transfer register (DirectX 5 wants this)
			if (dev->ibm8514.bus_size == 0)
			{
				dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffffff00) | data;
				dev->ibm8514_wait_draw();
			}
			if (dev->ibm8514.bus_size == 1)
			{
				switch (offset & 0x0001)
				{
				case 0:
				default:
					dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffffff00) | data;
					break;
				case 1:
					dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffff00ff) | (data << 8);
					dev->ibm8514_wait_draw();
					break;
				}
			}
			if (dev->ibm8514.bus_size == 2)
			{
				switch (offset & 0x0003)
				{
				case 0:
				default:
					dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffffff00) | data;
					break;
				case 1:
					dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffff00ff) | (data << 8);
					break;
				case 2:
					dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xff00ffff) | (data << 16);
					break;
				case 3:
					dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0x00ffffff) | (data << 24);
					dev->ibm8514_wait_draw();
					break;
				}
			}
			return;
		}
		printf("mem_w offset: 0x%05x data: 0x%02x\n", (unsigned)offset, (unsigned)data);
		switch (offset)
		{
		case 0x8100:
		case 0x82e8:
			dev->ibm8514.curr_y = (dev->ibm8514.curr_y & 0xff00) | data;
			dev->ibm8514.prev_y = (dev->ibm8514.prev_y & 0xff00) | data;
			break;
		case 0x8101:
		case 0x82e9:
			dev->ibm8514.curr_y = (dev->ibm8514.curr_y & 0x00ff) | (data << 8);
			dev->ibm8514.prev_y = (dev->ibm8514.prev_y & 0x00ff) | (data << 8);
			break;
		case 0x8102:
		case 0x86e8:
			dev->ibm8514.curr_x = (dev->ibm8514.curr_x & 0xff00) | data;
			dev->ibm8514.prev_x = (dev->ibm8514.prev_x & 0xff00) | data;
			break;
		case 0x8103:
		case 0x86e9:
			dev->ibm8514.curr_x = (dev->ibm8514.curr_x & 0x00ff) | (data << 8);
			dev->ibm8514.prev_x = (dev->ibm8514.prev_x & 0x00ff) | (data << 8);
			break;
		case 0x8108:
		case 0x8ae8:
			dev->ibm8514.line_axial_step = (dev->ibm8514.line_axial_step & 0xff00) | data;
			dev->ibm8514.dest_y = (dev->ibm8514.dest_y & 0xff00) | data;
			break;
		case 0x8109:
		case 0x8ae9:
			dev->ibm8514.line_axial_step = (dev->ibm8514.line_axial_step & 0x00ff) | ((data & 0x3f) << 8);
			dev->ibm8514.dest_y = (dev->ibm8514.dest_y & 0x00ff) | (data << 8);
			break;
		case 0x810a:
		case 0x8ee8:
			dev->ibm8514.line_diagonal_step = (dev->ibm8514.line_diagonal_step & 0xff00) | data;
			dev->ibm8514.dest_x = (dev->ibm8514.dest_x & 0xff00) | data;
			break;
		case 0x810b:
		case 0x8ee9:
			dev->ibm8514.line_diagonal_step = (dev->ibm8514.line_diagonal_step & 0x00ff) | ((data & 0x3f) << 8);
			dev->ibm8514.dest_x = (dev->ibm8514.dest_x & 0x00ff) | (data << 8);
			break;
		case 0x8118:
		case 0x9ae8:
			s3.mmio_9ae8 = (s3.mmio_9ae8 & 0xff00) | data;
			break;
		case 0x8119:
		case 0x9ae9:
			s3.mmio_9ae8 = (s3.mmio_9ae8 & 0x00ff) | (data << 8);
			dev->ibm8514_cmd_w(s3.mmio_9ae8);
			break;
		case 0x8120:
		case 0xa2e8:
			dev->ibm8514.bgcolour = (dev->ibm8514.bgcolour & 0xff00) | data;
			break;
		case 0x8121:
		case 0xa2e9:
			dev->ibm8514.bgcolour = (dev->ibm8514.bgcolour & 0x00ff) | (data << 8);
			break;
		case 0x8124:
		case 0xa6e8:
			dev->ibm8514.fgcolour = (dev->ibm8514.fgcolour & 0xff00) | data;
			break;
		case 0x8125:
		case 0xa6e9:
			dev->ibm8514.fgcolour = (dev->ibm8514.fgcolour & 0x00ff) | (data << 8);
			break;
		case 0x8128:
		case 0xaae8:
			dev->ibm8514.write_mask = (dev->ibm8514.write_mask & 0xff00) | data;
			break;
		case 0x8129:
		case 0xaae9:
			dev->ibm8514.write_mask = (dev->ibm8514.write_mask & 0x00ff) | (data << 8);
			break;
		case 0x812c:
		case 0xaee8:
			dev->ibm8514.read_mask = (dev->ibm8514.read_mask & 0xff00) | data;
			break;
		case 0x812d:
		case 0xaee9:
			dev->ibm8514.read_mask = (dev->ibm8514.read_mask & 0x00ff) | (data << 8);
			break;
		case 0xb6e8:
		case 0x8134:
			dev->ibm8514.bgmix = (dev->ibm8514.bgmix & 0xff00) | data;
			break;
		case 0x8135:
		case 0xb6e9:
			dev->ibm8514.bgmix = (dev->ibm8514.bgmix & 0x00ff) | (data << 8);
			break;
		case 0x8136:
		case 0xbae8:
			dev->ibm8514.fgmix = (dev->ibm8514.fgmix & 0xff00) | data;
			break;
		case 0x8137:
		case 0xbae9:
			dev->ibm8514.fgmix = (dev->ibm8514.fgmix & 0x00ff) | (data << 8);
			break;
		case 0x8138:
			dev->ibm8514.scissors_top = (dev->ibm8514.scissors_top & 0xff00) | data;
			break;
		case 0x8139:
			dev->ibm8514.scissors_top = (dev->ibm8514.scissors_top & 0x00ff) | (data << 8);
			break;
		case 0x813a:
			dev->ibm8514.scissors_left = (dev->ibm8514.scissors_left & 0xff00) | data;
			break;
		case 0x813b:
			dev->ibm8514.scissors_left = (dev->ibm8514.scissors_left & 0x00ff) | (data << 8);
			break;
		case 0x813c:
			dev->ibm8514.scissors_bottom = (dev->ibm8514.scissors_bottom & 0xff00) | data;
			break;
		case 0x813d:
			dev->ibm8514.scissors_bottom = (dev->ibm8514.scissors_bottom & 0x00ff) | (data << 8);
			break;
		case 0x813e:
			dev->ibm8514.scissors_right = (dev->ibm8514.scissors_right & 0xff00) | data;
			break;
		case 0x813f:
			dev->ibm8514.scissors_right = (dev->ibm8514.scissors_right & 0x00ff) | (data << 8);
			break;
		case 0x8140:
			dev->ibm8514.pixel_control = (dev->ibm8514.pixel_control & 0xff00) | data;
			break;
		case 0x8141:
			dev->ibm8514.pixel_control = (dev->ibm8514.pixel_control & 0x00ff) | (data << 8);
			break;
		case 0x8146:
			dev->ibm8514.multifunc_sel = (dev->ibm8514.multifunc_sel & 0xff00) | data;
			break;
		case 0x8148:
			dev->ibm8514.rect_height = (dev->ibm8514.rect_height & 0xff00) | data;
			break;
		case 0x8149:
			dev->ibm8514.rect_height = (dev->ibm8514.rect_height & 0x00ff) | (data << 8);
			break;
		case 0x814a:
			dev->ibm8514.rect_width = (dev->ibm8514.rect_width & 0xff00) | data;
			break;
		case 0x814b:
			dev->ibm8514.rect_width = (dev->ibm8514.rect_width & 0x00ff) | (data << 8);
			break;
		case 0x8150:
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffffff00) | data;
			if (dev->ibm8514.state == IBM8514_DRAWING_RECT)
				dev->ibm8514_wait_draw();
			break;
		case 0x8151:
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffff00ff) | (data << 8);
			if (dev->ibm8514.state == IBM8514_DRAWING_RECT)
				dev->ibm8514_wait_draw();
			break;
		case 0x8152:
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xff00ffff) | (data << 16);
			if (dev->ibm8514.state == IBM8514_DRAWING_RECT)
				dev->ibm8514_wait_draw();
			break;
		case 0x8153:
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0x00ffffff) | (data << 24);
			if (dev->ibm8514.state == IBM8514_DRAWING_RECT)
				dev->ibm8514_wait_draw();
			break;
		case 0xbee8:
			s3.mmio_bee8 = (s3.mmio_bee8 & 0xff00) | data;
			break;
		case 0xbee9:
			s3.mmio_bee8 = (s3.mmio_bee8 & 0x00ff) | (data << 8);
			dev->ibm8514_multifunc_w(s3.mmio_bee8);
			break;
		case 0x96e8:
			s3.mmio_96e8 = (s3.mmio_96e8 & 0xff00) | data;
			break;
		case 0x96e9:
			s3.mmio_96e8 = (s3.mmio_96e8 & 0x00ff) | (data << 8);
			dev->ibm8514_width_w(s3.mmio_96e8);
			break;
		case 0xe2e8:
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffffff00) | data;
			dev->ibm8514_wait_draw();
			break;
		case 0xe2e9:
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffff00ff) | (data << 8);
			if (dev->ibm8514.state == IBM8514_DRAWING_RECT)
				dev->ibm8514_wait_draw();
			break;
		case 0xe2ea:
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xff00ffff) | (data << 16);
			if (dev->ibm8514.state == IBM8514_DRAWING_RECT)
				dev->ibm8514_wait_draw();
			break;
		case 0xe2eb:
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0x00ffffff) | (data << 24);
			if (dev->ibm8514.state == IBM8514_DRAWING_RECT)
				dev->ibm8514_wait_draw();
			break;
		default:
			LOG("S3: MMIO offset %05x write %02x\n", offset + 0xa0000, data);
			break;
		}
		return;
	}

	if (svga.rgb8_en || svga.rgb15_en || svga.rgb16_en || svga.rgb32_en)
	{
		//printf("%08x %02x (%02x %02x) %02X\n",offset,data,vga.sequencer.map_mask,svga.bank_w,(vga.sequencer.data[4] & 0x08));
		if (offset & 0x10000)
			return;
		if (vga.sequencer.data[4] & 0x8)
		{
			if ((offset + (svga.bank_w * 0x10000)) < vga.svga_intf.vram_size)
				vga.memory[(offset + (svga.bank_w * 0x10000))] = data;
		}
		else
		{
			int i;
			for (i = 0; i < 4; i++)
			{
				if (vga.sequencer.map_mask & 1 << i)
				{
					if ((offset * 4 + i + (svga.bank_w * 0x10000)) < vga.svga_intf.vram_size)
						vga.memory[(offset * 4 + i + (svga.bank_w * 0x10000))] = data;
				}
			}
		}
		return;
	}

	if ((offset + (svga.bank_w * 0x10000)) < vga.svga_intf.vram_size)
		CVGA::mem_w(offset, data);
}

uint32_t CS3Trio64::screen_update(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	CVGA::screen_update(bitmap, cliprect);

	uint8_t cur_mode = pc_vga_choosevideomode();

#ifdef DEBUG_VGA_RENDER
	static uint8_t last_mode = 0xff;
	if (cur_mode != last_mode) {
		LOG("S3: screen mode changed to %d (rgb8=%d sync=%d graphic=%d)\n",
			cur_mode, svga.rgb8_en, vga.crtc.sync_en, vga.gc.alpha_dis);
		last_mode = cur_mode;
	}

	// log screen configuration  
	static bool screen_diag = false;
	if (cur_mode == 6 && !screen_diag) {  // 6 = RGB8_MODE
		const rectangle& vis = cliprect;
		LOG("DIAG: screen visible_area=(%d,%d)-(%d,%d) bitmap=%dx%d\n",
			vis.min_x, vis.min_y, vis.max_x, vis.max_y,
			bitmap.width(), bitmap.height());
		LOG("DIAG: CRTC offset=%03x seq4=%02x chain4=%d height=%d LINES=%d VGA_COLUMNS=%d\n",
			vga.crtc.offset, vga.sequencer.data[4],
			(vga.sequencer.data[4] & 0x08) ? 1 : 0,
			vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1),
			(vga.crtc.vert_disp_end + 1), (vga.crtc.horz_disp_end + 1));
		screen_diag = true;

		static int dac_frame_count = 0;
		if (cur_mode == 6 && (dac_frame_count % 60 == 0) && dac_frame_count < 300) {
			LOG("DIAG[frame %d]: dac.dirty=%d DAC[0]=%02x,%02x,%02x DAC[2]=%02x,%02x,%02x\n",
				dac_frame_count, vga.dac.dirty,
				vga.dac.color[0], vga.dac.color[1], vga.dac.color[2],
				vga.dac.color[6], vga.dac.color[7], vga.dac.color[8]);
		}
		dac_frame_count++;
	}


		printf("PALETTE: dirty=%d DAC[0]=%02x,%02x,%02x DAC[2]=%02x,%02x,%02x DAC[130]=%02x,%02x,%02x\n",
			vga.dac.dirty,
			vga.dac.color[0], vga.dac.color[1], vga.dac.color[2],
			vga.dac.color[6], vga.dac.color[7], vga.dac.color[8],
			vga.dac.color[390], vga.dac.color[391], vga.dac.color[392]);


#endif

	// draw hardware graphics cursor
	// TODO: support 16 bit and greater video modes
	// TODO: should be a derived function from svga_device
	if (s3.cursor_mode & 0x01)  // if cursor is enabled
	{
		uint16_t cx = s3.cursor_x & 0x07ff;
		uint16_t cy = s3.cursor_y & 0x07ff;

		if (cur_mode == SCREEN_OFF || cur_mode == TEXT_MODE || cur_mode == MONO_MODE || cur_mode == CGA_MODE || cur_mode == EGA_MODE)
			return 0;  // cursor only works in VGA or SVGA modes

		uint32_t src = s3.cursor_start_addr * 1024;  // start address is in units of 1024 bytes

		uint32_t bg_col;
		uint32_t fg_col;
		int r, g, b;
		uint32_t datax;
		switch (cur_mode)
		{
		case RGB15_MODE:
		case RGB16_MODE:
			datax = s3.cursor_bg[0] | s3.cursor_bg[1] << 8;
			r = (datax & 0xf800) >> 11;
			g = (datax & 0x07e0) >> 5;
			b = (datax & 0x001f) >> 0;
			r = (r << 3) | (r & 0x7);
			g = (g << 2) | (g & 0x3);
			b = (b << 3) | (b & 0x7);
			bg_col = (0xff << 24) | (r << 16) | (g << 8) | (b << 0);

			datax = s3.cursor_fg[0] | s3.cursor_fg[1] << 8;
			r = (datax & 0xf800) >> 11;
			g = (datax & 0x07e0) >> 5;
			b = (datax & 0x001f) >> 0;
			r = (r << 3) | (r & 0x7);
			g = (g << 2) | (g & 0x3);
			b = (b << 3) | (b & 0x7);
			fg_col = (0xff << 24) | (r << 16) | (g << 8) | (b << 0);
			break;
		case RGB24_MODE:
			datax = s3.cursor_bg[0] | s3.cursor_bg[1] << 8 | s3.cursor_bg[2] << 16;
			r = (datax & 0xff0000) >> 16;
			g = (datax & 0x00ff00) >> 8;
			b = (datax & 0x0000ff) >> 0;
			bg_col = (0xff << 24) | (r << 16) | (g << 8) | (b << 0);

			datax = s3.cursor_fg[0] | s3.cursor_fg[1] << 8 | s3.cursor_fg[2] << 16;
			r = (datax & 0xff0000) >> 16;
			g = (datax & 0x00ff00) >> 8;
			b = (datax & 0x0000ff) >> 0;
			fg_col = (0xff << 24) | (r << 16) | (g << 8) | (b << 0);
			break;
		case RGB8_MODE:
		default:
			bg_col = pen(s3.cursor_bg[0]);
			fg_col = pen(s3.cursor_fg[0]);
			break;
		}

		//popmessage("%08x %08x",(s3.cursor_bg[0])|(s3.cursor_bg[1]<<8)|(s3.cursor_bg[2]<<16)|(s3.cursor_bg[3]<<24)
		//                    ,(s3.cursor_fg[0])|(s3.cursor_fg[1]<<8)|(s3.cursor_fg[2]<<16)|(s3.cursor_fg[3]<<24));
//      for(x=0;x<64;x++)
//          printf("%08x: %02x %02x %02x %02x\n",src+x*4,vga.memory[src+x*4],vga.memory[src+x*4+1],vga.memory[src+x*4+2],vga.memory[src+x*4+3]);
		for (int y = 0; y < 64; y++)
		{
			if (cy + y < cliprect.max_y && cx < cliprect.max_x)
			{
				uint32_t* const dst = &bitmap.pix(cy + y, cx);
				for (int x = 0; x < 64; x++)
				{
					uint16_t bita = (vga.memory[(src + 1) % vga.svga_intf.vram_size]
						| ((vga.memory[(src + 0) % vga.svga_intf.vram_size]) << 8))
						>> (15 - (x % 16));
					uint16_t bitb = (vga.memory[(src + 3) % vga.svga_intf.vram_size]
						| ((vga.memory[(src + 2) % vga.svga_intf.vram_size]) << 8))
						>> (15 - (x % 16));
					uint8_t val = ((bita & 0x01) << 1) | (bitb & 0x01);

					if (s3.extended_dac_ctrl & 0x10)
					{  // X11 mode
						switch (val)
						{
						case 0x00: break;                // no change
						case 0x01: break;                // no change
						case 0x02: dst[x] = bg_col; break;
						case 0x03: dst[x] = fg_col; break;
						}
					}
					else
					{  // Windows mode
						switch (val)
						{
						case 0x00: dst[x] = bg_col; break;
						case 0x01: dst[x] = fg_col; break;
						case 0x02: break;                // screen data
						case 0x03: dst[x] = ~(dst[x]); break; // inverted
						}
					}
					if (x % 16 == 15)
						src += 4;
				}
			}
		}
	}
	return 0;
}

// vision 964 stuff

// vision 968 stuff

// s3trio64_vga_device::s3trio64_vga_device(const machine_config& mconfig, const char* tag, device_t* owner, uint32_t clock)
//	: s3trio64_vga_device(mconfig, S3_TRIO64_VGA, tag, owner, clock)

// s3trio64_vga_device::s3trio64_vga_device(const machine_config& mconfig, device_type type, const char* tag, device_t* owner, uint32_t clock)
//	: s3vision968_vga_device(mconfig, type, tag, owner, clock)

// void s3trio64_vga_device::device_start()

// END MAME pc_vga_s3.cpp

// ES40 specific functions

// ---- Simple RGB332 quantiser so GUI's 8-bit tile path can show >8bpp ----
static inline u8 rgb_to_332(u8 r8, u8 g8, u8 b8) {
	return (u8)(((r8 >> 5) << 5) | ((g8 >> 5) << 2) | (b8 >> 6));
}

void CS3Trio64::ensure_rgb332_palette_loaded() {
	if (state.tc_rgb332_palette_loaded) return;
	for (int i = 0; i < 256; ++i) {
		const u8 r = (u8)((i >> 5) & 0x07);
		const u8 g = (u8)((i >> 2) & 0x07);
		const u8 b = (u8)(i & 0x03);
		const u8 r6 = (u8)((r * 63) / 7);
		const u8 g6 = (u8)((g * 63) / 7);
		const u8 b6 = (u8)((b * 63) / 3);
		bx_gui->palette_change((unsigned)i, (unsigned)(r6 << 2),
			(unsigned)(g6 << 2), (unsigned)(b6 << 2));
	}
	state.tc_rgb332_palette_loaded = true;
}

// Helper: compose an RGB888 into host pixel using GUI's mask/shift semantics.
static inline u32 pack_rgb_to_host(u8 R, u8 G, u8 B, const bx_svga_tileinfo_t& ti)
{
	// shifts in tileinfo are "top-of-8-bit channel" positions; see base GUI mapping
	// (e.g. for 32bpp: red_shift=24 means R occupies bits 16..23).
	return
		(((u32)R << (ti.red_shift - 8)) & (u32)ti.red_mask) |
		(((u32)G << (ti.green_shift - 8)) & (u32)ti.green_mask) |
		(((u32)B << (ti.blue_shift - 8)) & (u32)ti.blue_mask);
}


// Returns (A<<1)|B for pixel (sx,sy) from a 6464 cursor map.
// Supports:
//   Standard layout (16 bytes/row): A[15:0] then B[15:0] per 16 px (all non-16bpp modes)
//   64k layout (16bpp): A byte then B byte per 8 px, each bit horizontally doubled. 
//   Right-storage addressing when CR45 bit4 is set - via MAME 
static inline u8 s3_cursor_ab(const u8* vram, u32 vram_mask, u32 src_base, unsigned sx, unsigned sy, bool is16bpp, bool right_storage)
{
	// Compute row base with optional "right storage" skew.
	auto row_base_16bytes = [&](unsigned row) -> u32 {
		if (!right_storage) {
			return src_base + row * 16;                    // normal: 16 bytes/row
		}
		// right storage (non-16bpp path): last 256B of each 1KiB line, 4 lines  1KiB
		const unsigned lane = (row >> 4) & 0x3;            // 0..3
		const unsigned r = row & 0x0F;                  // 0..15 within lane
		return src_base + lane * 1024u + (1024u - 256u) + r * 16u;
		};

	if (!is16bpp) {
		// Standard 16B/row path: A[15:0] then B[15:0] per 16 pixels
		const unsigned group = sx >> 4;                    // 0..3
		const unsigned bit = 15 - (sx & 15);
		const u32 rb = row_base_16bytes(sy);
		const u32 src = rb + group * 4u;
		const u16 A = (u16)vram[(src + 1) & vram_mask] |
			((u16)vram[(src + 0) & vram_mask] << 8);
		const u16 B = (u16)vram[(src + 3) & vram_mask] |
			((u16)vram[(src + 2) & vram_mask] << 8);
		return (u8)(((A >> bit) & 1) << 1 | ((B >> bit) & 1));
	}
	else {
		// 64k (16bpp) path: one A byte + one B byte per 8 pixels, bits doubled horizontally. 
		// We still keep 16 bytes per row total: 8 groups  2 bytes.
		// Right storage variant uses last 512B of 22KiB lines.
		u32 rb;
		if (!right_storage) {
			rb = src_base + sy * 16u;
		}
		else {
			const unsigned lane = (sy >> 5) & 0x1;         // 0..1
			const unsigned r = sy & 0x1F;               // 0..31
			rb = src_base + lane * 2048u + (2048u - 512u) + r * 16u;
		}
		const unsigned group8 = sx >> 3;                   // 0..7 (8px groups)
		const u32 src = rb + group8 * 2u;                  // A byte, then B byte
		const u8 Abits = vram[(src + 0) & vram_mask];
		const u8 Bbits = vram[(src + 1) & vram_mask];
		const unsigned bit = 7 - ((sx & 7) >> 1);          // horizontal bit-doubling
		const u8 A = (Abits >> bit) & 1;
		const u8 B = (Bbits >> bit) & 1;
		return (u8)((A << 1) | B);
	}
}

/**
 * Set a specific tile's updated variable.
 *
 * Only reference the array if the tile numbers are within the bounds
 * of the array.  If out of bounds, do nothing.
 **/
#define SET_TILE_UPDATED(xtile, ytile, value)                    \
  do                                                             \
  {                                                              \
    if(((xtile) < BX_NUM_X_TILES) && ((ytile) < BX_NUM_Y_TILES)) \
      state.vga_tile_updated[(xtile)][(ytile)] = value;          \
  } while(0)

 /**
  * Get a specific tile's updated variable.
  *
  * Only reference the array if the tile numbers are within the bounds
  * of the array.  If out of bounds, return 0.
  **/
#define GET_TILE_UPDATED(xtile, ytile) \
    ((((xtile) < BX_NUM_X_TILES) && ((ytile) < BX_NUM_Y_TILES)) ? state.vga_tile_updated[(xtile)][(ytile)] : 0)

  /**
   * Thread entry point.
   *
   * The thread first initializes the GUI, and then starts looping the
   * following actions until interrupted (by StopThread being set to true)
   *   - Handle any GUI events (mouse moves, keypresses)
   *   - Update the GUI to match the screen buffer
   *   - Flush the updated GUI content to the screen
   *   .
   **/
void CS3Trio64::run()
{
	try
	{
		// initialize the GUI (and let it know our tilesize)
		bx_gui->init(state.x_tilesize, state.y_tilesize);
		bool was_paused = false;
		PauseAck.store(false, std::memory_order_release);
		for (;;)
		{
			// Terminate thread if StopThread is set to true
			if (StopThread)
				return;
			// Handle GUI events (50 times per second)
			bx_gui->lock();
			bx_gui->handle_events();
			bx_gui->unlock();
			CThread::sleep(10);

			// During firmware reset: keep pumping events (window stays alive),
			// but do NOT touch emulated VGA state.
			if (PauseThread.load(std::memory_order_acquire))
			{
				if (!was_paused)
				{
					bx_gui->lock();
					bx_gui->clear_screen();   // optional; comment out if you want last frame to remain
					bx_gui->unlock();
					was_paused = true;
				}
				PauseAck.store(true, std::memory_order_release);
				continue;
			}
			PauseAck.store(false, std::memory_order_release);
			was_paused = false;

			//Update the screen (50 times per second)
			bx_gui->lock();
			update();
			bx_gui->flush();
			bx_gui->unlock();
		}
	}

	catch (CException& e)
	{
		printf("Exception in S3 thread: %s.\n", e.displayText().c_str());

		// Let the thread die...
	}
}

/** Size of ROM image */
static unsigned int rom_max;

/** ROM image */
static u8           option_rom[65536];

/** PCI Configuration Space data block */
static u32                 s3_cfg_data[64] = {
	/*00*/ 0x88115333,            // CFID: vendor + device
	/*04*/ 0x011f0000,            // CFCS: command + status
	/*08*/ 0x03000002,            // CFRV: class + revision
	/*0c*/ 0x00000000,            // CFLT: latency timer + cache line size
	/*10*/ 0x00000000,            // BAR0: FB
	/*14*/ 0x00000000,            // BAR1:
	/*18*/ 0x00000000,            // BAR2:
	/*1c*/ 0x00000000,            // BAR3:
	/*20*/ 0x00000000,            // BAR4:
	/*24*/ 0x00000000,            // BAR5:
	/*28*/ 0x00000000,            // CCIC: CardBus
	/*2c*/ 0x00000000,            // CSID: subsystem + vendor
	/*30*/ 0x00000000,            // BAR6: expansion rom base
	/*34*/ 0x00000000,            // CCAP: capabilities pointer
	/*38*/ 0x00000000,
	/*3c*/ 0x281401ff,            // CFIT: interrupt configuration
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/** PCI Configuration Space mask block */
static u32                 s3_cfg_mask[64] = {
	/*00*/ 0x00000000,            // CFID: vendor + device
	/*04*/ 0x0000ffff,            // CFCS: command + status
	/*08*/ 0x00000000,            // CFRV: class + revision
	/*0c*/ 0x0000ffff,            // CFLT: latency timer + cache line size
	/*10*/ 0xfc000000,            // BAR0: FB
	/*14*/ 0x00000000,            // BAR1:
	/*18*/ 0x00000000,            // BAR2:
	/*1c*/ 0x00000000,            // BAR3:
	/*20*/ 0x00000000,            // BAR4:
	/*24*/ 0x00000000,            // BAR5:
	/*28*/ 0x00000000,            // CCIC: CardBus
	/*2c*/ 0x00000000,            // CSID: subsystem + vendor
	/*30*/ 0x00000000,            // BAR6: expansion rom base
	/*34*/ 0x00000000,            // CCAP: capabilities pointer
	/*38*/ 0x00000000,
	/*3c*/ 0x000000ff,            // CFIT: interrupt configuration
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/**
 * Constructor.
 *
 * Don't do anything, the real initialization is done by init()
 **/
CS3Trio64::CS3Trio64(CConfigurator* cfg, CSystem* c, int pcibus, int pcidev) : CVGA(cfg, c, pcibus, pcidev)
{
}

// --- S3 CR36 -----------------------------------------------------------------
// CR36 (Reset State Read 1) encodes DRAM type in the low and the actual VRAM 
// size in the high. 
//   EDO: <1M=0xFE, 1M=0xDE, 2M=0x9E, 3M=0x5E, 4M=0x1E, 8M=0x7E
// Trio64 max display memory is 4 MiB
// see DOSBox-X Reset State Read 1
// `use_edo` to false for FPM instead of EDO memory
static inline uint8_t s3_cr36_from_memsize(uint32_t bytes, bool use_edo /*=true*/) {
	const uint8_t type_nibble = use_edo ? 0x0E : 0x0A; // low nibble
	uint8_t size_nibble_high;
	if (bytes < (1u * 1024 * 1024))       size_nibble_high = 0xF0; // <1MB
	else if (bytes < (2u * 1024 * 1024))  size_nibble_high = 0xD0; // 1MB
	else if (bytes < (3u * 1024 * 1024))  size_nibble_high = 0x90; // 2MB
	else if (bytes < (4u * 1024 * 1024))  size_nibble_high = 0x50; // 3MB
	else                                  size_nibble_high = 0x10; // 4MB (or more -> clamp)
	return uint8_t(size_nibble_high | type_nibble);
}

// CR32 (Backward Compatibility 1 - BKWD_1).
// Writing 01xx10xx (e.g. 0x48) unlocks S3 VGA regs.
static inline bool s3_cr32_is_unlock(uint8_t v) {
	return ((v & 0xC0) == 0x40) && ((v & 0x0C) == 0x08);
}

inline uint32_t CS3Trio64::s3_lfb_base_from_regs() {
	// CR59 high, CR5A low, reported as (la_window << 16)
	const uint16_t la_window = (uint16_t(m_crtc_map.read_byte(0x59)) << 8) | uint16_t(m_crtc_map.read_byte(0x5A));
	return uint32_t(la_window) << 16;
}

inline uint8_t CS3Trio64::current_char_width_px() const {
	// If special blanking (CR33 bit5) is set, S3 forces 8-dot chars.
	if (m_crtc_map.read_byte(0x33) & 0x20) return 8;
	// Otherwise, use Sequencer reg1 bit0 like the rest of our text logic.
	return (vga.sequencer.data[1] & 0x01) ? 8 : 9;
}

void CS3Trio64::overlay_hw_cursor_on_tile(u8* tile8,
	unsigned xc, unsigned yc,
	unsigned tile_w, unsigned tile_h,
	unsigned tile_x0, unsigned tile_y0,
	int bpp_now, unsigned pitch_bytes,
	unsigned start_addr)
{
	(void)pitch_bytes; (void)start_addr; // we overlay on the already composed tile
	if ((s3.cursor_mode & 0x01) == 0) return; // disabled  

	// Cursor rectangle (64x64) in screen space
	u16 cx = (u16)(s3.cursor_x & 0x07FF);
	const u16 cy = (u16)(s3.cursor_y & 0x07FF);
	// Trio64: in 15/16-bpp modes, X is stored doubled; compensate here
	const bool is16bpp = (bpp_now == 2);
	if (is16bpp) cx >>= 1;
	const unsigned cur_w = 64, cur_h = 64;

	// Tile rectangle
	const unsigned rx0 = xc, ry0 = yc;
	const unsigned rx1 = xc + tile_w; const unsigned ry1 = yc + tile_h;
	// Intersection
	const int ix0 = (int)std::max<unsigned>(rx0, cx);
	const int iy0 = (int)std::max<unsigned>(ry0, cy);
	const int ix1 = (int)std::min<unsigned>(rx1, cx + cur_w);
	const int iy1 = (int)std::min<unsigned>(ry1, cy + cur_h);
	if (ix0 >= ix1 || iy0 >= iy1) return;

	// Cursor colours depend on mode: 8bpp -> palette indices; 15/16 -> RGB565/555; 24 -> 24bpp
	// We always convert to RGB332 index for the 8-bit GUI path.
	const u8 ext_dac = m_crtc_map.read_byte(0x55); // CR55 EX_DAC_CTL, bit4 selects Windows vs X11 mapping 
	const bool x11 = (ext_dac & 0x10) != 0;

	auto fg_idx_from_stack = [&]() -> u8 {
		if (bpp_now == 1) return s3.cursor_fg[0];
		if (bpp_now == 3) { // 24-bit
			u8 r = s3.cursor_fg[2], g = s3.cursor_fg[1], b = s3.cursor_fg[0];
			return rgb_to_332(r, g, b);

		}
		// 15/16 -> two bytes little-endian, treat as 565 (or 555 ok)
		u16 pix = u16(s3.cursor_fg[0]) | (u16(s3.cursor_fg[1]) << 8);
		u8 r5 = (bpp_now == 2 && ((s3.ext_misc_ctrl_2 >> 4) & 0x0F) == 0x03) ? ((pix >> 10) & 0x1F) : ((pix >> 11) & 0x1F);
		u8 g6 = (pix >> 5) & ((((s3.ext_misc_ctrl_2 >> 4) & 0x0F) == 0x05) ? 0x3F : 0x1F);
		u8 b5 = (pix >> 0) & 0x1F;
		u8 R = (r5 << 3) | (r5 >> 2), G = (g6 << 2) | (g6 >> 4), B = (b5 << 3) | (b5 >> 2);
		return rgb_to_332(R, G, B);
		};
	auto bg_idx_from_stack = [&]() -> u8 {
		if (bpp_now == 1) return s3.cursor_bg[0];
		if (bpp_now == 3) {
			u8 r = s3.cursor_bg[2], g = s3.cursor_bg[1], b = s3.cursor_bg[0];
			return rgb_to_332(r, g, b);
		}
		u16 pix = u16(s3.cursor_bg[0]) | (u16(s3.cursor_bg[1]) << 8);
		u8 r5 = (bpp_now == 2 && ((s3.ext_misc_ctrl_2 >> 4) & 0x0F) == 0x03) ? ((pix >> 10) & 0x1F) : ((pix >> 11) & 0x1F);
		u8 g6 = (pix >> 5) & ((((s3.ext_misc_ctrl_2 >> 4) & 0x0F) == 0x05) ? 0x3F : 0x1F);
		u8 b5 = (pix >> 0) & 0x1F;
		u8 R = (r5 << 3) | (r5 >> 2), G = (g6 << 2) | (g6 >> 4), B = (b5 << 3) | (b5 >> 2);
		return rgb_to_332(R, G, B);
		};
	const u8 fg_idx = fg_idx_from_stack();
	const u8 bg_idx = bg_idx_from_stack();

	const u32 vram_mask = s3_vram_mask();
	const u32 src_base = (u32(s3.cursor_start_addr) << 10); // 1 KiB units  
	const u8* vram = &vga.memory[0];
	// CR45 bit4: Right aligned cursor addressing
	const bool right_storage = (s3.cursor_mode & 0x10) != 0;

	for (int py = iy0; py < iy1; ++py) {
		const int sy = (py - (int)cy + (int)s3.cursor_pattern_y) & 63; // wrap within 64
		const int ty = py - (int)yc; // tile row index
		for (int px = ix0; px < ix1; ++px) {
			const int sx = (px - (int)cx + (int)s3.cursor_pattern_x) & 63; // wrap
			const u8 ab = s3_cursor_ab(vram, vram_mask, src_base, sx, sy, is16bpp, right_storage);
			// Destination in our 16x16 tile buffer
			const int tx = px - (int)xc;
			const int ty_local = py - (int)yc;
			const int pos = ty_local * (int)X_TILESIZE + tx;
			u8& dst = tile8[pos];
			if (x11) {
				// X11 mapping: 0: no change, 1: no change, 2: bg, 3: fg  
				if (ab == 2) dst = bg_idx;
				else if (ab == 3) dst = fg_idx;

			}
			else {
				// Windows mapping: 0:bg, 1:fg, 2:no change, 3:invert  
				if (ab == 0) dst = bg_idx;
				else if (ab == 1) dst = fg_idx;
				else if (ab == 3) dst ^= 0xFF; // 8-bit index invert approximation
			}
		}
	}
}

// MMIO alias base offset helper (CR53 bit5: 0=A0000, 1=B8000).
inline uint32_t CS3Trio64::s3_mmio_base_off(SS3_state& s) {
	// Trio64 CR53 bit5 (A0000/B8000 MMIO base select):
	//  - 0: A0000...AFFFF is MMIO alias space when CR53 bit4=0
	//  - 1: B8000..BFFFF is MMIO alias space when CR53 bit4=1
	return (s3.cr53 & 0x20) ? 0x18000u : 0u;
}

// Trio64: "new" MMIO 64 KiB window at LFB + 0x0100_0000 when CR53 bit3 = 1.
// We'll at minimum wire the lower 0x8000 to PIXTRANS FIFO (0xE2E8 port semantics).
inline bool CS3Trio64::s3_new_mmio_enabled()
{
	// CR53 - see 86Box behavior gating several "new-mmio" aliases. 
	return (s3.cr53 & 0x08) != 0;
}

static inline uint32_t s3_lfb_size_from_cr58(uint8_t cr58) {
	switch (cr58 & 0x03) {
	case 0: return 64 * 1024;
	case 1: return 1u * 1024 * 1024;
	case 2: return 2u * 1024 * 1024;
	default: return 4u * 1024 * 1024;
	}
}

static inline bool s3_lfb_enabled(uint8_t cr58) {
	return (cr58 & 0x10) != 0; // ENB LA (Enable Linear Addressing)
}

void CS3Trio64::update_linear_mapping()
{
	// BAR-only mode: no per-device mapping. PCI core decodes BAR0 and gates
	// access via COMMAND.MSE. We keep these fields for debug only.
	lfb_active = s3_lfb_enabled(m_crtc_map.read_byte(0x58));
	lfb_size = s3_lfb_size_from_cr58(m_crtc_map.read_byte(0x58));
	lfb_base = s3_lfb_base_from_regs();
#if S3_LFB_TRACE
	printf("LFB (BAR-only): CR58=%02x base=%08x size=%x active=%d\n",
		m_crtc_map.read_byte(0x58), lfb_base, lfb_size, lfb_active);
#endif
}

void CS3Trio64::on_crtc_linear_regs_changed()
{
	const u8 cr58 = m_crtc_map.read_byte(0x58);
	const u8 cr59 = m_crtc_map.read_byte(0x59);
	const u8 cr5a = m_crtc_map.read_byte(0x5A);

	// Enable via CR58.ENB_LA (bit 4)
	lfb_active = s3_lfb_enabled(cr58);

	// Trio64 size: CR58[1:0] 00=64K, 01=1M, 10=2M, 11=4M
	lfb_size = s3_lfb_size_from_cr58(cr58);

	// Base: CR59:CR5A form LA_WINDOW; reported/programmed as (la_window << 16)
	const u32 la_window = (u32(cr59) << 8) | u32(cr5a);
	lfb_base = la_window << 16;

	// Apply/unapply the mapping now that CR regs changed
	lfb_recalc_and_map();
	trace_lfb_if_changed("CR58/59/5A");
}

/**
 * Initialize the S3 device.
 **/
void CS3Trio64::init()
{
	// Register PCI device
	add_function(0, s3_cfg_data, s3_cfg_mask);

	// Make the 21272/21274 PCI config window visible for this device now.
	// NetBSD reads PCI_ID_REG at 0/1/0 during console bring-up; without this
	// mapping it sees ~0/0 and panics with "no device at 255/255/0".
	ResetPCI();


	// Initialize all state variables to 0
	memset((void*)&state, 0, sizeof(state));

	// initialize MAME S3 and VGA fields and values to 0
	memset(&s3, 0, sizeof(s3));
	memset(&vga, 0, sizeof(vga));
	memset(&svga, 0, sizeof(svga));
	memset(&timing, 0, sizeof(timing));
	memset(vga.dac.color, 0, sizeof(vga.dac.color));
	memset(vga.dac.loading, 0, sizeof(vga.dac.loading));

	// Register VGA I/O ports at 3b4, 3b5, 3ba, 3c0..cf, 3d4, 3d5, 3da
	add_legacy_io(1, 0x3b4, 2);
	add_legacy_io(3, 0x3ba, 2);
	add_legacy_io(2, 0x3c0, 16);
	add_legacy_io(8, 0x3d4, 2);
	add_legacy_io(9, 0x3da, 1);

	// Register CRTC address-map handlers.  Must be called before any
	// m_crtc_map.write_byte() so that writes dispatch through handlers
	// and update decomposed vga.crtc.* / s3.* fields + trigger recomputes.
	init_maps();

	vga.attribute.state = atc_flip_flop() ? 1 : 0;
	vga.attribute.index = vga.attribute.index & 0x1f;

	vga.gc.bit_mask = 0xFF;

	vga.sequencer.char_sel.base[0] = 0x20000;  // font B (attr bit3=0)
	vga.sequencer.char_sel.base[1] = 0x20000;  // font A (attr bit3=1)

	// 8514/A-style S3 accel ports (byte-wide) - always register;
	// runtime gating is done via CR40 (state.accel.enabled).
	add_legacy_io(10, 0x42E8, 2); // SUBSYS_CNTL/STAT
	add_legacy_io(11, 0x4AE8, 2); // ADVFUNC_CNTL
	add_legacy_io(12, 0x46E8, 2); // CUR_X
	add_legacy_io(13, 0x4EE8, 2); // CUR_Y
	add_legacy_io(14, 0x86E8, 2); // DEST_X
	add_legacy_io(15, 0x8EE8, 2); // DEST_Y/AXSTP
	add_legacy_io(16, 0x96E8, 2); // MAJ_AXIS_PCNT
	add_legacy_io(17, 0x9AE8, 2); // CMD
	add_legacy_io(18, 0xA2E8, 2); // BKGD_COLOR
	add_legacy_io(19, 0xA6E8, 2); // FRGD_COLOR
	add_legacy_io(20, 0xAAE8, 2); // WRT_MASK
	add_legacy_io(21, 0xAEE8, 2); // RD_MASK
	add_legacy_io(22, 0xB6E8, 2); // BKGD_MIX
	add_legacy_io(23, 0xBAE8, 2); // FRGD_MIX
	add_legacy_io(24, 0xE2E8, 8); // PIX_TRANS (0xE2E8..0xE2EF)
	add_legacy_io(25, 0xB2E8, 2); // PIX_CNTL or ALT PIX_TRANS (gated by MULTIFUNC[E].bit8)
	add_legacy_io(26, 0xBEE8, 2); // MULTIFUNC_CNTL (word)
	add_legacy_io(27, 0xD2E8, 2); // ROP_MIX       (word; some paths read this)
	add_legacy_io(28, 0x9EE8, 2); // SHORT_STROKE  (word; latch)
	add_legacy_io(29, 0xCAE8, 2); // DESTY/AXSTP alias (might be wrong)
	add_legacy_io(30, 0x82E8, 2);  // CUR_Y 
	add_legacy_io(31, 0x92E8, 2);  // ERR_TERM 


	/* The VGA BIOS we use sends text messages to port 0x500.
	   We listen for these messages at port 500. */
	add_legacy_io(7, 0x500, 1);
	bios_message_size = 0;
	bios_message[0] = '\0';

	// Legacy video address space: A0000 -> bffff
	add_legacy_mem(4, 0xa0000, 128 * 1024);

	// Default: no linear window until guest enables CR58 bit 0.
	// Seed base/size from PCI config defaults; CR58/59 will override when written.
	lfb_active = false;
	lfb_base = s3_cfg_data[0x10 >> 2] & 0xFC000000;  // BAR0 default (aligned)
	lfb_size = vga.svga_intf.vram_size;                        // clamp to VRAM for now

	// Reset the base PCI device
	ResetPCI();

	/* The configuration file variable "rom" should point to a VGA BIOS
	   image. If not, try "vgabios.bin". */
	FILE* rom = fopen(myCfg->get_text_value("rom", "vgabios.bin"), "rb");
	if (!rom)
	{
		FAILURE_1(FileNotFound, "s3 rom file %s not found",
			myCfg->get_text_value("rom", "vgabios.bin"));
	}

	rom_max = (unsigned)fread(option_rom, 1, 65536, rom);
	fclose(rom);

	// Option ROM address space: C0000
	add_legacy_mem(5, 0xc0000, rom_max);

	vga.attribute.state = 1;

	vga.crtc.line_compare = 1023;
	vga.crtc.vert_disp_end = 399;

	vga.dac.mask = 0xff;
	vga.dac.dirty = 1;
	vga.dac.state = 0;
	vga.dac.read = 0;

	vga.gc.memory_map_sel = 2; // monochrome text mode

	vga.sequencer.data[0] = 0x03;  // reset1=1, reset2=1
	vga.sequencer.data[4] = 0x06;  // extended_mem=1, odd_even=1, chain_four=0
	s3.sr15 = 0;               // CLKSYN Control 2 Register (SR15) 00H poweron
	vga.sequencer.data[0x0A] = 0;                // External Bus Control Register (SRA) 00H poweron
	vga.sequencer.data[0x0B] = 0;                // Miscellaneous Extended Sequencer Register 00H poweron
	vga.sequencer.data[0x0D] = 0;                // Extended Sequencer Register (EX_SR_D) (SRD) 00H poweron
	vga.sequencer.data[0x09] = 0;                // Extended Sequencer Register 9 (SR9) poweron 00H

	// MCLK PLL defaults (MAME values)
	s3.sr10 = 0x42;
	s3.sr11 = 0x41;

	// DCLK PLL defaults (MAME values)
	s3.sr12 = 0x00;
	s3.sr13 = 0x00;
	s3.clk_pll_n = 0x00;
	s3.clk_pll_r = 0x00;

	// Use VIDEO_RAM_SIZE (in bits) to size VRAM. With 22 this is 4 MB.
	vga.svga_intf.vram_size = 1u << VIDEO_RAM_SIZE;
	vga.memory = new u8[vga.svga_intf.vram_size];
	memset(vga.memory, 0, vga.svga_intf.vram_size);

	state.last_bpp = 8;

	s3.id_cr30 = 0xE1; // Chip ID/REV register CR30, dosbox-x implementation returns 0x00 for our use case. poweron default is E1H however.
	m_crtc_map.write_byte(0x32, 0x00); // Locked by default
	m_crtc_map.write_byte(0x33, 0x00); // CR33 (Backward Compatibility 2) default 00h (no locks).
	m_crtc_map.write_byte(0x36, s3_cr36_from_memsize(vga.svga_intf.vram_size, true)); // Configuration 2 Register (CONFG_REG1) (CR36) - bootstrap config
	m_crtc_map.write_byte(0x37, 0xE5); // Configuration 2 Register (CONFG_REG2) (CR37)  - bootstrap read, sane value from 86box
	m_crtc_map.write_byte(0x3B, 0x00); // CR3B: Data Transfer Position (DTPC)
	m_crtc_map.write_byte(0x3C, 0x00); // CR3C: IL_RTSTART defaulting
	m_crtc_map.write_byte(0x40, 0x30); // System Configuration Register, power on default 30h
	m_crtc_map.write_byte(0x42, 0x00); // Mode Control 2- can set interlace vs non
	printf("%u", s3_cr36_from_memsize(vga.svga_intf.vram_size, true));
	vga.gc.memory_map_sel = 3; // color text mode
	state.vga_mem_updated = 1;

	// init MAME fields
	s3.id_high = 0x88;    // 86C764 Trio64
	s3.id_low = 0x11;
	s3.revision = 0x00;
	s3.id_cr30 = 0xE1;    // Trio64 (per datasheet)

	// Hardware cursor defaults (MAME says windows 95 doesn't program these but it applies it regardless to everything)
	for (int i = 0; i < 4; i++) {
		s3.cursor_fg[i] = 0xFF;
		s3.cursor_bg[i] = 0x00;
	}

	// CR56: External Sync Control 1 (EX_SYNC_1) power-on default 00h
	m_crtc_map.write_byte(0x56, 0x00);

	// CR57: EX_SYNC_2 (VSYNC reset adjust), power-on default 00h
	m_crtc_map.write_byte(0x57, 0x00);

	// EX_SYNC_3 (CR63)
	m_crtc_map.write_byte(0x63, 0x00);

	// CNFG-REG-3 (CR68) poweron strap; datasheet says power-on samples PD[23:16].
	// 00h per 86Box-compatible. If needed, set CRTC.reg[0x68] before 
	// recompute_config3() for different.
	m_crtc_map.write_byte(0x68, 0x00);

	// CR65: Extended Miscellaneous Control Register (EXT-MISC-CTL) (CR65)
	m_crtc_map.write_byte(0x65, 0x00);

	m_8514.set_vga_ptr(this);
	m_8514.start();

	refresh_pitch_offset(); // do it initially, just for sanity sake

	myThread = 0;

	printf("%s: $Id$\n",
		devid_string);
}


void CS3Trio64::recompute_scanline_layout()
{
	const uint8_t cr5d = m_crtc_map.read_byte(0x5d);

	auto xbit = [&](int b) -> uint16_t { return (cr5d >> b) & 1u; };

	// still some compute/reliance, need to sync up here...

	// vga.crtc.horz_total - MAME sets low 8, we overlay bit8 from CR5D.
	vga.crtc.horz_total = (vga.crtc.horz_total & 0xff) | (xbit(0) << 8);

	// vga.crtc.horz_disp_end — same pattern.
	vga.crtc.horz_disp_end = (vga.crtc.horz_disp_end & 0xff) | (xbit(1) << 8);

	// vga.crtc.horz_blank_start — can only hold low 8 bits.
	// Compose full 9-bit value 
	vga.crtc.horz_blank_start = uint16_t(vga.crtc.horz_blank_start) | (xbit(2) << 8);

	// vga.crtc.horz_retrace_start — same.
	vga.crtc.horz_retrace_start = uint16_t(vga.crtc.horz_retrace_start) | (xbit(4) << 8);

	// vga.crtc.horz_blank_end, holds bits 0-5 (CR03 + CR05).
	uint16_t hb_end_base = uint16_t(vga.crtc.horz_blank_end & 0x3f);
	vga.crtc.horz_blank_end = hb_end_base + (xbit(3) ? 64 : 0);

	// vga.crtc.horz_retrace_end, holds bits 0-4 (from CR05).
	uint16_t hs_end_base = uint16_t(vga.crtc.horz_retrace_end & 0x1f);
	vga.crtc.horz_retrace_end = hs_end_base + (xbit(5) ? 32 : 0);

	// S3 special blanking (CR33 bit5)
	if (m_crtc_map.read_byte(0x33) & 0x20) {
		vga.crtc.horz_blank_start = vga.crtc.horz_disp_end;
		vga.crtc.horz_blank_end = (vga.crtc.horz_total - 1) & 0x1FF;
	}

	// 9-bit masking
	vga.crtc.horz_total &= 0x1FF;
	vga.crtc.horz_disp_end &= 0x1FF;
	vga.crtc.horz_blank_start &= 0x1FF;
	vga.crtc.horz_blank_end &= 0x1FF;
	vga.crtc.horz_retrace_start &= 0x1FF;
	vga.crtc.horz_retrace_end &= 0x1FF;
}

void CS3Trio64::recompute_params()
{
	recompute_scanline_layout();
	refresh_pitch_offset();
	redraw_area(0, 0, old_iWidth, old_iHeight);
}

void CS3Trio64::attribute_map(address_map& map)
{
	map.global_mask(0x3f);
	map.unmap_value_high();

	// Palette Index Registers 0x00..0x0f
	map(0x00, 0x0f).lrw8(
		NAME([this](offs_t offset) {
			return vga.attribute.data[offset & 0x1f];
			}),
		NAME([this](offs_t offset, u8 data) {
			// CR33 bit6: Lock Palette/Overscan Registers
			if (m_crtc_map.read_byte(0x33) & 0x40)
				return;
			u8 idx = offset & 0x0f;
			if (vga.attribute.data[idx] != (data & 0x3f)) {
				vga.attribute.data[idx] = data & 0x3f;
				redraw_area(0, 0, old_iWidth, old_iHeight);
			}
			})
	);

	// 0x20-0x2f mirrors — NOP (MAME: map(0x20, 0x2f).noprw())
	// Our address_map doesn't have noprw(); unmap_value_high covers reads.
	// Writes to this range are simply ignored by having no handler installed.

	// Mode Control (index 0x10, mirrored at 0x30)
	map(0x10, 0x10).mirror(0x20).lrw8(
		NAME([this](offs_t offset) {
			return vga.attribute.data[0x10];
			}),
		NAME([this](offs_t offset, u8 data) {
			const u8 prev = vga.attribute.data[0x10];
			vga.attribute.data[0x10] = data & 0x3f; // MAME canonical

			// ES40 side-effects: detect bit changes from previous value
			if (BIT(data, 2) != BIT(prev, 2)) {  // enable_line_graphics
				bx_gui->lock();
				bx_gui->set_text_charmap(
					&vga.memory[0x20000 + vga.sequencer.char_sel.A]);
				bx_gui->unlock();
				state.vga_mem_updated = 1;
			}
			if (BIT(data, 7) != BIT(prev, 7)) {  // internal_palette_size
				redraw_area(0, 0, old_iWidth, old_iHeight);
			}
			})
	);

	// Overscan Color (index 0x11, mirrored at 0x31)
	map(0x11, 0x11).mirror(0x20).lrw8(
		NAME([this](offs_t offset) {
			return vga.attribute.data[0x11];
			}),
		NAME([this](offs_t offset, u8 data) {
			// CR33 bit6: Lock Palette/Overscan Registers
			if (m_crtc_map.read_byte(0x33) & 0x40)
				return;
			vga.attribute.data[0x11] = data & 0x3f; // MAME canonical
			})
	);

	// Color Plane Enable (index 0x12, mirrored at 0x32)
	map(0x12, 0x12).mirror(0x20).lrw8(
		NAME([this](offs_t offset) {
			return vga.attribute.data[0x12];
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.attribute.data[0x12] = data & 0x3f; // MAME canonical
			redraw_area(0, 0, old_iWidth, old_iHeight);
			})
	);

	// Horizontal PEL Shift (index 0x13, mirrored at 0x33)
	map(0x13, 0x13).mirror(0x20).lrw8(
		NAME([this](offs_t offset) {
			return vga.attribute.data[0x13];
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.attribute.pel_shift_latch = data & 0xf; // MAME canonical
			vga.attribute.data[0x13] = data & 0xf;      // MAME canonical
			redraw_area(0, 0, old_iWidth, old_iHeight);
			})
	);

	// Color Select (index 0x14, mirrored at 0x34)
	map(0x14, 0x14).mirror(0x20).lrw8(
		NAME([this](offs_t offset) {
			return vga.attribute.data[0x14];
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.attribute.pel_shift_latch = data & 0xf; // MAME canonical (yes, same field)
			vga.attribute.data[0x14] = data & 0xf;      // MAME canonical
			redraw_area(0, 0, old_iWidth, old_iHeight);
			})
	);
}

/**************************************
 *
 * GC
 *
 *************************************/

void CS3Trio64::gc_map(address_map& map)
{
	map.unmap_value_high();
	map(0x00, 0x00).lrw8(
		NAME([this](offs_t offset) {
			return vga.gc.set_reset & 0xf;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.gc.set_reset = data & 0xf;
			})
	);
	map(0x01, 0x01).lrw8(
		NAME([this](offs_t offset) {
			return vga.gc.enable_set_reset & 0xf;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.gc.enable_set_reset = data & 0xf;
			})
	);
	map(0x02, 0x02).lrw8(
		NAME([this](offs_t offset) {
			return vga.gc.color_compare & 0xf;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.gc.color_compare = data & 0xf;
			})
	);
	map(0x03, 0x03).lrw8(
		NAME([this](offs_t offset) {
			return ((vga.gc.logical_op & 3) << 3) | (vga.gc.rotate_count & 7);
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.gc.logical_op = (data & 0x18) >> 3;
			vga.gc.rotate_count = data & 7;
			})
	);
	map(0x04, 0x04).lrw8(
		NAME([this](offs_t offset) {
			return vga.gc.read_map_sel & 3;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.gc.read_map_sel = data & 3;
			})
	);
	map(0x05, 0x05).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.gc.shift256 & 1) << 6;
			res |= (vga.gc.shift_reg & 1) << 5;
			res |= (vga.gc.host_oe & 1) << 4;
			res |= (vga.gc.read_mode & 1) << 3;
			res |= (vga.gc.write_mode & 3);
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.gc.shift256 = BIT(data, 6);
			vga.gc.shift_reg = BIT(data, 5);
			vga.gc.host_oe = BIT(data, 4);
			vga.gc.read_mode = BIT(data, 3);
			vga.gc.write_mode = data & 3;
			//if(data & 0x10 && vga.gc.alpha_dis)
			//  popmessage("Host O/E enabled, contact MAMEdev");
			})
	);
	map(0x06, 0x06).lrw8(
		NAME([this](offs_t offset) {
			u8 res = (vga.gc.memory_map_sel & 3) << 2;
			res |= (vga.gc.chain_oe & 1) << 1;
			res |= (vga.gc.alpha_dis & 1);
			return res;
			}),
		NAME([this](offs_t offset, u8 data) {
			u8 prev_memory_mapping = vga.gc.memory_map_sel; //es40ism
			bool prev_alpha_dis = vga.gc.alpha_dis; // es40ism
			// MAME
			vga.gc.memory_map_sel = (data & 0xc) >> 2;
			vga.gc.chain_oe = BIT(data, 1);
			vga.gc.alpha_dis = BIT(data, 0);
			//if(data & 2 && vga.gc.alpha_dis)
			//  popmessage("Chain O/E enabled, contact MAMEdev");
			// ES40 side-effects: redraw on mapping/mode change
			if (prev_memory_mapping != vga.gc.memory_map_sel)
				redraw_area(0, 0, old_iWidth, old_iHeight);
			if (prev_alpha_dis != vga.gc.alpha_dis) {
				redraw_area(0, 0, old_iWidth, old_iHeight);
				old_iHeight = 0;
			}
			})
	);
	map(0x07, 0x07).lrw8(
		NAME([this](offs_t offset) {
			return vga.gc.color_dont_care & 0xf;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.gc.color_dont_care = data & 0xf;
			})
	);
	map(0x08, 0x08).lrw8(
		NAME([this](offs_t offset) {
			return vga.gc.bit_mask & 0xff;
			}),
		NAME([this](offs_t offset, u8 data) {
			vga.gc.bit_mask = data & 0xff;
			})
	);
}

void CS3Trio64::recompute_params_clock(int divisor, int xtal)
{
	// Store timing parameters for renderer/debug use -- ES40 specific
	timing.xtal_hz = xtal;
	timing.divisor = divisor;
	// es40 specific end

	int vblank_period, hblank_period;
	attoseconds_t refresh;
	uint8_t hclock_m = (!GRAPHIC_MODE) ? VGA_CH_WIDTH : 8;
	int pixel_clock;

	/* safety check */
	if (!vga.crtc.horz_disp_end || !vga.crtc.vert_disp_end || !vga.crtc.horz_total) // check needs 'vga.crtc.vert_total' but we don't implement.... yet
		return;

	const u8 is_interlace_mode = get_interlace_mode() + 1;
	const int display_lines = vga.crtc.vert_disp_end * is_interlace_mode;

	//rectangle visarea(0, ((vga.crtc.horz_disp_end + 1) * ((float)(hclock_m) / divisor)) - 1, 0, display_lines);

	vblank_period = (vga.crtc.vert_total + 2) * is_interlace_mode;
	hblank_period = ((vga.crtc.horz_total + 5) * ((float)(hclock_m) / divisor));

	// TODO: improve/complete clocking modes
	pixel_clock = xtal / ((x_dotclockdiv2() >> 3) + 1);

	refresh = HZ_TO_ATTOSECONDS(pixel_clock) * (hblank_period)*vblank_period;
	//screen().configure((hblank_period), (vblank_period), visarea, refresh);
	//m_vblank_timer->adjust(screen().time_until_pos(vga.crtc.vert_blank_start + vga.crtc.vert_blank_end));

	// ES40 specific here - MAME: pixel_clock = xtal / (((vga.sequencer.data[1]&8) >> 3) + 1);
	const int seq_div = ((vga.sequencer.data[1] & 0x08) >> 3) + 1;
	timing.pixel_clock_hz = (seq_div > 0) ? (xtal / seq_div) : xtal;

	// Recompute line offset (pitch) — ES40's existing function
	refresh_pitch_offset();

	// Mark display dirty so the renderer picks up changes
	state.vga_mem_updated = 1;
}

/**
 * Create and start thread.
 **/
void CS3Trio64::start_threads()
{
	// Resume after reset if the thread already exists
	PauseThread.store(false, std::memory_order_release);

	if (!myThread)
	{
		myThread = new CThread("s3");
		printf(" %s", myThread->getName().c_str());
		StopThread = false;
		myThread->start(*this);
	}
}

/**
 * Stop and destroy thread.
 **/
void CS3Trio64::stop_threads()
{
	// During firmware reset, do NOT kill the S3 thread (it owns the SDL window).
	// Just pause it so the window stays alive.
	if (cSystem && cSystem->IsResetInProgress())
	{
		PauseThread.store(true, std::memory_order_release);

		// Wait briefly until the S3 thread acknowledges pause
		if (myThread)
		{
			for (int spin = 0; spin < 600; spin++) // up to ~600ms
			{
				if (PauseAck.load(std::memory_order_acquire))
					break;
				CThread::sleep(1);
			}
			// Make it visible in the log whether we paused or stopped
			printf(" s3(pause)");
		}
		return;
	}

	// Normal shutdown: actually stop the thread.
	StopThread = true;
	if (myThread)
	{
		printf(" %s", myThread->getName().c_str());
		myThread->join();
		delete myThread;
		myThread = 0;
	}
}

/**
 * Destructor.
 **/
CS3Trio64::~CS3Trio64()
{
	stop_threads();
}

/**
 * Read from one of the Legacy (fixed-address) memory ranges.
 **/
u32 CS3Trio64::ReadMem_Legacy(int index, u32 address, int dsize)
{
	u32 data = 0;
	switch (index)
	{
		// IO Port 0x3b4
	case 1:
		data = io_read(address + 0x3b4, dsize);
		break;

		// IO Port 0x3c0..0x3cf
	case 2:
		data = io_read(address + 0x3c0, dsize);
		break;

		// IO Port 0x3ba
	case 3:
		data = io_read(address + 0x3ba, dsize);
		break;

		// VGA Memory
	case 4:
		data = legacy_read(address, dsize);
		break;

		// ROM
	case 5:
		data = rom_read(address, dsize);
		break;

		// IO Port 0x3d4
	case 8:
		data = io_read(address + 0x3d4, dsize);
		break;

		// IO Port 0x3da
	case 9:
		data = io_read(address + 0x3da, dsize);
		break;

	case 10: data = io_read(address + 0x42E8, dsize); break;
	case 11: data = io_read(address + 0x4AE8, dsize); break;
	case 12: data = io_read(address + 0x46E8, dsize); break;
	case 13: data = io_read(address + 0x4EE8, dsize); break;
	case 14: data = io_read(address + 0x86E8, dsize); break;
	case 15: data = io_read(address + 0x8EE8, dsize); break;
	case 16: data = io_read(address + 0x96E8, dsize); break;
	case 17: data = io_read(address + 0x9AE8, dsize); break;
	case 18: data = io_read(address + 0xA2E8, dsize); break; // BKGD_COLOR
	case 19: data = io_read(address + 0xA6E8, dsize); break; // FRGD_COLOR
	case 20: data = io_read(address + 0xAAE8, dsize); break; // WRT_MASK
	case 21: data = io_read(address + 0xAEE8, dsize); break; // RD_MASK
	case 22: data = io_read(address + 0xB6E8, dsize); break; // BKGD_MIX
	case 23: data = io_read(address + 0xBAE8, dsize); break; // FRGD_MIX
	case 24: data = io_read(address + 0xE2E8, dsize); break; // PIX_TRANS (8 bytes)
	case 25: data = io_read(address + 0xB2E8, dsize); break;
	case 26: data = io_read(address + 0xBEE8, dsize); break;
	case 27: data = io_read(address + 0xD2E8, dsize); break;
	case 28: data = io_read(address + 0x9EE8, dsize); break;
	case 29: data = io_read(address + 0xCAE8, dsize); break;
	}

	return data;
}

/**
 * Write to one of the Legacy (fixed-address) memory ranges.
 **/
void CS3Trio64::WriteMem_Legacy(int index, u32 address, int dsize, u32 data)
{
	switch (index)
	{
		// IO Port 0x3b4
	case 1:
		io_write(address + 0x3b4, dsize, data);
		return;

		// IO Port 0x3c0..0x3cf
	case 2:
		io_write(address + 0x3c0, dsize, data);
		return;

		// IO Port 0x3ba
	case 3:
		io_write(address + 0x3ba, dsize, data);
		return;

		// VGA Memory
	case 4:
		legacy_write(address, dsize, data);
		return;

		// BIOS Message IO Port (0x500)
	case 7:
		bios_message[bios_message_size++] = (char)data & 0xff;
		if (((data & 0xff) == 0x0a) || ((data & 0xff) == 0x0d))
		{
			if (bios_message_size > 1)
			{
				bios_message[bios_message_size - 1] = '\0';
				printf("s3: %s\n", bios_message);
			}

			bios_message_size = 0;
		}

		return;

		// IO Port 0x3d4
	case 8:
		io_write(address + 0x3d4, dsize, data);
		return;

		// IO Port 0x3da
	case 9:
		io_write(address + 0x3da, dsize, data);
		return;

	case 10: io_write(address + 0x42E8, dsize, data); break;
	case 11: io_write(address + 0x4AE8, dsize, data); break;
	case 12: io_write(address + 0x46E8, dsize, data); break;
	case 13: io_write(address + 0x4EE8, dsize, data); break;
	case 14: io_write(address + 0x86E8, dsize, data); break;
	case 15: io_write(address + 0x8EE8, dsize, data); break;
	case 16: io_write(address + 0x96E8, dsize, data); break;
	case 17: io_write(address + 0x9AE8, dsize, data); break;
	case 18: io_write(address + 0xA2E8, dsize, data); break; // BKGD_COLOR
	case 19: io_write(address + 0xA6E8, dsize, data); break; // FRGD_COLOR
	case 20: io_write(address + 0xAAE8, dsize, data); break; // WRT_MASK
	case 21: io_write(address + 0xAEE8, dsize, data); break; // RD_MASK
	case 22: io_write(address + 0xB6E8, dsize, data); break; // BKGD_MIX
	case 23: io_write(address + 0xBAE8, dsize, data); break; // FRGD_MIX
	case 24: io_write(address + 0xE2E8, dsize, data); break; // PIX_TRANS (8 bytes)
	case 25: io_write(address + 0xB2E8, dsize, data); break;
	case 26: io_write(address + 0xBEE8, dsize, data); break;
	case 27: io_write(address + 0xD2E8, dsize, data); break;
	case 28: io_write(address + 0x9EE8, dsize, data); break;
	case 29: io_write(address + 0xCAE8, dsize, data); break;
	}
}

int CS3Trio64::BytesPerPixel() const
{
	const uint8_t pf = (s3.ext_misc_ctrl_2 >> 4) & 0x0F;
	switch (pf) {
	case 0x01: return 1; // Mode 8: 8-bit packed
	case 0x02: return 2; // Mode 1: 15-bit (2 VCLK/pixel)
	case 0x03: return 2; // Mode 9: 15-bit (1 VCLK/pixel)
	case 0x04: return 3; // Mode 2: 24-bit (3 VCLK/pixel)
	case 0x05: return 2; // Mode 10: 16-bit (1 VCLK/pixel)
	case 0x06: return 2; // Mode 3: 16-bit (2 VCLK/pixel)
	case 0x07: return 4; // Mode 11: 32-bit (2 VCLK/pixel) 
	case 0x0D: return 4; // Mode 13: 32-bit alternate
	default:   return 1; // Mode 0: 8bpp indexed (or VGA)
	}
}

u32 CS3Trio64::PitchBytes() const
{
	return (uint32_t)vga.crtc.offset;
}

static inline u32 clamp_vram_addr(u32 a, u32 vram_size) {
	return (vram_size == 0) ? a : (a % vram_size);
}

void CS3Trio64::accel_reset() {
	m_8514.start();
	m_8514.ibm8514.enabled = (s3.cr40 & 0x01);
}


// -------------------------
// Minimal accel window I/O
// -------------------------
u8 CS3Trio64::AccelIORead(u32 port)
{
	ibm8514a_device* dev = get_8514();
	if (!s3.enable_8514) return 0xFF;

	switch (port & 0xFFFE) {
	case 0x9AE8: {
		uint16_t ret = dev->ibm8514_gpstatus_r();
		return (port & 1) ? (uint8_t)(ret >> 8) : (uint8_t)(ret & 0xff);
	}

	case 0x42E8: {
		uint16_t v = dev->ibm8514_substatus_r();
		return (port & 1) ? (v >> 8) : (v & 0xff);
	}

			   // coordinate & size readbacks
	case 0x82E8: { uint16_t v = dev->ibm8514_currenty_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }
	case 0x86E8: { uint16_t v = dev->ibm8514_currentx_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }
	case 0x8AE8: { uint16_t v = dev->ibm8514_desty_r();    return (port & 1) ? (v >> 8) : (v & 0xff); }
	case 0x8EE8: { uint16_t v = dev->ibm8514_destx_r();    return (port & 1) ? (v >> 8) : (v & 0xff); }
	case 0x92E8: { uint16_t v = dev->ibm8514_line_error_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }
	case 0x96E8: { uint16_t v = dev->ibm8514_width_r();    return (port & 1) ? (v >> 8) : (v & 0xff); }

	case 0x9EE8: { uint16_t v = dev->ibm8514_ssv_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }

	case 0xA2E8: { uint16_t v = dev->ibm8514_bgcolour_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }
	case 0xA6E8: { uint16_t v = dev->ibm8514_fgcolour_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }

	case 0xAAE8: { uint16_t v = dev->ibm8514_write_mask_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }
	case 0xAEE8: { uint16_t v = dev->ibm8514_read_mask_r();  return (port & 1) ? (v >> 8) : (v & 0xff); }

	case 0xB6E8: { uint16_t v = dev->ibm8514_backmix_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }
	case 0xBAE8: { uint16_t v = dev->ibm8514_foremix_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }

	case 0xBEE8: { uint16_t v = dev->ibm8514_multifunc_r(); return (port & 1) ? (v >> 8) : (v & 0xff); }

	default:
		return 0x00;
	}
}

static inline void write16_low_high(u16& reg, u32 port, u8 data) {
	if ((port & 1) == 0) reg = (reg & 0xFF00u) | data;
	else                 reg = (reg & 0x00FFu) | (u16)data << 8;
}

void CS3Trio64::AccelIOWrite(u32 port, u8 data)
{
	ibm8514a_device* dev = get_8514();
	if (!s3.enable_8514) return;

	switch (port & 0xFFFE) {

		// SUBSYS_CNTL (42E8h write)
	case 0x42E8:
		if ((port & 1) == 0)
			s3.mmio_42e8 = (s3.mmio_42e8 & 0xff00) | data;
		else
			s3.mmio_42e8 = (s3.mmio_42e8 & 0x00ff) | (data << 8);
		dev->ibm8514_subcontrol_w(s3.mmio_42e8);
		break;

		// ADVFUNC_CNTL (4AE8h)
	case 0x4AE8:
		if ((port & 1) == 0)
			s3.mmio_4ae8 = (s3.mmio_4ae8 & 0xff00) | data;
		else
			s3.mmio_4ae8 = (s3.mmio_4ae8 & 0x00ff) | (data << 8);
		dev->ibm8514_advfunc_w(s3.mmio_4ae8);
		break;

		// CUR_Y (82E8h) — MAME sets prev_y too
	case 0x82E8:
		if ((port & 1) == 0) {
			dev->ibm8514.curr_y = (dev->ibm8514.curr_y & 0xff00) | data;
			dev->ibm8514.prev_y = (dev->ibm8514.prev_y & 0xff00) | data;
		}
		else {
			dev->ibm8514.curr_y = (dev->ibm8514.curr_y & 0x00ff) | (data << 8);
			dev->ibm8514.prev_y = (dev->ibm8514.prev_y & 0x00ff) | (data << 8);
		}
		break;

		// CUR_X (86E8h) — MAME sets prev_x too
	case 0x86E8:
		if ((port & 1) == 0) {
			dev->ibm8514.curr_x = (dev->ibm8514.curr_x & 0xff00) | data;
			dev->ibm8514.prev_x = (dev->ibm8514.prev_x & 0xff00) | data;
		}
		else {
			dev->ibm8514.curr_x = (dev->ibm8514.curr_x & 0x00ff) | (data << 8);
			dev->ibm8514.prev_x = (dev->ibm8514.prev_x & 0x00ff) | (data << 8);
		}
		break;

		// DESTY/AXSTP (8AE8h) — dual-purpose register
	case 0x8AE8:
		if ((port & 1) == 0) {
			dev->ibm8514.line_axial_step = (dev->ibm8514.line_axial_step & 0xff00) | data;
			dev->ibm8514.dest_y = (dev->ibm8514.dest_y & 0xff00) | data;
		}
		else {
			dev->ibm8514.line_axial_step = (dev->ibm8514.line_axial_step & 0x00ff) | ((data & 0x3f) << 8);
			dev->ibm8514.dest_y = (dev->ibm8514.dest_y & 0x00ff) | (data << 8);
		}
		break;

		// DESTX/DIASTP (8EE8h) — dual-purpose register
	case 0x8EE8:
		if ((port & 1) == 0) {
			dev->ibm8514.line_diagonal_step = (dev->ibm8514.line_diagonal_step & 0xff00) | data;
			dev->ibm8514.dest_x = (dev->ibm8514.dest_x & 0xff00) | data;
		}
		else {
			dev->ibm8514.line_diagonal_step = (dev->ibm8514.line_diagonal_step & 0x00ff) | ((data & 0x3f) << 8);
			dev->ibm8514.dest_x = (dev->ibm8514.dest_x & 0x00ff) | (data << 8);
		}
		break;

		// ERR_TERM (92E8h)
	case 0x92E8:
		if ((port & 1) == 0)
			s3.mmio_92e8 = (s3.mmio_92e8 & 0xff00) | data;
		else {
			s3.mmio_92e8 = (s3.mmio_92e8 & 0x00ff) | (data << 8);
			dev->ibm8514_line_error_w(s3.mmio_92e8);
		}
		break;

		// MAJ_AXIS_PCNT (96E8h)
	case 0x96E8:
		if ((port & 1) == 0)
			s3.mmio_96e8 = (s3.mmio_96e8 & 0xff00) | data;
		else {
			s3.mmio_96e8 = (s3.mmio_96e8 & 0x00ff) | (data << 8);
			dev->ibm8514_width_w(s3.mmio_96e8);
		}
		break;

		// CMD (9AE8h) — high byte triggers execution!
	case 0x9AE8:
		if ((port & 1) == 0)
			s3.mmio_9ae8 = (s3.mmio_9ae8 & 0xff00) | data;
		else {
			s3.mmio_9ae8 = (s3.mmio_9ae8 & 0x00ff) | (data << 8);
			dev->ibm8514_cmd_w(s3.mmio_9ae8);
		}
		break;

		// SSV (9EE8h) — high byte triggers execution
	case 0x9EE8:
		if ((port & 1) == 0)
			s3.mmio_9ee8 = (s3.mmio_9ee8 & 0xff00) | data;
		else {
			s3.mmio_9ee8 = (s3.mmio_9ee8 & 0x00ff) | (data << 8);
			dev->ibm8514_ssv_w(s3.mmio_9ee8);
		}
		break;

		// BKGD_COLOR (A2E8h)
	case 0xA2E8:
	{
		unsigned s = (port & 1) ? 8 : 0;
		dev->ibm8514.bgcolour = (dev->ibm8514.bgcolour & ~(0xFFu << s)) | ((u32)data << s);
	}
	break;

	// FRGD_COLOR (A6E8h)
	case 0xA6E8:
	{
		unsigned s = (port & 1) ? 8 : 0;
		dev->ibm8514.fgcolour = (dev->ibm8514.fgcolour & ~(0xFFu << s)) | ((u32)data << s);
	}
	break;

	// WRT_MASK (AAE8h)
	case 0xAAE8:
	{
		unsigned s = (port & 1) ? 8 : 0;
		dev->ibm8514.write_mask = (dev->ibm8514.write_mask & ~(0xFFu << s)) | ((u32)data << s);
	}
	break;

	// RD_MASK (AEE8h)
	case 0xAEE8:
	{
		unsigned s = (port & 1) ? 8 : 0;
		dev->ibm8514.read_mask = (dev->ibm8514.read_mask & ~(0xFFu << s)) | ((u32)data << s);
	}
	break;

	// BKGD_MIX (B6E8h)
	case 0xB6E8:
		if ((port & 1) == 0)
			dev->ibm8514.bgmix = (dev->ibm8514.bgmix & 0xff00) | data;
		else
			dev->ibm8514.bgmix = (dev->ibm8514.bgmix & 0x00ff) | (data << 8);
		break;

		// FRGD_MIX (BAE8h)
	case 0xBAE8:
		if ((port & 1) == 0)
			dev->ibm8514.fgmix = (dev->ibm8514.fgmix & 0xff00) | data;
		else
			dev->ibm8514.fgmix = (dev->ibm8514.fgmix & 0x00ff) | (data << 8);
		break;

		// MULTIFUNC_CNTL (BEE8h) — high byte triggers dispatch
	case 0xBEE8:
		if ((port & 1) == 0)
			s3.mmio_bee8 = (s3.mmio_bee8 & 0xff00) | data;
		else {
			s3.mmio_bee8 = (s3.mmio_bee8 & 0x00ff) | (data << 8);
			dev->ibm8514_multifunc_w(s3.mmio_bee8);
		}
		break;

		// PIX_TRANS (E2E8h..E2EFh) — host data upload
		// Uses the ibm8514a's bus_size-aware accumulation + wait_draw()
	case 0xE2E8: case 0xE2EA: case 0xE2EC: case 0xE2EE:
	{
		if (dev->ibm8514.bus_size == 0) {
			dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffffff00) | data;
			dev->ibm8514_pixel_xfer_complete();
		}
		else if (dev->ibm8514.bus_size == 1) {
			switch (port & 0x0001) {
			case 0: dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffffff00) | data; break;
			case 1: dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffff00ff) | (data << 8);
				dev->ibm8514_pixel_xfer_complete();
				break;
			}
		}
		else if (dev->ibm8514.bus_size == 2) {
			switch (port & 0x0003) {
			case 0: dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffffff00) | data; break;
			case 1: dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xffff00ff) | (data << 8); break;
			case 2: dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0xff00ffff) | (data << 16); break;
			case 3: dev->ibm8514.pixel_xfer = (dev->ibm8514.pixel_xfer & 0x00ffffff) | (data << 24);
				dev->ibm8514_pixel_xfer_complete();
				break;
			}
		}
		break;
	}

	default:
		LOG("S3 Accel: unhandled I/O write port=%04x data=%02x\n", port, data);
		break;
	}
}

bool CS3Trio64::IsAccelPort(u32 p) const {
	switch (p & 0xFFFE) { // word regs, we accept low/high bytes
		// status/control
	case 0x42E8: // SUBSYS_CNTL / SUBSYS_STAT (w/r)
	case 0x4AE8: // ADVFUNC_CNTL
		// coordinates
	case 0x46E8: // CUR_X
	case 0x4EE8: // CUR_Y
	case 0x86E8: // DESTX
	case 0x8EE8: // DESTY / AXSTP (S3 encodes AXSTP here)
	case 0xCAE8: // DESTY / AXSTP alias (seen in 86Box mappings)
		// dimensions / count
	case 0x96E8: // MAJ_AXIS_PCNT
		// mixes, masks, colors
	case 0xA2E8: // BKGD_COLOR
	case 0xA6E8: // FRGD_COLOR
	case 0xAAE8: // WRT_MASK
	case 0xAEE8: // RD_MASK
	case 0xB6E8: // BKGD_MIX
	case 0xBAE8: // FRGD_MIX
		// extra control regs used by some S3 paths
	case 0xD2E8: // ROP_MIX (word)
	case 0xBEE8: // MULTIFUNC_CNTL (word)
	case 0x9EE8: // SHORT_STROKE (word)
	case 0x9D48: // SSV (older window alias)
	case 0xB2E8: // PIX_CNTL or ALT PIX_TRANS (word/byte window depending on gate)
	case 0x9AE8: // CMD
	case 0x82E8:
	case 0x92E8:
		return true;
	default:
		break;
	}
	// Host data (PIX_TRANS): accept 0xE2E8..0xE2EF byte-wise for color fills
	if ((p & 0xFFF0u) == 0xE2E0u) return true; // PIX_TRANS/host
	return false;
}

/**
 * Read from one of the PCI BAR (configurable address) memory ranges.
 **/
u32 CS3Trio64::ReadMem_Bar(int func, int bar, u32 address, int dsize)
{
	if (lfb_trace_needs_first_access_note) {
		printf("%s: LFB first BAR access @+%llx size=%d\n",
			devid_string, (unsigned long long)address, dsize);
		lfb_trace_needs_first_access_note = false;
	}

	switch (bar)
	{
		// PCI memory range
	case 0:
		if (!lfb_active) {
			// No decode when LFB disabled  mimic bus-float/read-as-FFs
			return (dsize == 1) ? 0xFFu : (dsize == 2) ? 0xFFFFu : 0xFFFFFFFFu;
		}
		return mem_read(address, dsize);
	}

	return 0;
}

/**
 * Write to one of the PCI BAR (configurable address) memory ranges.
 **/
void CS3Trio64::WriteMem_Bar(int func, int bar, u32 address, int dsize, u32 data)
{
#if DEBUG_PCI
	printf("[S3::WriteMem_Bar] func=%d bar=%d addr=%08X dsize=%d data=%08X\n",
		func, bar, address, dsize, data);
#endif
	if (lfb_trace_needs_first_access_note) {
		printf("%s: LFB first BAR access @+%llx size=%d (W)\n",
			devid_string, (unsigned long long)address, dsize);
		lfb_trace_needs_first_access_note = false;
	}

	switch (bar)
	{
		// PCI Memory range
	case 0:
		if (!lfb_active) {
			// Ignore writes when LFB disabled
			return;
		}
		mem_write(address, dsize, data);
		return;
	}
}

// --- Only include LFB here; legacy VGA paths fall through to CVGA ---
u64 CS3Trio64::ReadMem(int index, u64 address, int dsize)
{
	// LFB window (registered by update_linear_mapping)
	if (index == DEV_LFB_IDX && lfb_active && lfb_size)
	{
		const u64 off = address; // dispatcher already subtracts base
		// Trio64 "new MMIO" 128 KiB window at LFB+0x0100_0000 (CR53 bit3)
		//  - Lower half : PIX_TRANS FIFO (0xE2E8..0xE2EB) for 8/16/32-bit reads
		//  - Upper half : 8514/A register mirror at *E8 offsets (IsAccelPort())
		if (s3_new_mmio_enabled()) {
			printf("NEW MMIO READ !!!\n");
			const u64 win_lo = 0x01000000ull;
			const u64 win_mid = 0x01008000ull;
			const u64 win_hi = 0x01020000ull;
			if (off >= win_lo && off < win_hi) {
				if (off < win_mid) {
					switch (dsize) {
					case 8:
						return (u64)AccelIORead(0xE2E8);
					case 16:
						return (u64)AccelIORead(0xE2E8) | ((u64)AccelIORead(0xE2E9) << 8);
					case 32:
						return (u64)AccelIORead(0xE2E8) | ((u64)AccelIORead(0xE2E9) << 8) | ((u64)AccelIORead(0xE2EA) << 16) | ((u64)AccelIORead(0xE2EB) << 24);
					default: FAILURE(InvalidArgument, "Unsupported dsize");
					}
				}
				else {
					const u32 p = (u32)(off - win_lo); // ports by offset
					if (IsAccelPort(p)) {
						switch (dsize) {
						case 8:
							return (u64)AccelIORead(p);
						case 16:
							return (u64)AccelIORead(p + 0) | ((u64)AccelIORead(p + 1) << 8);
						case 32:
							return (u64)AccelIORead(p + 0) | ((u64)AccelIORead(p + 1) << 8) | ((u64)AccelIORead(p + 2) << 16) | ((u64)AccelIORead(p + 3) << 24);
						default: FAILURE(InvalidArgument, "Unsupported dsize");
						}
					}
				}
			}
		}
		if (off >= lfb_size) return 0;

		// Read little-endian from linear VRAM
		switch (dsize)
		{
		case 1:  return vga.memory[off];
		case 2:  return *(u16*)(vga.memory + off);
		case 4:  return *(u32*)(vga.memory + off);
		case 8: {
			u64 v = *(u32*)(vga.memory + off);
			v |= (u64) * (u32*)(vga.memory + off + 4) << 32;
#if S3_LFB_TRACE
			printf("%s: LFB R size=%d @%llx => %08x (off=%llx)\n",
				devid_string, dsize, (unsigned long long)address,
				v, (unsigned long long)(address - lfb_base));
#endif
			return v;
		}
		default: // fall back byte-by-byte
		{
			u64 v = 0;
			for (int i = 0; i < dsize; ++i) v |= (u64)vga.memory[off + i] << (i * 8);
			return v;
		}
		}
	}

	// Everything else (all legacy VGA ports & A0000 region, option ROM, etc.)
	return CVGA::ReadMem(index, address, dsize);
}

void CS3Trio64::WriteMem(int index, u64 address, int dsize, u64 data)
{
	if (index == DEV_LFB_IDX && lfb_active && lfb_size)
	{
		const u64 off = address; // dispatcher already subtracts base
		// Trio64 "new MMIO" 128 KiB window at LFB+0x0100_0000 (CR53 bit3)
		//  - Lower half : PIX_TRANS FIFO (0xE2E8..0xE2EB)
		//  - Upper half : 8514/A registers mirrored at *E8 offsets
		if (s3_new_mmio_enabled()) {
			const u64 win_lo = 0x01000000ull;
			const u64 win_hi = 0x01020000ull;
			if (off >= win_lo && off < win_hi) {
				const u32 mmio_off = (u32)(off - win_lo);
				switch (dsize) {
				case 8:  mem_w(mmio_off, (u8)(data)); return;
				case 16: mem_w(mmio_off, (u8)(data)); mem_w(mmio_off + 1, (u8)(data >> 8)); return;
				case 32: mem_w(mmio_off, (u8)(data)); mem_w(mmio_off + 1, (u8)(data >> 8));
					mem_w(mmio_off + 2, (u8)(data >> 16)); mem_w(mmio_off + 3, (u8)(data >> 24)); return;
				default: FAILURE(InvalidArgument, "Unsupported dsize");
				}
			}
		}
		if (off >= lfb_size) return;

		// Write little-endian into linear VRAM
		switch (dsize)
		{
#if S3_LFB_TRACE
			printf("%s: LFB W size=%d @%llx <= %08x (off=%llx)\n",
				devid_string, dsize, (unsigned long long)address,
				data, (unsigned long long)(address));
#endif
		case 1:  vga.memory[off] = (u8)data; break;
		case 2:  *(u16*)(vga.memory + off) = (u16)data; break;
		case 4:  *(u32*)(vga.memory + off) = (u32)data; break;
		case 8: {
			*(u32*)(vga.memory + off) = (u32)(data & 0xffffffffu);
			*(u32*)(vga.memory + off + 4) = (u32)(data >> 32);
			break;
		}
		default:
			for (int i = 0; i < dsize; ++i)
				vga.memory[off + i] = (u8)(data >> (i * 8));
			break;
		}

		state.vga_mem_updated = 1;
		return;
	}

	CVGA::WriteMem(index, address, dsize, data);
}


static inline u64 alpha_pio_phys_from_linear_base(u32 base)
{
	// Typhoon: PIO vs system memory is selected by physical bit<43>.
	// We map PCI memory space windows by setting that bit.
	// 0x0000_0800_0000_0000 is the simplest way to assert <43>.
	return U64(0x0000080000000000) | (u64)base;
}

void CS3Trio64::lfb_recalc_and_map()
{
	// BAR-only implementation: rely on BAR0 decoding in PCI core.
	// Just refresh cached enable/base/size; no RegisterMemory calls here.
	lfb_recalc_and_cache();
}


u32 CS3Trio64::config_read_custom(int func, u32 address, int dsize, u32 cur)
{
	// For Trio64 we can just return the base value for now.
	// (TODO: synthesize bits in BAR0 reads from CR58..5A)
	return cur;
}

void CS3Trio64::config_write_custom(int func, u32 address, int dsize,
	u32 old_data, u32 new_data, u32 raw)
{
	// Watch COMMAND (0x04..0x05) and BAR0 (0x10..0x13)..
	const bool is_command = (address == 0x04 || address == 0x05);
	const bool is_bar0 = (address >= 0x10 && address <= 0x13);

	if (is_command || is_bar0) {
		lfb_recalc_and_cache();
		// Apply/unapply DEV_LFB mapping when MSE or BAR0 changes
		lfb_recalc_and_map();
		trace_lfb_if_changed(is_command ? "PCI COMMAND" : "PCI BAR0");
	}
}

void CS3Trio64::trace_lfb_if_changed(const char* reason) {
	const bool cr58_on = s3_lfb_enabled(m_crtc_map.read_byte(0x58));
	const uint32_t sz = s3_lfb_size_from_cr58(m_crtc_map.read_byte(0x58));
	const uint32_t base = pci_bar0;  // BAR-only base of truth
	const bool eff = pci_mem_enable && cr58_on && (base != 0);

	if (!lfb_trace_initialized ||
		eff != lfb_trace_enabled_prev ||
		base != lfb_trace_base_prev ||
		sz != lfb_trace_size_prev) {

		printf("%s: LFB %s - MSE=%d CR58=%02x base=%08x size=%x (reason=%s)\n",
			devid_string, eff ? "ACTIVE(BAR)" : "INACTIVE(BAR)",
			(int)pci_mem_enable, m_crtc_map.read_byte(0x58),
			base, sz, reason ? reason : "n/a");

		lfb_trace_initialized = true;
		lfb_trace_enabled_prev = eff;
		lfb_trace_base_prev = base;
		lfb_trace_size_prev = sz;
	}
	if (eff) lfb_trace_needs_first_access_note = true;
}

void CS3Trio64::lfb_recalc_and_cache()
{
	// COMMAND bit 1 (Memory Space Enable)
	const u32 cmd = config_read(0, 0x04, 2);            // 16-bit read is enough for COMMAND

	// BAR0: 32-bit memory BAR, mask off attribute bits
	const u32 bar0 = config_read(0, 0x10, 4) & 0xFFFFFFF0u;

	pci_mem_enable = (cmd & 0x0002) != 0; // saner, i think...

	pci_bar0 = bar0;

	// Honor CR58 enable/size while keeping BAR0 as the effective mapping base.
	const u8 cr58 = m_crtc_map.read_byte(0x58);
	lfb_base_ = bar0;                        // effective CPU-visible base = BAR0
	lfb_size_ = s3_lfb_size_from_cr58(cr58); // 64K/1M/2M/4M per Trio64
	lfb_enabled_ = pci_mem_enable && s3_lfb_enabled(cr58) && (bar0 != 0);
}



/**
 * Check if threads are still running.
 **/
void CS3Trio64::check_state()
{
	if (myThread && !myThread->isRunning())
		FAILURE(Thread, "S3 thread has died");
}

static u32  s3_magic1 = 0x53338811;
static u32  s3_magic2 = 0x88115333;

/**
 * Save state to a Virtual Machine State file.
 **/
int CS3Trio64::SaveState(FILE* f)
{
	long  ss = sizeof(state);
	int   res;

	if ((res = CPCIDevice::SaveState(f)))
		return res;

	fwrite(&s3_magic1, sizeof(u32), 1, f);
	fwrite(&ss, sizeof(long), 1, f);
	fwrite(&state, sizeof(state), 1, f);
	fwrite(&s3_magic2, sizeof(u32), 1, f);
	printf("%s: %d bytes saved.\n", devid_string, (int)ss);
	return 0;
}

/**
 * Restore state from a Virtual Machine State file.
 **/
int CS3Trio64::RestoreState(FILE* f)
{
	long    ss;
	u32     m1;
	u32     m2;
	int     res;
	size_t  r;

	if ((res = CPCIDevice::RestoreState(f)))
		return res;

	r = fread(&m1, sizeof(u32), 1, f);
	if (r != 1)
	{
		printf("%s: unexpected end of file!\n", devid_string);
		return -1;
	}

	if (m1 != s3_magic1)
	{
		printf("%s: MAGIC 1 does not match!\n", devid_string);
		return -1;
	}

	fread(&ss, sizeof(long), 1, f);
	if (r != 1)
	{
		printf("%s: unexpected end of file!\n", devid_string);
		return -1;
	}

	if (ss != sizeof(state))
	{
		printf("%s: STRUCT SIZE does not match!\n", devid_string);
		return -1;
	}

	fread(&state, sizeof(state), 1, f);
	if (r != 1)
	{
		printf("%s: unexpected end of file!\n", devid_string);
		return -1;
	}

	r = fread(&m2, sizeof(u32), 1, f);
	if (r != 1)
	{
		printf("%s: unexpected end of file!\n", devid_string);
		return -1;
	}

	if (m2 != s3_magic2)
	{
		printf("%s: MAGIC 1 does not match!\n", devid_string);
		return -1;
	}

	printf("%s: %d bytes restored.\n", devid_string, (int)ss);
	return 0;
}

/**
 * Read from Framebuffer.
 *
 * Not functional.
 **/
u32 CS3Trio64::mem_read(u32 address, int dsize)
{
	const u32 mv = s3_vram_mask();          // (memsize - 1)
	const u32 off = address & mv;

	if (address >= 0xA0000 && address <= 0xBFFFF) {
		uint32_t offset = address - 0xA0000;
		// For SVGA modes, use MAME banking path
		return mem_r(offset);
	}

	switch (dsize) {
	case 8:
		return (u32)vga.memory[off];
	case 16:
		return (u32)vga.memory[off] | ((u32)vga.memory[(off + 1) & mv] << 8);
	case 32:
		return (u32)vga.memory[off] | ((u32)vga.memory[(off + 1) & mv] << 8) | ((u32)vga.memory[(off + 2) & mv] << 16) | ((u32)vga.memory[(off + 3) & mv] << 24);
	default:
		FAILURE(InvalidArgument, "Unsupported dsize");
	}
}

/**
 * Write to Framebuffer.
 *
 * Not functional.
 **/
void CS3Trio64::mem_write(u32 address, int dsize, u32 data)
{
	const u32 mv = s3_vram_mask();         // (memsize - 1)
	const u32 off = address & mv;

	// Little-endian store into VRAM
	switch (dsize) {
	case 8:
		vga.memory[off] = (u8)data;
		break;
	case 16:
		vga.memory[off] = (u8)(data);
		vga.memory[(off + 1) & mv] = (u8)(data >> 8);
		break;
	case 32:
		vga.memory[off] = (u8)(data);
		vga.memory[(off + 1) & mv] = (u8)(data >> 8);
		vga.memory[(off + 2) & mv] = (u8)(data >> 16);
		vga.memory[(off + 3) & mv] = (u8)(data >> 24);
		break;
	default:
		FAILURE(InvalidArgument, "Unsupported dsize");
	}

	// Mark the affected tiles dirty so update() will serialize to the screen.
	state.vga_mem_updated = 1;
	if (vga.crtc.offset) {
		const unsigned nbytes = (dsize == 8) ? 1u : (dsize == 16 ? 2u : 4u);
		for (unsigned i = 0; i < nbytes; ++i) {
			const u32 p = (off + i) & mv;
			const u32 line = (vga.crtc.offset ? (p / vga.crtc.offset) : 0);
			const u32 col = (vga.crtc.offset ? (p % vga.crtc.offset) : 0);
			const unsigned xti = col / X_TILESIZE;
			const unsigned yti = line / Y_TILESIZE;
			SET_TILE_UPDATED(xti, yti, 1);
		}
	}
}


/**
 * Read from Legacy VGA Memory
 *
 * Calls vga_mem_read to read the data 1 byte at a time.
 **/
u32 CS3Trio64::legacy_read(u32 address, int dsize)
{
	// MMIO alias active ?
	if (s3.cr53 & 0x10) {
		const u32 base = s3_mmio_base_off(state);
		if (address >= base && address <= base + 0xFFFFu) {
			const u32 off = address - base;
			if (off < 0x8000) {
				// PIX_TRANS read — same as MAME
				switch (dsize) {
				case 8:  return (u32)AccelIORead(0xE2E8);
				case 16: return (u32)AccelIORead(0xE2E8) | ((u32)AccelIORead(0xE2E9) << 8);
				case 32: return (u32)AccelIORead(0xE2E8) | ((u32)AccelIORead(0xE2E9) << 8) |
					((u32)AccelIORead(0xE2EA) << 16) | ((u32)AccelIORead(0xE2EB) << 24);
				default: FAILURE(InvalidArgument, "Unsupported dsize");
				}
			}
			// Upper half: register reads via AccelIORead
			if (IsAccelPort(off)) {
				switch (dsize) {
				case 8:  return AccelIORead(off);
				case 16: return AccelIORead(off) | ((u32)AccelIORead(off + 1) << 8);
				case 32: return AccelIORead(off) | ((u32)AccelIORead(off + 1) << 8) |
					((u32)AccelIORead(off + 2) << 16) | ((u32)AccelIORead(off + 3) << 24);
				default: FAILURE(InvalidArgument, "Unsupported dsize");
				}
			}
		}
	}

	u32 data = 0;
	switch (dsize)
	{
	case 32:
		data |= (u32)mem_r(address + 3) << 24;
		data |= (u32)mem_r(address + 2) << 16;
		[[fallthrough]];
	case 16:
		data |= (u32)mem_r(address + 1) << 8;
		[[fallthrough]];
	case 8:
		data |= (u32)mem_r(address + 0);
		break;
	default:
		FAILURE(InvalidArgument, "Unsupported dsize");
	}

	return data;

}

/**
 * Write to Legacy VGA Memory
 *
 * Calls vga_mem_write to write the data 1 byte at a time.
 **/
 // --- Legacy VGA memory write with S3 MMIO alias support ---
void CS3Trio64::legacy_write(u32 address, int dsize, u32 data)
{
	switch (dsize) {
	case 8:
		mem_w(address, (u8)data);
		break;

	case 16:
		mem_w(address, (u8)data);
		mem_w(address + 1, (u8)(data >> 8));
		break;

	case 32:
		mem_w(address, (u8)data);
		mem_w(address + 1, (u8)(data >> 8));
		mem_w(address + 2, (u8)(data >> 16));
		mem_w(address + 3, (u8)(data >> 24));
		break;

	default:
		FAILURE(InvalidArgument, "Unsupported dsize");
	}
}

/**
 * Read from Option ROM
 */
u32 CS3Trio64::rom_read(u32 address, int dsize)
{
	u32   data = 0x00;
	u8* x = (u8*)option_rom;
	if (address <= rom_max)
	{
		x += address;
		switch (dsize)
		{
		case 8:   data = (u32)endian_8((*((u8*)x)) & 0xff); break;
		case 16:  data = (u32)endian_16((*((u16*)x)) & 0xffff); break;
		case 32:  data = (u32)endian_32((*((u32*)x)) & 0xffffffff); break;
		}

		//printf("S3 rom read: %" PRIx64 ", %d, %" PRIx64 "\n", address, dsize,data);
	}
	else
	{

		//printf("S3 (BAD) rom read: %" PRIx64 ", %d, %" PRIx64 "\n", address, dsize,data);
	}

	return data;
}

/**
 * Read from I/O Port
 */
u32 CS3Trio64::io_read(u32 address, int dsize)
{
	u32 data = 0;
	// Always intercept S3 8514/A-style ports. If the port block is not enabled
	// yet (CR40 == 0), hardware behaves benignly: reads return bus pull-ups,
	// writes are ignored.
	if (IsAccelPort(address)) {
		const bool ge_enabled = (m_crtc_map.read_byte(0x40) & 0x01) != 0;
		if (!ge_enabled) {
			switch (dsize) {
			case 8: return 0xFF;
			case 16: return 0xFFFF;
			case 32: return 0xFFFFFFFF;
			default: FAILURE(InvalidArgument, "Unsupported dsize");
			}
		}
		if ((m_crtc_map.read_byte(0x40) & 0x01) && IsAccelPort(address)) {
			switch (dsize) {
			case 8:  return AccelIORead(address);
			case 16: return (u32)AccelIORead(address + 0) |
				((u32)AccelIORead(address + 1) << 8);
			case 32: return (u32)AccelIORead(address + 0) |
				((u32)AccelIORead(address + 1) << 8) |
				((u32)AccelIORead(address + 2) << 16) |
				((u32)AccelIORead(address + 3) << 24);
			default: FAILURE(InvalidArgument, "Unsupported dsize");
			}
		}
	}

	if (dsize != 8)
		FAILURE(InvalidArgument, "Unsupported dsize");


	switch (address)
	{
	case 0x3c0:
		data = atc_address_r(0);
		break;

	case 0x3c1:
		data = atc_data_r(0);
		break;

	case 0x3c2:
		data = read_b_3c2();
		break;

	case 0x3c3:
		data = m_vga_subsys_enable ? 0x01 : 0x00;
		break;

	case 0x3c4:
		data = sequencer_address_r(0);
		break;

	case 0x3c5:
		if (vga.sequencer.index > 0x08 && vga.sequencer.data[0x08] != 0x06)
			data = vga.sequencer.data[vga.sequencer.index];
		else
			data = sequencer_data_r(0);
		break;

	case 0x3c6:
		data = ramdac_mask_r(0);
		break;

	case 0x3c7:
		data = ramdac_state_r(0);
		break;

	case 0x3c8:
		data = ramdac_write_index_r(0);
		break;

	case 0x3c9:
		data = ramdac_data_r(0);
		break;

	case 0x3ca:
		data = read_b_3ca();
		break;

	case 0x3cc:
		data = miscellaneous_output_r(0);
		break;

	case 0x3cf:
		data = gc_data_r(0);
		break;

	case 0x3b4:
	case 0x3d4:
		data = crtc_address_r(0);
		break;

	case 0x3b5:
	case 0x3d5:
		data = crtc_data_r(0);
		break;

	case 0x3ba:
	case 0x3da:
	{
		// Input Status Register 1 — ES40 wall-clock vblank (no CRT timing engine)
		using clock = std::chrono::steady_clock;
		static auto t0 = clock::now();
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			clock::now() - t0).count();

		const int frame_ms = 1000 / 70;  // ~70Hz
		const int vblank_ms = 1;

		data = 0;
		if ((ms % frame_ms) < vblank_ms)
			data |= 0x08 | 0x01;

		vga.attribute.state = 0;  // ATC flip-flop reset
		break;
	}

	case 0x3bb: /* Feature Control (mono) readback; mirror 3CA behavior */
		data = read_b_3ca();
		break;
	case 0x3db: /* Feature Control (color) readback; same treatment */
		data = read_b_3ca();
		break;

	default:
		printf("S3: Unhandled io port %x read\n", address);
	}

	return data;
}

/**
 * Write to I/O Port
 *
 * Calls io_write_b to write the data 1 byte at a time.
 */
void CS3Trio64::io_write(u32 address, int dsize, u32 data)
{
	// 8514/A-style accel window (S3 engine). Intercept first, and swallow writes
	// until CR40 enables the port block (to avoid falling through to VGA path).
	if (IsAccelPort(address)) {
		const bool ge_enabled = (m_crtc_map.read_byte(0x40) & 0x01) != 0;
		if (!ge_enabled) {
			// Ignore early probes safely (hardware no-op)
			return;
		}

		printf("ACCEL HIT @%04X dsize=%d data=%08X\n",
			(unsigned)address, dsize, (unsigned)data);
		switch (dsize) {
		case 8:
			AccelIOWrite(address, (u8)data);
			return;
		case 16:
			AccelIOWrite(address + 0, (u8)(data & 0xFF));
			AccelIOWrite(address + 1, (u8)((data >> 8) & 0xFF));
			return;
		case 32:
			AccelIOWrite(address + 0, (u8)((data >> 0) & 0xFF));
			AccelIOWrite(address + 1, (u8)((data >> 8) & 0xFF));
			AccelIOWrite(address + 2, (u8)((data >> 16) & 0xFF));
			AccelIOWrite(address + 3, (u8)((data >> 24) & 0xFF));
			return;
		default:
			FAILURE(InvalidArgument, "Unsupported dsize");
		}
	}

	//  printf("S3 io write: %" PRIx64 ", %d, %" PRIx64 "   \n", address+VGA_BASE, dsize, data);
	switch (dsize)
	{
	case 8:
		io_write_b(address, (u8)data);
		break;

	case 16:
		io_write_b(address, (u8)data);
		io_write_b(address + 1, (u8)(data >> 8));
		break;

	case 32:
		/*
		printf("S3 Weird Size io write: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, data);
		io_write_b(address, (u8)data);
		io_write_b(address + 1, (u8)(data >> 8));
		io_write_b(address + 1, (u8)(data >> 16));
		io_write_b(address + 1, (u8)(data >> 24));
		*/
		break;

	default:
#if DEBUG_VGA
		printf("S3 Weird Size io write: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, data);
#endif
		FAILURE(InvalidArgument, "Weird IO size");
	}
}

/**
 * Write one byte to a VGA I/O port.
 **/
void CS3Trio64::io_write_b(u32 address, u8 data)
{
	switch (address)
	{
	case 0x3c0:
	{
		bool was_index_phase = (vga.attribute.state == 0);
		// Snapshot previous video-enabled state BEFORE the MAME canonical write
		bool prev_ve = atc_video_enabled();
		atc_address_data_w(0, data);
		if (was_index_phase) {
			// Detect video enable/disable transitions from MAME canonical source
			bool new_ve = atc_video_enabled();
			if (!new_ve && prev_ve) {
				bx_gui->lock();
				bx_gui->clear_screen();
				bx_gui->unlock();
			}
			else if (new_ve && !prev_ve) {
				redraw_area(0, 0, old_iWidth, old_iHeight);
			}
		}
		break;
	}

	case 0x3c2:
		write_b_3c2(data);
		m_ioas = bool(BIT(data, 0));
		break;

	case 0x3c3:
		m_vga_subsys_enable = (data & 0x01) != 0;
		break;

	case 0x3c4:
		sequencer_address_w(0, data);
		break;

	case 0x3c5:
		// PLL lock gate: SR09+ requires SR08 == 0x06
		if (vga.sequencer.index > 0x08 && vga.sequencer.data[0x08] != 0x06)
			break;
		// SR1A/SR1B: not in sequencer_map, but in 86box
		if (vga.sequencer.index == 0x1a) { s3.sr1a = data; break; }
		if (vga.sequencer.index == 0x1b) { s3.sr1b = data; break; }
		sequencer_data_w(0, data);
		break;

	case 0x3c6:
		if (m_crtc_map.read_byte(0x33) & 0x10) break;
		ramdac_mask_w(0, data);
		break;

	case 0x3c7:
		ramdac_read_index_w(0, data);
		break;

	case 0x3c8:
		if (m_crtc_map.read_byte(0x33) & 0x10) break;
		ramdac_write_index_w(0, data);
		break;

	case 0x3c9:
		if (m_crtc_map.read_byte(0x33) & 0x10) break;
		ramdac_data_w(0, data);
		break;

	case 0x3ce:
		gc_address_w(0, data);
		break;

	case 0x3cf:
		gc_data_w(0, data);
		break;

	case 0x3ba:
	case 0x3da:
		feature_control_w(0, data);
		break;

	case 0x3b4:
	case 0x3d4:
		vga.crtc.index = data & 0x7f;
		break;

	case 0x3b5:
	case 0x3d5:
		crtc_data_w(0, data);
		break;

	case 0x3bb:
		return;

	default:
#if DEBUG_VGA
		printf("\nFAILURE ON BELOW LISTED PORT BINARY VALUE=" PRINTF_BINARY_PATTERN_INT8 " HEX VALUE=0x%02x\n", PRINTF_BYTE_TO_BINARY_INT8(data), data);
#endif
		FAILURE_1(NotImplemented, "Unhandled port %x write", address);
	}
}

/**
 * Write to the VGA Miscellaneous Output Register (0x3c2)
 *
 * \code
 * +-+-+-+-+---+-+-+
 * |7|6|5| |3 2|1|0|
 * +-+-+-+-+---+-+-+
 *  ^ ^ ^    ^  ^ ^
 *  | | |    |  | +- 0: I/OAS -- Input/Output Address Select: Selects the CRT
 *  | | |    |  |       controller addresses.
 *  | | |    |  |         0: Compatibility with monochrome adapter
 *  | | |    |  |            (0x3b4,0x3b5,0x03ba)
 *  | | |    |  |         1: Compatibility with color graphics adapter (CGA)
 *  | | |    |  |            (0x3d4,0x3d5,0x03da)
 *  | | |    |  +--- 1: RAM Enable: Controls access from the system:
 *  | | |    |            0: Disables access to the display buffer
 *  | | |    |            1: Enables access to the display buffer
 *  | | |    +--- 2..3: Clock Select: Controls the selection of the dot clocks
 *  | | |               used in driving the display timing:
 *  | | |                 00: Select 25 Mhz clock (320/640 pixel wide modes)
 *  | | |                 01: Select 28 Mhz clock (360/720 pixel wide modes)
 *  | | |                 10: Undefined (possible external clock)
 *  | | |                 11: Undefined (possible external clock)
 *  | | +----------- 5: Odd/Even Page Select: Selects the upper/lower 64K page
 *  | |                 of memory when the system is in an even/odd mode.
 *  | |                   0: Selects the low page.
 *  | |                   1: Selects the high page.
 *  | +------------- 6: Horizontal Sync Polarity
 *  |                     0: Positive sync pulse.
 *  |                     1: Negative sync pulse.
 *  +--------------- 7: Vertical Sync Polarity
 *                        0: Positive sync pulse.
 *                        1: Negative sync pulse.
 * \endcode
 **/
void CS3Trio64::write_b_3c2(u8 value)
{
	// ES40 extension: CR34 bit7 locks clock select bits
	if (m_crtc_map.read_byte(0x34) & 0x80) {
		// Preserve current clock_select (bits 3:2), take everything else from value
		value = (value & ~0x0C) | (vga.miscellaneous_output & 0x0C);
	}

	// MAME canonical store (flat byte)
	vga.miscellaneous_output = value;

#if DEBUG_VGA_NOISY
	printf("io write 3c2: misc_output = 0x%02x\n", value);
	printf("  color_emulation = %u, enable_ram = %u, clock_select = %u\n",
		(unsigned)state.misc_output.color_emulation,
		(unsigned)state.misc_output.enable_ram,
		(unsigned)state.misc_output.clock_select);
	printf("  select_high_bank = %u, horiz_sync_pol = %u, vert_sync_pol = %u\n",
		(unsigned)state.misc_output.select_high_bank,
		(unsigned)state.misc_output.horiz_sync_pol,
		(unsigned)state.misc_output.vert_sync_pol);
#endif
}

/**
 * Read from the VGA Input Status register (0x3c2)
 *
 * \code
 * +-----+-+-------+
 * |     |4|       |
 * +-----+-+-------+
 *        ^
 *        +--------- 4: Switch Sense:
 *                      Returns the status of the four sense switches as selected by the
 *                      Clock Select field of the Miscellaneous Output Register (See
 *                      CCirrus::write_b_3c2)
 * \endcode
 **/
u8 CS3Trio64::read_b_3c2()
{
	u8 res = 0x60; // is VGA (bits 5-6 set)

	// Sense bit readback: select which of 4 sense switches based on clock select
	// MAME: const u8 sense_bit = (3 - (vga.miscellaneous_output >> 2)) & 3;
	//        if(BIT(m_input_sense->read(), sense_bit)) res |= 0x10;
	const u8 sense_bit = (3 - ((vga.miscellaneous_output >> 2) & 3)) & 3;
	if (BIT(0x0F, sense_bit))  // all sense pins active
		res |= 0x10;

	res |= vga.crtc.irq_latch << 7;
	return res;
}

/**
 * Read from the VGA Enable register (0x3c3)
 *
 * (Not sure where this comes from; doesn't seem to be in the VGA specs.)
 **/
u8 CS3Trio64::read_b_3c3()
{
#if DEBUG_VGA_NOISY
	printf("VGA: 3c3 READ VGA ENABLE 0x%02x\n", vga_enabled());
#endif
	return vga_enabled();
}

u8 CS3Trio64::read_b_3ca()
{
	return 0;
}

u8 CS3Trio64::get_actl_palette_idx(u8 index)
{
	return atc_palette(index);
}

void CS3Trio64::redraw_area(unsigned x0, unsigned y0, unsigned width,
	unsigned height)
{
	unsigned  xti;

	unsigned  yti;

	unsigned  xt0;

	unsigned  xt1;

	unsigned  yt0;

	unsigned  yt1;

	unsigned  xmax;

	unsigned  ymax;

	if ((width == 0) || (height == 0))
	{
		return;
	}

	state.vga_mem_updated = 1;

	if (vga.gc.alpha_dis)
	{

		// graphics mode
		xmax = old_iWidth;
		ymax = old_iHeight;
		xt0 = x0 / X_TILESIZE;
		yt0 = y0 / Y_TILESIZE;
		if (x0 < xmax)
		{
			xt1 = (x0 + width - 1) / X_TILESIZE;
		}
		else
		{
			xt1 = (xmax - 1) / X_TILESIZE;
		}

		if (y0 < ymax)
		{
			yt1 = (y0 + height - 1) / Y_TILESIZE;
		}
		else
		{
			yt1 = (ymax - 1) / Y_TILESIZE;
		}

		for (yti = yt0; yti <= yt1; yti++)
		{
			for (xti = xt0; xti <= xt1; xti++)
			{
				SET_TILE_UPDATED(xti, yti, 1);
			}
		}
	}
	else
	{

		// text mode
		memset(state.text_snapshot, 0, sizeof(state.text_snapshot));
	}
}

void CS3Trio64::update(void)
{
	unsigned  iHeight;

	unsigned  iWidth;

#ifdef DEBUG_VGA_UPDATES
	// ============== diag log
	// every 600th call 
	s3_diag_update_counter++;
	const bool do_diag = (s3_diag_update_counter % 600 == 1);

	if (do_diag) {
		s3_diag_frame_counter++;
		printf("\n=== S3 UPDATE DIAGNOSTIC (frame %d) ===\n", s3_diag_frame_counter);
		printf("  vga_mem_updated=%d\n", state.vga_mem_updated);
		printf("  vga_enabled=%d, video_enabled=%d\n",
			vga_enabled(), atc_video_enabled());
		printf("  exsync_blank=%d, reset1=%d, reset2=%d\n",
			exsync_blank(), seq_reset1(), seq_reset2());
		printf("  vga.gc.alpha_dis=%d (CRITICAL: must be 1 for graphics)\n",
			vga.gc.alpha_dis);
		printf("  graphics_ctrl.memory_mapping=%d (1=A0000-AFFFF for gfx)\n",
			vga.gc.memory_map_sel);
		printf("  gc.shift_reg=%d\n", vga.gc.shift_reg);
		printf("  sequencer chain_four=%d\n", seq_chain_four());
		printf("  line_offset=%u (must be >= width for packed 8bpp)\n", vga.crtc.offset);

		// Calculate BytesPerPixel and dimensions
		int bpp = BytesPerPixel();
		unsigned h, w;
		determine_screen_dimensions(&h, &w);
		printf("  BytesPerPixel()=%d, dimensions: %ux%u\n", bpp, w, h);
		printf("  looks_packed_8 check: bpp==1=%d, line_offset>=%u = %d\n",
			(bpp == 1), w, (vga.crtc.offset >= w));

		// Display start address
		unsigned long start = latch_start_addr();
		printf("  latch_start_addr()=0x%08lx\n", start);
		printf("  CR0C=%02x CR0D=%02x CR69=%02x CR31=%02x\n",
			m_crtc_map.read_byte(0x0C), m_crtc_map.read_byte(0x0D),
			m_crtc_map.read_byte(0x69), m_crtc_map.read_byte(0x31));
		printf("  CR67=%02x (pixel format), CRTC13=%02x (offset low)\n",
			s3.ext_misc_ctrl_2, m_crtc_map.read_byte(0x13));
		printf("  CR51=%02x (offset high bits[5:4])\n", m_crtc_map.read_byte(0x51));
		printf("=====================================\n");

	}
	// ============== end diag log
#endif

	/* no screen update necessary */
	if (!m_vga_subsys_enable || !vga_enabled() || !atc_video_enabled() || !state.vga_mem_updated)
		return;

	const uint8_t cur_mode = pc_vga_choosevideomode();

	if (cur_mode == SCREEN_OFF)
	{
		state.vga_mem_updated = 0;
		return;
	}

	if (cur_mode == TEXT_MODE)
	{
		update_text_mode();
		return;
	}

	// All graphics modes: MAME rendering pipeline
	mame_render_to_gui();
}

void CS3Trio64::determine_screen_dimensions(unsigned* piHeight,
	unsigned* piWidth)
{
	int ai[0x20];
	int i;
	int h;
	int v;
	for (i = 0; i < 0x20; i++)
		ai[i] = m_crtc_map.read_byte(i);

	h = (ai[1] + 1) * 8;
	v = (ai[18] | ((ai[7] & 0x02) << 7) | ((ai[7] & 0x40) << 3)) + 1;
	// S3 CR5E extends V* with bit10 (0x400)
	if (m_crtc_map.read_byte(0x5E) & 0x02) v |= 0x400;

	if (vga.gc.shift256)
	{
		// was shift_reg == 2 mode 13h / 256-color byte mode
		// chain_four vs modeX 
		*piWidth = h;
		*piHeight = v;
	}
	else if (vga.gc.shift_reg)
	{
		// was shift_reg == 1 CGA 4-color interleave
		if (x_dotclockdiv2())
			h <<= 1;
		*piWidth = h;
		*piHeight = v;
	}
	else
	{
		// was shift_reg == 0 standard VGA planar / EGA
		*piWidth = 640;
		*piHeight = 480;
		if (m_crtc_map.read_byte(0x06) == 0xBF)
		{
			if (m_crtc_map.read_byte(0x17) == 0xA3 && m_crtc_map.read_byte(0x14) == 0x40
				&& m_crtc_map.read_byte(0x09) == 0x41)
			{
				*piWidth = 320;
				*piHeight = 240;
			}
			else
			{
				if (x_dotclockdiv2())
					h <<= 1;
				*piWidth = h;
				*piHeight = v;
			}
		}
		else if ((h >= 640) && (v >= 480))
		{
			*piWidth = h;
			*piHeight = v;
		}
	}
}

inline uint32_t CS3Trio64::s3_vram_mask() const
{
	const uint32_t sz = vga.svga_intf.vram_size ? vga.svga_intf.vram_size : (8u * 1024u * 1024u);
	return sz - 1u;
}

void CS3Trio64::palette_update()
{
	CVGA::palette_update();

	for (int i = 0; i < 256; i++) {
		// pal6bit: expand 6-bit color to 8-bit
		u8 r = (vga.dac.color[3 * (i & vga.dac.mask) + 0] & 0x3f);
		u8 g = (vga.dac.color[3 * (i & vga.dac.mask) + 1] & 0x3f);
		u8 b = (vga.dac.color[3 * (i & vga.dac.mask) + 2] & 0x3f);
		// Expand 6-bit to 8-bit: (val << 2) | (val >> 4)
		r = (r << 2) | (r >> 4);
		g = (g << 2) | (g >> 4);
		b = (b << 2) | (b >> 4);
		bx_gui->palette_change((unsigned)i, (unsigned)r, (unsigned)g, (unsigned)b);
	}
}

uint8_t CS3Trio64::get_video_depth()
{
	switch (pc_vga_choosevideomode())
	{
	case VGA_MODE:
	case RGB8_MODE:    return 8;
	case RGB15_MODE:
	case RGB16_MODE:   return 16;
	case RGB24_MODE:
	case RGB32_MODE:   return 32;
	default:           return 0;
	}
}

void CS3Trio64::mem_linear_w(uint32_t offset, uint8_t data)
{
	CVGA::mem_linear_w(offset, data);
	state.vga_mem_updated = 1;
}


// Draws the 64x64 S3 hardware graphics cursor over a pre-rendered framebuffer.
// Supports Windows mode and X11 mode (CR55 bit 4), and all color depths.
void CS3Trio64::s3_draw_hardware_cursor(
	uint32_t* pixels, int pitch_px,
	int clip_width, int clip_height,
	uint8_t cur_mode)
{
	// Only draw if cursor is enabled
	if (!(s3.cursor_mode & 0x01))
		return;

	// Cursor only works in VGA or SVGA modes
	if (cur_mode == SCREEN_OFF || cur_mode == TEXT_MODE ||
		cur_mode == MONO_MODE || cur_mode == CGA_MODE ||
		cur_mode == EGA_MODE)
		return;

	uint16_t cx = s3.cursor_x & 0x07FF;
	uint16_t cy = s3.cursor_y & 0x07FF;

	// Start address is in units of 1024 bytes
	uint32_t src = (uint32_t)s3.cursor_start_addr * 1024;

	// Decode foreground/background colors 
	uint32_t bg_col, fg_col;

	auto decode_rgb16 = [](const uint8_t* raw) -> uint32_t {
		uint32_t datax = raw[0] | (raw[1] << 8);
		int r = (datax & 0xF800) >> 11;
		int g = (datax & 0x07E0) >> 5;
		int b = (datax & 0x001F) >> 0;
		r = (r << 3) | (r & 0x7);
		g = (g << 2) | (g & 0x3);
		b = (b << 3) | (b & 0x7);
		return 0xFF000000u | (r << 16) | (g << 8) | b;
		};

	auto decode_rgb24 = [](const uint8_t* raw) -> uint32_t {
		uint32_t datax = raw[0] | (raw[1] << 8) | (raw[2] << 16);
		int r = (datax & 0xFF0000) >> 16;
		int g = (datax & 0x00FF00) >> 8;
		int b = (datax & 0x0000FF) >> 0;
		return 0xFF000000u | (r << 16) | (g << 8) | b;
		};

	switch (cur_mode)
	{
	case RGB15_MODE:
	case RGB16_MODE:
		bg_col = decode_rgb16(s3.cursor_bg);
		fg_col = decode_rgb16(s3.cursor_fg);
		break;

	case RGB24_MODE:
		bg_col = decode_rgb24(s3.cursor_bg);
		fg_col = decode_rgb24(s3.cursor_fg);
		break;

	case RGB8_MODE:
	default:
		bg_col = pen(s3.cursor_bg[0]);
		fg_col = pen(s3.cursor_fg[0]);
		break;
	}

	// Draw the 64x64 cursor bitmap 
	// Cursor data: 64 rows, each row = 16 bytes (4 words of 16 bits A + 16 bits B)
	// Pattern origin offset from CR4E/CR4F
	const int pat_x = s3.cursor_pattern_x & 0x3F;
	const int pat_y = s3.cursor_pattern_y & 0x3F;

	for (int y = 0; y < 64; y++)
	{
		int screen_y = cy + y - pat_y;
		if (screen_y < 0 || screen_y >= clip_height)
		{
			// Still need to advance src through the row's cursor data
			// Each row: 4 groups of 4 bytes = 16 bytes
			// But we advance per-group below, so just skip
			// We need 4 groups * 4 bytes = 16 bytes per row
			src += 16; // skip this row's data
			continue;
		}

		uint32_t* dst = pixels + (screen_y * pitch_px);
		uint32_t row_src = src;

		for (int x = 0; x < 64; x++)
		{
			// Each 16-pixel group uses 4 bytes: 2 bytes for A-plane, 2 for B-plane
			// Bit extraction from MAME:
			uint16_t bita = (vga.memory[(row_src + 1) % vga.svga_intf.vram_size] |
				((vga.memory[(row_src + 0) % vga.svga_intf.vram_size]) << 8))
				>> (15 - (x % 16));
			uint16_t bitb = (vga.memory[(row_src + 3) % vga.svga_intf.vram_size] |
				((vga.memory[(row_src + 2) % vga.svga_intf.vram_size]) << 8))
				>> (15 - (x % 16));
			uint8_t val = ((bita & 0x01) << 1) | (bitb & 0x01);

			int screen_x = cx + x - pat_x;
			if (screen_x >= 0 && screen_x < clip_width)
			{
				if (s3.extended_dac_ctrl & 0x10)
				{
					// X11 mode
					switch (val)
					{
					case 0x00: /* no change - transparent */   break;
					case 0x01: /* no change - transparent */   break;
					case 0x02: dst[screen_x] = bg_col;        break;
					case 0x03: dst[screen_x] = fg_col;        break;
					}
				}
				else
				{
					// Windows mode
					switch (val)
					{
					case 0x00: dst[screen_x] = bg_col;            break;
					case 0x01: dst[screen_x] = fg_col;            break;
					case 0x02: /* screen data - no change */       break;
					case 0x03: dst[screen_x] = ~(dst[screen_x]);  break; // invert
					}
				}
			}

			// Advance source pointer every 16 pixels
			if (x % 16 == 15)
				row_src += 4;
		}
		src = row_src; // advance to next row
	}
}

void CS3Trio64::mame_render_to_gui()
{
	unsigned iWidth, iHeight;
	determine_screen_dimensions(&iHeight, &iWidth);

	if (iWidth == 0 || iHeight == 0)
		return;

	// Update screen shim's visible area
	screen().set_visible_area(iWidth, iHeight);

	// Ensure bitmap is large enough
	m_render_bitmap.allocate(iWidth, iHeight);

	// Render via MAME's screen_update pipeline
	rectangle clip = m_render_bitmap.cliprect();
	screen_update(m_render_bitmap, clip);

	// Tick the frame counter (for cursor blink)
	screen().tick_frame();

	// MAME always produces ARGB32 — tell SDL we're in 32bpp mode.
	if (state.last_bpp != 32 || iWidth != old_iWidth || iHeight != old_iHeight)
	{
		bx_gui->dimension_update(iWidth, iHeight, 0, 0, 32);
		old_iWidth = iWidth;
		old_iHeight = iHeight;
		state.last_bpp = 32;
	}

	// bitmap_rgb32 stores uint32_t pixels as 0xAARRGGBB in host word order.
	// The X11/Win32 GUI at 32bpp also interprets tile data as native-order
	// uint32_t pixels, so we can memcpy directly — no byte swizzling needed.
	//
	// state.tile is u8[X_TILESIZE * Y_TILESIZE * 4] — exactly sized for 32bpp.
	for (unsigned y = 0; y < iHeight; y += Y_TILESIZE)
	{
		for (unsigned x = 0; x < iWidth; x += X_TILESIZE)
		{
			const unsigned tw = std::min((unsigned)X_TILESIZE, iWidth - x);
			const unsigned th = std::min((unsigned)Y_TILESIZE, iHeight - y);

			u8* dst = state.tile;
			for (unsigned row = 0; row < th; row++)
			{
				const uint32_t* src_row = m_render_bitmap.rowptr(y + row) + x;
				memcpy(dst, src_row, tw * sizeof(uint32_t));
				// Pad partial tile columns with black
				if (tw < (unsigned)X_TILESIZE)
					memset(dst + tw * 4, 0, ((unsigned)X_TILESIZE - tw) * 4);
				dst += X_TILESIZE * 4;
			}
			// Pad partial tile rows with black
			for (unsigned row = th; row < (unsigned)Y_TILESIZE; row++)
			{
				memset(dst, 0, X_TILESIZE * 4);
				dst += X_TILESIZE * 4;
			}

			bx_gui->graphics_tile_update(state.tile, x, y);
		}
	}

	state.vga_mem_updated = 0;
}

void CS3Trio64::update_text_mode()
{
	unsigned      iHeight, iWidth;
	unsigned long start_address;
	unsigned long cursor_address;
	unsigned long cursor_x;
	unsigned long cursor_y;
	bx_vga_tminfo_t tm_info;
	unsigned      VDE;
	unsigned      MSL;
	unsigned      cols;
	unsigned      rows;
	unsigned      cWidth;

	tm_info.start_address = 2 * ((m_crtc_map.read_byte(0x0C) << 8)
		+ m_crtc_map.read_byte(0x0D));
	tm_info.cs_start = m_crtc_map.read_byte(0x0a) & 0x3f;
	tm_info.cs_end = m_crtc_map.read_byte(0x0b) & 0x1f;
	tm_info.line_offset = m_crtc_map.read_byte(0x13) << 2;
	tm_info.line_compare = vga.crtc.line_compare;
	tm_info.h_panning = vga.attribute.pel_shift & 0x0f;
	tm_info.v_panning = m_crtc_map.read_byte(0x08) & 0x1f;
	tm_info.line_graphics = !!(vga.attribute.data[0x10] & 0x04);
	tm_info.split_hpanning = !!(vga.attribute.data[0x10] & 0x20);

	if ((vga.sequencer.data[1] & 0x01) == 0)
	{
		if (tm_info.h_panning >= 8)
			tm_info.h_panning = 0;
		else
			tm_info.h_panning++;
	}
	else
	{
		tm_info.h_panning &= 0x07;
	}

	VDE = vga.crtc.vert_disp_end;
	MSL = vga.crtc.maximum_scan_line - 1;
	if (MSL == 0)
		return;

	cols = vga.crtc.horz_disp_end + 1;
	if ((MSL == 1) && (VDE == 399))
		MSL = 3;  // emulated CGA 160x100x16

	rows = (VDE + 1) / (MSL + 1);
	if (rows > BX_MAX_TEXT_LINES)
		return;

	// S3 Trio64: CR33 bit5 forces 8-pixel char width
	if (m_crtc_map.read_byte(0x33) & 0x20)
		cWidth = 8;
	else
		cWidth = ((vga.sequencer.data[1] & 0x01) == 1) ? 8 : 9;

	iWidth = cWidth * cols;
	iHeight = VDE + 1;

	bx_gui->dimension_update(iWidth, iHeight, MSL + 1, cWidth);

	if ((iWidth != old_iWidth) || (iHeight != old_iHeight)
		|| (MSL != old_MSL) || (state.last_bpp > 8))
	{
		old_iWidth = iWidth;
		old_iHeight = iHeight;
		old_MSL = MSL;
		state.last_bpp = 8;
	}

	start_address = 2 * ((m_crtc_map.read_byte(0x0C) << 8)
		+ m_crtc_map.read_byte(0x0D));
	cursor_address = 2 * ((m_crtc_map.read_byte(0x0e) << 8)
		+ m_crtc_map.read_byte(0x0F));

	if (cursor_address < start_address)
	{
		cursor_x = 0xffff;
		cursor_y = 0xffff;
	}
	else
	{
		cursor_x = ((cursor_address - start_address) / 2) % (iWidth / cWidth);
		cursor_y = ((cursor_address - start_address) / 2) / (iWidth / cWidth);
	}

	{ // HACK HACK HACK: We're not receiving the VBIOS fonts properly, so let's do that..... 
	  // render path uses its own charmap, not the VGA memory
		uint8_t ch = vga.memory[0];
		for (int ch = 0; ch < 256; ch++)
			for (int i = 0; i < 32; i++)
				bx_gui->set_text_charbyte(ch * 32 + i, vga.memory[0x20000 + ch * 32 + i]);
	}

	bx_gui->text_update(state.text_snapshot, &vga.memory[start_address],
		cursor_x, cursor_y, tm_info, rows);

	memcpy(state.text_snapshot, &vga.memory[start_address], 2 * cols * rows);
	state.vga_mem_updated = 0;
}
