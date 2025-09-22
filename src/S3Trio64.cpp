/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project & Others
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
#include <chrono>
#include "gui/gui.h"

  // turn on or off debug output
#define DEBUG_VGA 1
#define DEBUG_VGA_NOISY 0
#define DEBUG_PCI 0
#define S3_LFB_TRACE 1

static unsigned old_iHeight = 0, old_iWidth = 0, old_MSL = 0;

static const u8 ccdat[16][4] = {
  {0x00, 0x00, 0x00, 0x00},
  {0xff, 0x00, 0x00, 0x00},
  {0x00, 0xff, 0x00, 0x00},
  {0xff, 0xff, 0x00, 0x00},
  {0x00, 0x00, 0xff, 0x00},
  {0xff, 0x00, 0xff, 0x00},
  {0x00, 0xff, 0xff, 0x00},
  {0xff, 0xff, 0xff, 0x00},
  {0x00, 0x00, 0x00, 0xff},
  {0xff, 0x00, 0x00, 0xff},
  {0x00, 0xff, 0x00, 0xff},
  {0xff, 0xff, 0x00, 0xff},
  {0x00, 0x00, 0xff, 0xff},
  {0xff, 0x00, 0xff, 0xff},
  {0x00, 0xff, 0xff, 0xff},
  {0xff, 0xff, 0xff, 0xff},
};

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
		for (;;)
		{
			// Terminate thread if StopThread is set to true
			if (StopThread)
				return;
			// Handle GUI events (100 times per second)
			for (int i = 0; i < 10; i++)
			{
				bx_gui->lock();
				bx_gui->handle_events();
				bx_gui->unlock();
				CThread::sleep(10);
			}
			//Update the screen (10 times per second)
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

// proper CR69 handling
inline uint32_t CS3Trio64::compose_display_start() const {
	uint32_t sa = (uint32_t(state.CRTC.reg[0x0C]) << 8) | uint32_t(state.CRTC.reg[0x0D]);
	const uint8_t ext = state.CRTC.reg[0x69] & 0x0F;
	if (ext) {
		sa |= uint32_t(ext) << 16;  // CR69 overrides when non-zero
	}
	else {
		sa |= uint32_t(state.CRTC.reg[0x31] & 0x30) << 12; // old bits 17:16
		sa |= uint32_t(state.CRTC.reg[0x51] & 0x03) << 18; // old bits 19:18
	}
	return sa;
}

inline bool CS3Trio64::s3_mmio_enabled(const SS3_state& s) {
	return (s.CRTC.reg[0x53] & 0x10) != 0; // EXT_MEM_CNTL1 bit 4
}

inline uint8_t CS3Trio64::current_char_width_px() const {
	// If special blanking (CR33 bit5) is set, S3 forces 8-dot chars.
	if (state.CRTC.reg[0x33] & 0x20) return 8;
	// Otherwise, use Sequencer reg1 bit0 like the rest of our text logic.
	return (state.sequencer.reg1 & 0x01) ? 8 : 9;
}

void CS3Trio64::recompute_data_transfer_position()
{
	// Snapshot old for change-sensitive tracing
	const bool     old_en = state.dtp_enabled;
	const uint8_t  old_raw = state.dtp_raw;
	const uint16_t old_chars = state.dtp_hpos_chars;
	const uint16_t old_px = state.dtp_hpos_pixels;

	// Enabled via CR34 bit4 (ENB DTPC)
	state.dtp_enabled = (state.CRTC.reg[0x34] & 0x10) != 0;
	state.dtp_raw = state.CRTC.reg[0x3B];

	// Express CR3B (character clocks) in both chars and pixels for future fetch logic
	state.dtp_hpos_chars = uint16_t(state.dtp_raw);
	const uint8_t cwidth = current_char_width_px();  // 8 or 9 (forced 8 under special blanking)
	state.dtp_hpos_pixels = uint16_t(state.dtp_hpos_chars) * uint16_t(cwidth);

	// Trace only when something materially changes
	if (old_en != state.dtp_enabled ||
		old_raw != state.dtp_raw ||
		old_chars != state.dtp_hpos_chars ||
		old_px != state.dtp_hpos_pixels) {
		DTP_TRACE("S3 DTP: %s CR3B=0x%02X -> %u chars / %u px (charw=%u)\n",
			state.dtp_enabled ? "EN" : "DIS",
			state.dtp_raw, state.dtp_hpos_chars, state.dtp_hpos_pixels, cwidth);
	}
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
	lfb_active = s3_lfb_enabled(state.CRTC.reg[0x58]);
	lfb_size = s3_lfb_size_from_cr58(state.CRTC.reg[0x58]);
	lfb_base = (u32)state.CRTC.reg[0x59] << 24;
#if S3_LFB_TRACE
	printf("LFB (BAR-only): CR58=%02x base=%08x size=%x active=%d\n",
		state.CRTC.reg[0x58], lfb_base, lfb_size, lfb_active);
#endif
}

void CS3Trio64::on_crtc_linear_regs_changed()
{
	const u8 cr58 = state.CRTC.reg[0x58];
	const u8 cr59 = state.CRTC.reg[0x59];

	// Enable via CR58.ENB_LA (bit 4)
	lfb_active = s3_lfb_enabled(cr58);

	// Aperture size selector (bits 6:4). Use common S3 encodings: 1..128MB.
	// For Trio64, 64MB apertures are typical; clamp to VRAM for now.
	static const u32 sizes[8] = {
	  1u << 20, 2u << 20, 4u << 20, 8u << 20, 16u << 20, 32u << 20, 64u << 20, 128u << 20
	};
	const u32 sel = (cr58 >> 4) & 7u;
	u32 size = sizes[sel];
	if (size > state.memsize) size = state.memsize;
	lfb_size = size;

	// Base: CR59 holds bits [31:24]
	lfb_base = (u32)cr59 << 24;

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

	// Initialize all state variables to 0
	memset((void*)&state, 0, sizeof(state));

	// Register VGA I/O ports at 3b4, 3b5, 3ba, 3c0..cf, 3d4, 3d5, 3da
	add_legacy_io(1, 0x3b4, 2);
	add_legacy_io(3, 0x3ba, 2);
	add_legacy_io(2, 0x3c0, 16);
	add_legacy_io(8, 0x3d4, 2);
	add_legacy_io(9, 0x3da, 1);

	// 8514/A-style S3 accel ports (byte-wide)
#if ES40_S3_ACCEL_ENABLE
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
#endif


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
	lfb_size = state.memsize;                        // clamp to VRAM for now

	// Ensure we have a stable starting point (disabled = no mapping)
	// this was a bad idea. don't touch till CR58... .
	//update_linear_mapping();


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

	state.vga_enabled = 1;
	state.misc_output.color_emulation = 1;
	state.misc_output.enable_ram = 1;
	state.misc_output.horiz_sync_pol = 1;
	state.misc_output.vert_sync_pol = 1;

	state.attribute_ctrl.mode_ctrl.enable_line_graphics = 1;

	state.line_offset = 80;
	state.line_compare = 1023;
	state.vertical_display_end = 399;

	state.attribute_ctrl.video_enabled = 1;
	state.attribute_ctrl.color_plane_enable = 0x0f;

	state.pel.dac_state = 0x01;
	state.pel.mask = 0xff;

	state.graphics_ctrl.memory_mapping = 2; // monochrome text mode

	state.sequencer.reset1 = 1;
	state.sequencer.reset2 = 1;
	state.sequencer.extended_mem = 1;       // display mem greater than 64K
	state.sequencer.odd_even = 1;           // use sequential addressing mode
	state.sequencer.sr15 = 0;               // CLKSYN Control 2 Register (SR15) 00H poweron
	state.sequencer.srA = 0;                // External Bus Control Register (SRA) 00H poweron
	state.sequencer.srB = 0;                // Miscellaneous Extended Sequencer Register 00H poweron
	state.sequencer.srD = 0;                // Extended Sequencer Register (EX_SR_D) (SRD) 00H poweron
	state.sequencer.sr9 = 0;                // Extended Sequencer Register 9 (SR9) poweron 00H

	// Use VIDEO_RAM_SIZE (in bits) to size VRAM. With 22 this is 4 MB.
	state.memsize = 1u << VIDEO_RAM_SIZE;
	state.memory = new u8[state.memsize];
	memset(state.memory, 0, state.memsize);

	state.last_bpp = 8;

	state.CRTC.reg[0x09] = 16; // Maximum Scan Line Register (MAX_S_LN) (CR9) - poweron undefined. default scan lines per char row.
	state.CRTC.reg[0x30] = 0xe1;   // Chip ID/REV register CR30, dosbox-x implementation returns 0x00 for our use case. poweron default is E1H however.
	state.CRTC.reg[0x32] = 0x00; // Locked by default
	state.CRTC.reg[0x33] = 0x00; // CR33 (Backward Compatibility 2) — default 00h (no locks).
	state.CRTC.reg[0x36] = s3_cr36_from_memsize(state.memsize, true); // Configuration 2 Register (CONFG_REG1) (CR36) - bootstrap config
	state.CRTC.reg[0x37] = 0xE5; // Configuration 2 Register (CONFG_REG2) (CR37)  - bootstrap read, sane value from 86box
	state.CRTC.reg[0x3B] = 0x00;  // CR3B: Data Transfer Position (DTPC)
	state.CRTC.reg[0x3C] = 0x00; // CR3C: IL_RTSTART defaulting
	state.CRTC.reg[0x40] = 0x30; // System Configuration Register, power on default 30h
	state.CRTC.reg[0x42] = 0x00; // Mode Control 2 - can set interlace vs non
	printf("%u", s3_cr36_from_memsize(state.memsize, true));
	state.graphics_ctrl.memory_mapping = 3; // color text mode
	state.vga_mem_updated = 1;

	// CR56: External Sync Control 1 (EX_SYNC_1) power-on default 00h
	state.CRTC.reg[0x56] = 0x00;
	state.exsync1 = 0x00;
	state.exsync_remote = false;
	state.hsync_drive = true;   // don't blank until the guest writes CR56
	state.vsync_drive = true;
	state.exsync_vreset_only = false;
	state.exsync_preset_odd = false;
	state.exsync_blank = false;

	// CR57: EX_SYNC_2 (VSYNC reset adjust), power-on default 00h
	state.CRTC.reg[0x57] = 0x00;
	state.exsync2_raw = 0x00;
	state.exsync2_delay_lines = 0x00;

	// EX_SYNC_3 (CR63)
	state.CRTC.reg[0x63] = 0x00;
	state.exsync3_raw = 0x00;
	state.exsync3_hsreset_chars = 0;
	state.exsync3_charclk_delay = 0;
	state.exsync3_active = false;

	// CNFG-REG-3 (CR68) poweron strap; datasheet says power-on samples PD[23:16].
	// 00h per 86Box-compatible. If needed, set CRTC.reg[0x68] before 
	// recompute_config3() for different.
	state.CRTC.reg[0x68] = 0x00;
	state.cr68_raw = 0x00;
	state.cr68_casoe_we = 0;
	state.cr68_ras_low = false;
	state.cr68_ras_pcg = false;
	state.cr68_mon_inf = 0;
	state.cr68_up_add = false;
	recompute_config3();

	// CR65: Extended Miscellaneous Control Register (EXT-MISC-CTL) (CR65)
	state.CRTC.reg[0x65] = 0x00;
	state.cr65_raw = 0x00;
	state.cr65_enb_3c3 = true;  // we always expose 3C3h; this flag is just informative
	state.cr65_blank_dly = 0;

	// DTP derived state (disabled until CR34 bit4 is set)
	state.dtp_enabled = false;
	state.dtp_raw = 0;
	state.dtp_hpos_chars = 0;
	state.dtp_hpos_pixels = 0;

	// ILRT derived state
	state.ilrt_enabled = false;
	state.ilrt_raw = 0;
	state.ilrt_chars = 0;
	state.ilrt_pixels = 0;


	state.accel.enabled = (ES40_S3_ACCEL_ENABLE != 0);
	state.accel.busy = false;
	state.accel.subsys_cntl = 0;
	state.accel.subsys_stat = 0xFFFF; // bus-float lookalike when disabled

	recompute_line_offset(); // do it initially, just for sanity sake

	myThread = 0;

	printf("%s: $Id$\n",
		devid_string);
}


static inline uint32_t s3_lfb_base_from_regs(const uint8_t* cr) {
	// CR59 high, CR5A low, reported as (la_window << 16)
	const uint16_t la_window = (uint16_t(cr[0x59]) << 8) | uint16_t(cr[0x5A]);
	return uint32_t(la_window) << 16;
}

void CS3Trio64::recompute_scanline_layout()
{
	const uint8_t cr5d = state.CRTC.reg[0x5d];

	auto xbit = [&](int b) -> uint16_t { return (cr5d >> b) & 1u; };

	// ---- Base (8-bit) fields extended by CR5D "bit 8" carriers ----
	// CR5D bit0 -> HTotal bit8
	state.h_total = uint16_t(state.CRTC.reg[0x00]) | (xbit(0) << 8);

	// CR5D bit1 -> HDisplayEnd bit8
	state.h_display_end = uint16_t(state.CRTC.reg[0x01]) | (xbit(1) << 8);

	// CR5D bit2 -> HBlankStart bit8
	state.h_blank_start = uint16_t(state.CRTC.reg[0x02]) | (xbit(2) << 8);

	// CR5D bit4 -> HSyncStart bit8
	state.h_sync_start = uint16_t(state.CRTC.reg[0x04]) | (xbit(4) << 8);

	// ---- End fields: take your VGA base value, then apply S3 extensions ----------
	uint16_t hb_end_base = uint16_t(state.CRTC.reg[0x03] & 0x1F);  // VGA: End Horizontal Blanking (low 5)
	uint16_t hs_end_base = uint16_t(state.CRTC.reg[0x05] & 0x1F);  // VGA: Horizontal Sync End   (low 5)

	// CR5D bit3 adds +64 to End Horizontal Blanking
	state.h_blank_end = uint16_t(hb_end_base + (xbit(3) ? 64 : 0));

	// CR5D bit5 adds +32 to Horizontal Sync End
	state.h_sync_end = uint16_t(hs_end_base + (xbit(5) ? 32 : 0));

	// --- S3 special blanking (CR33 bit5) ---
		// In this mode, HBLANK runs from HDisplayEnd to HTotal-1.
	if (state.CRTC.reg[0x33] & 0x20) {
		state.h_blank_start = state.h_display_end;
		state.h_blank_end = (state.h_total - 1) & 0x1FF;
	}

	// shorten to 9-bit because CR5D only gives one extra bit 
	state.h_total &= 0x1FF;
	state.h_display_end &= 0x1FF;
	state.h_blank_start &= 0x1FF;
	state.h_blank_end &= 0x1FF;
	state.h_sync_start &= 0x1FF;
	state.h_sync_end &= 0x1FF;
}

void CS3Trio64::recompute_line_offset()
{
	// CR13  = CRTC Offset (in character clocks) -> base in bytes = CR13 * 2
	// CR43b2= Offset bit 8 (EXT_MODE[2])
	// CR51b5:4 = Offset bits 9:8 (EXT_SYSCTL2[5:4]) overlaps with CR43b2 for bit 8
	// CR14b6 / CR17b6 select addressing unit:
	//   if CR14[6]=1 -> DWORD addressing (x4)
	//   else if CR17[6]=0 -> WORD addressing (x2)
	//   else -> BYTE addressing (x1)
	uint32_t off = (uint32_t(state.CRTC.reg[0x13]) << 1);          // *2
	off |= (uint32_t(state.CRTC.reg[0x43] & 0x04) << 6);           // bit8 via CR43 (0x04 << 6) = 0x100
	off |= (uint32_t(state.CRTC.reg[0x51] & 0x30) << 4);           // bits9:8 via CR51 (0x30 << 4) = 0x300
	if (state.CRTC.reg[0x14] & 0x40) {
		off <<= 2;          // *4
	}
	else if ((state.CRTC.reg[0x17] & 0x40) == 0) {
		off <<= 1;        // *2
	}
	state.line_offset = (uint16_t)off;
}

void CS3Trio64::recompute_interlace_retrace_start()
{
	// Snapshot old values for change-sensitive tracing
	const bool     old_en = state.ilrt_enabled;
	const uint8_t  old_rw = state.ilrt_raw;
	const uint16_t old_ch = state.ilrt_chars;
	const uint16_t old_px = state.ilrt_pixels;

	// Enabled when CR42 bit5 is set
	state.ilrt_enabled = (state.CRTC.reg[0x42] & 0x20) != 0;
	state.ilrt_raw = state.CRTC.reg[0x3C];     // raw chars per datasheet

	// Convert to chars & pixels using current character width
	state.ilrt_chars = uint16_t(state.ilrt_raw);
	const uint8_t cwp = current_char_width_px();  // 8 or 9
	state.ilrt_pixels = uint16_t(state.ilrt_chars) * uint16_t(cwp);

	if (old_en != state.ilrt_enabled || old_rw != state.ilrt_raw ||
		old_ch != state.ilrt_chars || old_px != state.ilrt_pixels) {
		ILRT_TRACE("S3 ILRT: %s CR3C=0x%02X -> %u chars / %u px (charw=%u)\n",
			state.ilrt_enabled ? "EN" : "DIS",
			state.ilrt_raw, state.ilrt_chars, state.ilrt_pixels, cwp);
	}
}

void CS3Trio64::recompute_external_sync_1()
{
	const uint8_t r = state.CRTC.reg[0x56] & 0x1F; // bits 7-5 reserved
	state.exsync1 = r;
	state.exsync_remote = (r & 0x01) != 0;
	state.hsync_drive = (r & 0x02) != 0;  // 1 = driver enabled, 0 = tri-stated
	state.vsync_drive = (r & 0x04) != 0;  // 1 = driver enabled, 0 = tri-stated
	state.exsync_vreset_only = (r & 0x08) != 0;
	state.exsync_preset_odd = (r & 0x10) != 0;

	const bool new_blank = (!state.hsync_drive) || (!state.vsync_drive);
	if (new_blank != state.exsync_blank) {
		state.exsync_blank = new_blank;
		// Changing sync-drive affects monitor visible so full redraw.
		redraw_area(0, 0, old_iWidth, old_iHeight);

	}
	// Remote mode influences EX_SYNC_2 & EX_SYNC_3
	recompute_external_sync_2();
	recompute_external_sync_3();

	EX1_TRACE("S3 EX_SYNC_1: CR56=%02X remote=%d hs_drv=%d vs_drv=%d v_only=%d odd=%d blank=%d\n",
		r, state.exsync_remote, state.hsync_drive, state.vsync_drive,
		state.exsync_vreset_only, state.exsync_preset_odd, state.exsync_blank);
}

void CS3Trio64::recompute_external_sync_2()
{
	const uint8_t old_raw = state.exsync2_raw;
	const uint8_t old_delay = state.exsync2_delay_lines;

	state.exsync2_raw = state.CRTC.reg[0x57]; // 8-bit line delay from VSYNC fall
	// Datasheet: "must be non-zero in Remote mode (CR56 bit0=1)".
	// Don’t silently change the raw register; derive an effective delay instead.
	const bool remote = state.exsync_remote;
	const uint8_t eff = (remote && state.exsync2_raw == 0) ? 1 : state.exsync2_raw;
	state.exsync2_delay_lines = eff;

	if (old_raw != state.exsync2_raw || old_delay != state.exsync2_delay_lines) {
		if (remote && state.exsync2_raw == 0) {
			EX2_TRACE("S3 EX_SYNC_2: CR57=0x%02X -> effective delay=1 line (remote mode requires non-zero)\n",
				state.exsync2_raw);
		}
		else {
			EX2_TRACE("S3 EX_SYNC_2: CR57=0x%02X -> effective delay=%u lines\n",
				state.exsync2_raw, (unsigned)state.exsync2_delay_lines);
		}
	}
}

void CS3Trio64::recompute_ext_misc_ctl()
{
	const uint8_t r = state.CRTC.reg[0x65];
	state.cr65_raw = r;
	state.cr65_enb_3c3 = (r & 0x04) != 0;           // bit2
	state.cr65_blank_dly = (r >> 3) & 0x03;           // bits4:3
	// no effects in 86box. We keep it for future use. only bits 4-2 mean anything on trio64
	// enable 3c3h or 46EBh, we already always expose 3c3h it, so no use..... for now
}

void CS3Trio64::recompute_external_sync_3()
{
	const uint8_t old_raw = state.exsync3_raw;
	const uint8_t old_hs = state.exsync3_hsreset_chars;
	const uint8_t old_cc = state.exsync3_charclk_delay;
	const bool    old_act = state.exsync3_active;

	const uint8_t r = state.CRTC.reg[0x63];
	state.exsync3_raw = r;
	state.exsync3_hsreset_chars = r & 0x0F;       // chars
	state.exsync3_charclk_delay = (r >> 4) & 0x0F; // DCLKs
	state.exsync3_active = state.exsync_remote; // gated by CR56 bit0

	if (old_raw != r || old_hs != state.exsync3_hsreset_chars ||
		old_cc != state.exsync3_charclk_delay || old_act != state.exsync3_active) {
		EX3_TRACE("S3 EX_SYNC_3: CR63=%02X -> HSYNC+%u chars, CHARCLK+%u DCLKs, %s\n",
			r, state.exsync3_hsreset_chars, state.exsync3_charclk_delay,
			state.exsync3_active ? "ACTIVE (Remote)" : "inactive");
	}
}

void CS3Trio64::recompute_config3()
{
	const uint8_t r = state.CRTC.reg[0x68];
	state.cr68_raw = r;
	state.cr68_casoe_we = r & 0x03;      // bits 1:0
	state.cr68_ras_low = (r & 0x04) != 0;       // bit 2
	state.cr68_ras_pcg = (r & 0x08) != 0;       // bit 3
	state.cr68_mon_inf = (r >> 4) & 0x07;      // bits 6:4
	state.cr68_up_add = (r & 0x80) != 0;       // bit 7
	// No behavioral side-effects in our emulation (matches 86Box).
}


/**
 * Create and start thread.
 **/
void CS3Trio64::start_threads()
{
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
	// Signal the thread to stop
	StopThread = true;
	if (myThread)
	{
		printf(" %s", myThread->getName().c_str());
		// Wait for the thread to end execution
		myThread->join();
		// And delete the Thread object
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
	case 18: data = io_read(address + 0xA0E8, dsize); break;
	case 19: data = io_read(address + 0xA2E8, dsize); break;
	case 20: data = io_read(address + 0xA6E8, dsize); break;
	case 21: data = io_read(address + 0xAAE8, dsize); break;
	case 22: data = io_read(address + 0xE2E8, dsize); break;
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
	case 18: io_write(address + 0xA0E8, dsize, data); break;
	case 19: io_write(address + 0xA2E8, dsize, data); break;
	case 20: io_write(address + 0xA6E8, dsize, data); break;
	case 21: io_write(address + 0xAAE8, dsize, data); break;
	case 22: io_write(address + 0xE2E8, dsize, data); break;

	}
}

int CS3Trio64::BytesPerPixel() const
{
	const uint8_t pf = (state.CRTC.reg[0x67] >> 4) & 0x0F;
	switch (pf) {
	case 0x03: return 2; // 15bpp
	case 0x05: return 2; // 16bpp
	case 0x07: return 3; // 24bpp (packed)
	case 0x0D: return 4; // 32bpp (xRGB)
	default:   return 1; // 8bpp indexed
	}
}

u32 CS3Trio64::PitchBytes() const
{
	return (uint32_t)state.line_offset;
}

static inline u32 clamp_vram_addr(u32 a, u32 vram_size) {
	return (vram_size == 0) ? a : (a % vram_size);
}

void CS3Trio64::AccelExecute()
{
	// Busy during the operation (we execute synchronously for now)
	state.accel.busy = true;
	state.accel.subsys_stat |= 0x02; // GE busy
	state.accel.subsys_stat &= ~0x08; // FIFO not empty during op

	const int bpp = BytesPerPixel();
	const u32 pitch = PitchBytes();

	const u32 w = (u32)state.accel.maj_axis_pcnt + 1u;
	const u32 h = (u32)state.accel.desty_axstp + 1u;

	// Minimal decode: SRCCOPY blit vs solid fill.
	// FRGD_MIX lower nibble is a ROP2. 0x0D (SRCCOPY) is the common one.
	const u8  rop = (state.accel.frgd_mix & 0x0F);
	const bool is_copy = (rop == 0x0D); // SRCCOPY
	const bool is_solid = (rop == 0x0F) || (rop == 0x0C) || (rop == 0x05); // PAINT family -> treat as fill

	// Coordinates: CUR_* = source; DEST_* = dest
	const u32 src_x = state.accel.cur_x;
	const u32 src_y = state.accel.cur_y;
	const u32 dst_x = state.accel.dest_x;
	const u32 dst_y = state.accel.desty_axstp /*S3 aliases*/ ? state.accel.dest_y : state.accel.dest_y;

	// vRAM addressing is linear top-left origin. Compute byte offsets.
	auto row_off = [&](u32 y) -> u32 { return y * pitch; };
	auto px_off = [&](u32 x) -> u32 { return x * (u32)bpp; };

	// NOTE: We write VRAM through existing helpers so the rest of your pipeline
	// (dirty tracking/updates) keeps working.
	auto put_px = [&](u32 addr, u32 color) {
		// write color as 8/16/32
		switch (bpp) {
		case 1:
			s3_vram_write8(addr, (uint8_t)(color & 0xFF));
		case 2:
			s3_vram_write8(addr + 0, (uint8_t)(color & 0xFF));
			s3_vram_write8(addr + 1, (uint8_t)((color >> 8) & 0xFF));
			break;
		case 3: // 24bpp packed, little-endian: B, G, R
			s3_vram_write8(addr + 0, (uint8_t)(color & 0xFF));
			s3_vram_write8(addr + 1, (uint8_t)((color >> 8) & 0xFF));
			s3_vram_write8(addr + 2, (uint8_t)((color >> 16) & 0xFF));
			break;

		default: // 32bpp 
			s3_vram_write8(addr + 0, (uint8_t)(color & 0xFF));
			s3_vram_write8(addr + 1, (uint8_t)((color >> 8) & 0xFF));
			s3_vram_write8(addr + 2, (uint8_t)((color >> 16) & 0xFF));
			s3_vram_write8(addr + 3, (uint8_t)((color >> 24) & 0xFF));
			break;
		}
		};

	auto get_px = [&](u32 addr) -> u32 {
		switch (bpp) {
		case 1:
			return s3_vram_read8(addr);
		case 2: {
			uint32_t b0 = s3_vram_read8(addr + 0);
			uint32_t b1 = s3_vram_read8(addr + 1);
			return b0 | (b1 << 8);
		}
		case 3: {
			uint32_t b0 = s3_vram_read8(addr + 0);
			uint32_t b1 = s3_vram_read8(addr + 1);
			uint32_t b2 = s3_vram_read8(addr + 2);
			return b0 | (b1 << 8) | (b2 << 16);
		}
		default: {
			uint32_t b0 = s3_vram_read8(addr + 0);
			uint32_t b1 = s3_vram_read8(addr + 1);
			uint32_t b2 = s3_vram_read8(addr + 2);
			uint32_t b3 = s3_vram_read8(addr + 3);
			return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
		}
		}
		};

	// VRAM size guard 
	const u32 vram_mask = (state.memsize ? state.memsize : (8u * 1024u * 1024u)) - 1u;


	if (is_copy) {
		// Screen-to-screen BLT with overlap-safe order
		const bool ydec = (dst_y > src_y) && (dst_y < src_y + h);
		const bool xdec = (dst_x > src_x) && (dst_x < src_x + w);

		if (ydec) {
			for (int yy = (int)h - 1; yy >= 0; --yy) {
				if (xdec) {
					for (int xx = (int)w - 1; xx >= 0; --xx) {
						u32 sa = clamp_vram_addr(row_off(src_y + yy) + px_off(src_x + xx), vram_mask + 1);
						u32 da = clamp_vram_addr(row_off(dst_y + yy) + px_off(dst_x + xx), vram_mask + 1);
						put_px(da, get_px(sa));
					}
				}
				else {
					for (u32 xx = 0; xx < w; ++xx) {
						u32 sa = clamp_vram_addr(row_off(src_y + yy) + px_off(src_x + xx), vram_mask + 1);
						u32 da = clamp_vram_addr(row_off(dst_y + yy) + px_off(dst_x + xx), vram_mask + 1);
						put_px(da, get_px(sa));
					}
				}
			}
		}
		else {
			for (u32 yy = 0; yy < h; ++yy) {
				if (xdec) {
					for (int xx = (int)w - 1; xx >= 0; --xx) {
						u32 sa = clamp_vram_addr(row_off(src_y + yy) + px_off(src_x + xx), vram_mask + 1);
						u32 da = clamp_vram_addr(row_off(dst_y + yy) + px_off(dst_x + xx), vram_mask + 1);
						put_px(da, get_px(sa));
					}
				}
				else {
					for (u32 xx = 0; xx < w; ++xx) {
						u32 sa = clamp_vram_addr(row_off(src_y + yy) + px_off(src_x + xx), vram_mask + 1);
						u32 da = clamp_vram_addr(row_off(dst_y + yy) + px_off(dst_x + xx), vram_mask + 1);
						put_px(da, get_px(sa));
					}
				}
			}
		}
	}
	else if (is_solid) {
		const u32 color = state.accel.frgd_color;
		for (u32 yy = 0; yy < h; ++yy) {
			for (u32 xx = 0; xx < w; ++xx) {
				u32 da = clamp_vram_addr(row_off(dst_y + yy) + px_off(dst_x + xx), vram_mask + 1);
				put_px(da, color);
			}
		}
	}
	else {
		// Unknown/unsupported ROP -> no-op for now
	}

	redraw_area(dst_x, dst_y, w, h);
	state.accel.busy = false;
	state.accel.subsys_stat &= ~0x02; // GE idle
	state.accel.subsys_stat |= 0x08;  // FIFO empty
}


// -------------------------
// Minimal accel window I/O
// -------------------------
u8 CS3Trio64::AccelIORead(u32 port)
{
	if (!state.accel.enabled) return 0xFF;

	switch (port & 0xFFFE) {
	case 0x42E8: {
		// 0x42E8 -> SUBSYS_STAT (low); 0x42E9 -> SUBSYS_CNTL high byte
		if ((port & 1) == 0) return (u8)(state.accel.subsys_stat);
		else return (u8)(state.accel.subsys_cntl >> 8);
	}

			   // coordinate & size readbacks
	case 0x46E8: return (u8)((state.accel.cur_x >> ((port & 1) ? 8 : 0)) & 0xFF);
	case 0x4EE8: return (u8)((state.accel.cur_y >> ((port & 1) ? 8 : 0)) & 0xFF);
	case 0x86E8: return (u8)((state.accel.dest_x >> ((port & 1) ? 8 : 0)) & 0xFF);
	case 0x8EE8: return (u8)((state.accel.desty_axstp >> ((port & 1) ? 8 : 0)) & 0xFF);
	case 0x96E8: return (u8)((state.accel.maj_axis_pcnt >> ((port & 1) ? 8 : 0)) & 0xFF);

	case 0xAAE8: // WRT_MASK
		return (u8)((state.accel.wrt_mask >> ((port & 1) ? 8 : 0)) & 0xFF);
	case 0xAEE8: // RD_MASK
		return (u8)((state.accel.rd_mask >> ((port & 1) ? 8 : 0)) & 0xFF);

	case 0xA2E8: // BKGD_COLOR (16-bit window)
		return (u8)((state.accel.bkgd_color >> ((port & 1) ? 8 : 0)) & 0xFF);
	case 0xA6E8: // FRGD_COLOR (16-bit window)
		return (u8)((state.accel.frgd_color >> ((port & 1) ? 8 : 0)) & 0xFF);

	case 0xB6E8: // BKGD_MIX
		return ((port & 1) == 0) ? state.accel.bkgd_mix : 0xFF;
	case 0xBAE8: // FRGD_MIX
		return ((port & 1) == 0) ? state.accel.frgd_mix : 0xFF;

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
	if (!state.accel.enabled) return;

	switch (port & 0xFFFE) {

	case 0x42E8: // SUBSYS_STAT clear-on-write (S3 behavior)
		state.accel.subsys_stat &= ~data;
		break;

	case 0x42E9: // SUBSYS_CNTL
		write16_low_high(state.accel.subsys_cntl, port, data);
		break;

	case 0x4AE8: // ADVFUNC_CNTL (packed 4/8bpp mode etc.), store & ignore for now
		write16_low_high(state.accel.advfunc_cntl, port, data);
		break;

		// XY and sizes
	case 0x46E8: write16_low_high(state.accel.cur_x, port, data); break;
	case 0x4EE8: write16_low_high(state.accel.cur_y, port, data); break;
	case 0x86E8: write16_low_high(state.accel.dest_x, port, data); break;
	case 0x8EE8: write16_low_high(state.accel.desty_axstp, port, data); break; // we use as "height-1"
	case 0x96E8: write16_low_high(state.accel.maj_axis_pcnt, port, data); break; // "width-1"

		// colors/mixes/masks
	case 0xA2E8: // BKGD_COLOR
	{ unsigned s = (port & 1) ? 8 : 0; state.accel.bkgd_color = (state.accel.bkgd_color & ~(0xFFu << s)) | ((u32)data << s); }
	break;
	case 0xA6E8: // FRGD_COLOR (16-bit)
	{ unsigned s = (port & 1) ? 8 : 0; state.accel.frgd_color = (state.accel.frgd_color & ~(0xFFu << s)) | ((u32)data << s); }
	break;
	case 0xAAE8: // WRT_MASK
	{ unsigned s = (port & 1) ? 8 : 0; state.accel.wrt_mask = (state.accel.wrt_mask & ~(0xFFu << s)) | ((u32)data << s); }
	break;
	case 0xAEE8: // RD_MASK
	{ unsigned s = (port & 1) ? 8 : 0; state.accel.rd_mask = (state.accel.rd_mask & ~(0xFFu << s)) | ((u32)data << s); }
	break;
	case 0xB6E8: // BKGD_MIX
		if ((port & 1) == 0) state.accel.bkgd_mix = data;
		break;
	case 0xBAE8: // FRGD_MIX
		if ((port & 1) == 0) state.accel.frgd_mix = data;
		break;


		// command
	case 0x9AE8:
		write16_low_high(state.accel.cmd, port, data);
		if ((port & 1) == 1) { // high byte completes write
			const bool cpu_src = (state.accel.cmd & 0x0100) != 0; // S3: bit8 => host source
			if (cpu_src) {
				// arm PIX_TRANS streaming
				state.accel.host_bpp = (uint32_t)BytesPerPixel();          // 1/2/3/4
				const uint32_t w = (uint32_t)state.accel.maj_axis_pcnt + 1u;
				const uint32_t h = (uint32_t)state.accel.desty_axstp + 1u;
				state.accel.host_total_pixels = w * h;
				state.accel.host_pixels_rcvd = 0;
				state.accel.host_cur_x = 0;
				state.accel.host_cur_y = 0;
				state.accel.host_byte_count = 0;
				state.accel.host_xfer_active = true;

				// busy while we consume host pixels
				state.accel.busy = true;
				state.accel.subsys_stat |= 0x02;  // GE busy
				state.accel.subsys_stat &= ~0x08; // FIFO not empty during op
			}
			else {
				// high byte just arrived -> start the op right away
				AccelExecute();
			}
		}
		break;


	case 0xE2E8: // PIX_TRANS (byte-wide window 0xE2E8..0xE2EF)
	case 0xE2EA:
	case 0xE2EC:
	case 0xE2EE:
	case 0xE2E9:
	case 0xE2EB:
	case 0xE2ED:
	case 0xE2EF:
	{
		printf("PIXTRANS!!!\n");
		if (!state.accel.host_xfer_active) {
			// Nothing armed; drop the byte (matches hardware "ignored" semantics)
			break;
		}

		// Accumulate bytes for one destination pixel
		state.accel.host_byte_accum[state.accel.host_byte_count++] = data;

		if (state.accel.host_byte_count >= state.accel.host_bpp) {
			// We have a full pixel; write it to VRAM at (dest_x + x, dest_y + y).
			const uint32_t bpp = state.accel.host_bpp;
			const uint32_t pitch = PitchBytes();
			const uint32_t dx = state.accel.dest_x + state.accel.host_cur_x;
			const uint32_t dy = state.accel.dest_y + state.accel.host_cur_y;

			// Compute byte address (top-left origin, linear)
			uint32_t addr = dy * pitch + dx * bpp;

			// Write the pixel exactly as host provided (little-endian byte order).
			// For 24bpp we just emit 3 bytes in-order.
			for (uint32_t i = 0; i < bpp; ++i) {
				s3_vram_write8(addr + i, state.accel.host_byte_accum[i]);
			}

			// Advance to the next pixel in the rectangle
			state.accel.host_pixels_rcvd++;
			state.accel.host_byte_count = 0;

			state.accel.host_cur_x++;
			const uint32_t rect_w = (uint32_t)state.accel.maj_axis_pcnt + 1u;
			if (state.accel.host_cur_x >= rect_w) {
				state.accel.host_cur_x = 0;
				state.accel.host_cur_y++;
			}

			// Done?
			if (state.accel.host_pixels_rcvd >= state.accel.host_total_pixels) {
				state.accel.host_xfer_active = false;

				// Mark the rectangle updated (one shot for the whole upload).
				const uint32_t rect_w = (uint32_t)state.accel.maj_axis_pcnt + 1u;
				const uint32_t rect_h = (uint32_t)state.accel.desty_axstp + 1u;
				redraw_area(state.accel.dest_x, state.accel.dest_y, rect_w, rect_h);

				state.accel.busy = false;
				state.accel.subsys_stat &= ~0x02; // GE idle
				state.accel.subsys_stat |= 0x08;  // FIFO empty
			}

		}
		break;
	}


	default:
		if ((port & 0xFFF0u) == 0xE2E0u) {
			// Most X paths for color fill don't rely on host data here for S3;
			// if they do, we could add a small FIFO. For now, ignore or extend later.
		}
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
		// dimensions / count
	case 0x96E8: // MAJ_AXIS_PCNT
		// mixes, masks, colors
	case 0xA2E8: // BKGD_COLOR
	case 0xA6E8: // FRGD_COLOR
	case 0xAAE8: // WRT_MASK
	case 0xAEE8: // RD_MASK
	case 0xB6E8: // BKGD_MIX
	case 0xBAE8: // FRGD_MIX
	case 0x9AE8: // CMD
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
		if (off >= lfb_size) return 0;

		// Read little-endian from linear VRAM
		switch (dsize)
		{
		case 1:  return state.memory[off];
		case 2:  return *(u16*)(state.memory + off);
		case 4:  return *(u32*)(state.memory + off);
		case 8: {
			u64 v = *(u32*)(state.memory + off);
			v |= (u64) * (u32*)(state.memory + off + 4) << 32;
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
			for (int i = 0; i < dsize; ++i) v |= (u64)state.memory[off + i] << (i * 8);
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
		if (off >= lfb_size) return;

		// Write little-endian into linear VRAM
		switch (dsize)
		{
#if S3_LFB_TRACE
			printf("%s: LFB W size=%d @%llx <= %08x (off=%llx)\n",
				devid_string, dsize, (unsigned long long)address,
				data, (unsigned long long)(address));
#endif
		case 1:  state.memory[off] = (u8)data; break;
		case 2:  *(u16*)(state.memory + off) = (u16)data; break;
		case 4:  *(u32*)(state.memory + off) = (u32)data; break;
		case 8: {
			*(u32*)(state.memory + off) = (u32)(data & 0xffffffffu);
			*(u32*)(state.memory + off + 4) = (u32)(data >> 32);
			break;
		}
		default:
			for (int i = 0; i < dsize; ++i)
				state.memory[off + i] = (u8)(data >> (i * 8));
			break;
		}

		// Mark display dirty (simple and safe)
		state.vga_mem_updated = 1;
		return;
	}

	// Everything else (legacy VGA ports/memory) stays in CVGA
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
	const bool cr58_on = s3_lfb_enabled(state.CRTC.reg[0x58]);
	const uint32_t sz = s3_lfb_size_from_cr58(state.CRTC.reg[0x58]);
	const uint32_t base = pci_bar0;  // BAR-only base of truth
	const bool eff = pci_mem_enable && cr58_on && (base != 0);

	if (!lfb_trace_initialized ||
		eff != lfb_trace_enabled_prev ||
		base != lfb_trace_base_prev ||
		sz != lfb_trace_size_prev) {

		printf("%s: LFB %s - MSE=%d CR58=%02x base=%08x size=%x (reason=%s)\n",
			devid_string, eff ? "ACTIVE(BAR)" : "INACTIVE(BAR)",
			(int)pci_mem_enable, state.CRTC.reg[0x58],
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

	// tie aperture size to VRAM size - need to honor CR58..CR5A as 86Box does. 
	lfb_base_ = bar0;
	lfb_size_ = (u32)state.memsize;   // clamp to what your Trio variant actually exposes if needed
	lfb_enabled_ = pci_mem_enable && (bar0 != 0);
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
	const uint8_t  cr58 = state.CRTC.reg[0x58];
	if (!s3_lfb_enabled(cr58)) {
		// Linear aperture disabled: read as bus-floating
		return (dsize == 8) ? 0xFFu : (dsize == 16 ? 0xFFFFu : 0xFFFFFFFFu);
	}

	const uint32_t win_size = s3_lfb_size_from_cr58(cr58);
	const uint32_t mask_win = win_size - 1u;
	const uint32_t mask_vram = state.memsize - 1u;   // state.memsize is power-of-two

	uint32_t off = (address & mask_win) & mask_vram;
	const uint8_t* vram = &state.memory[0];

	switch (dsize) {

	case 8:
		//        printf("S3 mem read: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, vram[off]);
		return vram[off];

	case 16:
		//        printf("S3 mem read: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, (uint32_t(vram[off]) | (uint32_t(vram[(off + 1) & mask_vram]) << 8)));
		return uint32_t(vram[off]) | (uint32_t(vram[(off + 1) & mask_vram]) << 8);

	default: // 32
		//        printf("S3 mem read: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, (uint32_t(vram[off]) |
		//            (uint32_t(vram[(off + 1) & mask_vram]) << 8) |
		//            (uint32_t(vram[(off + 2) & mask_vram]) << 16) |
		//            (uint32_t(vram[(off + 3) & mask_vram]) << 24)));
		return  uint32_t(vram[off]) |
			(uint32_t(vram[(off + 1) & mask_vram]) << 8) |
			(uint32_t(vram[(off + 2) & mask_vram]) << 16) |
			(uint32_t(vram[(off + 3) & mask_vram]) << 24);
	}
}

/**
 * Write to Framebuffer.
 *
 * Not functional.
 **/
void CS3Trio64::mem_write(u32 address, int dsize, u32 data)
{
	const uint8_t cr58 = state.CRTC.reg[0x58];
	const bool    ena = (cr58 & 0x10) != 0;
	const u32     win = s3_lfb_size_from_cr58(cr58);
	//    printf("[S3 LFB WRITE] ena=%d size=%u addr=%08X dsize=%d data=%08X\n",
	//        ena, win, address, dsize, data);
	if (!ena) return;

	if (!s3_lfb_enabled(cr58)) return;  // ignore if aperture disabled

	const uint32_t win_size = s3_lfb_size_from_cr58(cr58);
	const uint32_t mask_win = win_size - 1u;
	const uint32_t mask_vram = state.memsize - 1u;

	uint32_t off = (address & mask_win) & mask_vram;
	uint8_t* vram = &state.memory[0];

	// Little-endian store (matches legacy path style)
	switch (dsize) {
	case 8:
		vram[off] = uint8_t(data);
		break;
	case 16:
		vram[off] = uint8_t(data);
		vram[(off + 1) & mask_vram] = uint8_t(data >> 8);
		break;
	default: // 32
		vram[off] = uint8_t(data);
		vram[(off + 1) & mask_vram] = uint8_t(data >> 8);
		vram[(off + 2) & mask_vram] = uint8_t(data >> 16);
		vram[(off + 3) & mask_vram] = uint8_t(data >> 24);
		break;
	}

	// Mark the affected tiles dirty so update() will serialize to the screen.
	state.vga_mem_updated = 1;
	if (state.line_offset) {
		// Mark each byte we touched (dsize/8 bytes) — cheap and correct at tile granularity
		const unsigned nbytes = (dsize == 8) ? 1u : (dsize == 16 ? 2u : 4u);
		for (unsigned i = 0; i < nbytes; ++i) {
			const uint32_t p = (off + i) & mask_vram;
			const uint32_t line = (state.line_offset ? (p / state.line_offset) : 0);
			const uint32_t col = (state.line_offset ? (p % state.line_offset) : 0);
			const unsigned xti = col / X_TILESIZE;
			const unsigned yti = line / Y_TILESIZE;
			SET_TILE_UPDATED(xti, yti, 1);
		}
	}
	//  printf("S3 mem write: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, data);
}

/**
 * Read from Legacy VGA Memory
 *
 * Calls vga_mem_read to read the data 1 byte at a time.
 **/
u32 CS3Trio64::legacy_read(u32 address, int dsize)
{
#if ES40_S3_ACCEL_ENABLE
	if (s3_mmio_enabled(state) && IsAccelPort(address)) {
		switch (dsize) {
		case 8:  return AccelIORead(address);
		case 16: return (u32)AccelIORead(address + 0)
			| ((u32)AccelIORead(address + 1) << 8);
		case 32: return (u32)AccelIORead(address + 0)
			| ((u32)AccelIORead(address + 1) << 8)
			| ((u32)AccelIORead(address + 2) << 16)
			| ((u32)AccelIORead(address + 3) << 24);
		default: FAILURE(InvalidArgument, "Unsupported dsize");
		}
	}
#endif
	u32 data = 0;
	switch (dsize)
	{
	case 32:
		data |= (u64)vga_mem_read((u32)address + 0xA0003) << 24;
		data |= (u64)vga_mem_read((u32)address + 0xA0002) << 16;

	case 16:
		data |= (u64)vga_mem_read((u32)address + 0xA0001) << 8;

	case 8:
		data |= (u64)vga_mem_read((u32)address + 0xA0000);
	}

	//  //printf("S3 legacy read: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, data);
	return data;
}

/**
 * Write to Legacy VGA Memory
 *
 * Calls vga_mem_write to write the data 1 byte at a time.
 **/
void CS3Trio64::legacy_write(u32 address, int dsize, u32 data)
{
#if ES40_S3_ACCEL_ENABLE
	if (s3_mmio_enabled(state) && IsAccelPort(address)) {
		switch (dsize) {
		case 8:
			AccelIOWrite(address, (u8)data);
			return;
		case 16:
			AccelIOWrite(address + 0, (u8)(data & 0xFF));
			AccelIOWrite(address + 1, (u8)(data >> 8));
			return;
		case 32:
			AccelIOWrite(address + 0, (u8)(data & 0xFF));
			AccelIOWrite(address + 1, (u8)((data >> 8) & 0xFF));
			AccelIOWrite(address + 2, (u8)((data >> 16) & 0xFF));
			AccelIOWrite(address + 3, (u8)((data >> 24) & 0xFF));
			return;
		default:
			FAILURE(InvalidArgument, "Unsupported dsize");
		}
	}
#endif

	//  //printf("S3 legacy write: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, data);
	switch (dsize)
	{
	case 32:
		vga_mem_write((u32)address + 0xA0002, (u8)(data >> 16));
		vga_mem_write((u32)address + 0xA0003, (u8)(data >> 24));

	case 16:
		vga_mem_write((u32)address + 0xA0001, (u8)(data >> 8));

	case 8:
		vga_mem_write((u32)address + 0xA0000, (u8)(data));
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
#if ES40_S3_ACCEL_ENABLE
	// Accel ports accept byte/word/dword I/O. Handle them BEFORE size check.
	if (IsAccelPort(address)) {
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
#endif
	if (dsize != 8)
		FAILURE(InvalidArgument, "Unsupported dsize");


	switch (address)
	{
	case 0x3c0:
		data = read_b_3c0();
		break;

	case 0x3c1:
		data = read_b_3c1();
		break;

	case 0x3c2:
		data = read_b_3c2();
		break;

	case 0x3c3:
		data = read_b_3c3();
		break;

	case 0x3c4:
		data = read_b_3c4();
		break;

	case 0x3c5:
		data = read_b_3c5();
		break;

	case 0x3c6:
		data = read_b_3c6();
		break;

	case 0x3c7:
		data = read_b_3c7();
		break;

	case 0x3c8:
		break;

	case 0x3c9:
		data = read_b_3c9();
		break;

	case 0x3ca:
		data = read_b_3ca();
		break;

	case 0x3cc:
		data = read_b_3cc();
		break;

	case 0x3cf:
		data = read_b_3cf();
		break;

	case 0x3b4:
	case 0x3d4:
		data = read_b_3d4();
		break;

	case 0x3b5:
	case 0x3d5:
		data = read_b_3d5();
		break;

	case 0x3ba:
	case 0x3da:
		data = read_b_3da();
		break;

	default:
		FAILURE_1(NotImplemented, "Unhandled port %x read", address);
	}

	//printf("S3 io read: %" PRIx64 ", %d, %" PRIx64 "   \n", address, dsize, data);
	return data;
}

/**
 * Write to I/O Port
 *
 * Calls io_write_b to write the data 1 byte at a time.
 */
void CS3Trio64::io_write(u32 address, int dsize, u32 data)
{
	// 8514/A-style accel window (S3 engine)
#if ES40_S3_ACCEL_ENABLE
	if (IsAccelPort(address)) {
		// Debug so you can see the 42E8 word/dword traffic too
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
#endif

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
		write_b_3c0(data);
		break;

	case 0x3c2:
		write_b_3c2(data);
		break;

	case 0x3c3:
		write_b_3c3(data);
		break;

	case 0x3c4:
		write_b_3c4(data);
		break;

	case 0x3c5:
		write_b_3c5(data);
		break;

	case 0x3c6:
		write_b_3c6(data);
		break;

	case 0x3c7:
		write_b_3c7(data);
		break;

	case 0x3c8:
		write_b_3c8(data);
		break;

	case 0x3c9:
		write_b_3c9(data);
		break;

	case 0x3ce:
		write_b_3ce(data);
		break;

	case 0x3cf:
		write_b_3cf(data);
		break;

	case 0x3da:
		write_b_3da(data);
		break;

	case 0x3b4:
	case 0x3d4:
		write_b_3d4(data);
		break;

	case 0x3b5:
	case 0x3d5:
		write_b_3d5(data);
		break;

	default:
#if DEBUG_VGA
		printf("\nFAILURE ON BELOW LISTED PORT BINARY VALUE=" PRINTF_BINARY_PATTERN_INT8 " HEX VALUE=0x%02x\n", PRINTF_BYTE_TO_BINARY_INT8(data), data);
#endif
		FAILURE_1(NotImplemented, "Unhandled port %x write", address);
	}
}

/**
 * Write to the attribute controller I/O port (0x3c0)
 *
 * The attribute controller registers are used to select the 16 color
 * and 64 color palettes used for EGA/CGA compatibility.
 *
 * The attribute registers are accessed in an indexed fashion.
 * The address register is read and written via port 3C0h.
 * The data register is written to port 3C0h and read from port 3C1h.
 * The index and the data are written to the same port, one after
 * another. A flip-flop inside the card keeps track of whether the
 * next write will be handled is an index or data. Because there is
 * no standard method of determining the state of this flip-flop, the
 * ability to reset the flip-flop such that the next write will be
 * handled as an index is provided. This is accomplished by reading
 * the Input Status #1 Register (normally port 3DAh) (the data
 * received is not important.)
 *
 * Attribute registers:
 *   - Palette Index registers (index 0x00 - 0x0f)
 *   - Attribute Mode Control register (index 0x10)
 *   - Overscan Color register (index 0x11)
 *   - Color Plane Enable register (index 0x12)
 *   - Horizontal Pixel Panning register (index 0x13)
 *   - Color Select register (index 0x14)
 *   .
 *
 * \code
 * Attribute Address Register(3C0h)
 * +---+-+---------+
 * |   |5|4 3 2 1 0|
 * +---+-+---------+
 *      ^     ^
 *      |     +-- 0..4: Attribute Address: This field specifies the index
 *      |               value of the attribute register to be read or written
 *      +----------- 5: Palette Address Source: This bit is set to 0 to load
 *                      color values to the registers in the internal palette.
 *                      It is set to 1 for normal operation of the attribute
 *                      controller.
 *
 * Palette Index Registers (index 0x00 - 0x0f)
 * +---+-----------+
 * |   |5 4 3 2 1 0|
 * +---+-----------+
 *           ^
 *           +--- 0..5: Internal Palette Index: These 6-bit registers allow a
 *                      dynamic mapping between the text attribute or graphic
 *                      color input value and the display color on the CRT
 *                      screen. These internal palette values are sent off-chip
 *                      to the video DAC, where they serve as addresses into
 *                      the DAC registers.
 *
 * Attribute Mode Control Register (index 0x10)
 * +-+-+-+-+-+-+-+-+
 * |7|6|5| |3|2|1|0|
 * +-+-+-+-+-+-+-+-+
 *  ^ ^ ^   ^ ^ ^ ^
 *  | | |   | | | +- 0: ATGE - Attribute Controller Graphics Enable:
 *  | | |   | | |         0: Disables the graphics mode of operation.
 *  | | |   | | |         1: Selects the graphics mode of operation.
 *  | | |   | | +--- 1: MONO - Monochrome Emulation: This bit is present and
 *  | | |   | |         programmable in all of the hardware but it apparently
 *  | | |   | |         does nothing.
 *  | | |   | +----- 2: LGE - Line Graphics Enable: This field is used in 9
 *  | | |   |           bit wide character modes to provide continuity for the
 *  | | |   |           horizontal line characters in the range C0h-DFh:
 *  | | |   |             0: the 9th column is replicated from the 8th column.
 *  | | |   |             1: the 9th column is set to the background.
 *  | | |   +------- 3: BLINK - Blink Enable:
 *  | | |                 0: Bit 7 of the attribute selects the background
 *  | | |                    intensity (allows 16 colors for background).
 *  | | |                 1: Bit 7 of the attribute enables blinking.
 *  | | +----------- 5: PPM -- Pixel Panning Mode: Allows the upper half of
 *  | |                 the screen to pan independently of the lower screen.
 *  | |                   0: nothing special occurs during a successful line
 *  | |                      compare (see the Line Compare field.)
 *  | |                   1: upon a successful line compare, the bottom portion
 *  | |                      of the screen is displayed as if the Pixel Shift
 *  | |                      Count and Byte Panning fields are set to 0.
 *  | +------------- 6: 8BIT -- 8-bit Color Enable:
 *  |                     1: The video data is sampled so that eight bits are
 *  |                        available to select a color in the 256-color mode.
 *  |                     0: All other modes.
 *  +--------------- 7: P54S -- Palette Bits 5-4 Select: Selects the source for
 *                      the P5 and P4 video bits that act as inputs to the video
 *                      DAC.
 *                        0: P5 and P4 are the outputs of the Internal Palette
 *                           registers.
 *                        1: P5 and P4 are bits 1 and 0 of the Color Select
 *                           register.
 *
 * Overscan Color Register (index 0x11)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 *         +----- 0..7: Overscan Palette Index: Selects a color from one of the
 *                      DAC registers for the border.
 *
 * Color Plane Enable Register (index 0x12)
 * +-------+-------+
 * |       |3 2 1 0|
 * +-------+-------+
 *             ^
 *             +- 0..3: Color Plane Enable: Setting a bit to 1 enables the
 *                      corresponding display-memory color plane.
 *
 * Horizontal Pixel Panning Register (index 0x13)
 * +-------+-------+
 * |       |3 2 1 0|
 * +-------+-------+
 *             ^
 *             +- 0..3: Pixel Shift Count: These bits select the number of pels
 *                      that the video data is shifted to the left.
 *
 * Color Select Register (index 0x14)
 * +-------+---+---+
 * |       |3 2|1 0|
 * +-------+---+---+
 *           ^   ^
 *           |   +- 0..1: Color Select 5-4: These bits can be used in place of
 *           |            the P4 and P5 bits from the Internal Palette registers
 *           |            to form the  8-bit digital color value to the video DAC.
 *           |            Selecting these bits is done in the Attribute Mode
 *           |            Control register (index 0x10).
 *           +----- 2..3: Color Select 7-6: In modes other than mode 0x13
 *                        (256-color VGA), these are the two most-significant bits
 *                        of the 8-bit digital color value to the video DAC.
 * \endcode
 **/
void CS3Trio64::write_b_3c0(u8 value)
{
	// Variables to save old state (to detect transitions)
	bool  prev_video_enabled;
	bool  prev_line_graphics;
	bool  prev_int_pal_size;

	/* The flip-flop determines whether the write goes to the index-register
	   (address) or the data-register. */
	if (state.attribute_ctrl.flip_flop == 0)
	{
		// Write goes to the index-register.

		/* The index register also has a bit that controls whether video
		   output is enabled or not.
		   We check this bit, and compare it to it's previous state, to
		   determine whether we need to perform an enable or disable
		   transition. */
		prev_video_enabled = state.attribute_ctrl.video_enabled;
		state.attribute_ctrl.video_enabled = (value >> 5) & 0x01;
#if DEBUG_VGA_NOISY
		printf("io write 3c0: video_enabled = %u   \n",
			(unsigned)state.attribute_ctrl.video_enabled);
#endif
		if (state.attribute_ctrl.video_enabled == 0)
		{
			if (prev_video_enabled)
			{
#if DEBUG_VGA_NOISY
				printf("found disable transition   \n");
#endif
				// Video output has been disabled. Clear the screen.
				bx_gui->lock();
				bx_gui->clear_screen();
				bx_gui->unlock();
			}
		}
		else if (!prev_video_enabled)
		{
#if DEBUG_VGA_NOISY
			printf("found enable transition   \n");
#endif
			// Video output has been enabled. Draw the screen.
			redraw_area(0, 0, old_iWidth, old_iHeight);
		}

		// Determine what register should be addressed.
		value &= 0x1f;  /* address = bits 0..4 */
		state.attribute_ctrl.address = value;

		/* Registers 0x00..0x0f are palette selection registers.
		   Write a debugging message for all other registers. */
#if DEBUG_VGA_NOISY
		if (value > 0x0f)
			printf("io write 3c0: address mode reg=%u   \n", (unsigned)value);
#endif
	}
	else
	{
		// Write should go to the data-register.

		// Registers 0x00..0x0f are palette selection registers.
		if (state.attribute_ctrl.address <= 0x0f)
		{
			// CR33 bit6: Lock Palette/Overscan Registers
			if (!(state.CRTC.reg[0x33] & 0x40)) {
				// Update palette selection only of there is a change.
				if (value != state.attribute_ctrl.palette_reg[state.attribute_ctrl.
					address])
				{
					// Update the palette selection.
					state.attribute_ctrl.palette_reg[state.attribute_ctrl.address] = value;
					// Requires redrawing the screen.
					redraw_area(0, 0, old_iWidth, old_iHeight);
				}
			}
		}
		else
		{
			switch (state.attribute_ctrl.address)
			{
				// Mode control register
			case 0x10:
				prev_line_graphics = state.attribute_ctrl.mode_ctrl.enable_line_graphics;
				prev_int_pal_size = state.attribute_ctrl.mode_ctrl.internal_palette_size;
				state.attribute_ctrl.mode_ctrl.graphics_alpha = (value >> 0) & 0x01;
				state.attribute_ctrl.mode_ctrl.display_type = (value >> 1) & 0x01;
				state.attribute_ctrl.mode_ctrl.enable_line_graphics = (value >> 2) & 0x01;
				state.attribute_ctrl.mode_ctrl.blink_intensity = (value >> 3) & 0x01;
				state.attribute_ctrl.mode_ctrl.pixel_panning_compat = (value >> 5) & 0x01;
				state.attribute_ctrl.mode_ctrl.pixel_clock_select = (value >> 6) & 0x01;
				state.attribute_ctrl.mode_ctrl.internal_palette_size = (value >> 7) & 0x01;
				if (((value >> 2) & 0x01) != prev_line_graphics)
				{
					bx_gui->lock();
					bx_gui->set_text_charmap(&state.memory[0x20000 + state.charmap_address]);
					bx_gui->unlock();
					state.vga_mem_updated = 1;
				}

				if (((value >> 7) & 0x01) != prev_int_pal_size)
				{
					redraw_area(0, 0, old_iWidth, old_iHeight);
				}

#if DEBUG_VGA_NOISY
				printf("io write 3c0: mode control: %02x h   \n", (unsigned)value);
#endif
				break;

				// Overscan Color Register
			case 0x11:
				// CR33 bit6: Lock Palette/Overscan Registers
				if (!(state.CRTC.reg[0x33] & 0x40)) {
					/* We don't do anything with this. Our display doesn't
					   show the overscan part of the normal monitor. */
					state.attribute_ctrl.overscan_color = (value & 0x3f);
#if DEBUG_VGA_NOISY
					printf("io write 3c0: overscan color = %02x   \n", (unsigned)value);
#endif
				}
				break;

				// Color Plane Enable Register
			case 0x12:
				state.attribute_ctrl.color_plane_enable = (value & 0x0f);
				redraw_area(0, 0, old_iWidth, old_iHeight);
#if DEBUG_VGA_NOISY
				printf("io write 3c0: color plane enable = %02x   \n", (unsigned)value);
#endif
				break;

				// Horizontal Pixel Panning Register
			case 0x13:
				state.attribute_ctrl.horiz_pel_panning = (value & 0x0f);
				redraw_area(0, 0, old_iWidth, old_iHeight);
#if DEBUG_VGA_NOISY
				printf("io write 3c0: horiz pel panning = %02x   \n", (unsigned)value);
#endif
				break;

				// Color Select Register
			case 0x14:
				state.attribute_ctrl.color_select = (value & 0x0f);
				redraw_area(0, 0, old_iWidth, old_iHeight);
#if DEBUG_VGA_NOISY
				printf("io write 3c0: color select = %02x   \n",
					(unsigned)state.attribute_ctrl.color_select);
#endif
				break;

			default:
				FAILURE_1(NotImplemented, "io write 3c0: data-write mode %02x h",
					(unsigned)state.attribute_ctrl.address);
			}
		}
	}

	// Flip the flip-flop
	state.attribute_ctrl.flip_flop = !state.attribute_ctrl.flip_flop;
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
	state.misc_output.color_emulation = (value >> 0) & 0x01;
	state.misc_output.enable_ram = (value >> 1) & 0x01;
	{
		u8 clk = (value >> 2) & 0x03;
		if (state.CRTC.reg[0x34] & 0x80)  // CR34 bit7: lock CKSL
			clk = state.misc_output.clock_select;
		state.misc_output.clock_select = clk;
	}
	state.misc_output.select_high_bank = (value >> 5) & 0x01;
	state.misc_output.horiz_sync_pol = (value >> 6) & 0x01;
	state.misc_output.vert_sync_pol = (value >> 7) & 0x01;
#if DEBUG_VGA_NOISY
	printf("io write 3c2:   \n");
	printf("  color_emulation = %u   \n",
		(unsigned)state.misc_output.color_emulation);
	printf("  enable_ram = %u   \n", (unsigned)state.misc_output.enable_ram);
	printf("  clock_select = %u   \n", (unsigned)state.misc_output.clock_select);
	printf("  select_high_bank = %u   \n",
		(unsigned)state.misc_output.select_high_bank);
	printf("  horiz_sync_pol = %u   \n",
		(unsigned)state.misc_output.horiz_sync_pol);
	printf("  vert_sync_pol = %u   \n", (unsigned)state.misc_output.vert_sync_pol);
#endif
}

/**
 * Write to the VGA sequencer index register (0x3c4)
 *
 * The Sequencer registers control how video data is sent to the DAC.
 *
 * The Sequencer registers are accessed in an indexed fashion. By writing a byte
 * to the Sequencer Index Register (0x3c4) equal to the index of the particular
 * sub-register you wish to access, one can address the data pointed to by that
 * index by reading and writing the Sequencer Data Register (0x3c5).
 *
 * Sequencer registers:
 *   - Reset register (index 0x00)
 *   - Clocking Mode register (index 0x01)
 *   - Map Mask register (index 0x02)
 *   - Character Map Select register (index 0x03)
 *   - Memory Mode register (index 0x04)
 *   .
 *
 * \code
 * Reset register (index 0x00)
 * +-----------+-+-+
 * |           |1|0|
 * +-----------+-+-+
 *              ^ ^
 *              | +- 0: Asynchronous Reset:
 *              |         0: Commands the sequencer to asynchronously clear and
 *              |            halt. Resetting the sequencer with this bit can
 *              |            cause loss of video data.
 *              |         1: Allows the sequencer to function normally.
 *              +--- 1: Sychnronous Reset:
 *                        0: Commands the sequencer to synchronously clear and
 *                           halt.
 *                        1: Allows the sequencer to function normally.
 * Bits 1 and 0 must be 1 to allow the sequencer to operate.
 * To prevent the loss of data, bit 1 must be set to 0 during the active display
 * interval before changing the clock selection. The clock is changed through the
 * Clocking Mode register or the Miscellaneous Output register.
 *
 * Clocking Mode register (index 0x01)
 * +---+-+-+-+-+-+-+
 * |   |5|4|3|2| |0|
 * +---+-+-+-+-+-+-+
 *      ^ ^ ^ ^   ^
 *      | | | |   +- 0: 9/8 Dot Mode: Selects whether a character is 8 or 9 dots
 *      | | | |         wide. This can be used to select between 720 and 640
 *      | | | |         pixel modes (or 360 and 320) and also is used to provide
 *      | | | |         9 bit wide character fonts in text mode:
 *      | | | |           0: Selects 9 dots per character.
 *      | | | |           1: Selects 8 dots per character.
 *      | | | +----- 2: Shift/Load Rate:
 *      | | |             0: Video serializers are loaded every character clock.
 *      | | |             1: Video serializers are loaded every other character
 *      | | |                clock, which is useful when 16 bits are fetched per
 *      | | |                cycle and chained together in the shift registers.
 *      | | +------- 3: Dot Clock Rate:
 *      | |               0: Selects the normal dot clocks derived from the
 *      | |                  sequencer master clock input.
 *      | |               1: The master clock will be divided by 2 to generate
 *      | |                  the dot clock. All other timings are affected
 *      | |                  because they are derived from the dot clock. The dot
 *      | |                  clock divided by 2 is used for 320 and 360 horizontal
 *      | |                  PEL modes.
 *      | +--------- 4: Shift Four Enable:
 *      |                 0: Video serializers are loaded every character clock.
 *      |                 1: Video serializers are loaded every fourth character
 *      |                    clock, which is useful when 32 bits are fetched per
 *      |                    cycle and chained together in the shift registers.
 *      +----------- 5: Screen Disable:
 *                        0: Display enabled.
 *                        1: Display blanked. Maximum memory bandwidth assigned to
 *                           the system.
 *
 * Map Mask register (index 0x02)
 * +-------+-------+
 * |       |3 2 1 0|
 * +-------+-------+
 *             ^
 *             +- 0..3: Memory Plane Write Enable: If a bit is set, then write
 *                      operations will modify the respective plane of display
 *                      memory. If a bit is not set then write operations will not
 *                      affect the respective plane of display memory.
 *
 * Character Map Select register (index 0x03)
 * +---+-+-+---+---+
 * |   |5|4|3 2|1 0|
 * +---+-+-+---+---+
 *      ^ ^  ^   ^
 *      | +--|---+- 0..1,4: Character Set B Select: This field is used to select the
 *      |    |              font that is used in text mode when bit 3 of the attribute
 *      |    |              byte for a character is set to 0. (*)
 *      +----+----- 2..3,5: Character Set A Select: This field is used to select the
 *                          font that is used in text mode when bit 3 of the attribute
 *                          byte for a character is set to 1. (*)
 *
 * (*) Note that this field is not contiguous in order to provide EGA compatibility.
 *     The font selected resides in plane 2 of display memory at the address specified
 *     by this field, as follows:
 * +------+---------------+
 * |  val | font at       |
 * +------+---------------+
 * | 000b | 0000h - 1FFFh |
 * | 001b | 4000h - 5FFFh |
 * | 010b | 8000h - 9FFFh |
 * | 011b | C000h - DFFFh |
 * | 100b | 2000h - 3FFFh |
 * | 101b | 6000h - 7FFFh |
 * | 110b | A000h - BFFFh |
 * | 111b | E000h - FFFFh |
 * +------+---------------+
 *
 * Memory Mode register (index 0x04)
 * +-------+-+-+-+-+
 * |       |3|2|1| |
 * +-------+-+-+-+-+
 *          ^ ^ ^
 *          | | +--- 1: Extended Memory:
 *          | |           0: 64 KB of video memory enabled
 *          | |           1: 256 KB of video memory enabled. This bit must be set to 1 to
 *          | |              enable the character map selection described for the
 *          | |              previous register.
 *          | +----- 2: Odd/Even Host Memory Write Adressing Disable:
 *          |             0: Even system addresses access maps 0 and 2, while odd system
 *          |                addresses access maps 1 and 3.
 *          |             1: System addresses sequentially access data within a bit map,
 *          |                and the maps are accessed according to the value in the Map
 *          |                Mask register (index 0x02).
 *          +------- 3: Chain 4 Enable: This bit controls the map selected during system
 *                      read operations.
 *                        0: Enables system addresses to sequentially access data within
 *                           a bit map by using the Map Mask register.
 *                        1: Causes the two low-order bits to select the map accessed as
 *                           shown below:
 *                           +----+----+--------------+
 *                           | A0 | A1 | Map Selected |
 *                           +----+----+--------------+
 *                           |  0 |  0 | 0            |
 *                           |  0 |  1 | 1            |
 *                           |  1 |  0 | 2            |
 *                           |  1 |  1 | 3            |
 *                           +----+----+--------------+
 * \endcode
 **/
void CS3Trio64::write_b_3c4(u8 value)
{
#if DEBUG_VGA_NOISY
	printf("VGA: 3c4 (SET SEQUENCE REGISTER INDEX) value=0x%02x \n", (unsigned)value);
#endif
	state.sequencer.index = value;
}

/**
 * Write to the VGA sequencer data register (0x3c5)
 *
 * For a description of the Sequencer registers, see CCirrus::write_b_3c4
 **/
void CS3Trio64::write_b_3c5(u8 value)
{
	unsigned  i;
	u8        charmap1;
	u8        charmap2;

	if (state.sequencer.index > 0x08 && state.sequencer.pll_lock != 0x6) return;

#if DEBUG_VGA_NOISY
	printf("VGA: 3c5 WRITE INDEX=0x%02x BINARY VALUE=" PRINTF_BINARY_PATTERN_INT8 " HEX VALUE=0x%02x\n", state.sequencer.index, PRINTF_BYTE_TO_BINARY_INT8(value), value);
#endif

	switch (state.sequencer.index)
	{
		// Sequencer: reset register
	case 0x00:
#if DEBUG_VGA_NOISY
		printf("write 0x3c5: sequencer reset: value=0x%02x   \n", (unsigned)value);
#endif
		if (state.sequencer.reset1 && ((value & 0x01) == 0))
		{
			state.sequencer.char_map_select = 0;
			state.charmap_address = 0;
			bx_gui->lock();
			bx_gui->set_text_charmap(&state.memory[0x20000 + state.charmap_address]);
			bx_gui->unlock();
			state.vga_mem_updated = 1;
		}

		state.sequencer.reset1 = (value >> 0) & 0x01;
		state.sequencer.reset2 = (value >> 1) & 0x01;
		break;

		// Sequencer: clocking mode register
	case 0x01:
#if DEBUG_VGA_NOISY
		printf("io write 3c5=%02x: clocking mode reg: ignoring   \n",
			(unsigned)value);
#endif
		{
			u8 newreg1 = value & 0x3f;
			// CR34 bit5 = lock 8/9-dot -> preserve the bit our text path uses (reg1 bit0).
			if (state.CRTC.reg[0x34] & 0x20) {
				newreg1 = (newreg1 & ~0x01) | (state.sequencer.reg1 & 0x01);
			}
			state.sequencer.reg1 = newreg1;
			state.x_dotclockdiv2 = ((newreg1 & 0x08) > 0);
			// Char width may have changed -> DTP pixel position changes -> recompute+trace
			recompute_data_transfer_position();  // prints if effective pixel pos changed
		}
		break;

		// Sequencer: map mask register
	case 0x02:
		state.sequencer.map_mask = (value & 0x0f);
		for (i = 0; i < 4; i++)
			state.sequencer.map_mask_bit[i] = (value >> i) & 0x01;
		break;

		// Sequencer: character map select register
	case 0x03:
		state.sequencer.char_map_select = value;
		charmap1 = value & 0x13;
		if (charmap1 > 3)
			charmap1 = (charmap1 & 3) + 4;
		charmap2 = (value & 0x2C) >> 2;
		if (charmap2 > 3)
			charmap2 = (charmap2 & 3) + 4;
		if (state.CRTC.reg[0x09] > 0)
		{
			state.charmap_address = (charmap1 << 13);
			bx_gui->lock();
			bx_gui->set_text_charmap(&state.memory[0x20000 + state.charmap_address]);
			bx_gui->unlock();
			state.vga_mem_updated = 1;
		}

		if (charmap2 != charmap1)
			printf("char map select: #2=%d (unused)   \n", charmap2);
		break;

		// Sequencer: memory mode register
	case 0x04:
		state.sequencer.extended_mem = (value >> 1) & 0x01;
		state.sequencer.odd_even = (value >> 2) & 0x01;
		state.sequencer.chain_four = (value >> 3) & 0x01;

#if DEBUG_VGA_NOISY
		printf("io write 3c5: index 4:   \n");
		printf("  extended_mem %u   \n", (unsigned)state.sequencer.extended_mem);
		printf("  odd_even %u   \n", (unsigned)state.sequencer.odd_even);
		printf("  chain_four %u   \n", (unsigned)state.sequencer.chain_four);
#endif
		break;

	case 0x08:
		state.sequencer.pll_lock = value;
		break;

	case 0x0A:
		state.sequencer.srA = value;
		break;

	case 0x0B:
		state.sequencer.srB = value;
		break;

	case 0x0D:
		state.sequencer.srD = value;
		break;

	case 0x09: // Extended Sequencer Register 9 (SR9) - all bits reserved
		state.sequencer.sr9 = value;
		break;

	case 0x10: // Memory PLL Data Low
		state.sequencer.sr10 = value;
		state.sequencer.mclkn = value & 0x1f;
		state.sequencer.mclkr = value >> 5;
		break;

	case 0x11:
		state.sequencer.mclkm = value;
		break;

	case 0x12: // video pll data low
		state.sequencer.sr10 = value;
		state.sequencer.clk3n = value & 0x1f;
		state.sequencer.clk3r = value >> 5;
		break;

	case 0x13: // DCLK Value High Register SR13 - here and 14 86box wants us to recalculate timings
		state.sequencer.sr13 = value;
		break;

	case 0x14:  // CLKSYN Control 1 Register (SR14) - So far only used to "power down" and "power up" MCLK and DCLK PLL 
		state.sequencer.sr14 = value;
		break;

	case 0x15: // CLKSYN Control 2 Register (SR15) - VGA_StartResize() called after setting value for dosbox-x, 86box does nothing
		state.sequencer.sr15 = value;
		break;

	case 0x18: // RAMDAC/CLKSYN Control Register (SR18)
		state.sequencer.sr18 = value;
		break;

		/* NOT DOCUMENTED - Sequence Register 1A & 1B - 86box for handling this is

		   if (svga->seqaddr >= 0x10 && svga->seqaddr < 0x20) {
			   svga->seqregs[svga->seqaddr] = val;
			   switch (svga->seqaddr) {
				  case 0x12:
				  case 0x13:
					  svga_recalctimings(svga);
					  return;

				  default:
					  break;  */
	case 0x1a: // not documented
		state.sequencer.sr1a = value;
		break;
	case 0x1b: // Not documented
		state.sequencer.sr1b = value;
		break;

	default:
		FAILURE_1(NotImplemented, "io write 3c5: index 0x%02x unhandled",
			(unsigned)state.sequencer.index);
	}
}

/**
 * Write to VGA DAC Pixel Mask register (0x3c6)
 *
 * The pixel inputs (R, G and B) are anded with this value. Set to FFh
 * for normal operation.
 **/
void CS3Trio64::write_b_3c6(u8 value)
{
	if (state.CRTC.reg[0x33] & 0x10)
		return;

	state.pel.mask = value;
#if DEBUG_VGA
	if (state.pel.mask != 0xff)
		printf("io write 3c6: PEL mask=0x%02x != 0xFF   \n", value);
#endif

	// state.pel.mask should be and'd with final value before
	// indexing into color register state.pel.data[]


}

u8 CS3Trio64::read_b_3c6()
{
	return state.pel.mask;
}

/**
 * Write VGA DAC Address Read Mode register (0x3c7)
 *
 * The Color Registers in the standard VGA provide a mapping between the
 * palette of between 2 and 256 colors to a larger 18-bit color space.
 * This capability allows for efficient use of video memory while
 * providing greater flexibility in color choice. The standard VGA has
 * 256 palette entries containing six bits each of red, green, and blue
 * values. The palette RAM is accessed via a pair of address registers
 * and a data register.
 *
 * To write a palette entry, output the palette entry's index value to
 * the DAC Address Write Mode Register (0x3c8) then perform 3 writes to
 * the DAC Data Register (0x3c9), loading the red, green, then blue
 * values into the palette RAM. The internal write address automatically
 * advances allowing the next value's RGB values to be loaded without
 * having to reprogram the DAC Address Write Mode Register. This allows
 * the entire palette to be loaded in one write operation.
 *
 * To read a palette entry, output the palette entry's index to the DAC
 * Address Read Mode Register (0x3c7). Then perform 3 reads from the DAC
 * Data Register (0x3c9), loading the red, green, then blue values from
 * palette RAM. The internal read address automatically advances
 * allowing the next RGB values to be read without having to reprogram
 * the DAC Address Read Mode Register.
 *
 * The data values are 6-bits each.
 **/
void CS3Trio64::write_b_3c7(u8 value)
{
	state.pel.read_data_register = value;
	state.pel.read_data_cycle = 0;
	state.pel.dac_state = 0x03;
}

u8 CS3Trio64::read_b_3c7()
{
	return state.pel.dac_state;
}

/**
 * Write VGA DAC Address Write Mode register (0x3c8)
 *
 * For a description of DAC registers see CCirrus::write_b_3c7
 **/
void CS3Trio64::write_b_3c8(u8 value)
{
	// CR33 bit4: Lock Video DAC Writes
	if (state.CRTC.reg[0x33] & 0x10)
		return;

	state.pel.write_data_register = value;
	state.pel.write_data_cycle = 0;
	state.pel.dac_state = 0x00;
}

u8 CS3Trio64::read_b_3c8()
{
	return state.pel.write_data_register;
}

/**
 * Write VGA DAC Data register (0x3c9)
 *
 * For a description of DAC registers see CCirrus::write_b_3c7
 **/
void CS3Trio64::write_b_3c9(u8 value)
{
	// CR33 bit4: Lock Video DAC Writes
	if (state.CRTC.reg[0x33] & 0x10)
		return;

	switch (state.pel.write_data_cycle)
	{
	case 0:
		state.pel.data[state.pel.write_data_register].red = value;
		break;

	case 1:
		state.pel.data[state.pel.write_data_register].green = value;
		break;

	case 2:
	{
		state.pel.data[state.pel.write_data_register].blue = value;
		// Palette write complete. Check if value has changed
		bx_gui->lock();
		bool  changed = bx_gui->palette_change(state.pel.write_data_register,
			state.pel.data[state.pel.write_data_register].red << 2,
			state.pel.data[state.pel.write_data_register].green << 2,
			state.pel.data[state.pel.write_data_register].blue << 2);
		bx_gui->unlock();
		// If palette value has changed, redraw the screen.
		if (changed)
			redraw_area(0, 0, old_iWidth, old_iHeight);
	}
	break;
	}

	// Move on to next RGB component
	state.pel.write_data_cycle++;

	// palette entry complete, move on to next one
	if (state.pel.write_data_cycle >= 3)
	{

		//BX_INFO(("state.pel.data[%u] {r=%u, g=%u, b=%u}",
		//  (unsigned) state.pel.write_data_register,
		//  (unsigned) state.pel.data[state.pel.write_data_register].red,
		//  (unsigned) state.pel.data[state.pel.write_data_register].green,
		//  (unsigned) state.pel.data[state.pel.write_data_register].blue);
		state.pel.write_data_cycle = 0;
		state.pel.write_data_register++;
	}
}

/**
 * Write to VGA Graphics Controller Index Register (0x3ce)
 *
 * The Graphics Controller registers control how the system accesses video RAM.
 *
 * The Graphics registers are accessed in an indexed fashion. By writing a byte
 * to the Graphics Index Register (0x3ce) equal to the index of the particular
 * sub-register you wish to access, one can address the data pointed to by that
 * index by reading and writing the Graphics Data Register (0x3cf).
 *
 * Graphics registers:
 *   - Set/Reset register (index 0x00)
 *   - Enable Set/Reset register (index 0x01)
 *   - Color Compare register (index 0x02)
 *   - Data Rotate register (index 0x03)
 *   - Read Map Select register (index 0x04)
 *   - Graphics Mode register (index 0x05)
 *   - Miscellaneous Graphics register (index 0x06)
 *   - Color Don't Care register (index 0x07)
 *   - Bit Mask register (index 0x08)
 *   .
 *
 * \code
 * Set/Reset register (index 0x00)
 * +-------+-------+
 * |       |3 2 1 0|
 * +-------+-------+
 *             ^
 *             +- 0..3: Set/Reset: Bits 3-0 of this field represent planes 3-0 of
 *                      the VGA display memory. This field is used by Write Mode
 *                      0 and Write Mode 3 (See the Write Mode field.) In Write
 *                      Mode 0, if the corresponding bit in the Enable Set/Reset
 *                      field is set, and in Write Mode 3 regardless of the Enable
 *                      Set/Reset field, the value of the bit in this field is
 *                      expanded to 8 bits and substituted for the data of the
 *                      respective plane and passed to the next stage in the
 *                      graphics pipeline, which for Write Mode 0 is the Logical
 *                      Operation unit and for Write Mode 3 is the Bit Mask unit.
 *
 * Enable Set/Reset Register (index 0x01)
 * +-------+-------+
 * |       |3 2 1 0|
 * +-------+-------+
 *             ^
 *             +- 0..3: Enable Set/Reset: Bits 3-0 of this field represent planes
 *                      3-0 of the VGA display memory. This field is used in Write
 *                      Mode 0 (See the Write Mode field) to select whether data
 *                      for each plane is derived from host data or from expansion
 *                      of the respective bit in the Set/Reset field.
 *
 * Color Compare Register (index 0x02)
 * +-------+-------+
 * |       |3 2 1 0|
 * +-------+-------+
 *             ^
 *             +- 0..3: Color Compare: Bits 3-0 of this field represent planes 3-0
 *                      of the VGA display memory. This field holds a reference
 *                      color that is used by Read Mode 1 (See the Read Mode field.)
 *                      Read Mode 1 returns the result of the comparison between
 *                      this value and a location of display memory, modified by
 *                      the Color Don't Care field.
 *
 * Data Rotate Register (index 0x03)
 * +-----+---+-----+
 * |     |4 3|2 1 0|
 * +-----+---+-----+
 *         ^    ^
 *         |    +- 0..2: Rotate Count:
 *         |             This field is used in Write Mode 0 and Write Mode 3 (See
 *         |             the Write Mode field.) In these modes, the host data is
 *         |             rotated to the right by the value specified by the value of
 *         |             this field. A rotation operation consists of moving bits
 *         |             7-1 right one position to bits 6-0, simultaneously
 *         |             wrapping bit 0 around to bit 7, and is repeated the number
 *         |             of times specified by this field.
 *         +------ 3..4: Logical Operation:
 *                       This field is used in Write Mode 0 and Write Mode 2 (See
 *                       the Write Mode field.) The logical operation stage of the
 *                       graphics pipeline is 32 bits wide (1 byte * 4 planes) and
 *                       performs the operations on its inputs from the previous
 *                       stage in the graphics pipeline and the latch register. The
 *                       latch register remains unchanged and the result is passed
 *                       on to the next stage in the pipeline. The results based on
 *                       the value of this field are:
 *                         00: Result is input from previous stage unmodified.
 *                         01: Result is input from previous stage logical ANDed
 *                             with latch register.
 *                         10: Result is input from previous stage logical ORed with
 *                             latch register.
 *                         11: Result is input from previous stage logical XORed
 *                             with latch register.
 *
 * Read Map Select register (index 0x04)
 * +-----------+---+
 * |           |1 0|
 * +-----------+---+
 *               ^
 *               +- 0..1: Read Map Select: The value of this field is used in Read
 *                        Mode 0 (see the Read Mode field) to specify the display
 *                        memory plane to transfer data from. Due to the arrangement
 *                        of video memory, this field must be modified four times to
 *                        read one or more pixels values in the planar video modes.
 *
 * Graphics Mode Register (index 0x05)
 * +-+-+-+-+-+-+---+
 * | |6|5|4|3| |1 0|
 * +-+-+-+-+-+-+---+
 *    ^ ^ ^ ^    ^
 *    | | | |    +- 0..1: Write Mode
 *    | | | |             This field selects between four write modes, simply known
 *    | | | |             as Write Modes 0-3, based upon the value of this field:
 *    | | | |               00: Write Mode 0: In this mode, the host data is first
 *    | | | |                   rotated as per the Rotate Count field, then the
 *    | | | |                   Enable Set/Reset mechanism selects data from this or
 *    | | | |                   the Set/Reset field. Then the selected Logical
 *    | | | |                   Operation is performed on the resulting data and the
 *    | | | |                   data in the latch register. Then the Bit Mask field
 *    | | | |                   is used to select which bits come from the resulting
 *    | | | |                   data and which come from the latch register. Finally,
 *    | | | |                   only the bit planes enabled by the Memory Plane Write
 *    | | | |                   Enable field are written to memory.
 *    | | | |               01: Write Mode 1: In this mode, data is transferred directly
 *    | | | |                   from the 32 bit latch register to display memory,
 *    | | | |                   affected only by the Memory Plane Write Enable field.
 *    | | | |                   The host data is not used in this mode.
 *    | | | |               10: Write Mode 2: In this mode, the bits 3-0 of the host
 *    | | | |                   data are replicated across all 8 bits of their
 *    | | | |                   respective planes. Then the selected Logical Operation
 *    | | | |                   is performed on the resulting data and the data in the
 *    | | | |                   latch register. Then the Bit Mask field is used to
 *    | | | |                   select which bits come from the resulting data and which
 *    | | | |                   come from the latch register. Finally, only the bit
 *    | | | |                   planes enabled by the Memory Plane Write Enable field
 *    | | | |                   are written to memory.
 *    | | | |               11: Write Mode 3: In this mode, the data in the Set/Reset
 *    | | | |                   field is used as if the Enable Set/Reset field were set
 *    | | | |                   to 1111b. Then the host data is first rotated as per the
 *    | | | |                   Rotate Count field, then logical ANDed with the value of
 *    | | | |                   the Bit Mask field. The resulting value is used on the
 *    | | | |                   data obtained from the Set/Reset field in the same way
 *    | | | |                   that the Bit Mask field would ordinarily be used. to
 *    | | | |                   select which bits come from the expansion of the
 *    | | | |                   Set/Reset field and which come from the latch register.
 *    | | | |                   Finally, only the bit planes enabled by the Memory Plane
 *    | | | |                   Write Enable field are written to memory.
 *    | | | +--------- 3: Read Mode:
 *    | | |               This field selects between two read modes, simply known as Read
 *    | | |               Mode 0, and Read Mode 1, based upon the value of this field:
 *    | | |                 0: Read Mode 0: In this mode, a byte from one of the four
 *    | | |                    planes is returned on read operations. The plane from
 *    | | |                    which the data is returned is determined by the value of
 *    | | |                    the Read Map Select field.
 *    | | |                 1: Read Mode 1: In this mode, a comparison is made between
 *    | | |                    display memory and a reference color defined by the Color
 *    | | |                    Compare field. Bit planes not set in the Color Don't Care
 *    | | |                    field then the corresponding color plane is not considered
 *    | | |                    in the comparison. Each bit in the returned result
 *    | | |                    represents one comparison between the reference color, with
 *    | | |                    the bit being set if the comparison is true.
 *    | | +----------- 4: Host Odd/Even Memory Read Addressing Enable:
 *    | |                   0: Selects the standard addressing mode.
 *    | |                   1: Selects the odd/even addressing mode used by the IBM CGA
 *    | |                      Adapter.
 *    | |                 Normally, the value here follows the value of Memory Mode
 *    | |                 register bit 2 in the sequencer."
 *    | +------------- 5: Shift Register Interleave Mode:
 *    |                     1: Directs the shift registers in the graphics controller to
 *    |                        format the serial data stream with even-numbered bits from
 *    |                        both maps on even-numbered maps, and odd-numbered bits from
 *    |                        both maps on the odd-numbered maps. This bit is used for
 *    |                        modes 4 and 5.
 *    +--------------- 6: 256-Color Shift Mode:
 *                          0: Allows bit 5 to control the loading of the shift registers.
 *                          1: Causes the shift registers to be loaded in a manner that
 *                             supports the 256-color mode.
 *
 * Miscellaneous Graphics register (index 0x06)
 * +-------+---+-+-+
 * |       |3 2|1|0|
 * +-------+---+-+-+
 *           ^  ^ ^
 *           |  | +- 0: Alphanumeric Mode Disable:
 *           |  |       This bit controls alphanumeric mode addressing.
 *           |  |         0: Text mode.
 *           |  |         1: Graphics modes, disables character generator latches.
 *           |  +--- 1: Chain Odd/Even Enable
 *           |            1: Directs the system address bit, A0, to be replaced by a
 *           |               higher-order bit. The odd map is then selected when A0 is 1,
 *           |               and the even map when A0 is 0.
 *           +--- 2..3: Memory Map Select
 *                      This field specifies the range of host memory addresses that is
 *                      decoded by the VGA hardware and mapped into display memory
 *                      accesses. The values of this field and their corresponding host
 *                      memory ranges are:
 *                        00: A0000h-BFFFFh (128K region)
 *                        01: A0000h-AFFFFh (64K region)
 *                        10: B0000h-B7FFFh (32K region)
 *                        11: B8000h-BFFFFh (32K region)
 *
 * Color Don't Care register (index 0x07)
 * +-------+-------+
 * |       |3 2 1 0|
 * +-------+-------+
 *             ^
 *             +- 0..3: Color Don't Care: Bits 3-0 of this field represent planes
 *                      3-0 of the VGA display memory. This field selects the
 *                      planes that are used in the comparisons made by Read Mode
 *                      1 (See the Read Mode field.) Read Mode 1 returns the
 *                      result of the comparison between the value of the Color
 *                      Compare field and a location of display memory. If a bit
 *                      in this field is set, then the corresponding display
 *                      plane is considered in the comparison. If it is not set,
 *                      then that plane is ignored for the results of the
 *                      comparison.
 *
 * Bit Mask register (index 0x08)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 *         +----- 0..7: Bit Mask: This field is used in Write Modes 0, 2, and 3
 *                      (See the Write Mode field.) It it is applied to one byte
 *                      of data in all four display planes. If a bit is set,
 *                      then the value of corresponding bit from the previous
 *                      stage in the graphics pipeline is selected; otherwise
 *                      the value of the corresponding bit in the latch register
 *                      is used instead. In Write Mode 3, the incoming data byte,
 *                      after being rotated is logical ANDed with this byte and
 *                      the resulting value is used in the same way this field
 *                      would normally be used by itself.
 * \endcode
 **/
void CS3Trio64::write_b_3ce(u8 value)
{
#if DEBUG_VGA
	if (value > 0x08)  /* ??? */
		printf("io write: 3ce: value > 8   \n");
#endif
	state.graphics_ctrl.index = value;
}

/**
 * Write to VGA Graphics Controller Data Register (0x3cf)
 *
 * For a description of the Graphics registers, see CCirrus::write_b_3ce
 **/
void CS3Trio64::write_b_3cf(u8 value)
{
	u8    prev_memory_mapping;
	bool  prev_graphics_alpha;
	bool  prev_chain_odd_even;

	/* Graphics Controller Registers 00..08 */
	switch (state.graphics_ctrl.index)
	{
	case 0:           /* Set/Reset */
		state.graphics_ctrl.set_reset = value & 0x0f;
		break;

	case 1:           /* Enable Set/Reset */
		state.graphics_ctrl.enable_set_reset = value & 0x0f;
		break;

	case 2:           /* Color Compare */
		state.graphics_ctrl.color_compare = value & 0x0f;
		break;

	case 3:           /* Data Rotate */
		state.graphics_ctrl.data_rotate = value & 0x07;

		/* ??? is this bits 3..4 or 4..5 */
		state.graphics_ctrl.raster_op = (value >> 3) & 0x03;  /* ??? */
		break;

	case 4:     /* Read Map Select */
		state.graphics_ctrl.read_map_select = value & 0x03;
#if DEBUG_VGA_NOISY
		printf("io write to 03cf = %02x (RMS)   \n", (unsigned)value);
#endif
		break;

	case 5:     /* Mode */
		state.graphics_ctrl.write_mode = value & 0x03;
		state.graphics_ctrl.read_mode = (value >> 3) & 0x01;
		state.graphics_ctrl.odd_even = (value >> 4) & 0x01;
		state.graphics_ctrl.shift_reg = (value >> 5) & 0x03;

#if DEBUG_VGA_NOISY
		if (state.graphics_ctrl.odd_even)
			printf("io write: 3cf: reg 05: value = %02xh   \n", (unsigned)value);
		if (state.graphics_ctrl.shift_reg)
			printf("io write: 3cf: reg 05: value = %02xh   \n", (unsigned)value);
#endif
		break;

	case 6:     /* Miscellaneous */
		prev_graphics_alpha = state.graphics_ctrl.graphics_alpha;
		prev_chain_odd_even = state.graphics_ctrl.chain_odd_even;
		prev_memory_mapping = state.graphics_ctrl.memory_mapping;

		state.graphics_ctrl.graphics_alpha = value & 0x01;
		state.graphics_ctrl.chain_odd_even = (value >> 1) & 0x01;
		state.graphics_ctrl.memory_mapping = (value >> 2) & 0x03;
#if DEBUG_VGA_NOISY
		printf("memory_mapping set to %u   \n",
			(unsigned)state.graphics_ctrl.memory_mapping);
		printf("graphics mode set to %u   \n",
			(unsigned)state.graphics_ctrl.graphics_alpha);
		printf("odd_even mode set to %u   \n",
			(unsigned)state.graphics_ctrl.odd_even);
		printf("io write: 3cf: reg 06: value = %02xh   \n", (unsigned)value);
#endif
		if (prev_memory_mapping != state.graphics_ctrl.memory_mapping)
		{
			redraw_area(0, 0, old_iWidth, old_iHeight);
		}

		if (prev_graphics_alpha != state.graphics_ctrl.graphics_alpha)
		{
			redraw_area(0, 0, old_iWidth, old_iHeight);
			old_iHeight = 0;
		}
		break;

	case 7:     /* Color Don't Care */
		state.graphics_ctrl.color_dont_care = value & 0x0f;
		break;

	case 8:     /* Bit Mask */
		state.graphics_ctrl.bitmask = value;
		break;

	default:

		/* ??? */
		FAILURE_1(NotImplemented, "io write: 3cf: index %u unhandled",
			(unsigned)state.graphics_ctrl.index);
	}
}

/**
 * Write to VGA CRTC Index Register (0x3b4 or 0x3d4)
 *
 * The VGA CRTC Registers control how the video is output to the display.
 *
 * The CRTC registers are accessed in an indexed fashion. By writing a byte
 * to the CRTC Index Register (0x3d4) equal to the index of the particular
 * sub-register you wish to access, one can address the data pointed to by that
 * index by reading and writing the CRTC Data Register (0x3d5).
 *
 * CRTC registers:
 *   - Horizontal Total Register (index 0x00)
 *   - End Horizontal Display Register (index 0x01)
 *   - Start Horizontal Blanking Register (index 0x02)
 *   - End Horizontal Blanking Register (index 0x03)
 *   - Start Horizontal Retrace Register (index 0x04)
 *   - End Horizontal Retrace Register (index 0x05)
 *   - Vertical Total Register (index 0x06)
 *   - Overflow Register (index 0x07)
 *   - Preset Row Scan Register (index 0x08)
 *   - Maximum Scan Line Register (index 0x09)
 *   - Cursor Start Register (index 0x0a)
 *   - Cursor End Register (index 0x0b)
 *   - Start Address High Register (index 0x0c)
 *   - Start Address Low Register (index 0x0d)
 *   - Cursor Location High Register (index 0x0e)
 *   - Cursor Location Low Register (index 0x0f)
 *   - Vertical Retrace Start Register (index 0x10)
 *   - Vertical Retrace End Register (index 0x11)
 *   - Vertical Display End Register (index 0x12)
 *   - Offset Register (index 0x13)
 *   - Underline Location Register (index 0x14)
 *   - Start Vertical Blanking Register (index 0x15)
 *   - End Vertical Blanking (index 0x16)
 *   - CRTC Mode Control Register (index 0x17)
 *   - Line Compare Register (index 0x18)
 *   .
 *
 * \code
 * Horizontal Total register (index 0x00)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Horizontal Total:
 * This field is used to specify the number of character clocks per scan line.
 * This field, along with the dot rate selected, controls the horizontal
 * refresh rate of the VGA by specifying the amount of time one scan line
 * takes.  This field is not programmed with the actual number of character
 * clocks, however. Due to timing factors of the VGA hardware (which, for
 * compatibility purposes has been emulated by VGA compatible chipsets), the
 * actual horizontal total is 5 character clocks more than the value stored in
 * this field, thus one needs to subtract 5 from the actual horizontal total
 * value desired before programming it into this register.
 *
 * End Horizontal Display register (index 0x01)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: End Horizontal Display:
 * This field is used to control the point that the sequencer stops outputting
 * pixel values from display memory, and sequences the pixel value specified by
 * the Overscan Palette Index field for the remainder of the scan line. The
 * overscan begins the character clock after the the value programmed into this
 * field. This register should be programmed with the number of character
 * clocks in the active display - 1. Note that the active display may be
 * affected by the Display Enable Skew field.
 *
 * Start Horizontal Blanking register (index 0x02)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Start Horizontal Blanking:
 * This field is used to specify the character clock at which the horizontal
 * blanking period begins.  During the horizontal blanking period, the VGA
 * hardware forces the DAC into a blanking state, where all of the intensities
 * output are at minimum value, no matter what color information the attribute
 * controller is sending to the DAC.  This field works in conjunction with the
 * End Horizontal Blanking field to specify the horizontal blanking period.
 * Note that the horizontal blanking can be programmed to appear anywhere within
 * the scan line, as well as being programmed to a value greater than the
 * Horizontal Total field preventing the horizontal blanking from occurring at
 * all.
 *
 * End Horizontal Blanking register (index 0x03)
 * +-+---+---------+
 * |7|6 5|4 3 2 1 0|
 * +-+---+---------+
 *  ^  ^      ^
 *  |  |      +-- 0..4: End Horizontal Blanking:
 *  |  |                Contains bits 4-0 of the End Horizontal Blanking field
 *  |  |                which specifies the end of the horizontal blanking
 *  |  |                period.  Bit 5 is located in bit 7 of the End Horizontal
 *  |  |                Retrace register (index 0x05). After the period has
 *  |  |                begun as specified by the Start Horizontal Blanking
 *  |  |                field, the 6-bit value of this field is compared against
 *  |  |                the lower 6 bits of the character clock. When a match
 *  |  |                occurs, the horizontal blanking signal is disabled. This
 *  |  |                provides from 1 to 64 character clocks although some
 *  |  |                implementations may match in the character clock
 *  |  |                specified by the Start Horizontal Blanking field, in which
 *  |  |                case the range is 0 to 63.  Note that if blanking extends
 *  |  |                past the end of the scan line, it will end on the first
 *  |  |                match of this field on the next scan line.
 *  |  +--------- 5..6: Display Enable Skew:
 *  |                   This field affects the timings of the display enable
 *  |                   circuitry in the VGA. The value of this field is the number
 *  |                   of character clocks that the display enable "signal" is
 *  |                   delayed. In all known VGA cards, this field is always
 *  |                   programmed to 0. Programming it to non-zero values results
 *  |                   in the overscan being displayed over the number of
 *  |                   characters programmed into this field at the beginning of
 *  |                   the scan line, as well as the end of the active display
 *  |                   being shifted the number of characters programmed into this
 *  |                   field. The characters that extend past the normal end of the
 *  |                   active display can be garbled in certain circumstances that
 *  |                   is dependent on the particular VGA implementation. According
 *  |                   to documentation from IBM, "This skew control is needed to
 *  |                   provide sufficient time for the CRT controller to read a
 *  |                   character and attribute code from the video buffer, to gain
 *  |                   access to the character generator, and go through the
 *  |                   Horizontal PEL Panning register in the attribute controller.
 *  |                   Each access requires the 'display enable' signal to be
 *  |                   skewed one character clock so that the video output is
 *  |                   synchronized with the horizontal and vertical retrace
 *  |                   signals." as well as "Note: Character skew is not adjustable
 *  |                   on the Type 2 video and the bits are ignored; however,
 *  |                   programs should set these bits for the appropriate skew to
 *  |                   maintain compatibility."  This may be required for some early
 *  |                   IBM VGA implementations or may be simply an unused "feature"
 *  |                   carried over along with its register description from the IBM
 *  |                   EGA implementations that require the use of this field.
 *  +--------------- 7: Enable Vertical Retrace Access:
 *                      This field was used in the IBM EGA to provide access to the
 *                      light pen input values as the light pen registers were mapped
 *                      over CRTC indexes 10h-11h. The VGA lacks capability for light
 *                      pen input, thus this field is normally forced to 1 (although
 *                      always writing it as 1 might be a good idea for compatibility),
 *                      which in the EGA would enable access to the vertical retrace
 *                      fields instead of the light pen fields.
 *
 * Start Horizontal Retrace register (index 0x04)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Start Horizontal Retrace:
 * This field specifies the character clock at which the VGA begins sending the
 * horizontal synchronization pulse to the display which signals the monitor to retrace
 * back to the left side of the screen. The end of this pulse is controlled by the End
 * Horizontal Retrace field. This pulse may appear anywhere in the scan line, as well
 * as set to a position beyond the Horizontal Total field which effectively disables
 * the horizontal synchronization pulse.
 *
 * End Horizontal Retrace register (index 0x05)
 * +-+---+---------+
 * |7|6 5|4 3 2 1 0|
 * +-+---+---------+
 *  ^  ^      ^
 *  |  |      +-- 0..4: End Horizontal Retrace:
 *  |  |                This field specifies the end of the horizontal retrace period,
 *  |  |                which begins at the character clock specified in the Start
 *  |  |                Horizontal Retrace field.  The horizontal retrace signal is
 *  |  |                enabled until the lower 5 bits of the character counter match
 *  |  |                the 5 bits of this field.  This provides for a horizontal
 *  |  |                retrace period from 1 to 32 character clocks.  Note that some
 *  |  |                implementations may match immediately instead of 32 clocks
 *  |  |                away, making the effective range 0 to 31 character clocks.
 *  |  +--------- 5..6: Horizontal Retrace Skew:
 *  |                   This field delays the start of the horizontal retrace period
 *  |                   by the number of character clocks equal to the value of this
 *  |                   field.  From observation, this field is programmed to 0, with
 *  |                   the exception of the 40 column text modes where this field is
 *  |                   set to 1.  The VGA hardware simply acts as if this value is
 *  |                   added to the Start Horizontal Retrace field. According to IBM
 *  |                   documentation, "For certain modes, the 'horizontal retrace'
 *  |                   signal takes up the entire blanking interval. Some internal
 *  |                   timings are generated by the falling edge of the 'horizontal
 *  |                   retrace' signal. To ensure that the signals are latched
 *  |                   properly, the 'retrace' signal is started before the end of
 *  |                   the 'display enable' signal and then skewed several character
 *  |                   clock times to provide the proper screen centering." This does
 *  |                   not appear to be the case, leading me to believe this is yet
 *  |                   another holdout from the IBM EGA implementations that do
 *  |                   require the use of this field.
 *  +--------------- 7: End Horizontal Blanking (bit 5):
 *                      This contains bit 5 of the End Horizontal Blanking field in the
 *                      End Horizontal Blanking register (index 0x03).
 *
 * Vertical Total register (index 0x06)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Vertical Total
 * This contains the lower 8 bits of the Vertical Total field. Bits 9-8 of this field are
 * located in the Overflow Register (index 0x07). This field determines the number of
 * scanlines in the active display and thus the length of each vertical retrace. This
 * field contains the value of the scanline counter at the beginning of the last
 * scanline in the vertical period.
 *
 * Overflow register (index 0x07)
 * +-+-+-+-+-+-+-+-+
 * |7|6|5|4|3|2|1|0|
 * +-+-+-+-+-+-+-+-+
 *  ^ ^ ^ ^ ^ ^ ^ ^
 *  | | +-|-|-|-|-+- 0,5: Bit 8,9 of Vertical Total (index 0x06)
 *  | +---|-|-|-+--- 1,6: Bit 8,9 of Vertical Display End (index 0x12)
 *  +-----|-|-+----- 2,7: Bit 8,9 of Vertical Retrace Start (index 0x10)
 *        | +--------- 3: Bit 8 of Start Vertical Blanking (index 0x15)
 *        +----------- 4: Bit 8 of Line Compare (index 0x18)
 *
 * Preset Row Scan register (index 0x08)
 * +-+---+---------+
 * | |6 5|4 3 2 1 0|
 * +-+---+---------+
 *     ^      ^
 *     |      +-- 0..4: Preset Row Scan:
 *     |                This field is used when using text mode or any mode with a non-zero
 *     |                Maximum Scan Line field (index 0x09) to provide for more precise
 *     |                vertical scrolling than the Start Address Register provides. The
 *     |                value of this field specifies how many scan lines to scroll the
 *     |                display upwards. Valid values range from 0 to the value of the
 *     |                Maximum Scan Line field. Invalid values may cause undesired effects
 *     |                and seem to be dependent upon the particular VGA implementation.
 *     +--------- 5..6: Byte Panning:
 *                      The value of this field is added to the Start Address Register when
 *                      calculating the display memory address for the upper left hand pixel
 *                      or character of the screen. This allows for a maximum shift of 15,
 *                      31, or 35 pixels without having to reprogram the Start Address
 *                      Register.
 *
 * Maximum Scan Line register (index 0x09)
 * +-+-+-+---------+
 * |7|6|5|4 3 2 1 0|
 * +-+-+-+---------+
 *  ^ ^ ^     ^
 *  | | |     +-- 0..4: Maximum Scan Line:
 *  | | |               In text modes, this field is programmed with the character height - 1
 *  | | |               (scan line numbers are zero based.) In graphics modes, a non-zero
 *  | | |               value in this field will cause each scan line to be repeated by the
 *  | | |               value of this field + 1.
 *  | | +----------- 5: Bit 9 of Start Vertical Blanking (index 0x15)
 *  | +------------- 6: Bit 9 of Line Compare (index 0x18)
 *  +--------------- 7: Scan Doubling:
 *                      When this bit is set to 1, 200-scan-line video data is converted to
 *                      400-scan-line output. To do this, the clock in the row scan counter is
 *                      divided by 2, which allows the 200-line modes to be displayed as 400
 *                      lines on the display (this is called double scanning; each line is
 *                      displayed twice). When this bit is set to 0, the clock to the row scan
 *                      counter is equal to the horizontal scan rate.
 *
 * Cursor Start Register (index 0x0a)
 * +---+-+---------+
 * |   |5|4 3 2 1 0|
 * +---+-+---------+
 *      ^     ^
 *      |     +-- 0..4: Cursor Scan Line Start:
 *      |               This field controls the appearance of the text-mode cursor by
 *      |               specifying the scan line location within a character cell at which
 *      |               the cursor should begin, with the top-most scan line in a character
 *      |               cell being 0 and the bottom being with the value of the Maximum Scan
 *      |               Line field.
 *      +------------5: Cursor Disable:
 *                      This field controls whether or not the text-mode cursor is displayed:
 *                        0: Cursor Enabled.
 *                        1: Cursor Disabled.
 *
 * Cursor End Register (index 0x0b)
 * +-+---+---------+
 * | |6 5|4 3 2 1 0|
 * +-+---+---------+
 *     ^      ^
 *     |      +-- 0..4: Cursor Scan Line End:
 *     |                This field controls the appearance of the text-mode cursor by
 *     |                specifying the scan line location within a character cell at which
 *     |                the cursor should end, with the top-most scan line in a character
 *     |                cell being 0 and the bottom being with the value of the Maximum Scan
 *     |                Line field. If this field is less than the Cursor Scan Line Start
 *     |                field, the cursor is not drawn. Some graphics adapters, such as the
 *     |                IBM EGA display a split-block cursor instead.
 *     +------------ 5: Cursor Skew:
 *                      This field was necessary in the EGA to synchronize the cursor with
 *                      internal timing. In the VGA it basically is added to the cursor
 *                      location. In some cases when this value is non-zero and the cursor is
 *                      near the left or right edge of the screen, the cursor will not appear
 *                      at all, or a second cursor above and to the left of the actual one may
 *                      appear. This behavior may not be the same on all VGA compatible adapter
 *                      cards.
 *
 * Start Address High register (index 0x0c)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Bits 8..15 of the Start Address.
 * Bits 0..7 are in the Start Address Low register (index 0x0d). The Start Address field
 * specifies the display memory address of the upper left pixel or character of the screen.
 * Because the standard VGA has a maximum of 256K of memory, and memory is accessed 32 bits at
 * a time, this 16-bit field is sufficient to allow the screen to start at any memory address.
 * Normally this field is programmed to 0h, except when using virtual resolutions, paging,
 * and/or split-screen operation. Note that the VGA display will wrap around in display memory
 * if the starting address is too high. (This may or may not be desirable, depending on your
 * intentions.)
 *
 * Start Address Low register (index 0x0d)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Bits 0..7 of the Start Address. See Start Address High register (index 0x0c)
 *
 * Cursor Location High register (index 0x0e)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Bits 8..15 of the Cursor Location.
 * Bits 0..7 are in the Cursor Location Low register (index 0x0d). When the VGA hardware is
 * displaying text mode and the text-mode cursor is enabled, the hardware compares the address of
 * the character currently being displayed with sum of value of this field and the sum of the
 * Cursor Skew field. If the values equal then the scan lines in that character specified by the
 * Cursor Scan Line Start field and the Cursor Scan Line End field are replaced with the
 * foreground color.
 *
 * Cursor Location Low register (index 0x0f)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Bits 0..7 of the Cursor Location. See Cursor Location High register (index 0x0f)
 *
 * Vertical Retrace Start register (index 0x10)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Bits 0..7 of Vertical Retrace Start
 * Bits 8 and 9 are in the Overflow Register (index 0x07). This field controls the start of the
 * vertical retrace pulse which signals the display to move up to the beginning of the active
 * display. This field contains the value of the vertical scanline counter at the beginning of the
 * first scanline where the vertical retrace signal is asserted.
 *
 * Vertical Retrace End register (index 0x11)
 * +-+-+---+-------+
 * |7|6|5|4|3 2 1 0|
 * +-+-+---+-------+
 *  ^ ^ ^ ^   ^
 *  | | | |   +-- 0..3: Vertical Retrace End:
 *  | | | |             This field determines the end of the vertical retrace pulse, and thus its
 *  | | | |             length. This field contains the lower four bits of the vertical scanline
 *  | | | |             counter at the beginning of the scanline immediately after the last
 *  | | | |             scanline where the vertical retrace signal is asserted.
 *  | | | +--------- 4: End Vertical Interrupt
 *  | | +----------- 5: Enable Vertical Interrupt
 *  | +------------- 6: Memory Refresh Bandwidth:
 *  |                   Nearly all video chipsets include a few registers that control memory, bus,
 *  |                   or other timings not directly related to the output of the video card. Most
 *  |                   VGA/SVGA implementations ignore the value of this field; however, in the
 *  |                   least, IBM VGA adapters do utilize it and thus for compatibility with these
 *  |                   chipsets this field should be programmed. This register is used in the IBM
 *  |                   VGA hardware to control the number of DRAM refresh cycles per scan line.
 *  |                   The three refresh cycles per scanline is appropriate for the IBM VGA
 *  |                   horizontal frequency of approximately 31.5 kHz. For horizontal frequencies
 *  |                   greater than this, this setting will work as the DRAM will be refreshed more
 *  |                   often. However, refreshing not often enough for the DRAM can cause memory
 *  |                   loss. Thus at some point slower than 31.5 kHz the five refresh cycle setting
 *  |                   should be used. At which particular point this should occur, would require
 *  |                   better knowledge of the IBM VGA's schematics than I have available.
 *  |                   According to IBM documentation, "Selecting five refresh cycles allows use of
 *  |                   the VGA chip with 15.75 kHz displays." which isn't really enough to go by
 *  |                   unless the mode you are defining has a 15.75 kHz horizontal frequency.
 *  +--------------- 7: CRTC Registers Protect Enable:
 *                      This field is used to protect the video timing registers from being changed
 *                      by programs written for earlier graphics chipsets that attempt to program
 *                      these registers with values unsuitable for VGA timings. When this field is
 *                      set to 1, the CRTC register indexes 00h-07h ignore write access, with the
 *                      exception of bit 4 of the Overflow Register, which holds bit 8 of the Line
 *                      Compare field.
 *
 * Vertical Display End register (index 0x12)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Bits 0..7 of Vertical Display End
 * Bits 8 and 9 are in the Overflow Register (index 0x07). This field contains the value of the
 * vertical scanline counter at the beggining of the scanline immediately after the last scanline of
 * active display.
 *
 * Offset register (index 0x13)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Offset:
 * This field specifies the address difference between consecutive scan lines or two lines of
 * characters. Beginning with the second scan line, the starting scan line is increased by twice the
 * value of this register multiplied by the current memory address size (byte = 1, word = 2,
 * double-word = 4) each line. For text modes the following equation is used:
 *       Offset = Width / ( MemoryAddressSize * 2 )
 * and in graphics mode, the following equation is used:
 *       Offset = Width / ( PixelsPerAddress * MemoryAddressSize * 2 )
 * where Width is the width in pixels of the screen. This register can be modified to provide for a
 * virtual resolution, in which case Width is the width is the width in pixels of the virtual screen.
 * PixelsPerAddress is the number of pixels stored in one display memory address, and
 * MemoryAddressSize is the current memory addressing size.
 *
 * Underline Location register (index 0x14)
 * +-+-+-+---------+
 * | |6|5|4 3 2 1 0|
 * +-+-+-+---------+
 *    ^ ^     ^
 *    | |     +-- 0..4: Underline Location
 *    | |               These bits specify the horizontal scan line of a character row on which an
 *    | |               underline occurs. The value programmed is the scan line desired minus 1.
 *    | +----------- 5: Divide Memory Address Clock by 4:
 *    |                   1: The memory-address counter is clocked with the character clock divided
 *    |                      by 4, which is used when doubleword addresses are used.
 *    +------------- 6: Double-Word Addressing:
 *                        1: Memory addresses are doubleword addresses. See the description of the
 *                           word/byte mode bit (bit 6) in the CRT Mode Control Register (index 0x17)
 *
 * Start Vertical Blanking register (index 0x15)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Bits 0..7 of Start Vertical Blanking
 * Bit 8 is in the Overflow Register (index 0x07), and bit 9 is in the Maximum Scan Line register
 * (index 0x09). This field determines when the vertical blanking period begins, and contains the
 * value of the vertical scanline counter at the beginning of the first vertical scanline of
 * blanking.
 *
 * End Vertical Blanking register (index 0x16)
 * +-+-------------+
 * | |6 5 4 3 2 1 0|
 * +-+-------------+
 *          ^
 *          +---- 0..6: End Vertical Blanking:
 *                      This field determines when the vertical blanking period ends, and contains the
 *                      value of the vertical scanline counter at the beginning of the vertical
 *                      scanline immediately after the last scanline of blanking.
 *
 * CRTC Mode Control Register (index 0x17)
 * +-+-+-+-+-+-+-+-+
 * |7|6|5| |3|2|1|0|
 * +-+-+-+-+-+-+-+-+
 *  ^ ^ ^   ^ ^ ^ ^
 *  | | |   | | | +- 0: Map Display Address 13:
 *  | | |   | | |       This bit selects the source of bit 13 of the output multiplexer:
 *  | | |   | | |         0: Bit 0 of the row scan counter is the source.
 *  | | |   | | |         1: Bit 13 of the address counter is the source.
 *  | | |   | | |       The CRT controller used on the IBM Color/Graphics Adapter was capable of
 *  | | |   | | |       using 128 horizontal scan-line addresses. For the VGA to obtain 640-by-200
 *  | | |   | | |       graphics resolution, the CRT controller is programmed for 100 horizontal
 *  | | |   | | |       scan lines with two scan-line addresses per character row. Row scan address
 *  | | |   | | |       bit 0 becomes the most-significant address bit to the display buffer.
 *  | | |   | | |       Successive scan lines of the display image are displaced in 8KB of memory.
 *  | | |   | | |       This bit allows compatibility with the graphics modes of earlier adapters.
 *  | | |   | | +--- 1: Map Display Address 14:
 *  | | |   | |         This bit selects the source of bit 14 of the output multiplexer:
 *  | | |   | |           0: Bit 1 of the row scan counter is the source.
 *  | | |   | |           1: Bit 14 of the address counter is the source.
 *  | | |   | +----- 2: Divide Scan Line clock by 2:
 *  | | |   |           This bit selects the clock that controls the vertical timing counter:
 *  | | |   |             0: The horizontal retrace clock.
 *  | | |   |             1: The horizontal retrace clock divided by 2.
 *  | | |   |           Dividing the clock effectively doubles the vertical resolution of the CRT
 *  | | |   |           controller. The vertical counter has a maximum resolution of 1024 scan lines
 *  | | |   |           because the vertical total value is 10-bits wide. If the vertical counter is
 *  | | |   |           clocked with the horizontal retrace divided by 2, the vertical resolution is
 *  | | |   |           doubled to 2048 scan lines."
 *  | | |   +------- 3: Divide Memory Address clock by 2:
 *  | | |               This bit selects the clock that controlls the address counter:
 *  | | |                 0: The character clock.
 *  | | |                 1: The character clock divided by 2.
 *  | | |               This bit is used to create either a byte or word refresh address for the
 *  | | |               display buffer.
 *  | | +----------- 5: Address Wrap Select:
 *  | |                 This bit selects the memory-address bit, bit MA 13 or MA 15, that appears on
 *  | |                 the output pin MA 0, in the word address mode. If the VGA is not in the word
 *  | |                 address mode, bit 0 from the address counter appears on the output pin, MA 0.
 *  | |                   0: Selects MA 13. Used in applications where only 64KB of video memory is
 *  | |                      present.
 *  | |                   1: Selects MA 15. In odd/even mode, this bit should be set to 1 because
 *  | |                      256KB of video memory is installed on the system board.
 *  | |                 This function maintains compatibility with the IBM Color/Graphics Monitor
 *  | |                 Adapter.
 *  | +------------- 6: Word/Byte Mode Select:
 *  |                     0: Selects the word address mode. The word mode shifts the memory-address
 *  |                        counter bits to the left by one bit; the most-significant bit of the
 *  |                        counter appears on the least-significant bit of the memory address
 *  |                        outputs.
 *  |                     1: Selects the byte address mode.
 *  |                   The doubleword bit in the Underline Location register (index 0x14) also
 *  |                   controls the addressing. When the doubleword bit is 0, the word/byte bit
 *  |                   selects the mode. When the doubleword bit is set to 1, the addressing is
 *  |                   shifted by two bits.
 *  +--------------- 7: Sync Enable:
 *                        0: Disables the horizontal and vertical retrace signals and forces them to
 *                           an inactive level.
 *                        1: Enables the horizontal and vertical retrace signals.
 *                      This bit does not reset any other registers or signal outputs.
 *
 * Line Compare register (index 0x18)
 * +---------------+
 * |7 6 5 4 3 2 1 0|
 * +---------------+
 *         ^
 * 0..7: Bits 0..7 of Line Compare
 * Bit 8 is in the Overflow Register (index 0x07), and bit 9 is in the Maximum Scan Line register
 * (index 0x09). The Line Compare field specifies the scan line at which a horizontal division can
 * occur, providing for split-screen operation. If no horizontal division is required, this field
 * should be set to 3FFh. When the scan line counter reaches the value in the Line Compare field, the
 * current scan line address is reset to 0 and the Preset Row Scan is presumed to be 0. If the Pixel
 * Panning Mode field is set to 1 then the Pixel Shift Count and Byte Panning fields are reset to 0
 * for the remainder of the display cycle.
 * \endcode
 **/
void CS3Trio64::write_b_3d4(u8 value)
{
	state.CRTC.address = value & 0x7f;
#if DEBUG_VGA_NOISY
	printf("VGA: 3d4 (SETTING CRTC INDEX) CRTC INDEX=0x%02x\n", state.CRTC.address);
#endif
}

/**
 * Write to VGA CRTC Data Register (0x3b5 or 0x3d5)
 *
 * For a description of CRTC Registers, see CCirrus::write_b_3d4.
 **/
void CS3Trio64::write_b_3d5(u8 value)
{
#if DEBUG_VGA_NOISY
	printf("VGA: 3d5 WRITE CRTC register=0x%02x BINARY VALUE=" PRINTF_BINARY_PATTERN_INT8 " HEX VALUE=0x%02x\n", state.CRTC.address, PRINTF_BYTE_TO_BINARY_INT8(value), value);
#endif
	if (state.CRTC.write_protect && (state.CRTC.address < 0x08))
	{
		if (state.CRTC.address == 0x07)
		{
			state.CRTC.reg[state.CRTC.address] &= ~0x10;
			state.CRTC.reg[state.CRTC.address] |= (value & 0x10);
			state.line_compare &= 0x2ff;
			if (state.CRTC.reg[0x07] & 0x10)
				state.line_compare |= 0x100;
			redraw_area(0, 0, old_iWidth, old_iHeight);
			return;
		}
		else
		{
			return;
		}
	}

	// ---- S3 unlock gating (match 86Box) ----
	if (state.CRTC.address >= 0x20 && state.CRTC.address < 0x40 &&
		state.CRTC.address != 0x36 && state.CRTC.address != 0x38 && state.CRTC.address != 0x39 &&
		((state.CRTC.reg[0x38] & 0xCC) != 0x48)) {
		return;
	}
	if (state.CRTC.address >= 0x40 && ((state.CRTC.reg[0x39] & 0xE0) != 0xA0)) {
		return;
	}
	if (state.CRTC.address == 0x36 && (state.CRTC.reg[0x39] != 0xA5)) {
		return;
	}
	if (state.CRTC.address == 0x68 && state.CRTC.reg[0x39] != 0xA5) { // per datasheet...
		return;
	}


	if (value != state.CRTC.reg[state.CRTC.address])
	{
		switch (state.CRTC.address)
		{
		case 0x00:
		case 0x01:
		case 0x02:
		case 0x03:
		case 0x04:
		case 0x05:
		{
			state.CRTC.reg[state.CRTC.address] = value;
			recompute_scanline_layout();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;
		}

		case 0x07:
			state.CRTC.reg[state.CRTC.address] = value;

			state.vertical_display_end &= 0xff;
			if (state.CRTC.reg[0x07] & 0x02)
				state.vertical_display_end |= 0x100;
			if (state.CRTC.reg[0x07] & 0x40)
				state.vertical_display_end |= 0x200;
			state.line_compare &= 0x2ff;
			if (state.CRTC.reg[0x07] & 0x10)
				state.line_compare |= 0x100;
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x08:
			state.CRTC.reg[state.CRTC.address] = value;
			// Vertical pel panning change
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x09:
			state.CRTC.reg[state.CRTC.address] = value;
			state.y_doublescan = ((value & 0x9f) > 0);
			state.line_compare &= 0x1ff;
			if (state.CRTC.reg[0x09] & 0x40)
				state.line_compare |= 0x200;
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x0A:
		case 0x0B:
		case 0x0E:
		case 0x0F:
			state.CRTC.reg[state.CRTC.address] = value;
			// Cursor size / location change
			state.vga_mem_updated = 1;
			break;

		case 0x0C:
		case 0x0D:
			state.CRTC.reg[state.CRTC.address] = value;
			// Start address change
			if (state.graphics_ctrl.graphics_alpha)
			{
				redraw_area(0, 0, old_iWidth, old_iHeight);
			}
			else
			{
				state.vga_mem_updated = 1;
			}
			compose_display_start();
			break;

		case 0x10: // Vertical Retrace Start
			state.CRTC.reg[0x10] = value;
			break;

		case 0x11:
			state.CRTC.reg[0x11] = value;
			// Disable VDE Protection Override, CR33 Bit 1
			state.CRTC.write_protect = (value & 0x80) && !(state.CRTC.reg[0x33] & 0x02);
			break;

		case 0x12:
			state.CRTC.reg[state.CRTC.address] = value;
			state.vertical_display_end &= 0x300;
			state.vertical_display_end |= state.CRTC.reg[0x12];
			break;

		case 0x13: // Offset (low)
			state.CRTC.reg[0x13] = value;
			recompute_line_offset();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x14: // Underline Location (bit6 = dword addressing)
			state.CRTC.reg[0x14] = value;
			recompute_line_offset();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x17: // Mode Control (bit6 = byte/word addressing selector)
			state.CRTC.reg[0x17] = value;
			recompute_line_offset();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x18:
			state.CRTC.reg[state.CRTC.address] = value;
			state.line_compare &= 0x300;
			state.line_compare |= state.CRTC.reg[0x18];
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x30: // read only...
			printf("VGA 3d5 write: Attempted Write to 0x30 readonly\n");
			break;

		case 0x31:  // Memory Configuration
			state.CRTC.reg[0x31] = value;
			// Side effects (compat chain-4 + display start high bits)
			//   bits 4-5 -> display_start[16:17]  (low 16 in CR0C/CR0D)
			// track for stride/dirty-tiling; scanout uses our offsets.
			// DOSBox-X behavior (SVGA_S3_WriteCRTC 0x31). 
			break;

		case 0x32: // Backward Compatibility 1 (BKWD_1)
			state.CRTC.reg[0x32] = value;
			if (s3_cr32_is_unlock(value)) { /* unlock ext regs */ };
			break;

		case 0x33: // Backward Compatibility 2 (BKWD_2)        
			state.CRTC.reg[0x33] = value;
			// re-evaluate write-protect on CR33 changes (bit1 override)
			state.CRTC.write_protect = (state.CRTC.reg[0x11] & 0x80) && !(state.CRTC.reg[0x33] & 0x02);
			// CR33 bit5 can change blanking; recompute.
			recompute_scanline_layout();  // Special blanking forces 8-dot char width so DTP pixel pos may change
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x34: // Backward Compatibility 3 (CR34)
			state.CRTC.reg[0x34] = value;
			// Locks are enforced in 3C2/3C5; DTP enable lives here — recompute.
			recompute_data_transfer_position();
			return;


		case 0x35:  // CPU bank + timing locks
			if ((state.CRTC.reg[0x38] == 0x48) || s3_cr32_is_unlock(state.CRTC.reg[0x32])) {
				state.CRTC.reg[0x35] = (value & 0xF0) | (value & 0x0F); // both nibbles
				// No immediate work needed: vga_mem_read/write pick up the bank nibble live.
			}
			break;

		case 0x36: // Configuration 1 Register (CONFG_REG1) (CR36)
		{
			const u8 pre_reg = state.CRTC.reg[0x36];
			const u8 new_reg = (value & 0xFC) | (pre_reg & 0x03); // keep low 2 strap bits
			state.CRTC.reg[0x36] = new_reg;
			break;
		}

		case 0x37: // Configuration 2 Register (CONFG_REG2) (CR37)
			state.CRTC.reg[0x37] = value;
			break;

		case 0x38: // CR38 Register Lock 1
		case 0x39: // CR39 Register Lock 2
		case 0x3A: // Miscellaneous 1 Register (MISC_1) (CR3A) 
			state.CRTC.reg[state.CRTC.address] = value;

		case 0x3B: // Start Display FIFO Register (DT_EX-POS) (CR3B) - real effect is enabled by CR34 bit4,
			recompute_data_transfer_position();
			state.CRTC.reg[0x3B] = value;
			break;

		case 0x3C: // Interlace Retrace Start Register (IL_RTSTART) (CR3C)
			state.CRTC.reg[0x3C] = value;
			recompute_interlace_retrace_start();
			break;

		case 0x40: // CR40 system config
		case 0x41: // BIOS Flag Register (BIOS_FLAG) (CR41) 
			state.CRTC.reg[state.CRTC.address] = value;
			break;

		case 0x42:  // Mode Control Register (MODE_CTl) (CR42) Return 0x0d for non-interlaced. 
			state.CRTC.reg[0x42] = value;
			recompute_interlace_retrace_start();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x43: // Extended Mode Register (EXT_MODE)
			state.CRTC.reg[0x43] = value;
			recompute_line_offset();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;


		case 0x45: // Hardware Graphics Cursor Mode Register (HGC_MODE) (CR45) 
		case 0x46: // Hardware Graphics Cursor Origin-X Registers (HWGC_ORGX(H)(L)) (CR46, CR47) 
		case 0x47: // Hardware Graphics Cursor Origin-X Registers (HWGC_ORGX(H)(L)) (CR46, CR47) 
		case 0x48: // Hardware Graphics Cursor Origin-Y Registers (HWGC_ORGY(H)(L)) (CR48, CR49) 
		case 0x49: // Hardware Graphics Cursor Origin-Y Registers (HWGC_ORGY(H)(L)) (CR48, CR49) 
		case 0x4A: // Hardware Graphics Cursor Foreground Color Stack Register (HWGC_FGSTK) (CR4A) 
		case 0x4B: // Hardware Graphics Cursor Background Color Stack Register (HWGC_BGSTK) (CR4B) 
		case 0x4C: // Hardware Graphics Cursor Storage Start Address Registers (HWGC_STA(H)(L) (CR4C, CR4D) 
		case 0x4D: // Hardware Graphics Cursor Storage Start Address Registers (HWGC_STA(H)(L) (CR4C, CR4D) 
		case 0x4E: // Hardware Graphics Cursor Pattern Display Start X-PXL-Position Register (HWGC_DX) (CR4E) 
			state.CRTC.reg[state.CRTC.address] = value;

		case 0x4F: // Hardware Graphics Cursor Pattern Disp Start V-PXL-Position Register (HGC_DV) (CR4F) 
			state.CRTC.reg[state.CRTC.address] = value;
			// TODO: call a small overlay hook during scanout - for 4F
			break;

		case 0x50: // Extended System Control 1
			state.CRTC.reg[50] = value;
			break;

		case 0x51: // Extended System Control 2
			state.CRTC.reg[0x51] = value;
			recompute_line_offset();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x52: // Extended BIOS flag 1 register (EXT_BBFLG1) (CR52)
			state.CRTC.reg[0x52] = value;
			break;

		case 0x53: // Extended Memory Control 1 Register - dosbox calls VGA_SETUPHANDLERS(); inside if register != value
			state.CRTC.reg[0x53] = value;
			break;

		case 0x54: // Extended Memory Control 2 Register (EX_MCTL_2) (CR54) 
			state.CRTC.reg[0x54] = value;
			break;

		case 0x55: // Extended RAMDAC Control Register (EX_DAC_CT) (CR55) 
			state.CRTC.reg[0x55] = value;
			break;

		case 0x56: // External Sync Control 1 Register (EX_SYNC_1) (CR56)
			state.CRTC.reg[0x56] = value & 0x1F;  // bits 7–5 reserved
			recompute_external_sync_1();
			break;

		case 0x57: // External Sync Control 2 Register (EX_SYNC_2) (CR57)
			state.CRTC.reg[0x57] = value;
			recompute_external_sync_2();
			break;

		case 0x58: // Linear Address Window Control Register (LAW_CTL) (CR58) - dosbox calls VGA_StartUpdateLFB() after storing the value
			state.CRTC.reg[0x58] = value;
			on_crtc_linear_regs_changed();
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;


		case 0x59: // Linear Address Window Position High
			on_crtc_linear_regs_changed();
			state.CRTC.reg[state.CRTC.address] = value;
			break;

		case 0x5A: // Linear Address Window Position Low
			on_crtc_linear_regs_changed();
			state.CRTC.reg[state.CRTC.address] = value;
			lfb_recalc_and_map();
			break;
		case 0x5b: // undocumented on trio64?
		case 0x5c:  // General output port register - we don't use this (CR5C)
			state.CRTC.reg[state.CRTC.address] = value;
			break;

		case 0x5d: // Extended Horizontal Overflow
		{
			uint8_t prev = state.CRTC.reg[0x5D];
			if (prev == value) break;

			state.CRTC.reg[0x5D] = value;

			// snapshot old derived timings
			auto o_ht = state.h_total, o_hde = state.h_display_end;
			auto o_hbs = state.h_blank_start, o_hbe = state.h_blank_end;
			auto o_hss = state.h_sync_start, o_hse = state.h_sync_end;

			// recompute using CR00..CR05 + CR5D
			recompute_scanline_layout();

			// redraw only if something that affects the scanline changed
			if (state.h_total != o_ht ||
				state.h_display_end != o_hde ||
				state.h_blank_start != o_hbs ||
				state.h_blank_end != o_hbe ||
				state.h_sync_start != o_hss ||
				state.h_sync_end != o_hse) {
				redraw_area(0, 0, old_iWidth, old_iHeight);
			}
			break;
		}

		case 0x5E: // Extended Vertical Overflow Register (EXL_V_OVF) (CR5E)
			state.CRTC.reg[0x5E] = value;
			// vertical size may change (text height)
			redraw_area(0, 0, old_iWidth, old_iHeight);
			break;

		case 0x5F: // undocumented on trio64?
			state.CRTC.reg[0x5F] = value;
			break;

		case 0x60: // Extended Memory Control 3 Register (EXT-MCTL-3) (CR60) 
			state.CRTC.reg[0x60] = value;
			// controls fifo stuff, may need to compute derived bytes later if we use it;
			break;

		case 0x61: // Extended Memory Control 4 Register (EXT-MCTL-4) (CR61)
		case 0x62: // undocumented?
			state.CRTC.reg[state.CRTC.address] = value;
			break;

		case 0x63: // External Sync Control 3 Register (EX-SYNC-3) (CR63) 
			state.CRTC.reg[0x63] = value;
			recompute_external_sync_3();
			break;

		case 0x64: // undocumented?
			state.CRTC.reg[0x64] = value;
			break;

		case 0x65: // Extended Miscellaneous Control Register (EXT-MISC-CTL) (CR6S) 
			state.CRTC.reg[0x65] = value;   // keep full byte for readback
			recompute_ext_misc_ctl();       // recalc for genlock stuff.... we don't implement (maybe never?) but doc accurate
			return;

		case 0x66: // Extended Miscellaneous Control 1 Register (EXT-MISC-1) (CR66) - S3 BIOS writes 0 here - normal operation & PCI bus disconnect disabled
			state.CRTC.reg[0x66] = value;
			break;

		case 0x67: // Extended Miscellaneous Control 2 Register (EXT-MISC-2) (CR67) - Dosbox-X wants VGA_DetermineMode() here
			state.CRTC.reg[0x67] = value;
			break;

		case 0x68: // Configuration 3 Register (CNFG-REG-3) (CR68)
			state.CRTC.reg[0x68] = value;
			recompute_config3();
			return;

		case 0x69: // Extended System Control 3 Register (EXT-SCTL-3)(CR69) - overrides CR31/CR51 when non-zero
			state.CRTC.reg[0x69] = value & 0x0F;    // Trio64 uses 4 bits
			// Changing display-start high bits can affect panning; cheap redraw:
			redraw_area(0, 0, old_iWidth, old_iHeight);
			return;

		case 0x6A: // Extended System Control 4 Register (EXT-SCTL-4)(CR6A) ON TRIO64V+ - Seems Unused on Trio64 but driver uses it anyway
		case 0x6b: // Extended BIOS Flag 3 Register (EBIOS-FLG3) (CR6B) - Bios scratchpad
		case 0x6c: // Extended BIOS Flag 4 Register (EBIOS-FLG3) (CR6C) - Bios dcratchpad
		case 0x6d: // undocumented
			state.CRTC.reg[state.CRTC.address] = value;

		default:
			printf("VGA 3d5 write: unimplemented CRTC register 0x%02x\n", (unsigned)state.CRTC.address);
			state.CRTC.reg[state.CRTC.address] = value;

		}
	}
}

/**
 * Read from the attribute controller index register (0x3c0)
 *
 * For a description of the attribute controller registers, see CCirrus::write_b_3c0.
 **/
u8 CS3Trio64::read_b_3c0()
{
	if (state.attribute_ctrl.flip_flop == 0)
	{

		//BX_INFO(("io read: 0x3c0: flip_flop = 0"));
		return(state.attribute_ctrl.video_enabled << 5) | state.attribute_ctrl.address;
	}
	else
	{
		FAILURE(NotImplemented, "io read: 0x3c0: flip_flop != 0");
	}
}

/**
 * Read from the attribute controller data register (0x3c1)
 *
 * For a description of the attribute controller registers, see CCirrus::write_b_3c0.
 **/
u8 CS3Trio64::read_b_3c1()
{
	u8  retval;
	switch (state.attribute_ctrl.address)
	{
	case 0x00:
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
	case 0x05:
	case 0x06:
	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0a:
	case 0x0b:
	case 0x0c:
	case 0x0d:
	case 0x0e:
	case 0x0f:
		retval = state.attribute_ctrl.palette_reg[state.attribute_ctrl.address];
		return(retval);
		break;

	case 0x10:  /* mode control register */
		retval = (state.attribute_ctrl.mode_ctrl.graphics_alpha << 0) |
			(state.attribute_ctrl.mode_ctrl.display_type << 1) |
			(state.attribute_ctrl.mode_ctrl.enable_line_graphics << 2) |
			(state.attribute_ctrl.mode_ctrl.blink_intensity << 3) |
			(state.attribute_ctrl.mode_ctrl.pixel_panning_compat << 5) |
			(state.attribute_ctrl.mode_ctrl.pixel_clock_select << 6) |
			(state.attribute_ctrl.mode_ctrl.internal_palette_size << 7);
		return(retval);
		break;

	case 0x11:  /* overscan color register */
		return(state.attribute_ctrl.overscan_color);
		break;

	case 0x12:  /* color plane enable */
		return(state.attribute_ctrl.color_plane_enable);
		break;

	case 0x13:  /* horizontal PEL panning register */
		return(state.attribute_ctrl.horiz_pel_panning);
		break;

	case 0x14:  /* color select register */
		return(state.attribute_ctrl.color_select);
		break;

	default:
		FAILURE_1(NotImplemented, "io read: 0x3c1: unknown register 0x%02x",
			(unsigned)state.attribute_ctrl.address);
	}
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
#if DEBUG_VGA_NOISY
	printf("VGA: 3c2 INPUT STATUS REGISTER - ALWAYS ZERO\n");
#endif
	return 0;   // input status register
}

/**
 * Read from the VGA Enable register (0x3c3)
 *
 * (Not sure where this comes from; doesn't seem to be in the VGA specs.)
 **/
u8 CS3Trio64::read_b_3c3()
{
#if DEBUG_VGA_NOISY
	printf("VGA: 3c3 READ VGA ENABLE 0x%02x\n", state.vga_enabled);
#endif
	return state.vga_enabled;
}

/**
 * Write the VGA Enable register (0x3c3)
 *
 * (Not sure where this comes from; doesn't seem to be in the VGA specs.)
 **/
void CS3Trio64::write_b_3c3(u8 value)
{
#if DEBUG_VGA_NOISY
	printf("VGA: 3c3 WRITE VGA ENABLE 0x%02x\n", value);
#endif
	state.vga_enabled = value;
}


/**
 * Read from the VGA sequencer index register (0x3c4)
 *
 * For a description of the Sequencer registers, see CCirrus::write_b_3c4
 **/
u8 CS3Trio64::read_b_3c4()
{
#if DEBUG_VGA_NOISY
	printf("VGA: 3c4 READ Sequencer Index 0x%02x\n", state.sequencer.index);
#endif
	return state.sequencer.index;
}

/**
 * Read from the VGA sequencer data register (0x3c5)
 *
 * For a description of the Sequencer registers, see CCirrus::write_b_3c4
 **/
u8 CS3Trio64::read_b_3c5()
{
#if DEBUG_VGA_NOISY
	printf("VGA: 3c5 READ Sequencer register=0x%02x\n", state.sequencer.index);
#endif

	switch (state.sequencer.index)
	{
	case 0:     /* sequencer: reset */
#if DEBUG_VGA_NOISY
		BX_DEBUG(("io read 0x3c5: sequencer reset"));
#endif
		return(state.sequencer.reset1 ? 1 : 0) | (state.sequencer.reset2 ? 2 : 0);
		break;

	case 1:     /* sequencer: clocking mode */
#if DEBUG_VGA_NOISY
		BX_DEBUG(("io read 0x3c5: sequencer clocking mode"));
#endif
		return state.sequencer.reg1;
		break;

	case 2:     /* sequencer: map mask register */
		return state.sequencer.map_mask;
		break;

	case 3:     /* sequencer: character map select register */
		return state.sequencer.char_map_select;
		break;

	case 4:     /* sequencer: memory mode register */
		return(state.sequencer.extended_mem << 1) |
			(state.sequencer.odd_even << 2) |
			(state.sequencer.chain_four << 3);
		break;

	case 8:
		return state.sequencer.pll_lock;

	case 9: // Extended Sequence Register 9 (SR9)
		return state.sequencer.sr9;

	case 0x0a:
		return state.sequencer.srA;

	case 0x0b:
		return state.sequencer.srB;

	case 0x0d:
		return state.sequencer.srD;

	case 0x15:
		return state.sequencer.sr15;

	case 0x18:
		return state.sequencer.sr18;

	default:
		FAILURE_1(NotImplemented, "io read 0x3c5: index 0x%02x unhandled",
			(unsigned)state.sequencer.index);
	}
}

/**
 * Read from VGA DAC Data register (0x3c9)
 *
 * For a description of DAC registers see CCirrus::write_b_3c7
 **/
u8 CS3Trio64::read_b_3c9()
{
	u8  retval;
	if (state.pel.dac_state == 0x03)
	{
		switch (state.pel.read_data_cycle)
		{
		case 0:   retval = state.pel.data[state.pel.read_data_register].red; break;
		case 1:   retval = state.pel.data[state.pel.read_data_register].green; break;
		case 2:   retval = state.pel.data[state.pel.read_data_register].blue; break;
		default:  retval = 0; // keep compiler happy
		}

		state.pel.read_data_cycle++;
		if (state.pel.read_data_cycle >= 3)
		{
			state.pel.read_data_cycle = 0;
			state.pel.read_data_register++;
		}
	}
	else
	{
		retval = 0x3f;
	}

	return retval;
}

/**
 * Read from the VGA Feature Control register (index 0x3ca)
 *
 * \code
 * +-----------+-+-+
 * |           |1|0|
 * +-----------+-+-+
 *              ^ ^
 *              | +- 0: Feature Control 0 (reserved)
 *              +--- 1: Feature Control 1 (reserved)
 * \endcode
 **/
u8 CS3Trio64::read_b_3ca()
{
	return 0;
}

/**
 * Write to the VGA Miscellaneous Output Register (0x3cc)
 *
 * For a description of the Miscellaneous Output register, see CCirrus::write_b_3c2
 **/
u8 CS3Trio64::read_b_3cc()
{

	/* Miscellaneous Output / Graphics 1 Position ??? */
	return((state.misc_output.color_emulation & 0x01) << 0) |
		((state.misc_output.enable_ram & 0x01) << 1) |
		((state.misc_output.clock_select & 0x03) << 2) |
		((state.misc_output.select_high_bank & 0x01) << 5) |
		((state.misc_output.horiz_sync_pol & 0x01) << 6) |
		((state.misc_output.vert_sync_pol & 0x01) << 7);
}

/**
 * Read from VGA Graphics Controller Data Register (0x3cf)
 *
 * For a description of the Graphics registers, see CCirrus::write_b_3ce
 **/
u8 CS3Trio64::read_b_3cf()
{
	u8  retval;
	switch (state.graphics_ctrl.index)
	{
	case 0:               /* Set/Reset */
		return(state.graphics_ctrl.set_reset);
		break;

	case 1:               /* Enable Set/Reset */
		return(state.graphics_ctrl.enable_set_reset);
		break;

	case 2:               /* Color Compare */
		return(state.graphics_ctrl.color_compare);
		break;

	case 3:               /* Data Rotate */
		retval = ((state.graphics_ctrl.raster_op & 0x03) << 3) | ((state.graphics_ctrl.data_rotate & 0x07) << 0);
		return(retval);
		break;

	case 4:               /* Read Map Select */
		return(state.graphics_ctrl.read_map_select);
		break;

	case 5:               /* Mode */
		retval = ((state.graphics_ctrl.shift_reg & 0x03) << 5) |
			((state.graphics_ctrl.odd_even & 0x01) << 4) |
			((state.graphics_ctrl.read_mode & 0x01) << 3) |
			((state.graphics_ctrl.write_mode & 0x03) << 0);

#if DEBUG_VGA
		if (state.graphics_ctrl.odd_even || state.graphics_ctrl.shift_reg)
			BX_DEBUG(("io read 0x3cf: reg 05 = 0x%02x", (unsigned)retval));
#endif
		return(retval);
		break;

	case 6:               /* Miscellaneous */
		return((state.graphics_ctrl.memory_mapping & 0x03) << 2) |
			((state.graphics_ctrl.odd_even & 0x01) << 1) |
			((state.graphics_ctrl.graphics_alpha & 0x01) << 0);
		break;

	case 7:               /* Color Don't Care */
		return(state.graphics_ctrl.color_dont_care);
		break;

	case 8:               /* Bit Mask */
		return(state.graphics_ctrl.bitmask);
		break;

	default:
		FAILURE_1(NotImplemented, "io read: 0x3cf: index %u unhandled",
			(unsigned)state.graphics_ctrl.index);
	}
}

/**
 * Read from VGA CRTC Index Register (0x3b5 or 0x3d5)
 *
 * For a description of CRTC Registers, see CCirrus::write_b_3d4.
 **/
u8 CS3Trio64::read_b_3d4()
{
#if DEBUG_VGA
	printf("3d4 read register 0x%02x \n", (unsigned)state.CRTC.address);
#endif
	return state.CRTC.address;
}

/**
 * Read from VGA CRTC Data Register (0x3b5 or 0x3d5)
 *
 * For a description of CRTC Registers, see CCirrus::write_b_3d4.
 **/
u8 CS3Trio64::read_b_3d5()
{
	switch (state.CRTC.address)
	{
	case 0x2e: // Chip ID for S3, 0x11 == Trio64 (rev 00h) / Trio64V+ (rev 40h)
		printf("VGA: CRTC CHIP ID READ 0x2E HARDCODED 0x11 For TRIO64 maybe figure this out later\n");
		return 0x11;

	case 0x2f: // Revision ID, low byte of the PCI ID, in our case for Trio64, this will just be 0x00
		printf("VGA: CRTC CHIP REVISION ID READ 0x2F HARDCODED 0x11 FOR TRIO64 maybe figure this out later\n");
		return 0x00;

	case 0x30: // chip ID/Rev register
	case 0x31: // Memory Configuration
	case 0x32: // BKWD_1
	case 0x33: // BKWD_2
	case 0x34: // Backward Compatibility 3 Register (BKWD_3) (CR34) 
	case 0x35: // Bank & Lock - low nibble = CPU bank
	case 0x36: // Reset State Read 1 (read-only): VRAM size + DRAM type
	case 0x37: // Configuration 2 Register (CONFG_REG2) (CR37)
	case 0x38: // Lock 1
	case 0x39: // Lock 2
	case 0x3a: // Miscellaneous 1 Register (MISC_1) (CR3A) 
	case 0x3b: // Start Display FIFO Register (DT_EX-POS) (CR3B) 
	case 0x3c: // Interlace Retrace Start Register (IL_RTSTART) (CR3C)
		return state.CRTC.reg[state.CRTC.address];

	case 0x40: // System Configuration Register (SYS_CNFG) (CR40) 
	case 0x41: // BIOS Flag Register (BIOS_FLAG) (CR41) 
	case 0x42: // Mode Control Register (MODE_CTl) (CR42) - if you set 0x0d here, non-interlaced
	case 0x43: // Extended Mode Register (EXT_MODE) (CR43) 
		return state.CRTC.reg[state.CRTC.address];

	case 0x45: // Hardware Graphics Cursor Mode Register (HGC_MODE) (CR45) 
	case 0x46: // Hardware Graphics Cursor Origin-X Registers (HWGC_ORGX(H)(L)) (CR46, CR47) 
	case 0x47: // Hardware Graphics Cursor Origin-X Registers (HWGC_ORGX(H)(L)) (CR46, CR47) 
	case 0x48: // Hardware Graphics Cursor Origin-Y Registers (HWGC_ORGY(H)(L)) (CR48, CR49) 
	case 0x49: // Hardware Graphics Cursor Origin-Y Registers (HWGC_ORGY(H)(L)) (CR48, CR49) 
	case 0x4A: // Hardware Graphics Cursor Foreground Color Stack Register (HWGC_FGSTK) (CR4A) 
	case 0x4B: // Hardware Graphics Cursor Background Color Stack Register (HWGC_BGSTK) (CR4B) 
	case 0x4C: // Hardware Graphics Cursor Storage Start Address Registers (HWGC_STA(H)(L) (CR4C, CR4D) 
	case 0x4D: // Hardware Graphics Cursor Storage Start Address Registers (HWGC_STA(H)(L) (CR4C, CR4D) 
	case 0x4E: // Hardware Graphics Cursor Pattern Display Start X-PXL-Position Register (HWGC_DX) (CR4E) 
	case 0x4F: // Hardware Graphics Cursor Pattern Disp Start V-PXL-Position Register (HGC_DV) (CR4F) 
	case 0x50: // Extended System Cont 1 Register (EX_SCTL_1) (CR50) 
	case 0x51: // Extended System Control 2 Register (EX_SCTL_2) (CR51) 
	case 0x52: // Extended BIOS Flag 1 Register (EXT_BBFLG1) (CR52) 
	case 0x53: // Extended Memory Control 1 Register (EX_MCTL_1) (CR53) 
	case 0x54: // Extended Memory Control 2 Register (EX_MCTL_2) (CR54) 
	case 0x55: // Extended RAMDAC Control Register (EX_DAC_CT) (CR55) 
	case 0x56: // External Sync Control 1 Register (EX_SYNC_1) (CR56)
	case 0x57: // External Sync Control 2 Register (EX_SVNC_2) (CR57) 
	case 0x58: // Linear Address Window Control Register (LAW_CTL) (CR58) 
	case 0x59: // Linear Address Window Position Registers (LAW_POSIX) (CR59-5A) 
	case 0x5a: // Linear Address Window Position Registers (LAW_POSIX) (CR59-5A) 
	case 0x5b: // undocumented on trio64?
		return state.CRTC.reg[state.CRTC.address];

	case 0x5d: // Extended Horizontal Overflow
	case 0x5e: // Extended Vertical Overflow Register (EXL_V_OVF) (CR5E)
	case 0x5f: // undocumented on trio64?
	case 0x60: // Extended Memory Control 3 Register (EXT-MCTL-3) (CR60) 
	case 0x61: // ?Extended Memory Control 4 Register (EXT-MCTL-4) (CR61) - undocumented?
	case 0x62: // undocumented
	case 0x63: // External Sync Control 3 Register (EX-SYNC-3) (CR63) 
	case 0x64: // undocumented?
	case 0x65: // Extended Miscellaneous Control Register (EXT-MISC-CTL) (CR65)
		return state.CRTC.reg[state.CRTC.address];

	case 0x66: // Extended Miscellaneous Control 1 Register (EXT-MISC-1) (CR66) 
	case 0x67: // Extended Miscellaneous Control 2 Register (EXT-MISC-2)(CR67) 
	case 0x68: // Configuration 3 Register (CNFG-REG-3) (CR68) 
	case 0x69: // Extended System Control 3 Register (EXT-SCTL-3)(CR69) 
	case 0x6A: // Extended System Control 4 Register (EXT-SCTL-4)(CR6A) ON TRIO64V+ - Seems Unused on Trio64 but driver uses it anyway
	case 0x6b: // Extended BIOS Flag 3 Register (EBIOS-FLG3)(CR6B) 
	case 0x6c: // Extended BIOS Flag 4 Register (EBIOS-FLG4)(CR6C) 
	case 0x6d: // undocumented
		return state.CRTC.reg[state.CRTC.address];

	default:
#if DEBUG_VGA_NOISY
		printf("VGA: 3d5 READ CRTC register=0x%02x BINARY VALUE=" PRINTF_BINARY_PATTERN_INT8 " HEX VALUE=0x%02x\n", state.CRTC.address, \
			PRINTF_BYTE_TO_BINARY_INT8(state.CRTC.reg[state.CRTC.address]), state.CRTC.reg[state.CRTC.address]);
#endif
		printf("VGA: 3d5 read : unimplemented CRTC register 0x%02x   \n", (unsigned)state.CRTC.address);
		return state.CRTC.reg[state.CRTC.address];

	}
}

/**
 * Read from the VGA Input Status 1 register (0x3ba or 0x3da)
 *
 * \code
 * +-------+-+---+-+
 * |       |3|   |0|
 * +-------+-+---+-+
 *          ^     ^
 *          |     +- 0: Display Disabled:
 *          |             1: Indicates a horizontal or vertical retrace interval. This
 *          |                bit is the real-time status of the inverted 'display
 *          |                enable' signal. Programs have used this status bit to
 *          |                restrict screen updates to the inactive display intervals
 *          |                in order to reduce screen flicker. The video subsystem is
 *          |                designed to eliminate this software requirement; screen
 *          |                updates may be made at any time without screen degradation.
 *          +------- 1: Vertical Retrace:
 *                        1: Indicates that the display is in a vertical retrace interval.
 *                           This bit can be programmed, through the Vertical Retrace End
 *                           register, to generate an interrupt at the start of the
 *                           vertical retrace.
 * \endcode
 **/
 // above comment applies to the read, not this write function
void CS3Trio64::write_b_3da(u8 value) {
	state.port3da = value;
}

u8 CS3Trio64::read_b_3da()
{

	using clock = std::chrono::steady_clock;
	static auto t0 = clock::now();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - t0).count();

	const int refresh_hz = 70;                 // ~70Hz for classic VGA
	const int frame_ms = 1000 / refresh_hz;  // ~14ms
	const int vblank_ms = 1;                  // 1ms vblank window

	bool in_vblank = (ms % frame_ms) < vblank_ms;

	u8 ret = 0;
	if (in_vblank) { ret |= 0x08 | 0x01; }
	state.attribute_ctrl.flip_flop = 0;
	return ret;

	/* Input Status 1 (color emulation modes) */
   // u8  retval = 0;

	// bit3: Vertical Retrace
	//       0 = display is in the display mode
	//       1 = display is in the vertical retrace mode
	// bit0: Display Enable
	//       0 = display is in the display mode
	//       1 = display is not in the display mode; either the
	//           horizontal or vertical retrace period is active
	// using 72 Hz vertical frequency

	/*** TO DO ??? ***
		 usec = bx_pc_system.time_usec();
		 switch ( ( state.misc_output.vert_sync_pol << 1) | state.misc_output.horiz_sync_pol )
		 {
		   case 0: vertres = 200; break;
		   case 1: vertres = 400; break;
		   case 2: vertres = 350; break;
		   default: vertres = 480; break;
		 }
		 if ((usec % 13888) < 70) {
		   vert_retrace = 1;
		 }
		 if ((usec % (13888 / vertres)) == 0) {
		   horiz_retrace = 1;
		 }

		 if (horiz_retrace || vert_retrace)
		   retval = 0x01;
		 if (vert_retrace)
		   retval |= 0x08;

		 *** TO DO ??? ***/

		 /* reading this port resets the flip-flop to address mode */
		// state.attribute_ctrl.flip_flop = 0;
		// return retval;

}

u8 CS3Trio64::get_actl_palette_idx(u8 index)
{
	return state.attribute_ctrl.palette_reg[index];
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

	if (state.graphics_ctrl.graphics_alpha)
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

	/* no screen update necessary */
	if (state.vga_mem_updated == 0)
		return;

	/* skip screen update when vga/video is disabled or the sequencer is in reset mode */
	if (!state.vga_enabled || !state.attribute_ctrl.video_enabled || state.exsync_blank
		|| !state.sequencer.reset2 || !state.sequencer.reset1) return;

	// fields that effect the way video memory is serialized into screen output:
	// GRAPHICS CONTROLLER:
	//   state.graphics_ctrl.shift_reg:
	//     0: output data in standard VGA format or CGA-compatible 640x200 2 color
	//        graphics mode (mode 6)
	//     1: output data in CGA-compatible 320x200 4 color graphics mode
	//        (modes 4 & 5)
	//     2: output data 8 bits at a time from the 4 bit planes
	//        (mode 13 and variants like modeX)
	// if (state.vga_mem_updated==0 || state.attribute_ctrl.video_enabled == 0)
	if (state.graphics_ctrl.graphics_alpha)
	{
		u8            color;
		unsigned      bit_no;
		unsigned      r;
		unsigned      c;
		unsigned      x;
		unsigned      y;
		unsigned long byte_offset;
		unsigned long start_addr;
		unsigned      xc;
		unsigned      yc;
		unsigned      xti;
		unsigned      yti;

		start_addr = compose_display_start();

		// Expose per-line "fetch start" (in px) for future FIFO/prefetch logic.
		// Today its available, but not used to alter raster timing.
		const bool dtp_active = state.dtp_enabled;
		const unsigned fetch_start_pixels = dtp_active ? state.dtp_hpos_pixels : 0u;
		(void)fetch_start_pixels; // TODO: use in FIFO/prefetch scheduling

		// Interlace retrace start (for future interlace scheduling)
		const bool ilrt_active = state.ilrt_enabled;
		const unsigned ilrt_pixels = ilrt_active ? state.ilrt_pixels : 0u;
		(void)ilrt_pixels; // TODO: use to offset even/odd field start/end in interlaced modes


		//BX_DEBUG(("update: shiftreg=%u, chain4=%u, mapping=%u",
		//  (unsigned) state.graphics_ctrl.shift_reg,
		//  (unsigned) state.sequencer.chain_four,
		//  (unsigned) state.graphics_ctrl.memory_mapping);
		determine_screen_dimensions(&iHeight, &iWidth);
		if ((iWidth != old_iWidth) || (iHeight != old_iHeight) || (state.last_bpp > 8))
		{
			bx_gui->dimension_update(iWidth, iHeight);
			old_iWidth = iWidth;
			old_iHeight = iHeight;
			state.last_bpp = 8;
		}

		// --- Packed 8bpp linear scanout (S3 SVGA) ---
		const int bpp_now = BytesPerPixel();           // from CR67   (1/2/3/4)
		determine_screen_dimensions(&iHeight, &iWidth);
		if ((iWidth != old_iWidth) || (iHeight != old_iHeight) || (state.last_bpp > 8)) {
			bx_gui->dimension_update(iWidth, iHeight);
			old_iWidth = iWidth;
			old_iHeight = iHeight;
			state.last_bpp = 8;
		}

		// Heuristic: in packed 8bpp modes the pitch (line_offset) is >= visible width,
		// whereas planar VGA uses per-plane pitch that's much smaller.
		const bool looks_packed_8 =
			(bpp_now == 1) && (state.line_offset >= (unsigned)iWidth);

		if (looks_packed_8) {
			const unsigned long start_addr = compose_display_start();
			for (yc = 0, yti = 0; yc < iHeight; yc += Y_TILESIZE, yti++) {
				for (xc = 0, xti = 0; xc < iWidth; xc += X_TILESIZE, xti++) {
					if (GET_TILE_UPDATED(xti, yti)) {
						for (r = 0; r < Y_TILESIZE; r++) {
							const unsigned long pixely = yc + r;
							for (c = 0; c < X_TILESIZE; c++) {
								const unsigned long pixelx = xc + c;
								const unsigned long byte_offset =
									start_addr + pixely * state.line_offset + pixelx;
								state.tile[r * X_TILESIZE + c] =
									state.memory[byte_offset & s3_vram_mask()];
							}
						}
						SET_TILE_UPDATED(xti, yti, 0);
						bx_gui->graphics_tile_update(state.tile, xc, yc);
					}
				}
			}
			state.vga_mem_updated = 0;
			return; // we handled the frame
		}


		switch (state.graphics_ctrl.shift_reg)
		{
		case 0:
			u8 attribute, palette_reg_val, DAC_regno;

			unsigned long line_compare;
			u8* plane0;
			u8* plane1;
			u8* plane2;
			u8* plane3;

			if (state.graphics_ctrl.memory_mapping == 3)
			{                 // CGA 640x200x2
				for (yc = 0, yti = 0; yc < iHeight; yc += Y_TILESIZE, yti++)
				{
					for (xc = 0, xti = 0; xc < iWidth; xc += X_TILESIZE, xti++)
					{
						if (GET_TILE_UPDATED(xti, yti))
						{
							for (r = 0; r < Y_TILESIZE; r++)
							{
								y = yc + r;
								if (state.y_doublescan)
									y >>= 1;
								for (c = 0; c < X_TILESIZE; c++)
								{
									x = xc + c;

									/* 0 or 0x2000 */
									byte_offset = start_addr + ((y & 1) << 13);

									/* to the start of the line */
									byte_offset += (320 / 4) * (y / 2);

									/* to the byte start */
									byte_offset += (x / 8);

									bit_no = 7 - (x % 8);
									palette_reg_val = (((state.memory[byte_offset]) >> bit_no) & 1);
									DAC_regno = state.attribute_ctrl.palette_reg[palette_reg_val];
									state.tile[r * X_TILESIZE + c] = DAC_regno;
								}
							}

							SET_TILE_UPDATED(xti, yti, 0);
							bx_gui->graphics_tile_update(state.tile, xc, yc);
						}
					}
				}
			}
			else
			{                 // output data in serial fashion with each display plane
			  // output on its associated serial output.  Standard EGA/VGA format
				plane0 = &state.memory[0 << 16];
				plane1 = &state.memory[1 << 16];
				plane2 = &state.memory[2 << 16];
				plane3 = &state.memory[3 << 16];
				line_compare = state.line_compare;
				if (state.y_doublescan)
					line_compare >>= 1;

				for (yc = 0, yti = 0; yc < iHeight; yc += Y_TILESIZE, yti++)
				{
					for (xc = 0, xti = 0; xc < iWidth; xc += X_TILESIZE, xti++)
					{
						if (GET_TILE_UPDATED(xti, yti))
						{
							for (r = 0; r < Y_TILESIZE; r++)
							{
								y = yc + r;
								if (state.y_doublescan)
									y >>= 1;
								for (c = 0; c < X_TILESIZE; c++)
								{
									x = xc + c;
									if (state.x_dotclockdiv2)
										x >>= 1;
									bit_no = 7 - (x % 8);
									if (y > line_compare)
									{
										byte_offset = x / 8 + ((y - line_compare - 1) * state.line_offset);
									}
									else
									{
										byte_offset = start_addr + x / 8 + (y * state.line_offset);
									}

									attribute = (((plane0[byte_offset] >> bit_no) & 0x01) << 0) |
										(((plane1[byte_offset] >> bit_no) & 0x01) << 1) |
										(((plane2[byte_offset] >> bit_no) & 0x01) << 2) |
										(((plane3[byte_offset] >> bit_no) & 0x01) << 3);

									attribute &= state.attribute_ctrl.color_plane_enable;

									// undocumented feature ???: colors 0..7 high intensity, colors 8..15 blinking
									// using low/high intensity. Blinking is not implemented yet.
									if (state.attribute_ctrl.mode_ctrl.blink_intensity)
										attribute ^= 0x08;
									palette_reg_val = state.attribute_ctrl.palette_reg[attribute];
									if (state.attribute_ctrl.mode_ctrl.internal_palette_size)
									{

										// use 4 lower bits from palette register
										// use 4 higher bits from color select register
										// 16 banks of 16-color registers
										DAC_regno = (palette_reg_val & 0x0f) | (state.attribute_ctrl.color_select << 4);
									}
									else
									{

										// use 6 lower bits from palette register
										// use 2 higher bits from color select register
										// 4 banks of 64-color registers
										DAC_regno = (palette_reg_val & 0x3f) | ((state.attribute_ctrl.color_select & 0x0c) << 4);
									}

									// DAC_regno &= video DAC mask register ???
									state.tile[r * X_TILESIZE + c] = DAC_regno;
								}
							}

							SET_TILE_UPDATED(xti, yti, 0);
							bx_gui->graphics_tile_update(state.tile, xc, yc);
						}
					}
				}
			}
			break;            // case 0

		case 1:             // output the data in a CGA-compatible 320x200 4 color graphics
			// mode.  (modes 4 & 5)

			/* CGA 320x200x4 start */
			for (yc = 0, yti = 0; yc < iHeight; yc += Y_TILESIZE, yti++)
			{
				for (xc = 0, xti = 0; xc < iWidth; xc += X_TILESIZE, xti++)
				{
					if (GET_TILE_UPDATED(xti, yti))
					{
						for (r = 0; r < Y_TILESIZE; r++)
						{
							y = yc + r;
							if (state.y_doublescan)
								y >>= 1;
							for (c = 0; c < X_TILESIZE; c++)
							{
								x = xc + c;
								if (state.x_dotclockdiv2)
									x >>= 1;

								/* 0 or 0x2000 */
								byte_offset = start_addr + ((y & 1) << 13);

								/* to the start of the line */
								byte_offset += (320 / 4) * (y / 2);

								/* to the byte start */
								byte_offset += (x / 4);

								attribute = 6 - 2 * (x % 4);
								palette_reg_val = (state.memory[byte_offset]) >> attribute;
								palette_reg_val &= 3;
								DAC_regno = state.attribute_ctrl.palette_reg[palette_reg_val];
								state.tile[r * X_TILESIZE + c] = DAC_regno;
							}
						}

						SET_TILE_UPDATED(xti, yti, 0);
						bx_gui->graphics_tile_update(state.tile, xc, yc);
					}
				}
			}

			/* CGA 320x200x4 end */
			break;            // case 1

		case 2:             // output the data eight bits at a time from the 4 bit plane

			// (format for VGA mode 13 hex)
		case 3:             // FIXME: is this really the same ???
			if (state.sequencer.chain_four)
			{
				unsigned long pixely;

				unsigned long pixelx;

				unsigned long plane;

				if (state.misc_output.select_high_bank != 1)
				{
					FAILURE(NotImplemented, "update: select_high_bank != 1   \n");
				}

				for (yc = 0, yti = 0; yc < iHeight; yc += Y_TILESIZE, yti++)
				{
					for (xc = 0, xti = 0; xc < iWidth; xc += X_TILESIZE, xti++)
					{
						if (GET_TILE_UPDATED(xti, yti))
						{
							for (r = 0; r < Y_TILESIZE; r++)
							{
								pixely = yc + r;
								if (state.y_doublescan)
									pixely >>= 1;
								for (c = 0; c < X_TILESIZE; c++)
								{
									pixelx = (xc + c) >> 1;
									plane = (pixelx % 4);
									byte_offset = start_addr + (plane * 65536) + (pixely * state.line_offset) + (pixelx & ~0x03);
									color = state.memory[byte_offset];
									state.tile[r * X_TILESIZE + c] = color;
								}
							}

							SET_TILE_UPDATED(xti, yti, 0);
							bx_gui->graphics_tile_update(state.tile, xc, yc);
						}
					}
				}
			}
			else
			{                 // chain_four == 0, modeX
				unsigned long pixely;

				// chain_four == 0, modeX
				unsigned long pixelx;

				// chain_four == 0, modeX
				unsigned long plane;

				for (yc = 0, yti = 0; yc < iHeight; yc += Y_TILESIZE, yti++)
				{
					for (xc = 0, xti = 0; xc < iWidth; xc += X_TILESIZE, xti++)
					{
						if (GET_TILE_UPDATED(xti, yti))
						{
							for (r = 0; r < Y_TILESIZE; r++)
							{
								pixely = yc + r;
								if (state.y_doublescan)
									pixely >>= 1;
								for (c = 0; c < X_TILESIZE; c++)
								{
									pixelx = (xc + c) >> 1;
									plane = (pixelx % 4);
									byte_offset = (plane * 65536) + (pixely * state.line_offset) + (pixelx >> 2);
									color = state.memory[start_addr + byte_offset];
									state.tile[r * X_TILESIZE + c] = color;
								}
							}

							SET_TILE_UPDATED(xti, yti, 0);
							bx_gui->graphics_tile_update(state.tile, xc, yc);
						}
					}
				}
			}
			break;            // case 2

		default:
			FAILURE_1(NotImplemented, "update: shift_reg == %u   \n",
				(unsigned)state.graphics_ctrl.shift_reg);
		}

		state.vga_mem_updated = 0;
		return;
	}
	else
	{                     // text mode
		unsigned long   start_address;
		unsigned long   cursor_address;
		unsigned long   cursor_x;
		unsigned long   cursor_y;
		bx_vga_tminfo_t tm_info;
		unsigned        VDE;
		unsigned        MSL;
		unsigned        cols;
		unsigned        rows;
		unsigned        cWidth;

		tm_info.start_address = 2 * ((state.CRTC.reg[12] << 8) + state.CRTC.reg[13]);
		tm_info.cs_start = state.CRTC.reg[0x0a] & 0x3f;
		tm_info.cs_end = state.CRTC.reg[0x0b] & 0x1f;
		tm_info.line_offset = state.CRTC.reg[0x13] << 2;
		tm_info.line_compare = state.line_compare;
		tm_info.h_panning = state.attribute_ctrl.horiz_pel_panning & 0x0f;
		tm_info.v_panning = state.CRTC.reg[0x08] & 0x1f;
		tm_info.line_graphics = state.attribute_ctrl.mode_ctrl.enable_line_graphics;
		tm_info.split_hpanning = state.attribute_ctrl.mode_ctrl.pixel_panning_compat;
		if ((state.sequencer.reg1 & 0x01) == 0)
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

		// Verticle Display End: find out how many lines are displayed
		VDE = state.vertical_display_end;

		// Maximum Scan Line: height of character cell
		MSL = state.CRTC.reg[0x09] & 0x1f;
		if (MSL == 0)
		{
#if DEBUG_VGA
			BX_ERROR(("character height = 1, skipping text update"));
#endif
			return;
		}

		cols = state.CRTC.reg[1] + 1;
		if ((MSL == 1) && (VDE == 399))
		{

			// emulated CGA graphics mode 160x100x16 colors
			MSL = 3;
		}

		rows = (VDE + 1) / (MSL + 1);
		if (rows > BX_MAX_TEXT_LINES)
		{
			BX_PANIC(("text rows>%d: %d", BX_MAX_TEXT_LINES, rows));
			return;
		}

		// Force 8-dot characters if special blanking is enabled. Accuracy
		if (state.CRTC.reg[0x33] & 0x20)
			cWidth = 8;
		else
			cWidth = ((state.sequencer.reg1 & 0x01) == 1) ? 8 : 9;

		iWidth = cWidth * cols;
		iHeight = VDE + 1;
		if ((iWidth != old_iWidth) || (iHeight != old_iHeight) || (MSL != old_MSL)
			|| (state.last_bpp > 8))
		{
			bx_gui->dimension_update(iWidth, iHeight, MSL + 1, cWidth);
			old_iWidth = iWidth;
			old_iHeight = iHeight;
			old_MSL = MSL;
			state.last_bpp = 8;
		}

		// pass old text snapshot & new VGA memory contents
		start_address = 2 * ((state.CRTC.reg[12] << 8) + state.CRTC.reg[13]);
		cursor_address = 2 * ((state.CRTC.reg[0x0e] << 8) + state.CRTC.reg[0x0f]);
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

		bx_gui->text_update(state.text_snapshot, &state.memory[start_address],
			cursor_x, cursor_y, tm_info, rows);

		// screen updated, copy new VGA memory contents into text snapshot
		memcpy(state.text_snapshot, &state.memory[start_address], 2 * cols * rows);
		state.vga_mem_updated = 0;
	}
}

void CS3Trio64::determine_screen_dimensions(unsigned* piHeight,
	unsigned* piWidth)
{
	int ai[0x20];
	int i;
	int h;
	int v;
	for (i = 0; i < 0x20; i++)
		ai[i] = state.CRTC.reg[i];

	h = (ai[1] + 1) * 8;
	v = (ai[18] | ((ai[7] & 0x02) << 7) | ((ai[7] & 0x40) << 3)) + 1;
	// S3 CR5E extends V* with bit10 (0x400)
	if (state.CRTC.reg[0x5E] & 0x02) v |= 0x400;

	if (state.graphics_ctrl.shift_reg == 0)
	{
		*piWidth = 640;
		*piHeight = 480;

		if (state.CRTC.reg[6] == 0xBF)
		{
			if (state.CRTC.reg[23] == 0xA3 && state.CRTC.reg[20] == 0x40
				&& state.CRTC.reg[9] == 0x41)
			{
				*piWidth = 320;
				*piHeight = 240;
			}
			else
			{
				if (state.x_dotclockdiv2)
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
	else if (state.graphics_ctrl.shift_reg == 2)
	{
		if (state.sequencer.chain_four)
		{
			*piWidth = h;
			*piHeight = v;
		}
		else
		{
			*piWidth = h;
			*piHeight = v;
		}
	}
	else
	{
		if (state.x_dotclockdiv2)
			h <<= 1;
		*piWidth = h;
		*piHeight = v;
	}
}

inline uint32_t CS3Trio64::s3_vram_mask() const
{
	const uint32_t sz = state.memsize ? state.memsize : (8u * 1024u * 1024u);
	return sz - 1u;
}

inline uint8_t  CS3Trio64::s3_vram_read8(uint32_t addr) const
{
	return state.memory[(addr & s3_vram_mask())];
}

inline void CS3Trio64::s3_vram_write8(uint32_t addr, uint8_t v)
{
	state.memory[(addr & s3_vram_mask())] = v;
	state.vga_mem_updated = 1;
}


u8 CS3Trio64::vga_mem_read(u32 addr)
{
	u32   offset;
	u8* plane0;
	u8* plane1;
	u8* plane2;
	u8* plane3;
	u8    retval = 0;

	switch (state.graphics_ctrl.memory_mapping)
	{
	case 1:               // 0xA0000 .. 0xAFFFF
		if (addr > 0xAFFFF)
			return 0xff;
		offset = addr & 0xFFFF;
		break;

	case 2:               // 0xB0000 .. 0xB7FFF
		if ((addr < 0xB0000) || (addr > 0xB7FFF))
			return 0xff;
		offset = addr & 0x7FFF;
		break;

	case 3:               // 0xB8000 .. 0xBFFFF
		if (addr < 0xB8000)
			return 0xff;
		offset = addr & 0x7FFF;
		break;

	default:              // 0xA0000 .. 0xBFFFF
		offset = addr & 0x1FFFF;
	}

	// Apply S3 CPU bank (CR35 low nibble) only for graphics apertures:
	const bool bank_applies = (state.graphics_ctrl.memory_mapping == 0) ||
		(state.graphics_ctrl.memory_mapping == 1);
	const u32 bank_nib = (state.CRTC.reg[0x35] & 0x0F);
	const u32 bank_base = bank_applies
		? (bank_nib << (state.sequencer.chain_four ? 16 : 14))
		: 0u;


	if (state.sequencer.chain_four)
	{

		// Mode 13h: 320x200x8bpp (chained) — bank in 64 KiB units
		return state.memory[(offset & ~0x03) + (offset % 4) * 65536];
	}

	// Planar modes — bank in 16 KiB units
	plane0 = &state.memory[bank_base + (0 << 16)];
	plane1 = &state.memory[bank_base + (1 << 16)];
	plane2 = &state.memory[bank_base + (2 << 16)];
	plane3 = &state.memory[bank_base + (3 << 16)];

	/* addr between 0xA0000 and 0xAFFFF */
	switch (state.graphics_ctrl.read_mode)
	{
	case 0:               /* read mode 0 */
		state.graphics_ctrl.latch[0] = plane0[offset];
		state.graphics_ctrl.latch[1] = plane1[offset];
		state.graphics_ctrl.latch[2] = plane2[offset];
		state.graphics_ctrl.latch[3] = plane3[offset];
		retval = state.graphics_ctrl.latch[state.graphics_ctrl.read_map_select];
		break;

	case 1:               /* read mode 1 */
	{
		u8  color_compare;

		u8  color_dont_care;
		u8  latch0;
		u8  latch1;
		u8  latch2;
		u8  latch3;

		color_compare = state.graphics_ctrl.color_compare & 0x0f;
		color_dont_care = state.graphics_ctrl.color_dont_care & 0x0f;
		latch0 = state.graphics_ctrl.latch[0] = plane0[offset];
		latch1 = state.graphics_ctrl.latch[1] = plane1[offset];
		latch2 = state.graphics_ctrl.latch[2] = plane2[offset];
		latch3 = state.graphics_ctrl.latch[3] = plane3[offset];

		latch0 ^= ccdat[color_compare][0];
		latch1 ^= ccdat[color_compare][1];
		latch2 ^= ccdat[color_compare][2];
		latch3 ^= ccdat[color_compare][3];

		latch0 &= ccdat[color_dont_care][0];
		latch1 &= ccdat[color_dont_care][1];
		latch2 &= ccdat[color_dont_care][2];
		latch3 &= ccdat[color_dont_care][3];

		retval = ~(latch0 | latch1 | latch2 | latch3);
	}
	break;
	}

	return retval;
}

/**
 * Write to Legacy VGA Memory
 **/
void CS3Trio64::vga_mem_write(u32 addr, u8 value)
{
	u32       offset;
	u8        new_val[4];
	unsigned  start_addr;
	u8* plane0;
	u8* plane1;
	u8* plane2;
	u8* plane3;

	/* The memory_mapping bits of the graphics controller determine
	 * what window of VGA memory is available.
	 *
	 *  00: 0xA0000 .. 0xBFFFF (128K)
	 *  01: 0xA0000 .. 0xAFFFF (64K) (also used for VGA text mode)
	 *  02: 0xB0000 .. 0xB7FFF (32K)
	 *  03: 0xB8000 .. 0xBFFFF (32K) (also used for CGA text mode)
	 */
	switch (state.graphics_ctrl.memory_mapping)
	{
		// 0xA0000 .. 0xAFFFF
	case 1:
		if (addr > 0xAFFFF)
			return;
		offset = addr - 0xA0000;
		break;

		// 0xB0000 .. 0xB7FFF
	case 2:
		if ((addr < 0xB0000) || (addr > 0xB7FFF))
			return;
		offset = addr - 0xB0000;
		break;

		// 0xB8000 .. 0xBFFFF
	case 3:
		if (addr < 0xB8000)
			return;
		offset = addr - 0xB8000;
		break;

		// 0xA0000 .. 0xBFFFF
	default:
		offset = addr - 0xA0000;
	}

	start_addr = compose_display_start();

	// Apply S3 CPU bank (CR35 low nibble) only for graphics apertures:
	const bool bank_applies = (state.graphics_ctrl.memory_mapping == 0) ||
		(state.graphics_ctrl.memory_mapping == 1);
	const u32 bank_nib = (state.CRTC.reg[0x35] & 0x0F);
	const u32 bank_base = bank_applies
		? (bank_nib << (state.sequencer.chain_four ? 16 : 14)) : 0u;

	if (state.graphics_ctrl.graphics_alpha)
	{
		if (state.graphics_ctrl.memory_mapping == 3)
		{
			// Text mode, and memory 0xB8000 .. 0xBFFFF selected => CGA text mode
			unsigned  x_tileno;
			unsigned  x_tileno2;
			unsigned  y_tileno;

			/* CGA 320x200x4 / 640x200x2 start */
			state.memory[offset] = value;
			offset -= start_addr;
			if (offset >= 0x2000)
			{
				y_tileno = offset - 0x2000;
				y_tileno /= (320 / 4);
				y_tileno <<= 1; //2 * y_tileno;
				y_tileno++;
				x_tileno = (offset - 0x2000) % (320 / 4);
				x_tileno <<= 2; //*= 4;
			}
			else
			{
				y_tileno = offset / (320 / 4);
				y_tileno <<= 1; //2 * y_tileno;
				x_tileno = offset % (320 / 4);
				x_tileno <<= 2; //*=4;
			}

			x_tileno2 = x_tileno;
			if (state.graphics_ctrl.shift_reg == 0)
			{
				x_tileno *= 2;
				x_tileno2 += 7;
			}
			else
			{
				x_tileno2 += 3;
			}

			if (state.x_dotclockdiv2)
			{
				x_tileno /= (X_TILESIZE / 2);
				x_tileno2 /= (X_TILESIZE / 2);
			}
			else
			{
				x_tileno /= X_TILESIZE;
				x_tileno2 /= X_TILESIZE;
			}

			if (state.y_doublescan)
			{
				y_tileno /= (Y_TILESIZE / 2);
			}
			else
			{
				y_tileno /= Y_TILESIZE;
			}

			state.vga_mem_updated = 1;
			SET_TILE_UPDATED(x_tileno, y_tileno, 1);
			if (x_tileno2 != x_tileno)
			{
				SET_TILE_UPDATED(x_tileno2, y_tileno, 1);
			}

			return;

			/* CGA 320x200x4 / 640x200x2 end */
		}

		if (state.graphics_ctrl.memory_mapping != 1)
		{
			FAILURE_1(NotImplemented, "mem_write: graphics: mapping = %u  \n",
				(unsigned)state.graphics_ctrl.memory_mapping);
		}

		if (state.sequencer.chain_four)
		{
			unsigned  x_tileno;
			unsigned  y_tileno;

			// 320x200x8bpp (chained) — bank in 64 KiB units
			state.memory[bank_base + ((offset & ~0x03) + (offset % 4) * 65536)] = value;
			if (state.line_offset > 0)
			{
				offset -= start_addr;
				x_tileno = (offset % state.line_offset) / (X_TILESIZE / 2);
				if (state.y_doublescan)
				{
					y_tileno = (offset / state.line_offset) / (Y_TILESIZE / 2);
				}
				else
				{
					y_tileno = (offset / state.line_offset) / Y_TILESIZE;
				}

				state.vga_mem_updated = 1;
				SET_TILE_UPDATED(x_tileno, y_tileno, 1);
			}

			return;
		}
	}

	/* addr between 0xA0000 and 0xAFFFF (or graphics in 128 KiB window) */
	plane0 = &state.memory[bank_base + (0 << 16)];
	plane1 = &state.memory[bank_base + (1 << 16)];
	plane2 = &state.memory[bank_base + (2 << 16)];
	plane3 = &state.memory[bank_base + (3 << 16)];

	switch (state.graphics_ctrl.write_mode)
	{
		unsigned  i;
		// Write mode 0
	case 0:
	{
		/* Write Mode 0 is the standard and most general write mode.
		 * While the other write modes are designed to perform a specific
		 * task, this mode can be used to perform most tasks as all five
		 * operations are performed on the data:
		 *   - The data byte from the host is first rotated as specified
		 *     by the Rotate Count field, then is replicated across all
		 *     four planes.
		 *   - Then the Enable Set/Reset field selects which planes will
		 *     receive their values from the host data and which will
		 *     receive their data from that plane's Set/Reset field
		 *     location.
		 *   - Then the operation specified by the Logical Operation
		 *     field is performed on the resulting data and the data in
		 *     the read latches.
		 *   - The Bit Mask field is then used to select between the
		 *     resulting data and data from the latch register.
		 *   - Finally, the resulting data is written to the display
		 *     memory planes enabled in the Memory Plane Write Enable
		 *     field.
		 *   .
		 */
		const u8  bitmask = state.graphics_ctrl.bitmask;
		const u8  set_reset = state.graphics_ctrl.set_reset;
		const u8  enable_set_reset = state.graphics_ctrl.enable_set_reset;

		/* perform rotate on CPU data in case its needed */
		if (state.graphics_ctrl.data_rotate)
		{
			value = (value >> state.graphics_ctrl.data_rotate) | (value << (8 - state.graphics_ctrl.data_rotate));
		}

		new_val[0] = state.graphics_ctrl.latch[0] & ~bitmask;
		new_val[1] = state.graphics_ctrl.latch[1] & ~bitmask;
		new_val[2] = state.graphics_ctrl.latch[2] & ~bitmask;
		new_val[3] = state.graphics_ctrl.latch[3] & ~bitmask;
		switch (state.graphics_ctrl.raster_op)
		{
		case 0: // replace
			new_val[0] |=
				(
					(enable_set_reset & 1) ? ((set_reset & 1) ? bitmask : 0) :
					(value & bitmask)
					);
			new_val[1] |=
				(
					(enable_set_reset & 2) ? ((set_reset & 2) ? bitmask : 0) :
					(value & bitmask)
					);
			new_val[2] |=
				(
					(enable_set_reset & 4) ? ((set_reset & 4) ? bitmask : 0) :
					(value & bitmask)
					);
			new_val[3] |=
				(
					(enable_set_reset & 8) ? ((set_reset & 8) ? bitmask : 0) :
					(value & bitmask)
					);
			break;

		case 1: // AND
			new_val[0] |=
				(
					(enable_set_reset & 1) ?
					((set_reset & 1) ? (state.graphics_ctrl.latch[0] & bitmask) : 0) :
					(value & state.graphics_ctrl.latch[0]) & bitmask
					);
			new_val[1] |=
				(
					(enable_set_reset & 2) ?
					((set_reset & 2) ? (state.graphics_ctrl.latch[1] & bitmask) : 0) :
					(value & state.graphics_ctrl.latch[1]) & bitmask
					);
			new_val[2] |=
				(
					(enable_set_reset & 4) ?
					((set_reset & 4) ? (state.graphics_ctrl.latch[2] & bitmask) : 0) :
					(value & state.graphics_ctrl.latch[2]) & bitmask
					);
			new_val[3] |=
				(
					(enable_set_reset & 8) ?
					((set_reset & 8) ? (state.graphics_ctrl.latch[3] & bitmask) : 0) :
					(value & state.graphics_ctrl.latch[3]) & bitmask
					);
			break;

		case 2: // OR
			new_val[0] |=
				(
					(enable_set_reset & 1) ?
					(
						(set_reset & 1) ? bitmask :
						(state.graphics_ctrl.latch[0] & bitmask)
						) : ((value | state.graphics_ctrl.latch[0]) & bitmask)
					);
			new_val[1] |=
				(
					(enable_set_reset & 2) ?
					(
						(set_reset & 2) ? bitmask :
						(state.graphics_ctrl.latch[1] & bitmask)
						) : ((value | state.graphics_ctrl.latch[1]) & bitmask)
					);
			new_val[2] |=
				(
					(enable_set_reset & 4) ?
					(
						(set_reset & 4) ? bitmask :
						(state.graphics_ctrl.latch[2] & bitmask)
						) : ((value | state.graphics_ctrl.latch[2]) & bitmask)
					);
			new_val[3] |=
				(
					(enable_set_reset & 8) ?
					(
						(set_reset & 8) ? bitmask :
						(state.graphics_ctrl.latch[3] & bitmask)
						) : ((value | state.graphics_ctrl.latch[3]) & bitmask)
					);
			break;

		case 3: // XOR
			new_val[0] |=
				(
					(enable_set_reset & 1) ?
					(
						(set_reset & 1) ? (~state.graphics_ctrl.latch[0] & bitmask) :
						(state.graphics_ctrl.latch[0] & bitmask)
						) : (value ^ state.graphics_ctrl.latch[0]) & bitmask
					);
			new_val[1] |=
				(
					(enable_set_reset & 2) ?
					(
						(set_reset & 2) ? (~state.graphics_ctrl.latch[1] & bitmask) :
						(state.graphics_ctrl.latch[1] & bitmask)
						) : (value ^ state.graphics_ctrl.latch[1]) & bitmask
					);
			new_val[2] |=
				(
					(enable_set_reset & 4) ?
					(
						(set_reset & 4) ? (~state.graphics_ctrl.latch[2] & bitmask) :
						(state.graphics_ctrl.latch[2] & bitmask)
						) : (value ^ state.graphics_ctrl.latch[2]) & bitmask
					);
			new_val[3] |=
				(
					(enable_set_reset & 8) ?
					(
						(set_reset & 8) ? (~state.graphics_ctrl.latch[3] & bitmask) :
						(state.graphics_ctrl.latch[3] & bitmask)
						) : (value ^ state.graphics_ctrl.latch[3]) & bitmask
					);
			break;

		default:
			FAILURE_1(NotImplemented, "vga_mem_write: write mode 0: op = %u",
				(unsigned)state.graphics_ctrl.raster_op);
		}
	}
	break;

	// Write mode 1
	case 1:
		/* Write Mode 1 is used to transfer the data in the latches
		 * register directly to the screen, affected only by the
		 * Memory Plane Write Enable field. This can facilitate
		 * rapid transfer of data on byte boundaries from one area
		 * of video memory to another or filling areas of the
		 * display with a pattern of 8 pixels.
		 * When Write Mode 0 is used with the Bit Mask field set to
		 * 00000000b the operation of the hardware is identical to
		 * this mode.
		 */
		for (i = 0; i < 4; i++)
		{
			new_val[i] = state.graphics_ctrl.latch[i];
		}
		break;

		// Write mode 2
	case 2:
	{
		/* Write Mode 2 is used to unpack a pixel value packed into
		 * the lower 4 bits of the host data byte into the 4 display
		 * planes:
		 *   - In the byte from the host, the bit representing each
		 *     plane will be replicated across all 8 bits of the
		 *     corresponding planes.
		 *   - Then the operation specified by the Logical Operation
		 *     field is performed on the resulting data and the data
		 *     in the read latches.
		 *   - The Bit Mask field is then used to select between the
		 *     resulting data and data from the latch register.
		 *   - Finally, the resulting data is written to the display
		 *     memory planes enabled in the Memory Plane Write Enable
		 *     field.
		 *   .
		 */
		const u8  bitmask = state.graphics_ctrl.bitmask;

		new_val[0] = state.graphics_ctrl.latch[0] & ~bitmask;
		new_val[1] = state.graphics_ctrl.latch[1] & ~bitmask;
		new_val[2] = state.graphics_ctrl.latch[2] & ~bitmask;
		new_val[3] = state.graphics_ctrl.latch[3] & ~bitmask;
		switch (state.graphics_ctrl.raster_op)
		{
		case 0: // write
			new_val[0] |= (value & 1) ? bitmask : 0;
			new_val[1] |= (value & 2) ? bitmask : 0;
			new_val[2] |= (value & 4) ? bitmask : 0;
			new_val[3] |= (value & 8) ? bitmask : 0;
			break;

		case 1: // AND
			new_val[0] |= (value & 1) ? (state.graphics_ctrl.latch[0] & bitmask) : 0;
			new_val[1] |= (value & 2) ? (state.graphics_ctrl.latch[1] & bitmask) : 0;
			new_val[2] |= (value & 4) ? (state.graphics_ctrl.latch[2] & bitmask) : 0;
			new_val[3] |= (value & 8) ? (state.graphics_ctrl.latch[3] & bitmask) : 0;
			break;

		case 2: // OR
			new_val[0] |= (value & 1) ? bitmask : (state.graphics_ctrl.latch[0] & bitmask);
			new_val[1] |= (value & 2) ? bitmask : (state.graphics_ctrl.latch[1] & bitmask);
			new_val[2] |= (value & 4) ? bitmask : (state.graphics_ctrl.latch[2] & bitmask);
			new_val[3] |= (value & 8) ? bitmask : (state.graphics_ctrl.latch[3] & bitmask);
			break;

		case 3: // XOR
			new_val[0] |= (value & 1) ? (~state.graphics_ctrl.latch[0] & bitmask) : (state.graphics_ctrl.latch[0] & bitmask);
			new_val[1] |= (value & 2) ? (~state.graphics_ctrl.latch[1] & bitmask) : (state.graphics_ctrl.latch[1] & bitmask);
			new_val[2] |= (value & 4) ? (~state.graphics_ctrl.latch[2] & bitmask) : (state.graphics_ctrl.latch[2] & bitmask);
			new_val[3] |= (value & 8) ? (~state.graphics_ctrl.latch[3] & bitmask) : (state.graphics_ctrl.latch[3] & bitmask);
			break;
		}
	}
	break;

	// Write mode 3
	case 3:
	{
		/* Write Mode 3 is used when the color written is fairly
		 * constant but the Bit Mask field needs to be changed
		 * frequently, such as when drawing single color lines or
		 * text:
		 *   - The value of the Set/Reset field is expanded as if
		 *     the Enable Set/Reset field were set to 1111b,
		 *     regardless of its actual value.
		 *   - The host data is first rotated as specified by the
		 *     Rotate Count field, then is ANDed with the Bit
		 *     Mask field.
		 *   - The resulting value is used where the Bit Mask
		 *     field normally would be used, selecting data from
		 *     either the expansion of the Set/Reset field or the
		 *     latch register.
		 *   - Finally, the resulting data is written to the
		 *     display memory planes enabled in the Memory Plane
		 *     Write Enable field.
		 *   .
		 */
		const u8  bitmask = state.graphics_ctrl.bitmask & value;
		const u8  set_reset = state.graphics_ctrl.set_reset;

		/* perform rotate on CPU data */
		if (state.graphics_ctrl.data_rotate)
		{
			value = (value >> state.graphics_ctrl.data_rotate) | (value << (8 - state.graphics_ctrl.data_rotate));
		}

		new_val[0] = state.graphics_ctrl.latch[0] & ~bitmask;
		new_val[1] = state.graphics_ctrl.latch[1] & ~bitmask;
		new_val[2] = state.graphics_ctrl.latch[2] & ~bitmask;
		new_val[3] = state.graphics_ctrl.latch[3] & ~bitmask;

		value &= bitmask;

		switch (state.graphics_ctrl.raster_op)
		{
		case 0: // write
			new_val[0] |= (set_reset & 1) ? value : 0;
			new_val[1] |= (set_reset & 2) ? value : 0;
			new_val[2] |= (set_reset & 4) ? value : 0;
			new_val[3] |= (set_reset & 8) ? value : 0;
			break;

		case 1: // AND
			new_val[0] |= ((set_reset & 1) ? value : 0) & state.graphics_ctrl.latch[0];
			new_val[1] |= ((set_reset & 2) ? value : 0) & state.graphics_ctrl.latch[1];
			new_val[2] |= ((set_reset & 4) ? value : 0) & state.graphics_ctrl.latch[2];
			new_val[3] |= ((set_reset & 8) ? value : 0) & state.graphics_ctrl.latch[3];
			break;

		case 2: // OR
			new_val[0] |= ((set_reset & 1) ? value : 0) | state.graphics_ctrl.latch[0];
			new_val[1] |= ((set_reset & 2) ? value : 0) | state.graphics_ctrl.latch[1];
			new_val[2] |= ((set_reset & 4) ? value : 0) | state.graphics_ctrl.latch[2];
			new_val[3] |= ((set_reset & 8) ? value : 0) | state.graphics_ctrl.latch[3];
			break;

		case 3: // XOR
			new_val[0] |= ((set_reset & 1) ? value : 0) ^ state.graphics_ctrl.latch[0];
			new_val[1] |= ((set_reset & 2) ? value : 0) ^ state.graphics_ctrl.latch[1];
			new_val[2] |= ((set_reset & 4) ? value : 0) ^ state.graphics_ctrl.latch[2];
			new_val[3] |= ((set_reset & 8) ? value : 0) ^ state.graphics_ctrl.latch[3];
			break;
		}
	}
	break;

	default:
		FAILURE_1(NotImplemented, "vga_mem_write: write mode %u ?",
			(unsigned)state.graphics_ctrl.write_mode);
	}

	// state.sequencer.map_mask determines which bitplanes the write should actually go to
	if (state.sequencer.map_mask & 0x0f)
	{
		state.vga_mem_updated = 1;
		if (state.sequencer.map_mask & 0x01)
			plane0[offset] = new_val[0];
		if (state.sequencer.map_mask & 0x02)
			plane1[offset] = new_val[1];
		if (state.sequencer.map_mask & 0x04)
		{
			// Plane 2 contains the character map
			if ((offset & 0xe000) == state.charmap_address)
			{

				//printf("Updating character map %04x with %02x...\n  ", (offset & 0x1fff), new_val[2]);
				bx_gui->lock();
				bx_gui->set_text_charbyte((u16)(offset & 0x1fff), new_val[2]);
				bx_gui->unlock();
			}

			plane2[offset] = new_val[2];
		}

		if (state.sequencer.map_mask & 0x08)
			plane3[offset] = new_val[3];

		unsigned  x_tileno;

		unsigned  y_tileno;

		if (state.graphics_ctrl.shift_reg == 2)
		{
			offset -= start_addr;
			x_tileno = (offset % state.line_offset) * 4 / (X_TILESIZE / 2);
			if (state.y_doublescan)
			{
				y_tileno = (offset / state.line_offset) / (Y_TILESIZE / 2);
			}
			else
			{
				y_tileno = (offset / state.line_offset) / Y_TILESIZE;
			}

			SET_TILE_UPDATED(x_tileno, y_tileno, 1);
		}
		else
		{
			if (state.line_compare < state.vertical_display_end)
			{
				if (state.line_offset > 0)
				{
					if (state.x_dotclockdiv2)
					{
						x_tileno = (offset % state.line_offset) / (X_TILESIZE / 16);
					}
					else
					{
						x_tileno = (offset % state.line_offset) / (X_TILESIZE / 8);
					}

					if (state.y_doublescan)
					{
						y_tileno =
							(
								(offset / state.line_offset) *
								2 +
								state.line_compare +
								1
								) /
							Y_TILESIZE;
					}
					else
					{
						y_tileno = ((offset / state.line_offset) + state.line_compare + 1) / Y_TILESIZE;
					}

					SET_TILE_UPDATED(x_tileno, y_tileno, 1);
				}
			}

			if (offset >= start_addr)
			{
				offset -= start_addr;
				if (state.line_offset > 0)
				{
					if (state.x_dotclockdiv2)
					{
						x_tileno = (offset % state.line_offset) / (X_TILESIZE / 16);
					}
					else
					{
						x_tileno = (offset % state.line_offset) / (X_TILESIZE / 8);
					}

					if (state.y_doublescan)
					{
						y_tileno = (offset / state.line_offset) / (Y_TILESIZE / 2);
					}
					else
					{
						y_tileno = (offset / state.line_offset) / Y_TILESIZE;
					}

					SET_TILE_UPDATED(x_tileno, y_tileno, 1);
				}
			}
		}
	}
}
