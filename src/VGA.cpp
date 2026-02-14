/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * WWW    : http://sourceforge.net/projects/es40
 * E-mail : camiel@camicom.com
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
  * Contains the code for the VGA base class.
  *
  * $Id$
  *
  * X-1.6        Camiel Vanderhoeven                             24-MAR-2008
  *      Added comments.
  *
  * X-1.5        Camiel Vanderhoeven                             14-MAR-2008
  *      Formatting.
  *
  * X-1.4        Camiel Vanderhoeven                             14-MAR-2008
  *   1. More meaningful exceptions replace throwing (int) 1.
  *   2. U64 macro replaces X64 macro.
  *
  * X-1.3        Brian Wheeler                                   27-FEB-2008
  *      Avoid compiler warnings.
  *
  * X-1.2        Camiel Vanderhoeven                             28-DEC-2007
  *      Throw exceptions rather than just exiting when errors occur.
  *
  * X-1.1        Camiel Vanderhoeven                             10-DEC-2007
  *      Initial version in CVS.
  **/
#include "StdAfx.h"
#include "VGA.h"

  /**
   * Constructor.
   *
   * Checks if more than one VGA card is present. If so, throws a failure.
   **/
CVGA::CVGA(class CConfigurator* cfg, class CSystem* c, int pcibus, int pcidev) : CPCIDevice(cfg, c, pcibus, pcidev)
{
	if (theVGA != 0)
		FAILURE(Configuration, "More than one VGA");
	theVGA = this;
}

/**
 * Destructor.
 **/
CVGA::~CVGA(void)
{
}

/**************************************
 *
 * 3bx/3dx implementation
 *
 *************************************/

u8 CVGA::crtc_address_r(offs_t offset)
{
	return vga.crtc.index;
}

void CVGA::crtc_address_w(offs_t offset, u8 data)
{
	vga.crtc.index = data;
}

u8 CVGA::crtc_data_r(offs_t offset)
{
	return space(CRTC_REG).read_byte(vga.crtc.index);
}

void CVGA::crtc_data_w(offs_t offset, u8 data)
{
	vga.crtc.data[vga.crtc.index] = data;
	space(CRTC_REG).write_byte(vga.crtc.index, data);
}

u8 CVGA::input_status_1_r(offs_t offset)
{
	// ES40 uses CS3Trio64::read_b_3da() 
	// MAME needs screen().hblank() and vga_vblank() 
	u8 res = 0;
	vga.attribute.state = 0;
	// Return a plausible value.
	return res;
}

void CVGA::feature_control_w(offs_t offset, u8 data)
{
	vga.feature_control = data;
}

/**************************************
 *
 * 3cx implementation
 *
 *************************************/

u8 CVGA::atc_address_r(offs_t offset)
{
	return vga.attribute.index;
}

u8 CVGA::atc_data_r(offs_t offset)
{
	return space(ATC_REG).read_byte(vga.attribute.index);
}

void CVGA::atc_address_data_w(offs_t offset, u8 data)
{
	if (vga.attribute.state == 0)
	{
		vga.attribute.index = data;
	}
	else
	{
		space(ATC_REG).write_byte(vga.attribute.index, data);
	}

	vga.attribute.state = !vga.attribute.state;
}

u8 CVGA::input_status_0_r(offs_t offset)
{
	// ES40 uses CS3Trio64::read_b_3c2() with hardcoded sense 
	// MAME version needs m_input_sense ioport 
	u8 res = 0x60;  // bit 5-6 = "is VGA"
	res |= vga.crtc.irq_latch << 7;
	return res;

}

void CVGA::miscellaneous_output_w(offs_t offset, u8 data)
{
	vga.miscellaneous_output = data;
	recompute_params();
	m_ioas = bool(BIT(data, 0));
}

u8 CVGA::sequencer_address_r(offs_t offset)
{
	return vga.sequencer.index;
}

u8 CVGA::sequencer_data_r(offs_t offset)
{
	return space(SEQ_REG).read_byte(vga.sequencer.index);
}

void CVGA::sequencer_address_w(offs_t offset, u8 data)
{
	vga.sequencer.index = data;
}

void CVGA::sequencer_data_w(offs_t offset, u8 data)
{
	// TODO: temporary cheat for read-back
	vga.sequencer.data[vga.sequencer.index] = data;
	space(SEQ_REG).write_byte(vga.sequencer.index, data);
	recompute_params();
}

u8 CVGA::ramdac_mask_r(offs_t offset)
{
	return vga.dac.mask;
}

u8 CVGA::ramdac_state_r(offs_t offset)
{
	return (vga.dac.read) ? 3 : 0;
}

u8 CVGA::ramdac_write_index_r(offs_t offset)
{
	return vga.dac.write_index;
}

u8 CVGA::ramdac_data_r(offs_t offset)
{
	u8 res = 0xFF;
	if (vga.dac.read)
	{
		switch (vga.dac.state++)
		{
		case 0:
			res = vga.dac.color[3 * vga.dac.read_index];
			break;
		case 1:
			res = vga.dac.color[3 * vga.dac.read_index + 1];
			break;
		case 2:
			res = vga.dac.color[3 * vga.dac.read_index + 2];
			break;
		}

		if (vga.dac.state == 3)
		{
			vga.dac.state = 0;
			vga.dac.read_index++;
		}
	}
	return res;
}

u8 CVGA::feature_control_r(offs_t offset)
{
	return vga.feature_control;
}

u8 CVGA::miscellaneous_output_r(offs_t offset)
{
	return vga.miscellaneous_output;
}

u8 CVGA::gc_address_r(offs_t offset)
{
	return vga.gc.index;
}

u8 CVGA::gc_data_r(offs_t offset)
{
	return space(GC_REG).read_byte(vga.gc.index);
}


void CVGA::ramdac_mask_w(offs_t offset, u8 data)
{
	vga.dac.mask = data;
	vga.dac.dirty = 1;
}

void CVGA::ramdac_read_index_w(offs_t offset, u8 data)
{
	vga.dac.read_index = data;
	vga.dac.state = 0;
	vga.dac.read = 1;
}

void CVGA::ramdac_write_index_w(offs_t offset, u8 data)
{
	vga.dac.write_index = data;
	vga.dac.state = 0;
	vga.dac.read = 0;
}

void CVGA::ramdac_data_w(offs_t offset, u8 data)
{
	if (!vga.dac.read)
	{
		vga.dac.loading[vga.dac.state] = data;
		vga.dac.state++;

		if (vga.dac.state == 3)
		{
			vga.dac.color[3 * vga.dac.write_index] = vga.dac.loading[0];
			vga.dac.color[3 * vga.dac.write_index + 1] = vga.dac.loading[1];
			vga.dac.color[3 * vga.dac.write_index + 2] = vga.dac.loading[2];
			vga.dac.dirty = 1;
			vga.dac.state = 0;
			vga.dac.write_index++;
		}
	}
}

void CVGA::gc_address_w(offs_t offset, u8 data)
{
	vga.gc.index = data;
}

void CVGA::gc_data_w(offs_t offset, u8 data)
{
	space(GC_REG).write_byte(vga.gc.index, data);
}

// skip s ome stuff for now

uint16_t CVGA::offset()
{
	if (vga.crtc.dw)
		return vga.crtc.offset << 3;
	if (vga.crtc.word_mode)
		return vga.crtc.offset << 1;
	else
		return vga.crtc.offset << 2;
}

uint8_t CVGA::vga_latch_write(int offs, uint8_t data)
{
    uint8_t res = 0;

    switch (vga.gc.write_mode & 3)
    {
    case 0:
        data = rotate_right(data);
        if (vga.gc.enable_set_reset & 1 << offs)
            res = vga_logical_op((vga.gc.set_reset & 1 << offs) ? vga.gc.bit_mask : 0, offs, vga.gc.bit_mask);
        else
            res = vga_logical_op(data, offs, vga.gc.bit_mask);
        break;
    case 1:
        res = vga.gc.latch[offs];
        break;
    case 2:
        res = vga_logical_op((data & 1 << offs) ? 0xff : 0x00, offs, vga.gc.bit_mask);
        break;
    case 3:
        data = rotate_right(data);
        res = vga_logical_op((vga.gc.set_reset & 1 << offs) ? 0xff : 0x00, offs, data & vga.gc.bit_mask);
        break;
    }

    return res;
}

// more skips

void CVGA::palette_update()
{
	// Base class: no-op.  Subclasses update their palette here.
}

// more skips, pc_vga_choosevideomode() is in the subclass

// skip screen_update

// recompute_params_clock

void CVGA::recompute_params()
{
//	if (vga.miscellaneous_output & 8)
//		LOGWARN("Warning: VGA external clock latch selected\n");
//	else
//		recompute_params_clock(1, ((vga.miscellaneous_output & 0xc) ? XTAL(28'636'363) : XTAL(25'174'800)).value());
}

// vga_vblank

// device_input_ports

// TODO: convert to mapped space
uint8_t CVGA::mem_r(offs_t offset)
{
	switch (vga.gc.memory_map_sel & 0x03)
	{
	case 0: break;
	case 1: offset &= 0x0ffff; break;
	case 2: offset -= 0x10000; offset &= 0x07fff; break;
	case 3: offset -= 0x18000; offset &= 0x07fff; break;
	}

	if (vga.sequencer.data[4] & 4)
	{
		int data;
		if (/*!machine().side_effects_disabled() move this over in a refactor */ true)
		{
			vga.gc.latch[0] = vga.memory[(offset)];
			vga.gc.latch[1] = vga.memory[(offset)+0x10000];
			vga.gc.latch[2] = vga.memory[(offset)+0x20000];
			vga.gc.latch[3] = vga.memory[(offset)+0x30000];
		}

		if (vga.gc.read_mode)
		{
			// In Read Mode 1 latch is checked against this
			// cfr. lombrall & intsocch where they RMW sprite-like objects
			// and anything outside this formula goes transparent.
			const u8 target_color = (vga.gc.color_compare & vga.gc.color_dont_care);
			data = 0;

			for (u8 byte = 0; byte < 8; byte++)
			{
				u8 fill_latch = 0;
				for (u8 layer = 0; layer < 4; layer++)
				{
					if (vga.gc.latch[layer] & 1 << byte)
						fill_latch |= 1 << layer;
				}
				fill_latch &= vga.gc.color_dont_care;
				if (fill_latch == target_color)
					data |= 1 << byte;
			}
		}
		else
			data = vga.gc.latch[vga.gc.read_map_sel];

		return data;
	}
	else
	{
		uint8_t i, data;

		data = 0;
		//printf("%08x\n",offset);

		for (i = 0; i < 4; i++)
		{
			if (vga.sequencer.map_mask & 1 << i)
				data |= vga.memory[offset + i * 0x10000];
		}

		return data;
	}

	// never executed
	//return 0;
}

void CVGA::mem_w(offs_t offset, uint8_t data)
{
	//Inside each case must prevent writes to non-mapped VGA memory regions, not only mask the offset.
	switch (vga.gc.memory_map_sel & 0x03)
	{
	case 0: break;
	case 1:
		if (offset & 0x10000)
			return;

		offset &= 0x0ffff;
		break;
	case 2:
		if ((offset & 0x18000) != 0x10000)
			return;

		offset &= 0x07fff;
		break;
	case 3:
		if ((offset & 0x18000) != 0x18000)
			return;

		offset &= 0x07fff;
		break;
	}

	{
		uint8_t i;

		for (i = 0; i < 4; i++)
		{
			if (vga.sequencer.map_mask & 1 << i)
				vga.memory[offset + i * 0x10000] = (vga.sequencer.data[4] & 4) ? vga_latch_write(i, data) : data;
		}
		return;
	}
}

// TODO: is there any non-SVGA capable board capable of linear access?
uint8_t CVGA::mem_linear_r(offs_t offset)
{
	return vga.memory[offset % vga.svga_intf.vram_size];
}

void CVGA::mem_linear_w(offs_t offset, uint8_t data)
{
	vga.memory[offset % vga.svga_intf.vram_size] = data;
}

/**
 * Variable pointer to the one and only VGA card.
 **/
CVGA* theVGA = 0;
