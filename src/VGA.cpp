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

uint8_t CVGA::mem_r(uint32_t offset)
{
    // --- memory_map_sel windowing ---
    switch (vga.gc.memory_map_sel & 0x03)
    {
    case 0: break;
    case 1: offset &= 0x0ffff; break;
    case 2: offset -= 0x10000; offset &= 0x07fff; break;
    case 3: offset -= 0x18000; offset &= 0x07fff; break;
    }

    if (vga.sequencer.data[4] & 4)
    {
        // Chain-4 / planar read (seq reg 4 bit 2 set) 

        // Load all four latches on every read
        vga.gc.latch[0] = vga.memory[(offset)];
        vga.gc.latch[1] = vga.memory[(offset)+0x10000];
        vga.gc.latch[2] = vga.memory[(offset)+0x20000];
        vga.gc.latch[3] = vga.memory[(offset)+0x30000];

        if (vga.gc.read_mode)
        {
            // Read Mode 1: color-compare against latches
            const uint8_t target_color = (vga.gc.color_compare & vga.gc.color_dont_care);
            int data = 0;

            for (uint8_t byte = 0; byte < 8; byte++)
            {
                uint8_t fill_latch = 0;
                for (uint8_t layer = 0; layer < 4; layer++)
                {
                    if (vga.gc.latch[layer] & 1 << byte)
                        fill_latch |= 1 << layer;
                }
                fill_latch &= vga.gc.color_dont_care;
                if (fill_latch == target_color)
                    data |= 1 << byte;
            }
            return (uint8_t)data;
        }
        else
        {
            // Read Mode 0: return selected plane via read_map_sel
            return vga.gc.latch[vga.gc.read_map_sel];
        }
    }
    else
    {
        // --- Non-chain-4: OR-combine enabled planes ---
        uint8_t data = 0;
        for (int i = 0; i < 4; i++)
        {
            if (vga.sequencer.map_mask & 1 << i)
                data |= vga.memory[offset + i * 0x10000];
        }
        return data;
    }
}

void CVGA::mem_w(uint32_t offset, uint8_t data)
{
    // --- memory_map_sel windowing (with boundary checks) ---
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

    // --- Per-plane write through map_mask ---
    for (int i = 0; i < 4; i++)
    {
        if (vga.sequencer.map_mask & 1 << i)
        {
            vga.memory[offset + i * 0x10000] =
                (vga.sequencer.data[4] & 4)
                ? vga_latch_write(i, data)
                : data;
        }
    }
}

void CVGA::mem_w(uint32_t offset, uint8_t data)
{
    // --- memory_map_sel windowing (with boundary checks) ---
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

    // --- Per-plane write through map_mask ---
    for (int i = 0; i < 4; i++)
    {
        if (vga.sequencer.map_mask & 1 << i)
        {
            vga.memory[offset + i * 0x10000] =
                (vga.sequencer.data[4] & 4)
                ? vga_latch_write(i, data)
                : data;
        }
    }
}

uint8_t CVGA::mem_linear_r(uint32_t offset)
{
    return vga.memory[offset % vga.svga_intf.vram_size];
}

void CVGA::mem_linear_w(uint32_t offset, uint8_t data)
{
    vga.memory[offset % vga.svga_intf.vram_size] = data;
}

uint16_t CVGA::offset()
{
    if (vga.crtc.dw)
        return vga.crtc.offset << 3;
    if (vga.crtc.word_mode)
        return vga.crtc.offset << 1;
    else
        return vga.crtc.offset << 2;
}

void CVGA::palette_update()
{
    // Base class: no-op.  Subclasses update their palette here.
}

/**
 * Variable pointer to the one and only VGA card.
 **/
CVGA* theVGA = 0;
