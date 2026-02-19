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

#include "logmacro.h"

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
#ifdef DEBUG_VGA_RENDER
	static int mask_read_count = 0;
	if (mask_read_count < 20)
		LOG("DIAG: ramdac_mask_r called (count=%d) mask=%02x\n", mask_read_count, vga.dac.mask);
	mask_read_count++;
#endif
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
#ifdef DEBUG_VGA_RENDER
		LOG("DIAG: ramdac_write_index_w data=%02x\n", data);
#endif
	vga.dac.write_index = data;
	vga.dac.state = 0;
	vga.dac.read = 0;
}

void CVGA::ramdac_data_w(offs_t offset, u8 data)
{
#ifdef DEBUG_VGA_RENDER
	// log first DAC writes
	static int dac_write_count = 0;
	if (dac_write_count < 20) {
		LOG("DIAG: ramdac_data_w data=%02x state=%d write_index=%02x read=%d\n",
			data, vga.dac.state, vga.dac.write_index, vga.dac.read);
		dac_write_count++;
	}

	printf("ramdac_data_w: data=%02x read=%d state=%d widx=%02x\n",
		data, vga.dac.read, vga.dac.state, vga.dac.write_index);

#endif
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

uint32_t CVGA::start_addr()
{
	//  popmessage("Offset: %04x  %s %s **",vga.crtc.offset,vga.crtc.dw?"DW":"--",vga.crtc.word_mode?"BYTE":"WORD");
	if (vga.crtc.dw)
		return vga.crtc.start_addr << 2;
	if (vga.crtc.word_mode)
		return vga.crtc.start_addr << 0;
	else
		return vga.crtc.start_addr << 1;
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

void CVGA::vga_vh_text(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	// TODO: make it a subclassable function, cfr. PR2 Video Select in Paradise family
#define TEXT_COPY_9COLUMN(ch) (((ch & 0xe0) == 0xc0) && (vga.attribute.data[0x10] & 4))

	int width = BIT(vga.sequencer.data[1], 0) ? 8 : 9, height = (vga.crtc.maximum_scan_line) * (vga.crtc.scan_doubling + 1);

	const u32 frame_number = screen().frame_number();

	if (vga.crtc.cursor_enable)
		vga.cursor.visible = frame_number & 0x10;
	else
		vga.cursor.visible = 0;

	bool blink_rate = frame_number & 0x20;
	const int TEXT_LINES = vga.crtc.vert_disp_end + 1;

	for (int addr = vga.crtc.start_addr, line = -vga.crtc.preset_row_scan; line < TEXT_LINES;
		line += height, addr += (offset() >> 1))
	{
		for (int pos = addr, column = 0; column < vga.crtc.horz_disp_end + 1; column++, pos++)
		{
			const uint8_t ch = vga.memory[(pos << 1) + 0];
			const uint8_t attr = vga.memory[(pos << 1) + 1];
			const uint32_t font_base = vga.sequencer.char_sel.base[BIT(attr, 3)] + (ch << 5);
			const bool blink_sel = !!BIT(vga.attribute.data[0x10], 3);
			const uint8_t blink_en = (blink_sel && blink_rate) ? BIT(attr, 7) : 0;

			uint8_t fore_col = attr & 0xf;
			uint8_t back_col = (attr & 0x70) >> 4;
			// blink disabled translates MSB of attribute as extra intensity for background color
			back_col |= (blink_sel) ? 0 : (BIT(attr, 7) >> 4);

			for (int h = std::max(-line, 0); (h < height) && (line + h < std::min(TEXT_LINES, bitmap.height())); h++)
			{
				uint32_t* const bitmapline = &bitmap.pix(line + h);
				uint8_t bits = vga.memory[font_base + (h >> (vga.crtc.scan_doubling))];

				int mask, w;
				for (mask = 0x80, w = 0; (w < width) && (w < 8); w++, mask >>= 1)
				{
					pen_t pen;
					if (bits & mask)
						pen = vga.pens[blink_en ? back_col : fore_col];
					else
						pen = vga.pens[back_col];

					const int dest_x = column * width + w;
					if (!screen().visible_area().contains(dest_x, line + h))
						continue;
					bitmapline[dest_x] = pen;

				}
				if (w < width)
				{
					/* 9 column */
					pen_t pen;
					if (TEXT_COPY_9COLUMN(ch) && (bits & 1))
						pen = vga.pens[blink_en ? back_col : fore_col];
					else
						pen = vga.pens[back_col];

					const int dest_x = column * width + w;

					if (!screen().visible_area().contains(dest_x, line + h))
						continue;
					bitmapline[dest_x] = pen;
				}
			}
			if (vga.cursor.visible && (pos == vga.crtc.cursor_addr))
			{
				for (int h = vga.crtc.cursor_scan_start;
					(h <= vga.crtc.cursor_scan_end) && (h < height) && (line + h < TEXT_LINES);
					h++)
				{
					const int dest_x = column * width;

					if (!screen().visible_area().contains(dest_x, line + h))
						continue;
					bitmap.plot_box(dest_x, line + h, width, 1, vga.pens[attr & 0xf]);
				}
			}
		}
	}
}

void CVGA::vga_vh_ega(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	const int height = vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1);
	const int pel_shift = (vga.attribute.pel_shift & 7);
	const int LINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);
	const int EGA_COLUMNS = vga.crtc.horz_disp_end + 1;

	for (int addr = vga.crtc.start_addr, line = 0; line < LINES; line += height, addr += offset())
	{
		for (int yi = 0; yi < height; yi++)
		{
			uint32_t* const bitmapline = &bitmap.pix(line + yi);
			// ibm_5150:batmanmv uses this on gameplay for both EGA and "VGA" modes
			// NB: EGA mode in that game sets 663, should be 303 like the other mode
			// causing no status bar to appear. This is a known btanb in how VGA
			// handles EGA mode, cfr. https://www.os2museum.com/wp/fantasyland-on-vga/
			if ((line + yi) == (vga.crtc.line_compare & 0x3ff))
				addr = 0;

			for (int pos = addr, c = 0, column = 0; column < EGA_COLUMNS + 1; column++, c += 8, pos = (pos + 1) & 0xffff)
			{
				int data[4] = {
						vga.memory[(pos & 0xffff)],
						vga.memory[(pos & 0xffff) + 0x10000] << 1,
						vga.memory[(pos & 0xffff) + 0x20000] << 2,
						vga.memory[(pos & 0xffff) + 0x30000] << 3 };

				for (int i = 7; i >= 0; i--)
				{
					pen_t pen = vga.pens[(data[0] & 1) | (data[1] & 2) | (data[2] & 4) | (data[3] & 8)];

					data[0] >>= 1;
					data[1] >>= 1;
					data[2] >>= 1;
					data[3] >>= 1;

					if (!screen().visible_area().contains(c + i - pel_shift, line + yi))
						continue;
					bitmapline[c + i - pel_shift] = pen;
				}
			}
		}
	}
}

// In mode 13h (256 colors) every pixel actually double height/width
// i.e. a 320x200 is really 640x400
void CVGA::vga_vh_vga(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	int height = vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1);
	int pel_shift = (vga.attribute.pel_shift & 6);
	int addrmask = vga.crtc.no_wrap ? -1 : 0xffff;

	const int LINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);
	/* line compare is screen sensitive */
	uint16_t mask_comp = 0x3ff; //| (LINES & 0x300);
	const int VGA_COLUMNS = vga.crtc.horz_disp_end + 1;
	//  popmessage("%02x %02x",vga.attribute.pel_shift,vga.sequencer.data[4] & 0x08);

	int curr_addr = 0;
	if (!(vga.sequencer.data[4] & 0x08))
	{
		for (int addr = start_addr(), line = 0; line < LINES; line += height, addr += offset(), curr_addr += offset())
		{
			for (int yi = 0; yi < height; yi++)
			{
				if ((line + yi) < (vga.crtc.line_compare & mask_comp))
					curr_addr = addr;
				if ((line + yi) == (vga.crtc.line_compare & mask_comp))
				{
					curr_addr = 0;
					pel_shift = 0;
				}
				uint32_t* const bitmapline = &bitmap.pix(line + yi);
				for (int pos = curr_addr, c = 0, column = 0; column < VGA_COLUMNS + 1; column++, c += 8, pos++)
				{
					if (pos > 0x80000 / 4)
						return;

					for (int xi = 0; xi < 8; xi++)
					{
						if (!screen().visible_area().contains(c + xi - (pel_shift), line + yi))
							continue;
						bitmapline[c + xi - (pel_shift)] = pen(vga.memory[(pos & addrmask) + ((xi >> 1) * 0x10000)]);
					}
				}
			}
		}
	}
	else
	{
		for (int addr = start_addr(), line = 0; line < LINES; line += height, addr += offset(), curr_addr += offset())
		{
			for (int yi = 0; yi < height; yi++)
			{
				if ((line + yi) < (vga.crtc.line_compare & mask_comp))
					curr_addr = addr;
				if ((line + yi) == (vga.crtc.line_compare & mask_comp))
					curr_addr = 0;
				uint32_t* const bitmapline = &bitmap.pix(line + yi);
				//addr %= 0x80000;
				for (int pos = curr_addr, c = 0, column = 0; column < VGA_COLUMNS + 1; column++, c += 0x10, pos += 0x8)
				{
					if (pos + 0x08 > 0x80000)
						return;

					for (int xi = 0; xi < 0x10; xi++)
					{
						if (!screen().visible_area().contains(c + xi - (pel_shift), line + yi))
							continue;
						bitmapline[c + xi - pel_shift] = pen(vga.memory[(pos + (xi >> 1)) & addrmask]);
					}
				}
			}
		}
	}
}

void CVGA::vga_vh_cga(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	const int LINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);

	const int height = (vga.crtc.scan_doubling + 1);
	const int width = (vga.crtc.horz_disp_end + 1) * 8;

	for (int y = 0; y < LINES; y++)
	{
		uint32_t addr = ((y & 1) * 0x2000) + (((y & ~1) >> 1) * width / 4);

		for (int x = 0; x < width; x += 4)
		{
			for (int yi = 0; yi < height; yi++)
			{
				uint32_t* const bitmapline = &bitmap.pix(y * height + yi);

				for (int xi = 0; xi < 4; xi++)
				{
					pen_t pen = vga.pens[(vga.memory[addr] >> (6 - xi * 2)) & 3];
					if (!screen().visible_area().contains(x + xi, y * height + yi))
						continue;
					bitmapline[x + xi] = pen;
				}
			}

			addr++;
		}
	}
}

void CVGA::vga_vh_mono(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	const int LINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);

	const int height = (vga.crtc.scan_doubling + 1);
	const int width = (vga.crtc.horz_disp_end + 1) * 8;

	for (int y = 0; y < LINES; y++)
	{
		uint32_t addr = ((y & 1) * 0x2000) + (((y & ~1) >> 1) * width / 8);

		for (int x = 0; x < width; x += 8)
		{
			for (int yi = 0; yi < height; yi++)
			{
				uint32_t* const bitmapline = &bitmap.pix(y * height + yi);

				for (int xi = 0; xi < 8; xi++)
				{
					pen_t pen = vga.pens[(vga.memory[addr] >> (7 - xi)) & 1];
					if (!screen().visible_area().contains(x + xi, y * height + yi))
						continue;
					bitmapline[x + xi] = pen;
				}
			}

			addr++;
		}
	}
}

void CVGA::palette_update()
{
	for (int i = 0; i < 256; i++)
	{
		set_pen_color(i,
			pal6bit(vga.dac.color[3 * (i & vga.dac.mask) + 0] & 0x3f),
			pal6bit(vga.dac.color[3 * (i & vga.dac.mask) + 1] & 0x3f),
			pal6bit(vga.dac.color[3 * (i & vga.dac.mask) + 2] & 0x3f)
		);
	}
#ifdef DEBUG_VGA_RENDER
	// dump first few palette entries
	static bool palette_dumped = false;
	if (!palette_dumped) {
		for (int i = 0; i < 8; i++) {
			LOG("DIAG: DAC[%d] = R:%02x G:%02x B:%02x (mask=%02x)\n",
				i,
				vga.dac.color[3 * (i & vga.dac.mask) + 0],
				vga.dac.color[3 * (i & vga.dac.mask) + 1],
				vga.dac.color[3 * (i & vga.dac.mask) + 2],
				vga.dac.mask);
		}
		palette_dumped = true;
	}
#endif
}

u16 CVGA::line_compare_mask()
{
	return 0x3ff;
}

void CVGA::svga_vh_rgb8(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	static int rgb8_dump_late = 0;
	if (rgb8_dump_late < 2 && pen(2) != 0) {
		printf("RGB8 LATE RENDER: start_addr=%04x start_shift=%d offset=%d\n",
			vga.crtc.start_addr,
			(!(vga.sequencer.data[4] & 0x08) || svga.ignore_chain4) ? 2 : 0,
			offset());
		printf("  pen(0)=%08x pen(2)=%08x pen(130)=%08x\n",
			pen(0), pen(2), pen(0x82));
		// Check VRAM at locations where color 2 fills should be
		// Fill at (6,252) means VRAM offset = 252*1024+6 = 258054
		int off1 = 252 * 1024 + 10;  // middle of first color-2 fill
		int off2 = 280 * 1024 + 500; // middle of the large fill area
		printf("  VRAM[0]=%02x VRAM[100]=%02x VRAM[%d]=%02x VRAM[%d]=%02x\n",
			vga.memory[0], vga.memory[100], off1, vga.memory[off1], off2, vga.memory[off2]);
		// Also dump first 16 bytes of line 252 (where color-2 fill starts)
		int line252 = 252 * 1024;
		printf("  LINE252[0..15]=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			vga.memory[line252 + 0], vga.memory[line252 + 1], vga.memory[line252 + 2], vga.memory[line252 + 3],
			vga.memory[line252 + 4], vga.memory[line252 + 5], vga.memory[line252 + 6], vga.memory[line252 + 7],
			vga.memory[line252 + 8], vga.memory[line252 + 9], vga.memory[line252 + 10], vga.memory[line252 + 11],
			vga.memory[line252 + 12], vga.memory[line252 + 13], vga.memory[line252 + 14], vga.memory[line252 + 15]);
		rgb8_dump_late++;
	}
	const int height = vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1);

	const uint16_t mask_comp = line_compare_mask();
	int curr_addr = 0;
	//  uint16_t line_length;
	//  if(vga.crtc.dw)
	//      line_length = vga.crtc.offset << 3;  // doubleword mode
	//  else
	//  {
	//      line_length = vga.crtc.offset << 4;
	//  }

	const int VGA_COLUMNS = vga.crtc.horz_disp_end + 1;
	const int LINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);

	uint8_t start_shift = (!(vga.sequencer.data[4] & 0x08) || svga.ignore_chain4) ? 2 : 0;
	for (int addr = vga.crtc.start_addr << start_shift, line = 0; line < LINES; line += height, addr += offset(), curr_addr += offset())
	{
		for (int yi = 0; yi < height; yi++)
		{
			if ((line + yi) < (vga.crtc.line_compare & mask_comp))
				curr_addr = addr;
			if ((line + yi) == (vga.crtc.line_compare & mask_comp))
				curr_addr = 0;
			uint32_t* const bitmapline = &bitmap.pix(line + yi);
			addr %= vga.svga_intf.vram_size;
			for (int pos = curr_addr, c = 0, column = 0; column < VGA_COLUMNS; column++, c += 8, pos += 0x8)
			{
				if (pos + 0x08 >= vga.svga_intf.vram_size)
					return;

				for (int xi = 0; xi < 8; xi++)
				{
					const int dest_x = c + xi;

					if (!screen().visible_area().contains(dest_x, line + yi))
						continue;
					bitmapline[dest_x] = pen(vga.memory[pos + xi]);
					// check what renderer is reading
#ifdef DEBUG_VGA_RENDER
					static int render_diag = 0;
					if (line == 255 && xi == 10 && render_diag < 5) {
						uint32_t addr_check = pos + xi;
						LOG("DIAG: rgb8 render line=%d pos=%d xi=%d addr=%06x "
							"memval=%02x pen=%08x offset=%d start_addr=%06x start_shift=%d\n",
							line, pos, xi, addr_check,
							vga.memory[addr_check], pen(vga.memory[addr_check]),
							offset(), vga.crtc.start_addr, start_shift);
						render_diag++;
					}
#endif
				}
			}
		}
	}
}

void CVGA::svga_vh_rgb15(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	constexpr uint32_t IV = 0xff000000;
	const int height = vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1);

	const int TLINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);
	const int TGA_COLUMNS = vga.crtc.horz_disp_end + 1;
	/* line compare is screen sensitive */
//  uint16_t mask_comp = 0xff | (TLINES & 0x300);
	int curr_addr = 0;
	int yi = 0;

	for (int addr = vga.crtc.start_addr << 2, line = 0; line < TLINES; line += height, addr += offset(), curr_addr += offset())
	{
		uint32_t* const bitmapline = &bitmap.pix(line);
		addr %= vga.svga_intf.vram_size;
		for (int pos = addr, c = 0, column = 0; column < TGA_COLUMNS; column++, c += 8, pos += 0x10)
		{
			if (pos + 0x10 >= vga.svga_intf.vram_size)
				return;
			for (int xi = 0, xm = 0; xi < 8; xi++, xm += 2)
			{
				const int dest_x = c + xi;

				if (!screen().visible_area().contains(dest_x, line + yi))
					continue;

				const u16 MV = (vga.memory[pos + xm] + (vga.memory[pos + xm + 1] << 8));
				int r = (MV & 0x7c00) >> 10;
				int g = (MV & 0x03e0) >> 5;
				int b = (MV & 0x001f) >> 0;
				r = (r << 3) | (r & 0x7);
				g = (g << 3) | (g & 0x7);
				b = (b << 3) | (b & 0x7);
				bitmapline[dest_x] = IV | (r << 16) | (g << 8) | (b << 0);
			}
		}
	}
}

void CVGA::svga_vh_rgb16(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	constexpr uint32_t IV = 0xff000000;
	const int height = vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1);
	const int TLINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);
	const int TGA_COLUMNS = vga.crtc.horz_disp_end + 1;

	/* line compare is screen sensitive */
//  uint16_t mask_comp = 0xff | (TLINES & 0x300);
	int curr_addr = 0;
	int yi = 0;

	for (int addr = vga.crtc.start_addr << 2, line = 0; line < TLINES; line += height, addr += offset(), curr_addr += offset())
	{
		uint32_t* const bitmapline = &bitmap.pix(line);
		addr %= vga.svga_intf.vram_size;
		for (int pos = addr, c = 0, column = 0; column < TGA_COLUMNS; column++, c += 8, pos += 0x10)
		{
			if (pos + 0x10 >= vga.svga_intf.vram_size)
				return;
			for (int xi = 0, xm = 0; xi < 8; xi++, xm += 2)
			{
				const int dest_x = c + xi;

				if (!screen().visible_area().contains(dest_x, line + yi))
					continue;

				const u16 MV = (vga.memory[pos + xm] + (vga.memory[pos + xm + 1] << 8));
				int r = (MV & 0xf800) >> 11;
				int g = (MV & 0x07e0) >> 5;
				int b = (MV & 0x001f) >> 0;
				r = (r << 3) | (r & 0x7);
				g = (g << 2) | (g & 0x3);
				b = (b << 3) | (b & 0x7);
				bitmapline[dest_x] = IV | (r << 16) | (g << 8) | (b << 0);
			}
		}
	}
}

void CVGA::svga_vh_rgb24(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	constexpr uint32_t ID = 0xff000000;
	const int height = vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1);
	const int TLINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);
	const int TGA_COLUMNS = vga.crtc.horz_disp_end + 1;

	/* line compare is screen sensitive */
//  uint16_t mask_comp = 0xff | (TLINES & 0x300);
	int curr_addr = 0;
	int yi = 0;

	for (int addr = vga.crtc.start_addr << 3, line = 0; line < TLINES; line += height, addr += offset(), curr_addr += offset())
	{
		uint32_t* const bitmapline = &bitmap.pix(line);
		addr %= vga.svga_intf.vram_size;
		for (int pos = addr, c = 0, column = 0; column < TGA_COLUMNS; column++, c += 8, pos += 24)
		{
			if (pos + 24 >= vga.svga_intf.vram_size)
				return;
			for (int xi = 0, xm = 0; xi < 8; xi++, xm += 3)
			{
				const int dest_x = c + xi;

				if (!screen().visible_area().contains(dest_x, line + yi))
					continue;

				const u32 MD = (vga.memory[pos + xm] + (vga.memory[pos + xm + 1] << 8) + (vga.memory[pos + xm + 2] << 16));

				int r = (MD & 0xff0000) >> 16;
				int g = (MD & 0x00ff00) >> 8;
				int b = (MD & 0x0000ff) >> 0;
				bitmapline[dest_x] = ID | (r << 16) | (g << 8) | (b << 0);
			}
		}
	}
}

void CVGA::svga_vh_rgb32(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	constexpr uint32_t ID = 0xff000000;
	const int height = vga.crtc.maximum_scan_line * (vga.crtc.scan_doubling + 1);
	const int TLINES = (vga.crtc.vert_disp_end + 1) * (get_interlace_mode() + 1);
	const int TGA_COLUMNS = vga.crtc.horz_disp_end + 1;

	//  uint16_t mask_comp;

		/* line compare is screen sensitive */
	//  mask_comp = 0xff | (TLINES & 0x300);
	int curr_addr = 0;
	int yi = 0;

	for (int addr = vga.crtc.start_addr << 2, line = 0; line < TLINES; line += height, addr += offset(), curr_addr += offset())
	{
		uint32_t* const bitmapline = &bitmap.pix(line);
		addr %= vga.svga_intf.vram_size;
		for (int pos = addr, c = 0, column = 0; column < TGA_COLUMNS; column++, c += 8, pos += 0x20)
		{
			if (pos + 0x20 >= vga.svga_intf.vram_size)
				return;
			for (int xi = 0, xm = 0; xi < 8; xi++, xm += 4)
			{
				if (!screen().visible_area().contains(c + xi, line + yi))
					continue;

				const u32 MD = (vga.memory[pos + xm] + (vga.memory[pos + xm + 1] << 8) + (vga.memory[pos + xm + 2] << 16));
				int r = (MD & 0xff0000) >> 16;
				int g = (MD & 0x00ff00) >> 8;
				int b = (MD & 0x0000ff) >> 0;
				bitmapline[c + xi] = ID | (r << 16) | (g << 8) | (b << 0);
			}
		}
	}
}

uint8_t CVGA::pc_vga_choosevideomode()
{
	if (vga.crtc.sync_en)
	{
		if (vga.dac.dirty)
		{
			palette_update();
			vga.dac.dirty = 0;
		}

		if (vga.attribute.data[0x10] & 0x80)
		{
			for (int i = 0; i < 16; i++)
			{
				vga.pens[i] = pen((vga.attribute.data[i] & 0x0f)
					| ((vga.attribute.data[0x14] & 0xf) << 4));
			}
		}
		else
		{
			for (int i = 0; i < 16; i++)
			{
				vga.pens[i] = pen((vga.attribute.data[i] & 0x3f)
					| ((vga.attribute.data[0x14] & 0xc) << 4));
			}
		}

		if (svga.rgb32_en)
		{
			return RGB32_MODE;
		}
		else if (svga.rgb24_en)
		{
			return RGB24_MODE;
		}
		else if (svga.rgb16_en)
		{
			return RGB16_MODE;
		}
		else if (svga.rgb15_en)
		{
			return RGB15_MODE;
		}
		else if (svga.rgb8_en)
		{
			return RGB8_MODE;
		}
		else if (!vga.gc.alpha_dis) // !GRAPHIC_MODE
		{
			return TEXT_MODE;
		}
		else if (vga.gc.shift256)
		{
			return VGA_MODE;
		}
		else if (vga.gc.shift_reg)
		{
			return CGA_MODE;
		}
		else if (vga.gc.memory_map_sel == 0x03)
		{
			return MONO_MODE;
		}
		else
		{
			return EGA_MODE;
		}
	}

	return SCREEN_OFF;
}

uint32_t CVGA::screen_update(bitmap_rgb32& bitmap, const rectangle& cliprect)
{
	uint8_t cur_mode = pc_vga_choosevideomode();

	switch (cur_mode)
	{
	case SCREEN_OFF:   bitmap.fill(black_pen(), cliprect); break;
	case TEXT_MODE:    vga_vh_text(bitmap, cliprect); break;
	case VGA_MODE:     vga_vh_vga(bitmap, cliprect); break;
	case EGA_MODE:     vga_vh_ega(bitmap, cliprect); break;
	case CGA_MODE:     vga_vh_cga(bitmap, cliprect); break;
	case MONO_MODE:    vga_vh_mono(bitmap, cliprect); break;
	case RGB8_MODE:    svga_vh_rgb8(bitmap, cliprect); break;
	case RGB15_MODE:   svga_vh_rgb15(bitmap, cliprect); break;
	case RGB16_MODE:   svga_vh_rgb16(bitmap, cliprect); break;
	case RGB24_MODE:   svga_vh_rgb24(bitmap, cliprect); break;
	case RGB32_MODE:   svga_vh_rgb32(bitmap, cliprect); break;
	}

	return 0;
}

// recompute_params_clock

void CVGA::recompute_params()
{
//	if (vga.miscellaneous_output & 8)
//		LOGWARN("Warning: VGA external clock latch selected\n");
//	else
//		recompute_params_clock(1, ((vga.miscellaneous_output & 0xc) ? XTAL(28'636'363) : XTAL(25'174'800)).value());
}

uint8_t CVGA::vga_vblank()
{
	uint8_t res;
	uint16_t vblank_start, vblank_end, vpos;

	/* calculate vblank start / end positions */
	res = 0;
	vblank_start = vga.crtc.vert_blank_start;
	vblank_end = vga.crtc.vert_blank_start + vga.crtc.vert_blank_end;
	vpos = screen().vpos();

	/* check if we are under vblank period */
	if (vblank_end > vga.crtc.vert_total)
	{
		vblank_end -= vga.crtc.vert_total;
		if (vpos >= vblank_start || vpos <= vblank_end)
			res = 1;
	}
	else
	{
		if (vpos >= vblank_start && vpos <= vblank_end)
			res = 1;
	}

	//popmessage("%d %d %d - SR1=%02x",vblank_start,vblank_end,vga.crtc.vert_total,vga.sequencer.data[1]);

	return res;
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
