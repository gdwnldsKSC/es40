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
  * Contains the definitions for the VGA base class.
  *
  * $Id$
  *
  * X-1.4        Camiel Vanderhoeven                             20-JAN-2008
  *      Added X11 GUI.
  *
  * X-1.3        Camiel Vanderhoeven                             02-JAN-2008
  *      Comments.
  *
  * X-1.2        Brian Wheeler                                   10-DEC-2007
  *      Made include's case-correct.
  *
  * X-1.1        Camiel Vanderhoeven                             10-DEC-2007
  *      Initial version in CVS.
  **/
#if !defined(__VGA_H__)
#define __VGA_H__

#include "PCIDevice.h"

  /**
   * \brief Abstract base class for PCI VGA cards.
   **/
class CVGA : public CPCIDevice
{
public:
  CVGA(class CConfigurator* cfg, class CSystem* c, int pcibus, int pcidev);
  ~CVGA(void);

  virtual u8    get_actl_palette_idx(u8 index) = 0;
  virtual void  redraw_area(unsigned x0, unsigned y0, unsigned width,
    unsigned height) = 0;

  virtual uint8_t mem_r(uint32_t offset);
  virtual void    mem_w(uint32_t offset, uint8_t data);
  virtual uint8_t mem_linear_r(uint32_t offset);
  virtual void    mem_linear_w(uint32_t offset, uint8_t data);

protected:
  struct vga_t
  {
    //vga_t(device_t &owner) { }
    struct
    {
      size_t vram_size;
    } svga_intf;

    // Non-owning pointer to the actual VRAM allocation.
    // Subclass sets this to its own backing store during init().
    // Layout: planar — plane 0 at +0, plane 1 at +0x10000,
    //                  plane 2 at +0x20000, plane 3 at +0x30000.
    uint8_t* memory = nullptr;

    uint32_t pens[16]; /* the current 16 pens */

    uint8_t miscellaneous_output;
    uint8_t feature_control;

    struct
    {
      uint8_t index;
      uint8_t data[0x100];
      uint8_t map_mask;
      struct
      {
        uint8_t A, B;
        u32 base[2];
      }char_sel;
    } sequencer;

    /* An empty comment at the start of the line indicates that register is currently unused */
    struct
    {
      uint8_t index;
      uint8_t data[0x100];
      uint16_t horz_total;
      uint16_t horz_disp_end;
      /**/    uint8_t horz_blank_start;
      /**/    uint8_t horz_blank_end;
      /**/    uint8_t horz_retrace_start;
      /**/    uint8_t horz_retrace_skew;
      /**/    uint8_t horz_retrace_end;
      /**/    uint8_t disp_enable_skew;
      /**/    uint8_t evra;
      uint16_t vert_total;
      uint16_t vert_disp_end;
      /**/    uint16_t vert_retrace_start;
      /**/    uint8_t vert_retrace_end;
      uint16_t vert_blank_start;
      uint16_t line_compare;
      uint32_t cursor_addr;
      /**/    uint8_t byte_panning;
      uint8_t preset_row_scan;
      uint8_t scan_doubling;
      uint8_t maximum_scan_line;
      uint8_t cursor_enable;
      uint8_t cursor_scan_start;
      /**/    uint8_t cursor_skew;
      uint8_t cursor_scan_end;
      uint32_t start_addr;
      uint32_t start_addr_latch;
      uint8_t protect_enable;
      /**/    uint8_t bandwidth;
      uint16_t offset;
      uint8_t word_mode;
      uint8_t dw;
      /**/    uint8_t div4;
      /**/    uint8_t underline_loc;
      uint16_t vert_blank_end;
      uint8_t sync_en;
      /**/    uint8_t aw;
      uint8_t div2;
      /**/    uint8_t sldiv;
      /**/    uint8_t map14;
      /**/    uint8_t map13;
      /**/    uint8_t irq_clear;
      /**/    uint8_t irq_disable;
      uint8_t irq_latch;
      uint8_t no_wrap;
    } crtc;

    struct
    {
      uint8_t index;
      uint8_t latch[4];
      uint8_t set_reset;
      uint8_t enable_set_reset;
      uint8_t color_compare;
      uint8_t logical_op;
      uint8_t rotate_count;
      uint8_t shift256;
      uint8_t shift_reg;
      uint8_t read_map_sel;
      uint8_t read_mode;
      uint8_t write_mode;
      uint8_t color_dont_care;
      uint8_t bit_mask;
      uint8_t alpha_dis;
      uint8_t memory_map_sel;
      uint8_t host_oe;
      uint8_t chain_oe;
    } gc;

    struct
    {
      uint8_t index, data[0x15]; int state;
      uint8_t prot_bit;
      uint8_t pel_shift;
      uint8_t pel_shift_latch;
    } attribute;

    struct {
      uint8_t read_index, write_index, mask;
      int read;
      int state;
      uint8_t color[0x300]; /* flat RGB triplets */
      int dirty;
      uint8_t loading[3];
    } dac;

    struct {
      uint8_t visible;
    } cursor;

    /* oak vga */
    struct { uint8_t reg; } oak;
  } vga;

  virtual uint8_t vga_latch_write(int offs, uint8_t data);
  
  inline uint8_t rotate_right(uint8_t val)
  {
    return (val >> vga.gc.rotate_count)
      | (val << (8 - vga.gc.rotate_count));
  }

  inline uint8_t vga_logical_op(uint8_t data, uint8_t plane, uint8_t mask)
  {
    uint8_t res = 0;
    switch (vga.gc.logical_op & 3)
    {
    case 0: /* NONE */
      res = (data & mask) | (vga.gc.latch[plane] & ~mask);
      break;
    case 1: /* AND */
      res = (data | ~mask) & (vga.gc.latch[plane]);
      break;
    case 2: /* OR */
      res = (data & mask) | (vga.gc.latch[plane]);
      break;
    case 3: /* XOR */
      res = (data & mask) ^ (vga.gc.latch[plane]);
      break;
    }
    return res;
  }

  // Subclasses may override these (S3 does for SVGA pitch, etc.)
  virtual uint16_t offset();
  virtual uint32_t latch_start_addr() { return vga.crtc.start_addr_latch; }
  virtual bool     get_interlace_mode() { return false; }
  virtual void     palette_update();
};

extern CVGA* theVGA;
#endif
