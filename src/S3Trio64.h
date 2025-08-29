/* ES40 emulator.
 * Copyright (C) 2007-2025 by the ES40 Emulator Project & Others
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
  * Contains the definitions for emulated S3 Trio 64 Video Card device.
  *
  * $Id$
  *
  * X-1.12       Camiel Vanderhoeven                             31-MAY-2008
  *      Changes to include parts of Poco.
  *
  * X-1.11       Camiel Vanderhoeven                             13-MAR-2008
  *      Create init() start_threads() and stop_threads() functions.
  *
  * X-1.10       Camiel Vanderhoeven                             05-MAR-2008
  *      Multi-threading version.
  *
  * X-1.9        Camiel Vanderhoeven                             20-JAN-2008
  *      Added X11 GUI.
  *
  * X-1.8        Camiel Vanderhoeven                             08-JAN-2008
  *      Comments.
  *
  * X-1.7        Camiel Vanderhoeven                             02-JAN-2008
  *      Cleanup.
  *
  * X-1.6        Camiel Vanderhoeven                             28-DEC-2007
  *      Keep the compiler happy.
  *
  * X-1.5        Camiel Vanderhoeven                             17-DEC-2007
  *      SaveState file format 2.1
  *
  * X-1.4        Brian Wheeler                                   10-DEC-2007
  *      Added SDL.h.
  *
  * X-1.3        Camiel Vanderhoeven                             10-DEC-2007
  *      Use new base class VGA.
  *
  * X-1.2        Camiel Vanderhoeven/Brian Wheeler               6-DEC-2007
  *      Changed implementation (with thanks to the Bochs project!!)
  *
  * X-1.1        Camiel Vanderhoeven                             1-DEC-2007
  *      Initial version in CVS.
  **/
#if !defined(INCLUDED_S3Trio64_H_)
#define INCLUDED_S3Trio64_H_

#include "VGA.h"
#include "gui/vga.h"

  /* video card has 4M of ram */
#define VIDEO_RAM_SIZE  22
#define CRTC_MAX        0x70

/**
 * \brief S3 Trio 64 Video Card
 *
 * Documentation consulted:
 *  - VGADOC4b
 *   (http://home.worldonline.dk/~finth/)
 *  .
 **/
class CS3Trio64 : public CVGA, public CRunnable
{
public:
  virtual int   SaveState(FILE* f);
  virtual int   RestoreState(FILE* f);
  virtual void  check_state();
  virtual void  WriteMem_Legacy(int index, u32 address, int dsize, u32 data);
  virtual u32   ReadMem_Legacy(int index, u32 address, int dsize);

  virtual void  WriteMem_Bar(int func, int bar, u32 address, int dsize,
    u32 data);
  virtual u32   ReadMem_Bar(int func, int bar, u32 address, int dsize);

  CS3Trio64(CConfigurator* cfg, class CSystem* c, int pcibus, int pcidev);
  virtual       ~CS3Trio64();

  void          update(void);
  virtual void  run(void);

  virtual u8    get_actl_palette_idx(u8 index);
  virtual void  redraw_area(unsigned x0, unsigned y0, unsigned width,
    unsigned height);

  virtual void  init();
  virtual void  start_threads();
  virtual void  stop_threads();
private:
  u32   mem_read(u32 address, int dsize);
  void  mem_write(u32 address, int dsize, u32 data);

  // accel I/O (S3 Trio uses 0x42E8/0x4AE8)
  void          AccelIOWrite(u32 port, u8 data);
  u8            AccelIORead(u32 port);
  // Execute a pending 2-D command (BitBLT/fill, minimal subset).
  void    AccelExecute();
  bool    IsAccelPort(u32 port) const;
  int     BytesPerPixel() const;
  u32     PitchBytes() const;   // from CRTC 13h + hi bits

  // Register helpers
  void recompute_scanline_layout();
  void recompute_line_offset();
  inline uint32_t compose_display_start() const;
  void recompute_data_transfer_position();
  inline uint8_t current_char_width_px() const;
  void recompute_interlace_retrace_start();
  void recompute_external_sync_1();
  void recompute_external_sync_2();
  void recompute_external_sync_3();

  u32   io_read(u32 address, int dsize);
  void  io_write(u32 address, int dsize, u32 data);

  void  io_write_b(u32 address, u8 data);

  void  write_b_3c0(u8 data);
  void  write_b_3c2(u8 data);
  void  write_b_3c3(u8 data);
  void  write_b_3c4(u8 data);
  void  write_b_3c5(u8 data);
  void  write_b_3c6(u8 data);
  void  write_b_3c7(u8 data);
  void  write_b_3c8(u8 data);
  void  write_b_3c9(u8 data);
  void  write_b_3ce(u8 data);
  void  write_b_3cf(u8 data);
  void  write_b_3d4(u8 data);
  void  write_b_3d5(u8 data);
  void  write_b_3da(u8 data);

  u8    read_b_3c0();
  u8    read_b_3c1();
  u8    read_b_3c2();
  u8    read_b_3c3();
  u8    read_b_3c4();
  u8    read_b_3c5();
  u8    read_b_3c6();
  u8    read_b_3c7();
  u8    read_b_3c8();
  u8    read_b_3c9();
  u8    read_b_3ca();
  u8    read_b_3cc();
  u8    read_b_3cf();
  u8    read_b_3d4();
  u8    read_b_3d5();
  u8    read_b_3da();

  u32   legacy_read(u32 address, int dsize);
  void  legacy_write(u32 address, int dsize, u32 data);

  u32   rom_read(u32 address, int dsize);

  void  determine_screen_dimensions(unsigned* piHeight, unsigned* piWidth);

  char  bios_message[200];
  int   bios_message_size;

  void  vga_mem_write(u32 addr, u8 value);
  u8    vga_mem_read(u32 addr);

  CThread* myThread;
  bool  StopThread;

  /// The state structure contains all elements that need to be saved to the statefile.
  struct SS3_state
  {
    bool      vga_enabled;
    bool      vga_mem_updated;
    u16       charmap_address;
    bool      x_dotclockdiv2;
    bool      y_doublescan;
    unsigned  line_offset;
    unsigned  line_compare;
    unsigned  vertical_display_end;
    uint16_t  h_total;         // derived from CR00 (+CR5D ext)
    uint16_t  h_display_end;   // CR01 (+CR5D ext)
    uint16_t  h_blank_start;   // CR02 (+CR5D ext)
    uint16_t  h_blank_end;     // CR03 (+CR5D ext bits)
    uint16_t  h_sync_start;    // CR04 (+CR5D ext)
    uint16_t  h_sync_end;      // CR05 (+CR5D ext bits)
    u8        text_snapshot[32 * 1024];           // current text snapshot
    bool      vga_tile_updated[BX_NUM_X_TILES][BX_NUM_Y_TILES];
    u8* memory;
    u32       memsize;
    u32       vram_display_mask; // active displayable VRAM mask
    u8        last_bpp;
    u8        tile[X_TILESIZE * Y_TILESIZE * 4];  /**< Currently allocates the tile as large as needed. */
    unsigned  x_tilesize;
    unsigned  y_tilesize;
    u8        port3da;
    // --- S3 Data Transfer Position (CR3B) derived state ---
    bool     dtp_enabled;        // CR34 bit4
    uint8_t  dtp_raw;            // CR3B value as written
    uint16_t dtp_hpos_chars;     // CR3B in character clocks (0..255)
    uint16_t dtp_hpos_pixels;    // CR3B converted to pixels (uses current char width)
    // --- S3 Interlace Retrace Start (CR3C) derived state ---
    bool     ilrt_enabled;     // enabled by CR42 bit5
    uint8_t  ilrt_raw;         // CR3C value (chars)
    uint16_t ilrt_chars;       // position in character clocks
    uint16_t ilrt_pixels;      // position in pixels (uses current char width)
    // --- External Sync Control 1 (CR56) derived state ---
    uint8_t   exsync1;            // raw CR56
    bool      exsync_remote;      // bit0: Remote Mode (genlock)
    bool      hsync_drive;        // bit1: 1 = HSYNC driver enabled, 0 = tri-stated
    bool      vsync_drive;        // bit2: 1 = VSYNC driver enabled, 0 = tri-stated
    bool      exsync_vreset_only; // bit3: vertical-only reset when genlocked
    bool      exsync_preset_odd;  // bit4: preset odd field after V reset (with genlock)
    bool      exsync_blank;       // derived: (!hsync_drive) || (!vsync_drive)

    // --- External Sync Control 2 (CR57) derived state ---
    uint8_t   exsync2_raw;          // CR57 as written (0..255 lines)
    uint8_t   exsync2_delay_lines;  // effective delay lines (>=1 when Remote=1)

    // --- External Sync Control 3 (CR63) derived state ---
    uint8_t   exsync3_raw;              // CR63 as written
    uint8_t   exsync3_hsreset_chars;    // low  nibble: HSYNC reset delay (chars)
    uint8_t   exsync3_charclk_delay;    // high nibble: char-clock reset delay (DCLKs) at end-of-line
    bool      exsync3_active;           // Remote mode gate (true when CR56 bit0=1)




    struct SS3_attr
    {
      bool      flip_flop;    /* 0 = address, 1 = data-write */
      unsigned  address;      /* register number */
      u8        video_enabled = 0x1;
      u8        palette_reg[16];
      u8        overscan_color;
      u8        color_plane_enable;
      u8        horiz_pel_panning;
      u8        color_select;
      struct SS3_mode
      {
        bool  graphics_alpha;
        bool  display_type;
        bool  enable_line_graphics;
        bool  blink_intensity;
        bool  pixel_panning_compat;
        bool  pixel_clock_select;
        bool  internal_palette_size;
      } mode_ctrl;
    } attribute_ctrl;

    struct SS3_misc
    {
      bool  color_emulation;  // 1=color emulation, base address = 3Dx

      // 0=mono emulation,  base address = 3Bx
      bool  enable_ram;       // enable CPU access to video memory if set
      u8    clock_select;     // 0=25Mhz 1=28Mhz
      bool  select_high_bank; // when in odd/even modes, select

      // high 64k bank if set
      bool  horiz_sync_pol;   // bit6: negative if set
      bool  vert_sync_pol;    // bit7: negative if set

      //   bit7,bit6 represent number of lines on display:
      //   0 = reserved
      //   1 = 400 lines
      //   2 = 350 lines
      //   3 - 480 lines
    } misc_output;

    struct SS3_seq
    {
      u8    index;
      u8    map_mask;
      bool  map_mask_bit[4];
      bool  reset1;
      bool  reset2;
      u8    reg1;
      u8    char_map_select;
      bool  extended_mem;
      bool  odd_even;
      bool  chain_four;
      u8    pll_lock;
      u8    mclkn; // Memory PLL Data Low (SR10)
      u8    mclkr; // SR10 continued
      u8    srA;   // External Bus Request Control Register (SRA)
      u8    srB;   // Miscellaneous External Sequencer Register (SRB)
      u8    srD;   // Extended Sequencer Register (EX_SR_D) (SRD)
      u8    sr9;   // Extended Sequence Register 9 (SR9)
      u8    sr10;
      u8    mclkm; // Memory PLL Data high (SR11)
      u8    sr11;  // store twice... two different implementation handling
      u8    clk3n; // Video PLL Data low (SR12)
      u8    clk3r; // SR12 continued
      u8    sr12; // store twice... two different implementation handling
      u8    sr13; // video PLL Data high
      u8    sr14;
      u8    sr15; // CLKSYN Control 2 Register (SR15)
      u8    sr18;
      u8    sr1a;
      u8    sr1b;
    } sequencer;

    struct SS3_pel
    {
      u8  write_data_register;
      u8  write_data_cycle;   /* 0, 1, 2 */
      u8  read_data_register;
      u8  read_data_cycle;    /* 0, 1, 2 */
      u8  dac_state;
      struct SS3_pel_data
      {
        u8  red;
        u8  green;
        u8  blue;
      } data[256];
      u8  mask;
    } pel;

    struct SS3_gfx
    {
      u8    index;
      u8    set_reset;
      u8    enable_set_reset;
      u8    color_compare;
      u8    data_rotate;
      u8    raster_op;
      u8    read_map_select;
      u8    write_mode;
      bool  read_mode;
      bool  odd_even;
      bool  chain_odd_even;
      u8    shift_reg;
      bool  graphics_alpha;
      u8    memory_mapping;   /* 0 = use A0000-BFFFF
                               * 1 = use A0000-AFFFF EGA/VGA graphics modes
                               * 2 = use B0000-B7FFF Monochrome modes
                               * 3 = use B8000-BFFFF CGA modes
                               */
      u8    color_dont_care;
      u8    bitmask;
      u8    latch[4];
    } graphics_ctrl;

    struct SS3_crtc
    {
      u8    address;
      u8    reg[CRTC_MAX];
      bool  write_protect;
    } CRTC;

    // Minimal 2-D engine skeleton (safe while disabled)
    struct {
      // 8514/A compat windows (S3 reused scheme)
      u16    subsys_cntl;     // write path (42E8)
      u16    subsys_stat;     // read path (42E8)
      u16    advfunc_cntl;    // misc control (4AE8)
      // Commonly ops
      u16    cur_x, cur_y;
      u16    dest_x, dest_y;
      u16    maj_axis_pcnt;
      u16    destx_distp;
      u16    desty_axstp;
      u16    cmd;             // start + op bits
      u32    frgd_color, bkgd_color;
      u32    wrt_mask, rd_mask;
      u8     frgd_mix, bkgd_mix;    // ROP2
      // State
      bool   enabled;         // expose accel ports?
      bool   busy;            // engine busy flag
    } accel;

  } state;
};

// keep it out... for now
#ifndef ES40_S3_ACCEL_ENABLE
#define ES40_S3_ACCEL_ENABLE 1
#endif

// ----- Debug tracing for Data Transfer Position (CR3B/CR34 bit4) -----
#ifndef S3_TRACE_DTP
#define S3_TRACE_DTP 1
#endif
#if S3_TRACE_DTP
#define DTP_TRACE(...) do { printf(__VA_ARGS__); } while (0)
#else
#define DTP_TRACE(...) do {} while (0)
#endif

// ----- Debug tracing for Interlace Retrace Start (CR3C / CR42 bit5) -----
#ifndef S3_TRACE_ILRT
#define S3_TRACE_ILRT 1
#endif
#if S3_TRACE_ILRT
#define ILRT_TRACE(...) do { printf(__VA_ARGS__); } while (0)
#else
#define ILRT_TRACE(...) do {} while (0)
#endif

#ifndef S3_TRACE_EXSYNC1
#define S3_TRACE_EXSYNC1 1
#endif
#if S3_TRACE_EXSYNC1
#define EX1_TRACE(...) do { printf(__VA_ARGS__); } while (0)
#else
#define EX1_TRACE(...) do {} while (0)
#endif

#ifndef S3_TRACE_EXSYNC2
#define S3_TRACE_EXSYNC2 1
#endif
#if S3_TRACE_EXSYNC2
#define EX2_TRACE(...) do { printf(__VA_ARGS__); } while (0)
#else
#define EX2_TRACE(...) do {} while (0)
#endif

#ifndef S3_TRACE_EXSYNC3
#define S3_TRACE_EXSYNC3 1
#endif
#if S3_TRACE_EXSYNC3
#define EX3_TRACE(...) do { printf(__VA_ARGS__); } while (0)
#else
#define EX3_TRACE(...) do {} while (0)
#endif

#endif // !defined(INCLUDED_S3Trio64_H_)
