/* ES40 emulator.
 * Copyright (C) 2007-2025 by the ES40 Emulator Project & Others
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
#include <atomic>
#include "coretmpl.h"
#include "attotime.h"
#include "mame_shims.h"
#include "address_map.h"

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
class CS3Trio64 : public CVGA, public CRunnable, public mame_machine_provider
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

  virtual u64 ReadMem(int index, u64 address, int dsize) override;
  virtual void WriteMem(int index, u64 address, int dsize, u64 data) override;


  // Observe PCI config-space accesses (BARs and COMMAND)
  u32  config_read_custom(int func, u32 address, int dsize, u32 cur) override;
  void config_write_custom(int func, u32 address, int dsize, u32 old_data, u32 new_data, u32 raw) override;



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
protected:
  enum
  {
    SCREEN_OFF = 0,
    TEXT_MODE,
    VGA_MODE,
    EGA_MODE,
    CGA_MODE,
    MONO_MODE,
    RGB8_MODE,
    RGB15_MODE,
    RGB16_MODE,
    RGB24_MODE,
    RGB32_MODE
  };

  // MAME VGA STRUCT
  struct vga_t
  {
    //vga_t(device_t &owner) { }
    struct
    {
      size_t vram_size;
    } svga_intf;

    //    std::unique_ptr<uint8_t[]> memory;
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

  virtual bool get_interlace_mode() { return false; }

  nop_callback m_vsync_cb;

  address_map m_crtc_map{ 256 };
  address_map m_seq_map{ 256 };
  // address_map m_gc_map{256};    // to be added
  // address_map m_atc_map{64};    // to be added

  void crtc_map(address_map& map);
  //void sequencer_map(address_map& map);

  void s3_define_video_mode();
  void refresh_pitch_offset();
  void recompute_params();

  void init_maps() {
    crtc_map(m_crtc_map);
    //sequencer_map(m_seq_map);
  }

private:
  u32   mem_read(u32 address, int dsize);
  void  mem_write(u32 address, int dsize, u32 data);


  // Keep SDL window alive across firmware reset:
  //  - PauseThread is set by stop_threads() when system reset is in progress
  //  - PauseAck is raised by the S3 thread once it is safely paused
  std::atomic<bool> PauseThread{ false };
  std::atomic<bool> PauseAck{ false };

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
  void recompute_ext_misc_ctl(); // CR65
  void recompute_config3(); // CR68
  void recompute_params_clock(int divisor, int xtal);
  void s3_sync_from_crtc();          // bulk-sync s3 struct from CRTC array

  // --- Rendering helpers (member functions; can access private 'state') ---
  // Compose the HW cursor over a prepared 8-bit tile (RGB332 indices in >8bpp).
  void  overlay_hw_cursor_on_tile(u8* tile8,
    unsigned xc, unsigned yc,
    unsigned tile_w, unsigned tile_h,
    unsigned tile_x0, unsigned tile_y0,
    int bpp_now, unsigned pitch_bytes,
    unsigned start_addr);
  // Ensure the GUI palette is the 256-entry RGB332 cube once when >8bpp.
  void  ensure_rgb332_palette_loaded();


  void  update_linear_mapping();
  void  on_crtc_linear_regs_changed();


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

  inline uint32_t s3_vram_mask() const;
  inline uint8_t  s3_vram_read8(uint32_t addr) const;
  inline void     s3_vram_write8(uint32_t addr, uint8_t v);

  void lfb_recalc_and_cache();  // recompute enable/base/size from COMMAND+BAR0 (and CR regs if you wish)
  void trace_lfb_if_changed(const char* reason);



  // cached state for LFB
  u32  lfb_base_ = 0;
  u32  lfb_size_ = 0;
  bool lfb_enabled_ = false;


  // LFB bookkeeping
  enum { DEV_LFB_IDX = 6 };      // free in this device (legacy used 4/5/7 etc.)
  u32   lfb_base = 0;            // guest-visible base (32-bit)
  u32   lfb_size = 0;            // 64K/1M/2M/4M
  u64   lfb_phys = 0;            // full physical mapping base we registered
  bool  lfb_active = false;      // effective enable (PCI + CR58)

  bool  pci_mem_enable = false;  // PCI Command.MSE cached
  u32   pci_bar0 = 0;            // cached BAR0 (optional; we treat CR58..5A as truth)

  void  lfb_recalc_and_map();    // (un)map according to CR58..5A & PCI
  inline u32 lfb_offset_from(u64 phys_addr) const {
    const u64 off = phys_addr - lfb_phys;
    return (u32)(off % state.memsize); // VRAM wraps modulo real size
  }

  bool lfb_trace_needs_first_access_note = false;
  bool lfb_trace_initialized = false;
  bool lfb_trace_enabled_prev = false;
  uint32_t lfb_trace_base_prev = 0;
  uint32_t lfb_trace_size_prev = 0;

  CThread* myThread;
  bool  StopThread;

  /// The state structure contains all elements that need to be saved to the statefile.
  struct SS3_state
  {
    bool      vga_enabled;
    // When >8bpp scanout is used we quantize to RGB332 and need to
    // load a fixed 256-color palette once.
    bool      tc_rgb332_palette_loaded;   // default 0 via memset
    bool      vga_mem_updated;
    u16       charmap_address;
    bool      x_dotclockdiv2;
    unsigned  line_offset;
    uint16_t  h_blank_start;   // CR02 (+CR5D ext)
    uint16_t  h_blank_end;     // CR03 (+CR5D ext bits)
    uint16_t  h_sync_start;    // CR04 (+CR5D ext)
    uint16_t  h_sync_end;      // CR05 (+CR5D ext bits)
    u8        text_snapshot[32 * 1024];           // current text snapshot
    bool      vga_tile_updated[BX_NUM_X_TILES][BX_NUM_Y_TILES];
    u8* memory; // the actual vram... probably should have notated this earlier
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

    // --- Extended Misc Control (CR65) ---
    uint8_t cr65_raw;       // as written
    bool    cr65_enb_3c3;   // bit2: 1=use 3C3h for video subsystem setup (no remap here)
    uint8_t cr65_blank_dly; // bits4:3: 0..3 DCLKs (Trio32 feature; latched only on Trio64)

    // --- Configuration 3 (CR68) strap/timing fields ---
    uint8_t   cr68_raw;        // as written / strapped
    uint8_t   cr68_casoe_we;   // bits 1:0  (CAS/OE stretch & WE delay)
    bool      cr68_ras_low;    // bit 2     (RAS low timing select)
    bool      cr68_ras_pcg;    // bit 3     (RAS precharge timing select)
    uint8_t   cr68_mon_inf;    // bits 6:4  (monitor info for BIOS)
    bool      cr68_up_add;     // bit 7     (upper address decode / bus size)



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

      u8   flat; // reconstructed byte, MAME vga.miscellaneous_output
    } misc_output;

    struct SS3_seq
    {
      u8    index;
      u8    map_mask;
      bool  map_mask_bit[4];
      bool  reset1;
      bool  reset2;
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
      u8    clk3m; // M-value - SR13 portion
      u8    sr12; // store twice... two different implementation handling
      u8    sr13; // video PLL Data high
      u8    sr14;
      u8    sr15; // CLKSYN Control 2 Register (SR15)
      u8    sr17; // CLKSYN Test Register (SR17) - decrements on read
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
      u8    memory_mapping;   /* 0 = use A0000-BFFFF
                               * 1 = use A0000-AFFFF EGA/VGA graphics modes
                               * 2 = use B0000-B7FFF Monochrome modes
                               * 3 = use B8000-BFFFF CGA modes
                               */
      u8    color_dont_care;
      u8    bitmask;
      u8    latch[4];
    } graphics_ctrl;

    // -------- Hardware Graphics Cursor (CR45..CR4F) --------
    // Modelled after MAME's s3 fields; see pc_vga_s3.cpp.  
    u8     cursor_mode = 0;       // CR45
    u16    cursor_x = 0;          // CR46/47
    u16    cursor_y = 0;          // CR48/49
    u8     cursor_fg[4] = { 0 };    // CR4A stack, auto-inc pointer
    u8     cursor_bg[4] = { 0 };    // CR4B stack, auto-inc pointer
    u8     hwc_fg_stack_pos = 0;   // CR4A foreground color stack pointer
    u8     hwc_bg_stack_pos = 0;   // CR4B background color stack pointer
    u32    hwc_fg_col = 0, hwc_bg_col = 0;
    u16    cursor_start_addr = 0; // CR4C/4D, in 1 KiB units
    u8     cursor_pattern_x = 0;  // CR4E (0..63)
    u8     cursor_pattern_y = 0;  // CR4F (0..63)



    struct SS3_crtc
    {
      u8    address;
      u8    reg[CRTC_MAX];
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

      // --- host (PIX_TRANS) streaming state ---
      bool     host_xfer_active;
      uint32_t host_total_pixels;
      uint32_t host_pixels_rcvd;
      uint32_t host_bpp;          // 1/2/3/4 based on current mode
      uint32_t host_cur_x, host_cur_y;  // position within the target rect
      uint8_t  host_byte_accum[4];      // up to 32bpp
      uint32_t host_byte_count;         // bytes collected toward one pixel

      u16    cmd;             // start + op bits
      u32    frgd_color, bkgd_color;
      u32    wrt_mask, rd_mask;
      u8     frgd_mix, bkgd_mix;    // ROP2

      // --- newly modeled registers/flags ---
      u16      ropmix;            // 0xD2E8
      u16      pix_cntl;          // 0xB2E8 when not acting as PIX_TRANS
      u16      short_stroke;      // 0x9EE8 - last 16-bit value written to SSV
      u16      multifunc_cntl;    // 0xBEE8 
      u16      multifunc[16];     // only [0xE] bit8 is used for B2E8 gating
      bool     b2e8_as_pixtrans;  // convenience cache for the gate (default true)

      // --- Short Stroke Vectors (0x9EE8/0x9D48) ---
      // Minimal subset: we draw short lines in 8 directions and advance CUR_X/CUR_Y.
      u8     ssv_len;         // helper (decoded from short_stroke byte)
      u8     ssv_dir;         // helper (decoded from short_stroke byte)


      // State
      bool   enabled;         // expose accel ports?
      bool   busy;            // engine busy flag
    } accel;

  } state;

  // MAME-compatible start - this way we can start to re-use code from MAME's
  // pc_vga_s3.cpp / pc_vga_s3.h
  // MAME struct "s3" field mapped to ES40 state

  // S3 CRTC extended registers (MAME: s3.xxx) 
  inline u8& s3_memory_config() { return state.CRTC.reg[0x31]; }
  inline u8        s3_memory_config() const { return state.CRTC.reg[0x31]; }

  inline u8& s3_ext_misc_ctrl_2() { return state.CRTC.reg[0x67]; }
  inline u8        s3_ext_misc_ctrl_2() const { return state.CRTC.reg[0x67]; }

  inline u8& s3_cr3a() { return state.CRTC.reg[0x3A]; }
  inline u8        s3_cr3a() const { return state.CRTC.reg[0x3A]; }

  inline u8& s3_cr42() { return state.CRTC.reg[0x42]; }
  inline u8        s3_cr42() const { return state.CRTC.reg[0x42]; }

  inline u8& s3_cr43() { return state.CRTC.reg[0x43]; }
  inline u8        s3_cr43() const { return state.CRTC.reg[0x43]; }

  inline u8& s3_cr51() { return state.CRTC.reg[0x51]; }
  inline u8        s3_cr51() const { return state.CRTC.reg[0x51]; }

  inline u8& s3_cr53() { return state.CRTC.reg[0x53]; }
  inline u8        s3_cr53() const { return state.CRTC.reg[0x53]; }

  // Lock registers
  inline u8& s3_crt_reg_lock() { return state.CRTC.reg[0x35]; }
  inline u8        s3_crt_reg_lock() const { return state.CRTC.reg[0x35]; }

  inline u8& s3_reg_lock1() { return state.CRTC.reg[0x38]; }
  inline u8        s3_reg_lock1() const { return state.CRTC.reg[0x38]; }

  inline u8& s3_reg_lock2() { return state.CRTC.reg[0x39]; }
  inline u8        s3_reg_lock2() const { return state.CRTC.reg[0x39]; }

  // Extended DAC control (CR55)
  inline u8& s3_extended_dac_ctrl() { return state.CRTC.reg[0x55]; }
  inline u8        s3_extended_dac_ctrl() const { return state.CRTC.reg[0x55]; }

  // Sequencer PLL registers (MAME: s3.srXX / s3.clk_pll_*) 
  inline u8& s3_sr10() { return state.sequencer.sr10; }
  inline u8        s3_sr10() const { return state.sequencer.sr10; }

  inline u8& s3_sr11() { return state.sequencer.sr11; }
  inline u8        s3_sr11() const { return state.sequencer.sr11; }

  inline u8& s3_sr12() { return state.sequencer.sr12; }
  inline u8        s3_sr12() const { return state.sequencer.sr12; }

  inline u8& s3_sr13() { return state.sequencer.sr13; }
  inline u8        s3_sr13() const { return state.sequencer.sr13; }

  inline u8& s3_sr15() { return state.sequencer.sr15; }
  inline u8        s3_sr15() const { return state.sequencer.sr15; }

  inline u8& s3_sr17() { return state.sequencer.sr17; }
  inline u8        s3_sr17() const { return state.sequencer.sr17; }

  // Extracted PLL values (latched on SR15 write)
  inline u8& s3_clk_pll_n() { return state.sequencer.clk3n; }
  inline u8        s3_clk_pll_n() const { return state.sequencer.clk3n; }

  inline u8& s3_clk_pll_r() { return state.sequencer.clk3r; }
  inline u8        s3_clk_pll_r() const { return state.sequencer.clk3r; }

  inline u8& s3_clk_pll_m() { return state.sequencer.clk3m; }
  inline u8        s3_clk_pll_m() const { return state.sequencer.clk3m; }

  // Hardware cursor (MAME: s3.cursor_*) 
  inline u8& s3_cursor_mode() { return state.cursor_mode; }
  inline u8        s3_cursor_mode() const { return state.cursor_mode; }

  inline u16& s3_cursor_x() { return state.cursor_x; }
  inline u16       s3_cursor_x() const { return state.cursor_x; }

  inline u16& s3_cursor_y() { return state.cursor_y; }
  inline u16       s3_cursor_y() const { return state.cursor_y; }

  inline u16& s3_cursor_start_addr() { return state.cursor_start_addr; }
  inline u16       s3_cursor_start_addr() const { return state.cursor_start_addr; }

  inline u8& s3_cursor_pattern_x() { return state.cursor_pattern_x; }
  inline u8        s3_cursor_pattern_x() const { return state.cursor_pattern_x; }

  inline u8& s3_cursor_pattern_y() { return state.cursor_pattern_y; }
  inline u8        s3_cursor_pattern_y() const { return state.cursor_pattern_y; }

  inline u8& s3_cursor_fg(int i) { return state.cursor_fg[i]; }
  inline u8        s3_cursor_fg(int i) const { return state.cursor_fg[i]; }

  inline u8& s3_cursor_bg(int i) { return state.cursor_bg[i]; }
  inline u8        s3_cursor_bg(int i) const { return state.cursor_bg[i]; }

  inline u8& s3_cursor_fg_ptr() { return state.hwc_fg_stack_pos; }
  inline u8        s3_cursor_fg_ptr() const { return state.hwc_fg_stack_pos; }

  inline u8& s3_cursor_bg_ptr() { return state.hwc_bg_stack_pos; }
  inline u8        s3_cursor_bg_ptr() const { return state.hwc_bg_stack_pos; }

  inline bool s3_mmio_enabled(const SS3_state& s);

  // VGA base registers (MAME: vga.miscellaneous_output) 
  // MAME stores as u8, let's rebuild it back for MAME code compat
  inline u8 vga_miscellaneous_output() const {
    return (state.misc_output.color_emulation ? 0x01 : 0)
      | (state.misc_output.enable_ram ? 0x02 : 0)
      | ((state.misc_output.clock_select & 3) << 2)
      | (state.misc_output.select_high_bank ? 0x20 : 0)
      | (state.misc_output.horiz_sync_pol ? 0x40 : 0)
      | (state.misc_output.vert_sync_pol ? 0x80 : 0);
  }

  // MAME S3 state
  struct {
    uint8_t memory_config;      // Memory Configuration (CR31)
    uint8_t ext_misc_ctrl_2;    // Extended Miscellaneous Control 2 Register (EXT-MISC-2)(CR67) 
    uint8_t crt_reg_lock;       // CPU bank + timing locks
    uint8_t reg_lock1;          // CR38 Register Lock 1
    uint8_t reg_lock2;          // CR39 Register Lock 2
    uint8_t enable_8514;        // CR40 system config bit0
    uint8_t enable_s3d;         // not used by trio64? kept for code compat
    uint8_t cr3a;               // Miscellaneous 1 Register (MISC_1) (CR3A) 
    uint8_t cr42;               // Mode Control Register (MODE_CTl) (CR42)
    uint8_t cr43;               // Extended Mode Register (EXT_MODE)
    uint8_t cr51;               // Extended System Control 2
    uint8_t cr53;               // Extended Memory Control 1 Register
    uint8_t id_high;            // Extended Chip ID (CR2D)
    uint8_t id_low;             // Chip ID for S3, 0x11 == Trio64 (rev 00h) / Trio64V+ (rev 40h)
    uint8_t revision;           // Revision ID, low byte of the PCI ID, in our case for Trio64, this will just be 0x00
    uint8_t id_cr30;            // chip ID/Rev register
    uint32_t strapping;         // CR36/CR68/CR69 combined strapping bits
    uint8_t sr10;               // MCLK PLL low
    uint8_t sr11;               // MCLK PLL high
    uint8_t sr12;               // DCLK PLL low  (Video PLL Data Low)
    uint8_t sr13;               // DCLK PLL high (Video PLL Data High)
    uint8_t sr15;               // CLKSYN control 2
    uint8_t sr17;               // CLKSYN test
    uint8_t clk_pll_r;         // Latched DCLK PLL R (from SR12 bits 6:5)
    uint8_t clk_pll_m;         // Latched DCLK PLL M (from SR13 bits 6:0)
    uint8_t clk_pll_n;         // Latched DCLK PLL N (from SR12 bits 4:0)

    // Hardware graphics cursor (same from state.cursor_* fields)
    uint8_t  cursor_mode;
    uint16_t cursor_x;
    uint16_t cursor_y;
    uint16_t cursor_start_addr;
    uint8_t  cursor_pattern_x;
    uint8_t  cursor_pattern_y;
    uint8_t  cursor_fg[4];
    uint8_t  cursor_bg[4];
    uint8_t  cursor_fg_ptr;
    uint8_t  cursor_bg_ptr;
    uint8_t  extended_dac_ctrl; // Extended RAMDAC Control Register (EX_DAC_CT) (CR55) 
  } s3;

  // SVGA mode flags (MAME: svga.rgb*_en) 
  // first new change to struct that's MAME-related
  // set by s3_define_video_mode(), consumed by renderer (soon)
  struct {
    u8 bank_r = 0;
    u8 bank_w = 0;
    u8 rgb8_en = 0;
    u8 rgb15_en = 0;
    u8 rgb16_en = 0;
    u8 rgb24_en = 0;
    u8 rgb32_en = 0;
  } svga;

  // computed video timing, MAME screen().configure() parameters
  struct {
    int      pixel_clock_hz = 0;    // computed pixel clock in Hz
    int      xtal_hz = 0;    // base or PLL-derived crystal frequency
    int      divisor = 1;    // VCLK divisor from color mode
    double   dclk_freq_mhz = 0.0;  // PLL output frequency in MHz (for debug)
  } timing;

  void s3_short_stroke_do(u8 code);
  inline uint32_t s3_mmio_base_off(SS3_state& s);
  void accel_reset();
  inline bool s3_new_mmio_enabled();
};

// ----- Debug tracing for Data Transfer Position (CR3B/CR34 bit4) -----
#ifndef S3_TRACE_DTP
#define S3_TRACE_DTP 1
#endif
#if S3_TRACE_DTP
#define DTP_TRACE(...) do { printf(__VA_ARGS__); } while (0)
#else
#define DTP_TRACE(...) do {} while (0)
#endif

#ifndef S3_ACCEL_TRACE
#define S3_ACCEL_TRACE 1
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
