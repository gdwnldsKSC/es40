#ifndef ES40_MAME_SHIM_H
#define ES40_MAME_SHIM_H

// mame code compat shims

#include "StdAfx.h"
#include <cstdint>
#include <cstring>
#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>

#ifdef _WIN32
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  ifdef max
#    undef max
#  endif
#  ifdef min
#    undef min
#  endif
#endif

using pen_t = uint32_t;

#ifndef popmessage
#define popmessage(...) do { printf(__VA_ARGS__); } while (0)
#endif

inline constexpr uint8_t pal6bit(uint8_t bits)
{
  return (bits << 2) | (bits >> 4);
}

struct rectangle
{
  int min_x = 0, max_x = 0;
  int min_y = 0, max_y = 0;

  rectangle() = default;
  rectangle(int x0, int x1, int y0, int y1)
    : min_x(x0), max_x(x1), min_y(y0), max_y(y1) {
  }

  int left()   const { return min_x; }
  int right()  const { return max_x; }
  int top()    const { return min_y; }
  int bottom() const { return max_y; }
  int width()  const { return max_x - min_x + 1; }
  int height() const { return max_y - min_y + 1; }

  void set(int x0, int x1, int y0, int y1) { min_x = x0; max_x = x1; min_y = y0; max_y = y1; }

  bool contains(int x, int y) const
  {
    return (x >= min_x) && (x <= max_x) && (y >= min_y) && (y <= max_y);
  }
};

class bitmap_rgb32
{
public:
  bitmap_rgb32() = default;

  void allocate(int w, int h)
  {
    if (w == m_width && h == m_height && !m_pixels.empty())
      return; // already correct size
    m_width = w;
    m_height = h;
    m_pixels.resize((size_t)w * h, 0);
  }

  int width()  const { return m_width; }
  int height() const { return m_height; }

  uint32_t& pix(int y)
  {
    return m_pixels[(size_t)y * m_width];
  }

  // pix(y, x) — returns a reference to pixel (x, y).
  uint32_t& pix(int y, int x)
  {
    return m_pixels[(size_t)y * m_width + x];
  }

  uint32_t* rowptr(int y)
  {
    return m_pixels.data() + (size_t)y * m_width;
  }
  const uint32_t* rowptr(int y) const
  {
    return m_pixels.data() + (size_t)y * m_width;
  }

  void fill(uint32_t color, const rectangle& clip)
  {
    int y0 = (std::max)(clip.min_y, 0);
    int y1 = (std::min)(clip.max_y, m_height - 1);
    int x0 = (std::max)(clip.min_x, 0);
    int x1 = (std::min)(clip.max_x, m_width - 1);
    for (int y = y0; y <= y1; y++)
    {
      uint32_t* row = &pix(y);
      for (int x = x0; x <= x1; x++)
        row[x] = color;
    }
  }

  // MAME text cursor: plot a horizontal box
  void plot_box(int x, int y, int w, int h, uint32_t color)
  {
    for (int dy = 0; dy < h; dy++)
    {
      if (y + dy < 0 || y + dy >= m_height) continue;
      uint32_t* row = &pix(y + dy);
      for (int dx = 0; dx < w; dx++)
      {
        if (x + dx >= 0 && x + dx < m_width)
          row[x + dx] = color;
      }
    }
  }

  // Direct pixel pointer access (whole buffer)
  const uint32_t* raw() const { return m_pixels.data(); }
  uint32_t* raw() { return m_pixels.data(); }

  // Clip rect covering the entire bitmap
  rectangle cliprect() const { return rectangle(0, m_width - 1, 0, m_height - 1); }

private:
  int m_width = 0;
  int m_height = 0;
  std::vector<uint32_t> m_pixels;
};

class screen_device_shim
{
public:
  // Set the logical visible area after mode change
  void set_visible_area(int w, int h)
  {
    m_visarea = rectangle(0, w - 1, 0, h - 1);
  }

  const rectangle& visible_area() const { return m_visarea; }

  uint64_t frame_number() const { return m_frame_number; }
  void     tick_frame() { m_frame_number++; }

  // Stubs for MAME timing APIs (not meaningful in ES40)
  void configure(int htotal, int vtotal, const rectangle& visarea, int64_t refresh)
  {
    m_visarea = visarea;
    // ES40 doesn't use attosecond timing; just store the visible area.
  }

  int  vpos()  const { return m_vpos; }
  int  hpos()  const { return 0; }
  bool hblank() const { return false; }

  void set_vpos(int v) { m_vpos = v; }

  // MAME uses screen().time_until_pos() for vblank timer.
  // Return a dummy value; ES40 drives its own timing.
  struct dummy_time { void adjust(...) {} };

private:
  rectangle  m_visarea;
  uint64_t   m_frame_number = 0;
  int        m_vpos = 0;
};

struct mame_machine_stub
{
  // MAME - returns true when using debugger to inspect registers,
  // without triggering their actions like ones that reset things on read
  bool side_effects_disabled() const { return false; }
};

struct mame_machine_provider
{
  mame_machine_stub m_machine_stub_;
  mame_machine_stub& machine() { return m_machine_stub_; }
  const mame_machine_stub& machine() const { return m_machine_stub_; }
};

struct nop_callback
{
  template <typename... Args>
  void operator()(Args&&...) const {}
};

#endif // ES40_MAME_SHIM_H
