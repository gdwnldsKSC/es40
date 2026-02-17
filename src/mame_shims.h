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

// useful soon for MAME code re-use
#ifndef ATTR_COLD
#define ATTR_COLD
#endif

#ifdef _WIN32 // this was a workaround for .... I think it was ... uh, windows.h being annoying.
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

struct rectangle
{
  int min_x = 0, max_x = 0;
  int min_y = 0, max_y = 0;

  rectangle() = default;
  rectangle(int x0, int x1, int y0, int y1)
    : min_x(x0), max_x(x1), min_y(y0), max_y(y1) {
  }

  int width()  const { return max_x - min_x + 1; }
  int height() const { return max_y - min_y + 1; }

  constexpr bool contains(int32_t x, int32_t y) const { return (x >= min_x) && (x <= max_x) && (y >= min_y) && (y <= max_y); }

  void set(int x0, int x1, int y0, int y1) { 
    min_x = x0; max_x = x1; min_y = y0; max_y = y1; 
  }
};

class bitmap_rgb32
{
public:
  bitmap_rgb32() = default;

  void allocate(int w, int h)
  {
    m_width = w;
    m_height = h;
    m_pixels.resize((size_t)w * h, 0);
  }

  void resize(int w, int h) { allocate(w, h); }

  int width() const { return m_width; }
  int height() const { return m_height; }
  bool valid() const { return m_width > 0 && m_height > 0; }

  // pix(y, x) — returns a reference to pixel (x, y).
  uint32_t& pix(int y, int x = 0)
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

class screen_device
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

class machine_config {};
using device_type = int;  // opaque handle, not used meaningfully in ES40

struct finder_base {
  static constexpr const char* DUMMY_TAG = "";
};

class device_t : public mame_machine_provider
{
public:
  device_t() = default;
  device_t(const machine_config& /*mconfig*/, device_type /*type*/,
    const char* /*tag*/, device_t* /*owner*/, uint32_t /*clock*/)
  {
  }

  virtual ~device_t() = default;

  // MAME lifecycle — overridden by subclasses
  virtual void device_start() {}
  virtual void device_reset() {}

  // Tag system (stub)
  const char* tag() const { return ""; }

  // Sub-device lookup stub
  template<typename T>
  T* subdevice(const char* /*tag*/) { return nullptr; }
};

template<typename T>
class required_device
{
public:
  required_device() : m_ptr(nullptr) {}

  // MAME constructor: required_device(device_t &owner, const char *tag)
  required_device(device_t& /*owner*/, const char* /*tag*/)
    : m_ptr(nullptr) {
  }

  // Set the target (called during ES40 init instead of MAME's device resolution)
  void set(T* ptr) { m_ptr = ptr; }

  // MAME API: set_tag (used in set_vga_owner, device_add_mconfig)
  void set_tag(const char* /*tag*/) {} // no-op in ES40; resolved manually
  template<typename U>
  void set_tag(U&& /*tag*/) {} // template overload

  // Smart pointer interface
  T* operator->() const { return m_ptr; }
  T& operator*() const { return *m_ptr; }
  operator T* () const { return m_ptr; }
  explicit operator bool() const { return m_ptr != nullptr; }
  T* target() const { return m_ptr; }

private:
  T* m_ptr;
};

#define DEFINE_DEVICE_TYPE(Type, Class, ShortName, FullName) \
  static const int Type = 0

#define DECLARE_DEVICE_TYPE(Type, Class) \
  extern const int Type

enum endianness_t { ENDIANNESS_LITTLE, ENDIANNESS_BIG };

using address_map_constructor = std::function<void(class address_map&)>;

struct address_space_config
{
  const char* m_name = nullptr;

  address_space_config() = default;
  address_space_config(const char* name, endianness_t, int, int, int,
    address_map_constructor = {})
    : m_name(name) {
  }
};

struct feature {
  using type = uint32_t;
  static constexpr type GRAPHICS = 0x01;
};

#ifndef popmessage
#define popmessage(...) do { printf(__VA_ARGS__); } while (0)
#endif

struct nop_callback
{
  template <typename... Args>
  void operator()(Args&&...) const {}
};

inline uint8_t pal6bit(uint8_t v)
{
  return (v << 2) | (v >> 4);
}

#endif // ES40_MAME_SHIM_H
