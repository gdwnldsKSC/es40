// Adapted from MAME xtal.h (license:BSD-3-Clause, copyright-holders:Olivier Galibert)
// just enough for S3 for now

// Usage:
//   XTAL(25'174'800).value()           -> 25174800  (u32)
//   XTAL(28'636'363).dvalue()          -> 28636363.0 (double)
//   (XTAL(14'318'181) * 2).value()     -> 28636362
//   XTAL(14'318'181) / 3              -> XTAL with current_clock ~ 4772727

#ifndef XTAL_H
#define XTAL_H

#include <cstdint>
#include <cmath>

class XTAL
{
public:
  constexpr explicit XTAL(double base_clock) noexcept : m_base_clock(base_clock), m_current_clock(base_clock) {}

  constexpr double   dvalue() const noexcept { return m_current_clock; }
  constexpr uint32_t value()  const noexcept { return uint32_t(m_current_clock + 1e-3); }
  constexpr double   base()   const noexcept { return m_base_clock; }

  template <typename T> constexpr XTAL operator *(T mult) const noexcept { return XTAL(m_base_clock, m_current_clock * mult); }
  template <typename T> constexpr XTAL operator /(T div) const noexcept { return XTAL(m_base_clock, m_current_clock / div); }

  friend constexpr XTAL operator *(int          mult, const XTAL& xtal);
  friend constexpr XTAL operator *(unsigned int mult, const XTAL& xtal);
  friend constexpr XTAL operator *(double       mult, const XTAL& xtal);

private:
  double m_base_clock, m_current_clock;

  constexpr XTAL(double base_clock, double current_clock) noexcept : m_base_clock(base_clock), m_current_clock(current_clock) {}
};

constexpr XTAL operator *(int          mult, const XTAL& xtal) { return XTAL(xtal.base(), mult * xtal.dvalue()); }
constexpr XTAL operator *(unsigned int mult, const XTAL& xtal) { return XTAL(xtal.base(), mult * xtal.dvalue()); }
constexpr XTAL operator *(double       mult, const XTAL& xtal) { return XTAL(xtal.base(), mult * xtal.dvalue()); }


constexpr XTAL operator ""_Hz_XTAL(long double clock) { return XTAL(double(clock)); }
constexpr XTAL operator ""_kHz_XTAL(long double clock) { return XTAL(double(clock * 1e3)); }
constexpr XTAL operator ""_MHz_XTAL(long double clock) { return XTAL(double(clock * 1e6)); }

constexpr XTAL operator ""_Hz_XTAL(unsigned long long clock) { return XTAL(double(clock)); }
constexpr XTAL operator ""_kHz_XTAL(unsigned long long clock) { return XTAL(double(clock) * 1e3); }
constexpr XTAL operator ""_MHz_XTAL(unsigned long long clock) { return XTAL(double(clock) * 1e6); }

// S3 crystal frequencies 
// These match MAME's pc_vga_s3.cpp s3_define_video_mode():
//   XTAL(25'174'800)  - 25.1748 MHz  (VGA 640px modes)
//   XTAL(28'636'363)  - 28.636363 MHz (VGA 720px modes, 8x NTSC subcarrier)
//   14.318 MHz         - XIN reference for S3 PLL (4x NTSC subcarrier)

#endif
