/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * Website: http://sourceforge.net/projects/es40
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
  * Contains code macros for the processor MVI (multimedia) instructions.
  *
  * $Id$
  *
  * X-1.4        Camiel Vanderhoeven                             14-MAR-2008
  *   1. More meaningful exceptions replace throwing (int) 1.
  *   2. U64 macro replaces X64 macro.
  *
  * X-1.3        Camiel Vanderhoeven                             11-APR-2007
  *      Moved all data that should be saved to a state file to a structure
  *      "state".
  *
  * X-1.2        Camiel Vanderhoeven                             30-MAR-2007
  *      Added old changelog comments.
  *
  * X-1.1        Camiel Vanderhoeven                             18-FEB-2007
  *      File created. Contains code previously found in AlphaCPU.h
  *
  * \author Camiel Vanderhoeven (camiel@camicom.com / http://www.camicom.com)
  **/
#if defined(_MSC_VER) && defined(_M_X64)
#include <immintrin.h>
#define ES40_HAVE_SSE2 1      // x64 on MSVC guarantees SSE2

  /* Optional: define these if you compile with /arch:AVX (/arch:AVX2) or
     explicitly via project Preprocessor Definitions. Otherwise the SSE2
     fallbacks are used automatically. */
#if defined(__AVX2__) || defined(__AVX__) || defined(__SSE4_1__)
#define ES40_HAVE_SSE41 1
#endif
#if defined(__SSSE3__) || defined(__SSE4_1__) || defined(__AVX__) || defined(__AVX2__)
#define ES40_HAVE_SSSE3 1
#endif
#endif

#if defined(ES40_HAVE_SSE2)
     /* tiny helpers for 64b <-> XMM */
static __forceinline __m128i es40_loadq(u64 x) { return _mm_cvtsi64_si128((__int64)x); }
static __forceinline u64     es40_storeq(__m128i v) { return (u64)_mm_cvtsi128_si64(v); }
#endif

#if defined(ES40_HAVE_SSE2)
#define DO_MINUB8 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i m = _mm_min_epu8(a, b);         \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#else
#define DO_MINUB8 do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 8) { \
      u8 a = (u8)((temp_64_1 >> i) & X64_BYTE), b = (u8)((temp_64_2 >> i) & X64_BYTE); \
      temp_64 |= ((u64)(a < b ? a : b) << i); \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSE2) && defined(ES40_HAVE_SSE41)
#define DO_MINSB8 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i m = _mm_min_epi8(a, b);         \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#elif defined(ES40_HAVE_SSE2)
#define DO_MINSB8 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i mask = _mm_cmpgt_epi8(b, a);    /* mask=(b>a) */ \
    __m128i m = _mm_or_si128(_mm_and_si128(a, mask), _mm_andnot_si128(mask, b)); \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#else
#define DO_MINSB8 do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 8) { \
      s8 a = (s8)((temp_64_1 >> i) & X64_BYTE), b = (s8)((temp_64_2 >> i) & X64_BYTE); \
      temp_64 |= (u64)((a < b ? (u8)a : (u8)b)) << i; \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSE2) && defined(ES40_HAVE_SSE41)
#define DO_MINUW4 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i m = _mm_min_epu16(a, b);        \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#elif defined(ES40_HAVE_SSE2)
#define DO_MINUW4 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    const __m128i bias = _mm_set1_epi16((short)0x8000); \
    __m128i ax = _mm_xor_si128(a, bias);    \
    __m128i bx = _mm_xor_si128(b, bias);    \
    __m128i m  = _mm_xor_si128(_mm_min_epi16(ax, bx), bias); \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#else
#define DO_MINUW4 do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 16) { \
      u16 a = (u16)((temp_64_1 >> i) & X64_WORD), b = (u16)((temp_64_2 >> i) & X64_WORD); \
      temp_64 |= ((u64)(a < b ? a : b) << i); \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSE2)
#define DO_MINSW4 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i m = _mm_min_epi16(a, b);        \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#else
#define DO_MINSW4 do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 16) { \
      s16 a = (s16)((temp_64_1 >> i) & X64_WORD), b = (s16)((temp_64_2 >> i) & X64_WORD); \
      temp_64 |= (u64)((a < b ? (u16)a : (u16)b)) << i; \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSE2)
#define DO_MAXUB8 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i m = _mm_max_epu8(a, b);         \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#else
#define DO_MAXUB8 do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 8) { \
      u8 a = (u8)((temp_64_1 >> i) & X64_BYTE), b = (u8)((temp_64_2 >> i) & X64_BYTE); \
      temp_64 |= ((u64)(a > b ? a : b) << i); \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSE2) && defined(ES40_HAVE_SSE41)
#define DO_MAXSB8 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i m = _mm_max_epi8(a, b);         \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#elif defined(ES40_HAVE_SSE2)
#define DO_MAXSB8 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i mask = _mm_cmpgt_epi8(a, b);    /* mask=(a>b) */ \
    __m128i m = _mm_or_si128(_mm_and_si128(a, mask), _mm_andnot_si128(mask, b)); \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#else
#define DO_MAXSB8 do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 8) { \
      s8 a = (s8)((temp_64_1 >> i) & X64_BYTE), b = (s8)((temp_64_2 >> i) & X64_BYTE); \
      temp_64 |= (u64)((a > b ? (u8)a : (u8)b)) << i; \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSE2) && defined(ES40_HAVE_SSE41)
#define DO_MAXUW4 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i m = _mm_max_epu16(a, b);        \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#elif defined(ES40_HAVE_SSE2)
#define DO_MAXUW4 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    const __m128i bias = _mm_set1_epi16((short)0x8000); \
    __m128i ax = _mm_xor_si128(a, bias);    \
    __m128i bx = _mm_xor_si128(b, bias);    \
    __m128i m  = _mm_xor_si128(_mm_max_epi16(ax, bx), bias); \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#else
#define DO_MAXUW4 do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 16) { \
      u16 a = (u16)((temp_64_1 >> i) & X64_WORD), b = (u16)((temp_64_2 >> i) & X64_WORD); \
      temp_64 |= (u64)(a > b ? a : b) << i; \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSE2)
#define DO_MAXSW4 do { \
    __m128i a = es40_loadq(state.r[REG_1]); \
    __m128i b = es40_loadq(V_2);            \
    __m128i m = _mm_max_epi16(a, b);        \
    state.r[REG_3] = es40_storeq(m);        \
  } while (0)
#else
#define DO_MAXSW4 do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 16) { \
      s16 a = (s16)((temp_64_1 >> i) & X64_WORD), b = (s16)((temp_64_2 >> i) & X64_WORD); \
      temp_64 |= (u64)((a > b ? (u16)a : (u16)b)) << i; \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSE2)
#define DO_PERR do { \
    __m128i a8  = es40_loadq(state.r[REG_1]); \
    __m128i b8  = es40_loadq(V_2);            \
    __m128i a16 = _mm_unpacklo_epi8(a8, _mm_cmpgt_epi8(_mm_setzero_si128(), a8)); \
    __m128i b16 = _mm_unpacklo_epi8(b8, _mm_cmpgt_epi8(_mm_setzero_si128(), b8)); \
    __m128i d16 = _mm_sub_epi16(a16, b16); \
    __m128i sgn = _mm_srai_epi16(d16, 15); \
    __m128i abs16 = _mm_sub_epi16(_mm_xor_si128(d16, sgn), sgn); \
    __m128i res8  = _mm_packus_epi16(abs16, _mm_setzero_si128()); \
    state.r[REG_3] = es40_storeq(res8); \
  } while (0)
#else
#define DO_PERR do { \
    temp_64 = 0; temp_64_1 = state.r[REG_1]; temp_64_2 = V_2; \
    for (i = 0; i < 64; i += 8) { \
      s8 a = (s8)((temp_64_1 >> i) & X64_BYTE), b = (s8)((temp_64_2 >> i) & X64_BYTE); \
      u8 d = (u8)((a > b) ? (a - b) : (b - a)); \
      temp_64 |= (u64)d << i; \
    } \
    state.r[REG_3] = temp_64; \
  } while (0)
#endif

#if defined(ES40_HAVE_SSSE3)
#define DO_PKLB do { \
    __m128i x = es40_loadq(V_2); \
    const __m128i m = _mm_setr_epi8(0,4,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80, \
                                    (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80); \
    state.r[REG_3] = es40_storeq(_mm_shuffle_epi8(x, m)); \
  } while (0)
#else
#define DO_PKLB do { u64 t = V_2; state.r[REG_3] = (t & U64(0x00000000000000ff)) | ((t & U64(0x000000ff00000000)) >> 24); } while (0)
#endif

#if defined(ES40_HAVE_SSSE3)
#define DO_PKWB do { \
    __m128i x = es40_loadq(V_2); \
    const __m128i m = _mm_setr_epi8(0,2,4,6,(char)0x80,(char)0x80,(char)0x80,(char)0x80, \
                                    (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80); \
    state.r[REG_3] = es40_storeq(_mm_shuffle_epi8(x, m)); \
  } while (0)
#else
#define DO_PKWB do { u64 t = V_2; state.r[REG_3] = (t & U64(0x00000000000000ff)) | \
    ((t & U64(0x0000000000ff0000)) >> 8) | ((t & U64(0x000000ff00000000)) >> 16) | ((t & U64(0x00ff000000000000)) >> 24); } while (0)
#endif

#if defined(ES40_HAVE_SSSE3)
#define DO_UNPKBL do { \
    __m128i x = es40_loadq(V_2); \
    const __m128i m = _mm_setr_epi8(0,(char)0x80,(char)0x80,(char)0x80, 1,(char)0x80,(char)0x80,(char)0x80, \
                                    (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80); \
    state.r[REG_3] = es40_storeq(_mm_shuffle_epi8(x, m)); \
  } while (0)
#else
#define DO_UNPKBL do { u64 t = V_2; state.r[REG_3] = (t & U64(0x000000ff)) | ((t & U64(0x0000ff00)) << 24); } while (0)
#endif

#if defined(ES40_HAVE_SSSE3)
#define DO_UNPKBW do { \
    __m128i x = es40_loadq(V_2); \
    const __m128i m = _mm_setr_epi8(0,(char)0x80,1,(char)0x80,2,(char)0x80,3,(char)0x80, \
                                    (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80); \
    state.r[REG_3] = es40_storeq(_mm_shuffle_epi8(x, m)); \
  } while (0)
#else
#define DO_UNPKBW do { u64 t = V_2; state.r[REG_3] = (t & U64(0x000000ff)) | ((t & U64(0x0000ff00)) << 8) | \
    ((t & U64(0x00ff0000)) << 16) | ((t & U64(0xff000000)) << 24); } while (0)
#endif
