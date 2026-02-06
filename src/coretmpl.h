// license:BSD-3-Clause
// copyright-holders:Aaron Giles, Vas Crabb
/***************************************************************************

    coretmpl.h

    Core templates for basic non-string types.

***************************************************************************/
// This file is part of the MAME project (https://www.mamedev.org/).
// Cut down for adaptation directly to ES40

#ifndef MAME_UTIL_CORETMPL_H
#define MAME_UTIL_CORETMPL_H

#pragma once

//#include "osdcomm.h"
//#include "vecstream.h"

#include <array>
#include <cstddef>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <utility>

/// \defgroup bitutils Useful functions for bit shuffling
/// \{

/// \brief Generate a right-aligned bit mask
///
/// Generates a right aligned mask of the specified width.  Works with
/// signed and unsigned integer types.
/// \tparam T Desired output type.
/// \tparam U Type of the input (generally resolved by the compiler).
/// \param [in] n Width of the mask to generate in bits.
/// \return Right-aligned mask of the specified width.

template <typename T, typename U> constexpr T make_bitmask(U n)
{
  return T((n < (8 * sizeof(T)) ? (std::make_unsigned_t<T>(1) << n) : std::make_unsigned_t<T>(0)) - 1);
}


/// \brief Extract a single bit from an integer
///
/// Extracts a single bit from an integer into the least significant bit
/// position.
///
/// \param [in] x The integer to extract the bit from.
/// \param [in] n The bit to extract, where zero is the least
///   significant bit of the input.
/// \return Zero if the specified bit is unset, or one if it is set.
/// \sa bitswap
template <typename T, typename U> constexpr T BIT(T x, U n) noexcept { return (x >> n) & T(1); }


/// \brief Extract a bit field from an integer
///
/// Extracts and right-aligns a bit field from an integer.
///
/// \param [in] x The integer to extract the bit field from.
/// \param [in] n The least significant bit position of the field to
///   extract, where zero is the least significant bit of the input.
/// \param [in] w The width of the field to extract in bits.
/// \return The field [n..(n+w-1)] from the input.
/// \sa bitswap
template <typename T, typename U, typename V> constexpr T BIT(T x, U n, V w)
{
  return (x >> n) & make_bitmask<T>(w);
}


/// \brief Extract bits in arbitrary order
///
/// Extracts bits from an integer.  Specify the bits in the order they
/// should be arranged in the output, from most significant to least
/// significant.  The extracted bits will be packed into a right-aligned
/// field in the output.
///
/// \param [in] val The integer to extract bits from.
/// \param [in] b The first bit to extract from the input
///   extract, where zero is the least significant bit of the input.
///   This bit will appear in the most significant position of the
///   right-aligned output field.
/// \param [in] c The remaining bits to extract, where zero is the
///   least significant bit of the input.
/// \return The extracted bits packed into a right-aligned field.
template <typename T, typename U, typename... V> constexpr T bitswap(T val, U b, V... c) noexcept
{
  if constexpr (sizeof...(c) > 0U)
    return (BIT(val, b) << sizeof...(c)) | bitswap(val, c...);
  else
    return BIT(val, b);
}


/// \brief Extract bits in arbitrary order with explicit count
///
/// Extracts bits from an integer.  Specify the bits in the order they
/// should be arranged in the output, from most significant to least
/// significant.  The extracted bits will be packed into a right-aligned
/// field in the output.  The number of bits to extract must be supplied
/// as a template argument.
///
/// A compile error will be generated if the number of bit positions
/// supplied does not match the specified number of bits to extract, or
/// if the output type is too small to hold the extracted bits.  This
/// guards against some simple errors.
///
/// \tparam B The number of bits to extract.  Must match the number of
///   bit positions supplied.
/// \param [in] val The integer to extract bits from.
/// \param [in] b Bits to extract, where zero is the least significant
///   bit of the input.  Specify bits in the order they should appear in
///   the output field, from most significant to least significant.
/// \return The extracted bits packed into a right-aligned field.
template <unsigned B, typename T, typename... U> constexpr T bitswap(T val, U... b) noexcept
{
  static_assert(sizeof...(b) == B, "wrong number of bits");
  static_assert((sizeof(std::remove_reference_t<T>) * 8) >= B, "return type too small for result");
  return bitswap(val, b...);
}

/// \}

#endif
