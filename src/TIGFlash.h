// TIGFlash.h
#if !defined(INCLUDED_TIGFLASH_H)
#define INCLUDED_TIGFLASH_H

#include "StdAfx.h"

/**
 * Tiny AM29F016 flash behind the TIG bus.
 * Addressing: expose one byte every 0x40 bytes of TIG (addr >> 6).
 * Size: 2 MiB total, organized in 64 KiB sectors.
 * Persistence: es40-tigflash.bin (created on first use, erased = 0xFF).
 */
u8  tigflash_read(u32 tig_offset);     // offset within 0x8010_0000_000..+0x07FF_FFF
void tigflash_write(u32 tig_offset, u8 data);
void tigflash_reset(void);

#endif // INCLUDED_TIGFLASH_H
