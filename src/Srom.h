/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * CPU-local SROM flash emulation.
 *
 * Real EV6/21264 systems fetch their very first PALcode instructions via the
 * SROM interface into the I-cache (entrypoint typically at 0x780 in PALmode).
 * On some platforms (e.g. AlphaPC 264) this is a separate flash device close
 * to the CPU. On ES40 hardware the initial SROM image is also present in the
 * 2MB system flash, but the CPU still boots via the SROM interface.
 *
 * This component models that CPU-local SROM flash and persists it to a
 * standalone file (default: rom\srom.rom) so that LFU/firmware updates that
 * modify the SROM image can survive across emulator runs.
 */

#if !defined(INCLUDED_SROM_H)
#define INCLUDED_SROM_H

#include "SystemComponent.h"

class CSrom : public CSystemComponent
{
public:
  CSrom(CConfigurator* cfg, class CSystem* c);
  virtual ~CSrom();

  // Not memory-mapped (CPU reads SROM via the SROM interface), but we still
  // support VM state save/restore.
  virtual int   SaveState(FILE* f);
  virtual int   RestoreState(FILE* f);

  void          SaveStateF();
  void          RestoreStateF();
  void          SaveStateF(const char* fn);
  void          RestoreStateF(const char* fn);

  void          FlushIfDirty();

  bool          HasValidImage() const;
  const u8*     GetBytes() const { return data; }
  u32           GetSize()  const { return kCpuSromSize; }
  u32           GetImageSize() const { return image_size; }

  // If no CPU SROM file exists yet, try to seed it from the system flash's
  // SROM partition (default ES40 layout: 0x100000..0x11FFFF).
  void          MaybeSeedFromSystemFlash(const u8* sysflash_dense, u32 sysflash_len);

  // Mirror system-flash writes into the CPU-local SROM image, so that firmware
  // update tools that (incorrectly) program the SROM through the system flash
  // still result in a consistent CPU SROM on next reset.
  void          MirrorSystemFlashProgram(u32 sysflash_off, u8 newval);
  void          MirrorSystemFlashErase(u32 sysflash_off, u32 len);

  bool          DebugEnabled() const { return debug; }

private:
  static const u32 kCpuSromSize = 512 * 1024;  // 512KiB max, compatible with AlphaPC 264

  bool dirty;
  bool debug;

  // Dense SROM bytes.
  u8   data[kCpuSromSize];

  // How many bytes of the image are meaningful (loaded/seeded). Used for
  // boot-time heuristics and progress output.
  u32  image_size;

  // Mapping of system flash region that contains the SROM image.
  u32  sysflash_srom_off;
  u32  sysflash_srom_len;

  // Instrumentation counters.
  u64  mirror_prog_count;
  u64  mirror_erase_count;
  u32  dirty_min;
  u32  dirty_max;
};

extern CSrom* theCPUSROM;

#endif // INCLUDED_SROM_H
