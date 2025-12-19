/* ES40 emulator.
 *
 * CPU-local SROM flash emulation.
 */

#include "StdAfx.h"
#include "Srom.h"

// A small self-contained CRC32 for instrumentation (polynomial 0xEDB88320).
static u32 crc32_update(u32 crc, const u8* buf, size_t len)
{
  static bool table_init = false;
  static u32 table[256];

  if (!table_init)
  {
    for (u32 i = 0; i < 256; i++)
    {
      u32 c = i;
      for (int j = 0; j < 8; j++)
        c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      table[i] = c;
    }
    table_init = true;
  }

  crc = ~crc;
  for (size_t i = 0; i < len; i++)
    crc = table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

static const u32 srom_magic1 = 0x4D4F5253; // "SROM" little-endian-ish
static const u32 srom_magic2 = 0x534F524D; // "MROS"

CSrom* theCPUSROM = 0;

CSrom::CSrom(CConfigurator* cfg, CSystem* c) : CSystemComponent(cfg, c)
{
  if (theCPUSROM)
    FAILURE(Configuration, "More than one CPU SROM");
  theCPUSROM = this;

  // Defaults match ES40/QEMU ES40 layout.
  sysflash_srom_off = (u32)myCfg->get_num_value("rom.sysflash.srom_off", false, 0x100000);
  sysflash_srom_len = (u32)myCfg->get_num_value("rom.sysflash.srom_len", false, 0x20000);

  debug = myCfg->get_bool_value("debug.srom", false);
  dirty = false;
  image_size = 0;
  mirror_prog_count = 0;
  mirror_erase_count = 0;
  dirty_min = 0xFFFFFFFFu;
  dirty_max = 0;

  memset(data, 0xFF, sizeof(data));
  RestoreStateF();

  // Replace the generic devid string with something more descriptive.
  // (Safe because CSystemComponent::~CSystemComponent frees devid_string.)
  free(devid_string);
  const char* name = "srom0(cpu_local)";
  devid_string = (char*)malloc(strlen(name) + 1);
  strcpy(devid_string, name);

  printf("%s: $Id$\n", devid_string);
}

CSrom::~CSrom()
{
  FlushIfDirty();
}

bool CSrom::HasValidImage() const
{
  // Typical EV6 SROM entrypoint is at 0x780.
  const u32 entry = 0x780;
  if (entry + 16 > sizeof(data))
    return false;

  bool all_ff = true;
  bool all_00 = true;
  for (u32 i = 0; i < 16; i++)
  {
    all_ff &= (data[entry + i] == 0xFF);
    all_00 &= (data[entry + i] == 0x00);
  }
  return !(all_ff || all_00);
}

void CSrom::FlushIfDirty()
{
  if (!dirty)
    return;

  SaveStateF();
  dirty = false;

  if (debug)
  {
    printf("%%SROM-I-MIRROR: mirrored %" PRIu64 " program bytes, %" PRIu64 " erase ops; dirty range [%08X..%08X]\n",
      mirror_prog_count, mirror_erase_count,
      (dirty_min == 0xFFFFFFFFu) ? 0 : dirty_min,
      dirty_max);
  }

  mirror_prog_count = 0;
  mirror_erase_count = 0;
  dirty_min = 0xFFFFFFFFu;
  dirty_max = 0;
}

void CSrom::SaveStateF(const char* fn)
{
  FILE* ff = fopen(fn, "wb");
  if (!ff)
  {
    printf("%%SROM-F-NOSAVE: CPU SROM could not be saved to %s\n", fn);
    return;
  }

  // Raw dump (dense bytes). This is intentionally not wrapped in the
  // CFlash SaveState container so it can be inspected easily.
  fwrite(data, 1, sizeof(data), ff);
  fclose(ff);

  const u32 crc = crc32_update(0, data, sizeof(data));
  printf("%%SROM-I-SAVEST: CPU SROM saved to %s (crc32=%08x)\n", fn, crc);
}

void CSrom::SaveStateF()
{
  SaveStateF(myCfg->get_text_value("rom.cpusrom", "rom\\srom.rom"));
}

void CSrom::RestoreStateF(const char* fn)
{
  FILE* ff = fopen(fn, "rb");
  if (!ff)
  {
    printf("%%SROM-F-NOREST: CPU SROM could not be restored from %s\n", fn);
    return;
  }

  // Determine file size.
  fseek(ff, 0, SEEK_END);
  const long fsz = ftell(ff);
  fseek(ff, 0, SEEK_SET);

  size_t r = 0;
  if (fsz > 0)
  {
    size_t toread = (size_t)fsz;
    if (toread > sizeof(data))
      toread = sizeof(data);
    r = fread(data, 1, toread, ff);
  }
  fclose(ff);

  if (r < sizeof(data))
    memset(data + r, 0xFF, sizeof(data) - r);

  image_size = (u32)((r < sizeof(data)) ? r : sizeof(data));
  if (image_size == 0)
    image_size = (sysflash_srom_len < (u32)sizeof(data)) ? sysflash_srom_len : (u32)sizeof(data);

  const u32 crc = crc32_update(0, data, sizeof(data));
  printf("%%SROM-I-RESTST: CPU SROM restored from %s (%ld bytes, crc32=%08x)\n", fn, fsz, crc);
}

void CSrom::RestoreStateF()
{
  RestoreStateF(myCfg->get_text_value("rom.cpusrom", "rom\\srom.rom"));
}

void CSrom::MaybeSeedFromSystemFlash(const u8* sysflash_dense, u32 sysflash_len)
{
  if (!sysflash_dense)
    return;
  if (sysflash_len < sysflash_srom_off + sysflash_srom_len)
    return;

  // If a valid CPU SROM is already present, do nothing.
  if (HasValidImage())
    return;

  // If the system-flash SROM region is all 0xFF or 0x00, it's not useful.
  const u8* src = sysflash_dense + sysflash_srom_off;
  bool all_ff = true;
  bool all_00 = true;
  for (u32 i = 0; i < sysflash_srom_len; i++)
  {
    all_ff &= (src[i] == 0xFF);
    all_00 &= (src[i] == 0x00);
  }

  if (all_ff || all_00)
  {
    printf("%%SROM-W-NOSEED: System flash SROM region looks blank; not seeding CPU SROM.\n");
    return;
  }

  // Seed CPU SROM from the system flash partition.
  memset(data, 0xFF, sizeof(data));
  {
    const u32 copy_len = (sysflash_srom_len < (u32)sizeof(data)) ? sysflash_srom_len : (u32)sizeof(data);
    memcpy(data, src, copy_len);
    image_size = copy_len;
  }
  dirty = true;
  dirty_min = 0;
  dirty_max = image_size ? (image_size - 1) : 0;

  const u32 crc = crc32_update(0, data, sizeof(data));
  printf("%%SROM-I-SEED: Seeded CPU SROM from system flash [0x%06X..0x%06X] (crc32=%08x)\n",
    sysflash_srom_off, sysflash_srom_off + sysflash_srom_len - 1, crc);

  FlushIfDirty();
}

void CSrom::MirrorSystemFlashProgram(u32 sysflash_off, u8 newval)
{
  if (sysflash_off < sysflash_srom_off)
    return;
  const u32 rel = sysflash_off - sysflash_srom_off;
  if (rel >= sysflash_srom_len || rel >= (u32)sizeof(data))
    return;

  // Mirror the already-ANDed value from the system flash emulation.
  if (data[rel] != newval)
  {
    data[rel] = newval;
    dirty = true;
    mirror_prog_count++;
    if (rel < dirty_min) dirty_min = rel;
    if (rel > dirty_max) dirty_max = rel;
  }
}

void CSrom::MirrorSystemFlashErase(u32 sysflash_off, u32 len)
{
  // Intersect erase range with SROM range.
  const u32 start = sysflash_off;
  const u32 end = sysflash_off + len;
  const u32 s_start = sysflash_srom_off;
  const u32 s_end = sysflash_srom_off + sysflash_srom_len;

  if (end <= s_start || start >= s_end)
    return;

  const u32 o_start = (start < s_start) ? s_start : start;
  const u32 o_end = (end > s_end) ? s_end : end;
  const u32 rel0 = o_start - sysflash_srom_off;
  const u32 rel1 = o_end - sysflash_srom_off;

  if (rel0 >= sizeof(data))
    return;

  const u32 rel_end = (rel1 < (u32)sizeof(data)) ? rel1 : (u32)sizeof(data);
  if (rel_end <= rel0)
    return;

  memset(&data[rel0], 0xFF, rel_end - rel0);
  dirty = true;
  mirror_erase_count++;
  if (rel0 < dirty_min) dirty_min = rel0;
  if ((rel_end - 1) > dirty_max) dirty_max = rel_end - 1;
}

int CSrom::SaveState(FILE* f)
{
  const u32 ss = (u32)sizeof(data);
  fwrite(&srom_magic1, sizeof(u32), 1, f);
  fwrite(&ss, sizeof(u32), 1, f);
  fwrite(data, 1, sizeof(data), f);
  fwrite(&srom_magic2, sizeof(u32), 1, f);
  printf("srom: %u bytes saved.\n", ss);
  return 0;
}

int CSrom::RestoreState(FILE* f)
{
  u32 m1 = 0, m2 = 0, ss = 0;
  size_t r;

  r = fread(&m1, sizeof(u32), 1, f);
  if (r != 1 || m1 != srom_magic1)
  {
    printf("srom: MAGIC 1 does not match!\n");
    return -1;
  }

  r = fread(&ss, sizeof(u32), 1, f);
  if (r != 1 || ss != sizeof(data))
  {
    printf("srom: STRUCT SIZE does not match!\n");
    return -1;
  }

  r = fread(data, 1, sizeof(data), f);
  if (r != sizeof(data))
  {
    printf("srom: unexpected end of file!\n");
    return -1;
  }

  r = fread(&m2, sizeof(u32), 1, f);
  if (r != 1 || m2 != srom_magic2)
  {
    printf("srom: MAGIC 2 does not match!\n");
    return -1;
  }

  image_size = (sysflash_srom_len < (u32)sizeof(data)) ? sysflash_srom_len : (u32)sizeof(data);
  printf("srom: %u bytes restored.\n", ss);
  return 0;
}
