// TIGFlash.cpp
#include "StdAfx.h"
#include <stdio.h>
#include <string.h>
#include "TIGFlash.h"

// --------------------------
// Device geometry & storage
// --------------------------
static const u32 FLASH_BYTES = 0x200000;  // 2 MiB
static const u32 FLASH_SECTOR_SZ = 0x10000;   // 64 KiB
static const char* FLASH_FILE = "es40-tigflash.bin";

// --------------------------
// Internal state
// --------------------------
static bool g_inited = false;
static u8   g_flash[FLASH_BYTES];
static int  g_cycle = 0;
static u8   g_cmd = 0;

// JEDEC helper (post-stride addresses)
static inline bool is_5555(u32 a) { return (a & 0xFFFFu) == 0x5555u; }
static inline bool is_2AAA(u32 a) { return (a & 0xFFFFu) == 0x2AAAu; }

static void persist_range(u32 off, u32 len) {
    printf("[TIGFLASH] persist_range off=%06x len=%06x\n", off, len);
    FILE* f = fopen(FLASH_FILE, "r+b");
    if (!f) {
        printf("[TIGFLASH] opening '%s' for write (create)\n", FLASH_FILE);
        f = fopen(FLASH_FILE, "w+b");
    }
    if (!f) {
        perror("[TIGFLASH] fopen failed");
        return;
    }
    fseek(f, off, SEEK_SET);
    fwrite(g_flash + off, 1, len, f);
    fclose(f);
}

static void init_once() {
    if (g_inited) return;

    bool need_full_write = false;

    FILE* f = fopen(FLASH_FILE, "rb");
    if (f) {
        size_t r = fread(g_flash, 1, FLASH_BYTES, f);
        fclose(f);
        // Pad the rest with erased bytes
        memset(g_flash + r, 0xFF, FLASH_BYTES - r);
        // On next persist, normalize the file to full 2 MiB
        need_full_write = true;
        printf("[TIGFLASH] existing image shorter than 2 MiB (%zu bytes); "
            "padding & extending on disk\n", r);
    }
    else {
        // No file yet: start with a completely erased 2 MiB part
        printf("[TIGFLASH] no existing image; creating fresh 2 MiB erased flash\n");
        memset(g_flash, 0xFF, sizeof g_flash);
        need_full_write = true;
    }
    g_cycle = 0;
    g_cmd = 0;
    g_inited = true;

    // Ensure on-disk file is a full 2 MiB image (erased where unused)
    if (need_full_write) {
        persist_range(0, FLASH_BYTES);
    }
}

u8 tigflash_read(u32 tig_offset)
{
    init_once();
    // One real byte every 0x40 bytes in TIG space
    u32 a = (tig_offset >> 6) % FLASH_BYTES;

    if (g_cmd == 0x90) {
        // Auto-select
        switch (a & 0xFF) {
        case 0x00: return 0x01; // AMD
        case 0x01: return 0xAD; // 29F016
        default:   return 0x00;
        }
    }
    return g_flash[a];
}

void tigflash_write(u32 tig_offset, u8 val)
{
    init_once();
    u32 a = (tig_offset >> 6) % FLASH_BYTES;

    printf("[TIGFLASH] write off=%08x (addr=%06x) val=%02x cycle=%d cmd=%02x\n",
        tig_offset, a, val, g_cycle, g_cmd);

    switch (g_cycle) {
    case 0:
        if (val == 0xF0) { g_cmd = 0; break; }              // Reset
        if (is_5555(a) && val == 0xAA) { g_cycle = 1; break; }
        break;

    case 1:
        if (is_2AAA(a) && val == 0x55) { g_cycle = 2; break; }
        g_cycle = 0; g_cmd = 0; break;

    case 2:
        if (is_5555(a)) {
            if (val == 0x90) { g_cmd = 0x90; g_cycle = 0; break; } // Auto-select
            if (val == 0xA0) { g_cmd = 0xA0; g_cycle = 3; break; } // Program
            if (val == 0x80) { g_cmd = 0x80; g_cycle = 3; break; } // Erase prefix
            if (val == 0xF0) { g_cmd = 0;    g_cycle = 0; break; } // Reset
        }
        g_cycle = 0; g_cmd = 0; break;

    case 3:
        if (g_cmd == 0xA0) {
            // Byte program: only 1->0 transitions
            g_flash[a] &= val;
            persist_range(a, 1);
            g_cycle = 0; g_cmd = 0;
            break;
        }
        if (g_cmd == 0x80 && is_5555(a) && val == 0xAA) { g_cycle = 4; break; }
        g_cycle = 0; g_cmd = 0; break;

    case 4:
        if (is_2AAA(a) && val == 0x55) { g_cycle = 5; break; }
        g_cycle = 0; g_cmd = 0; break;

    case 5:
        if (is_5555(a) && val == 0x10) {
            // Chip erase
            memset(g_flash, 0xFF, FLASH_BYTES);
            persist_range(0, FLASH_BYTES);
            g_cycle = 0; g_cmd = 0; break;
        }
        if (val == 0x30) {
            // Sector erase: pick sector by current address
            u32 base = a & ~(FLASH_SECTOR_SZ - 1u);
            memset(g_flash + base, 0xFF, FLASH_SECTOR_SZ);
            persist_range(base, FLASH_SECTOR_SZ);
            g_cycle = 0; g_cmd = 0; break;
        }
        g_cycle = 0; g_cmd = 0; break;
    }
}

void tigflash_reset(void)
{
    init_once();
    g_cycle = 0;
    g_cmd = 0;
}
