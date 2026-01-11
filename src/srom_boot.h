/* ES40 emulator - SROM Boot Implementation
 * Flash layout for AM29F016 (2MB, 32 x 64KB sectors):
 *   TIG:   sector 0        (0x000000, 64KB)
 *   SRM:   sectors 1-14    (0x010000, 896KB) - compressed console  
 *   SROM:  sectors 15-17   (0x0F0000, 192KB) - serial ROM bootstrap
 *   ARC:   sectors 18-30   (0x120000, 832KB)
 *   ARC variables: sector 31 (0x1F0000, 64KB)
 */

#ifndef SROM_BOOT_H
#define SROM_BOOT_H

#include "StdAfx.h"

 // SROM boot addresses (from real Tsunami/Typhoon hardware)
#define SROM_RESET_PC           U64(0x0000000000000000)  // Initial PC after reset
#define SROM_PAL_BASE           U64(0x0000000000000000)  // Initial PAL_BASE
#define SROM_FLASH_BASE         U64(0x0000080100000000)  // TIGbus Flash ROM address
#define SROM_DPR_BASE           U64(0x0000080110000000)  // Dual-Port RAM address

// SROM code entry points (typical for ES40/DS20)
#define SROM_CODE_START         U64(0x0000000000000000)
#define SROM_MINI_CONSOLE       U64(0x0000000000004000)

// SRM image signature
#define SRM_SIGNATURE_OFFSET    0x0000
#define SRM_ENTRY_OFFSET        0x0010
#define SRM_PAL_BASE_OFFSET     0x0018

/**
 * \brief SROM boot initialization structure
 * Contains information needed to boot from SROM
 */
struct SSROMBootInfo {
  u64 reset_pc;           // Initial program counter
  u64 pal_base;           // PAL base address
  u64 srm_entry;          // SRM console entry point
  bool valid;             // Boot info is valid
};

/**
 * Initialize the system for SROM-style boot
 * This sets up CPU and chipset state as the real SROM would
 */
bool srom_init_system(class CSystem* sys);

/**
 * Load boot firmware from flash
 * Returns boot information if successful
 */
SSROMBootInfo srom_load_boot_firmware(class CFlash* flash, class CSystem* sys);

/**
 * Configure CPU for SROM boot
 */
void srom_configure_cpu(class CAlphaCPU* cpu, const SSROMBootInfo& boot_info);

#endif // SROM_BOOT_H
