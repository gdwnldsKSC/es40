/* ES40 emulator - SROM Boot Implementation
 *
 * Flash layout for AM29F016 (2MB, 32 x 64KB sectors):
 *   TIG:   sector 0        (0x000000, 64KB)
 *   SRM:   sectors 1-14    (0x010000, 896KB) - compressed console
 *   SROM:  sectors 15-17   (0x0F0000, 192KB) - serial ROM bootstrap
 *   ARC:   sectors 18-30   (0x120000, 832KB)
 *   ARC variables: sector 31 (0x1F0000, 64KB)
 */

#include "StdAfx.h"
#include "srom_boot.h"
#include "System.h"
#include "AlphaCPU.h"
#include "Flash.h"
#include "DPR.h"

 // SROM partition in flash
static const u32 SROM_FLASH_OFFSET = 0x0F0000;  // Sectors 15-17 start at 960KB
static const u32 SROM_SIZE = 0x30000;           // 192KB (3 x 64KB sectors)

/**
 * Initialize system as SROM would after power-on reset.
 */
bool srom_init_system(CSystem* sys)
{
    if (!sys) return false;

    printf("%%SROM-I-INIT: Initializing system (true SROM boot mode)\n");

    if (theDPR) {
        printf("%%SROM-I-DPR: DPR present\n");
    }

    return true;
}

/**
 * Load SROM code from flash and prepare for execution.
 *
 * On a real ES40:
 * - EV68 comes out of reset, SROM is loaded into I-cache
 * - SROM executes, initializes chipset, memory, loads SRM console
 * - SROM jumps to SRM console
 *
 * For emulation, we copy SROM (192KB from flash offset 0x0F0000) to
 * physical address 0 and start the CPU there.
 */
SSROMBootInfo srom_load_boot_firmware(CFlash* flash, CSystem* sys)
{
    SSROMBootInfo info = { 0, 0, 0, false };

    if (!flash || !sys) {
        printf("%%SROM-E-NULLPTR: Flash or system not available\n");
        return info;
    }

    const u8* flash_data = flash->GetFlashBytes();
    if (!flash_data) {
        printf("%%SROM-E-NOFLASH: Cannot read flash data\n");
        return info;
    }

    // Get SROM partition (sectors 15-17, offset 0x0F0000)
    const u8* srom_data = flash_data + SROM_FLASH_OFFSET;
    
    printf("%%SROM-I-DEBUG: First 16 bytes at flash+0x%X:\n", SROM_FLASH_OFFSET);
    printf("  %02X %02X %02X %02X %02X %02X %02X %02X\n",
        srom_data[0], srom_data[1], srom_data[2], srom_data[3],
        srom_data[4], srom_data[5], srom_data[6], srom_data[7]);
    printf("  %02X %02X %02X %02X %02X %02X %02X %02X\n",
        srom_data[8], srom_data[9], srom_data[10], srom_data[11],
        srom_data[12], srom_data[13], srom_data[14], srom_data[15]);

    // Check first instruction - should NOT be CALL_PAL (opcode 0x00)
    u32 first_inst = srom_data[0] | (srom_data[1] << 8) |
        (srom_data[2] << 16) | (srom_data[3] << 24);

    if (first_inst == 0xFFFFFFFF) {
        printf("%%SROM-W-NOSROM: SROM partition appears erased at flash+0x%X\n",
            SROM_FLASH_OFFSET);
        return info;
    }

    // CALL_PAL has opcode 0x00 in bits 31:26 - not valid SROM entry
    if ((first_inst >> 26) == 0x00) {
        printf("%%SROM-W-INVALID: SROM starts with CALL_PAL (0x%08X) - invalid\n",
            first_inst);
        return info;
    }

    printf("%%SROM-I-SROM: Found SROM at flash offset 0x%X (%d KB)\n",
        SROM_FLASH_OFFSET, SROM_SIZE / 1024);
    printf("%%SROM-I-SROM: First instruction: 0x%08X\n", first_inst);

    // Copy SROM to physical address 0
    // Real hardware loads SROM into I-cache; we simulate by copying to RAM
    char* mem_ptr = sys->PtrToMem(0);
    if (!mem_ptr) {
        printf("%%SROM-E-MEMFAIL: Cannot get memory pointer for address 0\n");
        return info;
    }

    memcpy(mem_ptr, srom_data, SROM_SIZE);
    printf("%%SROM-I-SROM: Copied %d KB SROM to physical address 0\n", SROM_SIZE / 1024);

    // Verify the copy succeeded
    u32 verify_inst = ((u8*)mem_ptr)[0] | (((u8*)mem_ptr)[1] << 8) |
        (((u8*)mem_ptr)[2] << 16) | (((u8*)mem_ptr)[3] << 24);
    printf("%%SROM-I-SROM: Verify instruction at phys 0: 0x%08X\n", verify_inst);

    if (verify_inst != first_inst) {
        printf("%%SROM-E-VERIFY: Memory copy verification failed!\n");
        return info;
    }

    // EV68 reset state:
    // - PC = 0 (with PAL bit set -> 0x1)
    // - PAL_BASE = 0
    // - Processor starts in PAL mode
    info.reset_pc = 0;
    info.pal_base = 0;
    info.srm_entry = 0;
    info.valid = true;

    printf("%%SROM-I-SROM: Ready for SROM execution at PC=0x1 (PAL mode)\n");
    return info;
}

/**
 * Configure CPU state for SROM boot (true hardware reset state).
 */
void srom_configure_cpu(CAlphaCPU* cpu, const SSROMBootInfo& boot_info)
{
    if (!cpu || !boot_info.valid) return;

    printf("%%SROM-I-CPU: Configuring CPU %d for SROM boot\n", cpu->get_cpuid());

    // Flush I-cache to ensure CPU fetches fresh code from memory
    cpu->flush_icache();

    // PAL_BASE = 0: SROM itself serves as initial PALcode
    cpu->set_PAL_BASE(0);

    // PC = 0x1: Address 0 with PAL mode bit set
    cpu->set_pc(0x1);

    // Primary CPU starts immediately
    if (cpu->get_cpuid() == 0) {
        cpu->stop_waiting();
    }

    printf("%%SROM-I-CPU:   PAL_BASE: 0x%016" PRIx64 "\n", cpu->get_pal_base());
    printf("%%SROM-I-CPU:   PC: 0x%016" PRIx64 " (PAL mode at address 0)\n", cpu->get_pc());
}
