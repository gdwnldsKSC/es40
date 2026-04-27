/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * WWW    : http://es40.org
 * E-mail : camiel@es40.org
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
  * Contains the code for the emulated Floppy Controller devices.
  *
  * $Id$
  *
  * X-1.16       Camiel Vanderhoeven                             29-APR-2008
  *      Make floppy disk use CDisk images.
  *
  * X-1.15       Brian Wheeler                                   29-APR-2008
  *      Fixed floppy disk implementation.
  *
  * X-1.14       Brian Wheeler                                   29-APR-2008
  *      Floppy disk implementation.
  *
  * X-1.13       Camiel Vanderhoeven                             14-MAR-2008
  *      Formatting.
  *
  * X-1.12       Camiel Vanderhoeven                             14-MAR-2008
  *   1. More meaningful exceptions replace throwing (int) 1.
  *   2. U64 macro replaces X64 macro.
  *
  * X-1.11       Camiel Vanderhoeven                             30-DEC-2007
  *      Print file id on initialization.
  *
  * X-1.10       Camiel Vanderhoeven                             11-DEC-2007
  *      Don't claim IO port 3f6 as this is in use by the IDE controller.
  *
  * X-1.9        Camiel Vanderhoeven                             10-DEC-2007
  *      Use configurator.
  *
  * X-1.8        Camiel Vanderhoeven                             31-MAR-2007
  *      Added old changelog comments.
  *
  * X-1.7	Brian Wheeler					13-FEB-2007
  *	Formatting.
  *
  * X-1.6 	Camiel Vanderhoeven				12-FEB-2007
  *	Added comments.
  *
  * X-1.5        Camiel Vanderhoeven                             9-FEB-2007
  *      Added comments.
  *
  * X-1.4        Brian Wheeler                                   3-FEB-2007
  *      Formatting.
  *
  * X-1.3        Brian Wheeler                                   3-FEB-2007
  *      64-bit literals made compatible with Linux/GCC/glibc.
  *
  * X-1.2        Brian Wheeler                                   3-FEB-2007
  *      Includes are now case-correct (necessary on Linux)
  *
  * X-1.1        Camiel Vanderhoeven                             19-JAN-2007
  *      Initial version in CVS.
  *
  * \author Camiel Vanderhoeven (camiel@camicom.com / http://www.camicom.com)
  **/
#include "StdAfx.h"
#include "FloppyController.h"
#include "System.h"
#include "DMA.h"
#include "Disk.h"
#include "AliM1543C.h"

  /**
   * Constructor.
   **/
CFloppyController::CFloppyController(CConfigurator* cfg, CSystem* c, int id) : CSystemComponent(cfg, c), CDiskController(1, 2)
{
	c->RegisterMemory(this, 1536, U64(0x00000801fc0003f0) - (0x80 * id), 6);
	c->RegisterMemory(this, 1537, U64(0x00000801fc0003f7) - (0x80 * id), 1);

	memset(&state, 0, sizeof(state));
	state.dor = 0x0c;
	state.dor_dma_irq = true;
	state.dma = true;
	reset_controller(false);

	printf("%s: $Id$\n",
		devid_string);
}

/**
 * Destructor.
 **/
CFloppyController::~CFloppyController()
{
}

std::string datarate_name[] = {
	"500 Kb/S MFM",
	"300 Kb/S MFM",
	"250 Kb/S MFM",
	"1 Mb/S MFM"
};

struct cmdinfo_t {
	u8 command;
	u8 parms;
	u8 returns;
	std::string name;
}
cmdinfo[] = {
  { 0, 0, 0, ""},
  { 0, 0, 0, ""},
  { 2, 9, 7, "Read Track"},
  { 3, 3, 0, "Specify"},
  { 4, 2, 1, "Sense Drive Status"},
  { 5, 9, 7, "Write Data"},
  { 6, 9, 7, "Read Data"},
  { 7, 2, 0, "Recalibrate"},
  { 8, 1, 2, "Sense Interrupt Status"},
  { 9, 9, 7, "Write Deleted Data"},
  {10, 2, 7, "Read ID"},
  {11, 0, 0, ""},
  {12, 9, 7, "Read Deleted"},
  {13, 6, 7, "Format Track"},
  {14, 1, 10, "DumpReg"},
  {15, 3, 0, "Seek"},
  {16, 1, 1, "Version"},
  {17, 9, 7, "Scan Equal"},
  {18, 2, 0, "Perpendicular Mode"},
  {19, 4, 0, "Configure"},
  {20, 1, 1, "Lock"},
  {21, 0, 0, ""},
  {22, 9, 7, "Verify"},
  {23, 0, 0, ""},
  {24, 1, 1, "Part ID"},
  {25, 9, 7, "Scan Low or Equal"},
  {26, 0, 0, ""},
  {27, 0, 0, ""},
  {28, 0, 0, ""},
  {29, 9, 7, "Scan High or Equal"},
  {30, 0, 0, ""},
  {31, 0, 0, ""},
};

static u32 fdc_buffer_crc(const u8* buffer, size_t size)
{
	u32 crc = 2166136261u;
	for (size_t i = 0; i < size; i++)
	{
		crc ^= buffer[i];
		crc *= 16777619u;
	}
	return crc;
}

static void fdc_log_buffer_sample(const char* operation, const u8* buffer, size_t size)
{
	size_t sample = size < 16 ? size : 16;
	printf("FDC: %s sample: crc=%08x first", operation, fdc_buffer_crc(buffer, size));
	for (size_t i = 0; i < sample; i++)
		printf(" %02x", buffer[i]);
	printf("\n");
}

static void fdc_log_buffer_prefix(const char* operation, const u8* buffer, size_t size)
{
	size_t sample = size < 16 ? size : 16;
	printf("FDC: %s first", operation);
	for (size_t i = 0; i < sample; i++)
		printf(" %02x", buffer[i]);
	printf("\n");
}

static const char* fdc_status_text(bool value, const char* yes, const char* no)
{
	return value ? yes : no;
}

static void fdc_log_status_change(u8 status, bool rqm, bool dio, bool nondma, bool busy, bool seek0, bool seek1)
{
	static bool initialized = false;
	static u8 last_status = 0xff;

	if (initialized && status == last_status)
		return;

	initialized = true;
	last_status = status;
	printf("FDC Status[%02x]: %s, %s, %s, %s, %s, %s\n",
		status,
		fdc_status_text(rqm, "Data Register Ready", "No Access"),
		fdc_status_text(dio, "C->S", "S->C"),
		fdc_status_text(nondma, "No DMA", "DMA"),
		fdc_status_text(busy, "BUSY", "not busy"),
		fdc_status_text(seek0, "Disk 0 Seeking", "Disk 0 Idle"),
		fdc_status_text(seek1, "Disk 1 Seeking", "Disk 1 Idle"));
}

static u16 fdc_get_le16(const u8* buffer)
{
	return (u16)(buffer[0] | (buffer[1] << 8));
}

static u32 fdc_get_le32(const u8* buffer)
{
	return (u32)(buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24));
}

static bool fdc_fat_valid = false;
static u32 fdc_fat_root_lba = 0;
static u32 fdc_fat_root_sectors = 0;
static u32 fdc_fat_data_lba = 0;
static u8 fdc_fat_sectors_per_cluster = 0;
static u16 fdc_fat_bytes_per_sector = 0;
static u16 fdc_fat_sectors_per_track = 0;
static u16 fdc_fat_heads = 0;
static u32 fdc_fat_total_sectors = 0;
static u16 fdc_fat_reserved_sectors = 0;
static u16 fdc_fat_sectors_per_fat = 0;
static u16 fdc_fat_root_entries = 0;
static bool fdc_fat_cluster_map_ready = false;
static bool fdc_fat_cluster_mapped[4096];
static char fdc_fat_cluster_name[4096][13];
static u32 fdc_fat_cluster_offset[4096];
static u32 fdc_fat_cluster_size[4096];

static u16 fdc_fat12_next(const u8* fat, u16 cluster)
{
	u32 pos = cluster + (cluster / 2);
	if ((cluster & 1) == 0)
		return (u16)(fat[pos] | ((fat[pos + 1] & 0x0f) << 8));
	return (u16)(((fat[pos] >> 4) | (fat[pos + 1] << 4)) & 0x0fff);
}

static void fdc_fat_format_name(char* name, const u8* entry)
{
	int out = 0;
	for (int i = 0; i < 8 && entry[i] != ' '; i++)
		name[out++] = (char)entry[i];
	if (entry[8] != ' ')
	{
		name[out++] = '.';
		for (int i = 8; i < 11 && entry[i] != ' '; i++)
			name[out++] = (char)entry[i];
	}
	name[out] = 0;
}

static void fdc_cache_fat_cluster_map(CDisk* disk)
{
	if (!fdc_fat_valid || fdc_fat_cluster_map_ready || !disk ||
		fdc_fat_bytes_per_sector == 0 || fdc_fat_sectors_per_fat == 0 ||
		fdc_fat_root_entries == 0 || fdc_fat_sectors_per_cluster == 0)
		return;

	memset(fdc_fat_cluster_mapped, 0, sizeof(fdc_fat_cluster_mapped));
	memset(fdc_fat_cluster_name, 0, sizeof(fdc_fat_cluster_name));
	memset(fdc_fat_cluster_offset, 0, sizeof(fdc_fat_cluster_offset));
	memset(fdc_fat_cluster_size, 0, sizeof(fdc_fat_cluster_size));

	size_t fat_size = (size_t)fdc_fat_sectors_per_fat * fdc_fat_bytes_per_sector;
	size_t root_size = (size_t)fdc_fat_root_sectors * fdc_fat_bytes_per_sector;
	u8* fat = (u8*)malloc(fat_size);
	u8* root = (u8*)malloc(root_size);
	if (!fat || !root)
	{
		free(fat);
		free(root);
		return;
	}

	bool ok = disk->seek_byte((off_t_large)fdc_fat_reserved_sectors * fdc_fat_bytes_per_sector) &&
		disk->read_bytes(fat, fat_size) == fat_size &&
		disk->seek_byte((off_t_large)fdc_fat_root_lba * fdc_fat_bytes_per_sector) &&
		disk->read_bytes(root, root_size) == root_size;
	if (ok)
	{
		for (size_t pos = 0; pos + 32 <= root_size; pos += 32)
		{
			const u8* entry = root + pos;
			if (entry[0] == 0x00)
				break;
			if (entry[0] == 0xe5 || entry[11] == 0x0f || (entry[11] & 0x18))
				continue;

			u16 cluster = fdc_get_le16(entry + 26);
			u32 file_size = fdc_get_le32(entry + 28);
			if (cluster < 2 || cluster >= 0xff8)
				continue;

			char name[13];
			fdc_fat_format_name(name, entry);
			u32 file_offset = 0;
			u32 cluster_bytes = (u32)fdc_fat_sectors_per_cluster * fdc_fat_bytes_per_sector;
			for (int guard = 0; cluster >= 2 && cluster < 0xff8 && cluster < 4096 && guard < 4096; guard++)
			{
				fdc_fat_cluster_mapped[cluster] = true;
				strncpy(fdc_fat_cluster_name[cluster], name, sizeof(fdc_fat_cluster_name[cluster]) - 1);
				fdc_fat_cluster_offset[cluster] = file_offset;
				fdc_fat_cluster_size[cluster] = file_size;
				file_offset += cluster_bytes;
				if (file_offset >= file_size)
					break;
				cluster = fdc_fat12_next(fat, cluster);
			}
		}
		fdc_fat_cluster_map_ready = true;
		printf("FDC: FAT file map cached from root directory.\n");
	}

	free(fat);
	free(root);
}

static void fdc_log_fat_probe(CDisk* disk, const u8* buffer, size_t size, off_t_large lba, int sector_size)
{
	if (sector_size <= 0)
		return;

	size_t sectors_read = (size + (size_t)sector_size - 1) / (size_t)sector_size;
	if (lba == 0 && size >= 64)
	{
		u16 bytes_per_sector = fdc_get_le16(buffer + 11);
		u8 sectors_per_cluster = buffer[13];
		u16 reserved = fdc_get_le16(buffer + 14);
		u8 fats = buffer[16];
		u16 root_entries = fdc_get_le16(buffer + 17);
		u16 total_sectors16 = fdc_get_le16(buffer + 19);
		u8 media = buffer[21];
		u16 sectors_per_fat = fdc_get_le16(buffer + 22);
		u16 sectors_per_track = fdc_get_le16(buffer + 24);
		u16 heads = fdc_get_le16(buffer + 26);
		u32 hidden_sectors = fdc_get_le32(buffer + 28);
		u32 total_sectors32 = fdc_get_le32(buffer + 32);
		u32 total_sectors = total_sectors16 ? total_sectors16 : total_sectors32;
		u16 root_sectors = (u16)(((u32)root_entries * 32 + bytes_per_sector - 1) / bytes_per_sector);
		u32 root_lba = reserved + (u32)fats * sectors_per_fat;
		u32 data_lba = root_lba + root_sectors;
		fdc_fat_valid = bytes_per_sector != 0 && sectors_per_cluster != 0 &&
			fats != 0 && sectors_per_fat != 0 && root_sectors != 0;
		if (fdc_fat_valid)
		{
			fdc_fat_root_lba = root_lba;
			fdc_fat_root_sectors = root_sectors;
			fdc_fat_data_lba = data_lba;
			fdc_fat_sectors_per_cluster = sectors_per_cluster;
			fdc_fat_bytes_per_sector = bytes_per_sector;
			fdc_fat_sectors_per_track = sectors_per_track;
			fdc_fat_heads = heads;
			fdc_fat_total_sectors = total_sectors;
			fdc_fat_reserved_sectors = reserved;
			fdc_fat_sectors_per_fat = sectors_per_fat;
			fdc_fat_root_entries = root_entries;
			fdc_fat_cluster_map_ready = false;
		}
		printf("FDC: FAT BPB: bps=%u spc=%u reserved=%u fats=%u root_entries=%u media=%02x spf=%u spt=%u heads=%u hidden=%u total=%u root_lba=%u root_sectors=%u data_lba=%u\n",
			bytes_per_sector, sectors_per_cluster, reserved, fats, root_entries,
			media, sectors_per_fat, sectors_per_track, heads, hidden_sectors,
			total_sectors, root_lba, root_sectors, data_lba);
		if (fdc_fat_valid)
		{
			printf("FDC: FAT root directory expected: LBA=%u..%u\n",
				fdc_fat_root_lba, fdc_fat_root_lba + fdc_fat_root_sectors - 1);
			fdc_cache_fat_cluster_map(disk);
		}
	}
	else if (fdc_fat_valid && sectors_read > 0)
	{
		off_t_large end_lba = lba + (off_t_large)sectors_read - 1;
		off_t_large root_end = (off_t_large)fdc_fat_root_lba + fdc_fat_root_sectors - 1;
		if (lba <= root_end && end_lba >= (off_t_large)fdc_fat_root_lba)
			printf("FDC: FAT root directory read: LBA=%" PRId64 "..%" PRId64 "\n", lba, end_lba);
	}

	if (fdc_fat_valid && fdc_fat_cluster_map_ready && lba >= (off_t_large)fdc_fat_data_lba && sectors_read > 0)
	{
		for (size_t i = 0; i < sectors_read; i++)
		{
			off_t_large sector_lba = lba + (off_t_large)i;
			u32 cluster = (u32)((sector_lba - fdc_fat_data_lba) / fdc_fat_sectors_per_cluster) + 2;
			u32 sector_in_cluster = (u32)((sector_lba - fdc_fat_data_lba) % fdc_fat_sectors_per_cluster);
			if (cluster < 4096 && fdc_fat_cluster_mapped[cluster])
			{
				u32 file_offset = fdc_fat_cluster_offset[cluster] + sector_in_cluster * fdc_fat_bytes_per_sector;
				printf("FDC: FAT file data read: %s LBA=%" PRId64 " cluster=%u file_offset=%u size=%u\n",
					fdc_fat_cluster_name[cluster], sector_lba, cluster, file_offset, fdc_fat_cluster_size[cluster]);
			}
		}
	}

	for (size_t pos = 0; pos + 32 <= size; pos += 32)
	{
		const u8* entry = buffer + pos;
		if (entry[0] == 0x00 || entry[0] == 0xe5 || entry[11] == 0x0f)
			continue;
		if (memcmp(entry, "TXTSETUP", 8) == 0 && memcmp(entry + 8, "OEM", 3) == 0)
		{
			off_t_large entry_lba = lba + (off_t_large)(pos / sector_size);
			size_t entry_offset = pos % sector_size;
			u16 cluster = fdc_get_le16(entry + 26);
			u32 file_size = fdc_get_le32(entry + 28);
			printf("FDC: TXTSETUP.OEM directory entry found: LBA=%" PRId64 " offset=%zu attr=%02x cluster=%u size=%u",
				entry_lba, entry_offset, entry[11], cluster, file_size);
			if (fdc_fat_valid && cluster >= 2)
				printf(" data_lba=%u", fdc_fat_data_lba + ((u32)cluster - 2) * fdc_fat_sectors_per_cluster);
			printf("\n");
		}
	}
}

void CFloppyController::reset_controller(bool raise_irq)
{
	clear_interrupt();
	state.pio_active = false;
	state.pio_write = false;
	state.pio_size = 0;
	state.pio_pos = 0;
	state.cmd_parms_ptr = 0;
	state.cmd_res_ptr = 0;
	state.cmd_res_max = 0;
	state.status.rqm = true;
	state.status.dio = false;
	state.status.busy = false;
	state.status.nondma = false;
	for (int i = 0; i < 2; i++)
	{
		state.drive[i].seeking = 0;
		state.status.seeking[i] = false;
	}
	if (raise_irq)
		do_interrupt(0xc0, 0);
}

void CFloppyController::do_interrupt(u8 st0, u8 pcn)
{
	state.interrupt = true;
	state.interrupt_st0 = st0;
	state.interrupt_pcn = pcn;
	printf("FDC: IRQ6 assert, ST0=%02x PCN=%02x\n", st0, pcn);
	if (state.dor_dma_irq && theAli)
	{
		theAli->pic_deassert(0, 6);
		theAli->pic_interrupt(0, 6);
	}
}

void CFloppyController::clear_interrupt()
{
	if (state.interrupt && theAli)
		theAli->pic_deassert(0, 6);
	state.interrupt = false;
}

void CFloppyController::finish_result(int count, bool raise_irq)
{
	state.cmd_res_ptr = 0;
	state.cmd_res_max = (u8)count;
	state.status.rqm = true;
	state.status.dio = count > 0;
	if (raise_irq)
		do_interrupt(count > 0 ? state.cmd_res[0] : state.interrupt_st0,
			count > 3 ? state.cmd_res[3] : state.interrupt_pcn);
}

void CFloppyController::unsupported_command(int cmd)
{
	printf("Unhandled floppy command: %d = %s\n", cmd, cmdinfo[cmd].name.c_str());
	exit(1);
}

int CFloppyController::selected_drive(u8 drive_head)
{
	return drive_head & 0x03;
}

int CFloppyController::selected_head(u8 drive_head)
{
	return (drive_head >> 2) & 0x01;
}

bool CFloppyController::get_geometry(int drive, int eot, SGeometry* geometry)
{
	if (drive < 0 || drive >= 2 || !FDISK(drive))
		return false;

	off_t_large bytes = FDISK(drive)->get_byte_size();
	geometry->heads = 2;
	geometry->sectors = 18;
	geometry->cylinders = 80;

	if (fdc_fat_valid && fdc_fat_bytes_per_sector == 512 &&
		fdc_fat_sectors_per_track > 0 && fdc_fat_heads > 0)
	{
		u32 total_sectors = fdc_fat_total_sectors ? fdc_fat_total_sectors : (u32)(bytes / 512);
		geometry->heads = fdc_fat_heads;
		geometry->sectors = fdc_fat_sectors_per_track;
		geometry->cylinders = (int)(total_sectors / (geometry->heads * geometry->sectors));
		if (total_sectors % (geometry->heads * geometry->sectors))
			geometry->cylinders++;
		if (geometry->cylinders < 1)
			geometry->cylinders = 1;
	}
	else if (bytes == 368640)
	{
		geometry->cylinders = 40;
		geometry->heads = 2;
		geometry->sectors = 9;
	}
	else if (bytes == 737280)
	{
		geometry->cylinders = 80;
		geometry->heads = 2;
		geometry->sectors = 9;
	}
	else if (bytes == 1228800)
	{
		geometry->cylinders = 80;
		geometry->heads = 2;
		geometry->sectors = 15;
	}
	else if (bytes == 1474560)
	{
		geometry->cylinders = 80;
		geometry->heads = 2;
		geometry->sectors = 18;
	}
	else if (bytes == 2949120)
	{
		geometry->cylinders = 80;
		geometry->heads = 2;
		geometry->sectors = 36;
	}
	else
	{
		geometry->sectors = 18;
		geometry->cylinders = (int)(bytes / (512 * geometry->heads * geometry->sectors));
		if (bytes % (512 * geometry->heads * geometry->sectors))
			geometry->cylinders++;
		if (geometry->cylinders < 1)
			geometry->cylinders = 1;
	}

	return true;
}

int CFloppyController::bytes_per_sector(u8 n, u8 dtl)
{
	if (n == 0)
		return dtl ? dtl : 128;
	if (n > 7)
		return 0;
	return 128 << n;
}

int CFloppyController::track_eot(const SGeometry& geometry, int eot)
{
	// SRM uses EOT=36 on 1.44 MB media as one logical cylinder.
	if (eot > geometry.sectors && eot <= geometry.sectors * geometry.heads)
		return eot;
	if (eot > 0 && eot <= geometry.sectors)
		return eot;
	return geometry.sectors;
}

bool CFloppyController::chs_to_lba(int cylinder, int head, int sector, int eot, const SGeometry& geometry, off_t_large* lba)
{
	if (head < 0 || head >= geometry.heads || cylinder < 0 || cylinder >= geometry.cylinders)
		return false;

	int sectors = track_eot(geometry, eot);
	if (sector < 1 || sector > sectors)
		return false;

	if (sectors > geometry.sectors)
	{
		int logical_sector = sector;
		if (sector <= geometry.sectors)
			logical_sector += head * geometry.sectors;
		if (logical_sector > sectors)
			return false;
		*lba = ((off_t_large)cylinder * sectors) + (logical_sector - 1);
		return true;
	}

	*lba = (((off_t_large)cylinder * geometry.heads) + head) * geometry.sectors + (sector - 1);
	return true;
}

u8 CFloppyController::make_st0(int drive, int head, u8 flags)
{
	return (u8)((drive & ST0_DS) | (head ? ST0_HA : 0) | flags);
}

u8 CFloppyController::make_st3(int drive, int head)
{
	u8 st3 = (u8)((drive & ST0_DS) | (head ? ST3_HA : 0));
	if (drive >= 0 && drive < 2)
	{
		if (state.drive[drive].cylinder == 0)
			st3 |= ST3_TZ;
		if (FDISK(drive))
			st3 |= 0x20;  // ready
		if (FDISK(drive) && FDISK(drive)->ro())
			st3 |= ST3_WP;
	}
	return st3;
}

void CFloppyController::command_error(int drive, int head, u8 st0_flags, u8 st1, u8 st2)
{
	state.cmd_res[0] = make_st0(drive, head, st0_flags);
	state.cmd_res[1] = st1;
	state.cmd_res[2] = st2;
	state.cmd_res[3] = state.cmd_parms_ptr > 2 ? state.cmd_parms[2] : (drive >= 0 && drive < 2 ? state.drive[drive].cylinder : 0);
	state.cmd_res[4] = state.cmd_parms_ptr > 3 ? state.cmd_parms[3] : (u8)head;
	state.cmd_res[5] = state.cmd_parms_ptr > 4 ? state.cmd_parms[4] : 1;
	state.cmd_res[6] = state.cmd_parms_ptr > 5 ? state.cmd_parms[5] : 2;
	finish_result(7, true);
}

void CFloppyController::advance_chs(int* cylinder, int* head, int* sector, int sectors, const SGeometry& geometry, int eot, bool multi_track)
{
	int track_eot = this->track_eot(geometry, eot);
	if (track_eot > geometry.sectors)
	{
		if (*sector <= geometry.sectors)
			*sector += *head * geometry.sectors;
		*head = 0;
	}

	for (int i = 0; i < sectors; i++)
	{
		(*sector)++;
		if (*sector > track_eot)
		{
			*sector = 1;
			if (track_eot > geometry.sectors)
			{
				(*cylinder)++;
			}
			else
			{
				if (multi_track)
				{
					(*head)++;
					if (*head >= geometry.heads)
					{
						*head = 0;
						(*cylinder)++;
					}
				}
				else
				{
					(*cylinder)++;
				}
			}
		}
	}
}

bool CFloppyController::validate_transfer(int drive, int head, int cylinder, int sector, int eot, int sector_size, size_t bytes, SGeometry* geometry)
{
	if (drive < 0 || drive >= 2 || !FDISK(drive))
		return false;
	if (!get_geometry(drive, eot, geometry))
		return false;
	if (sector_size <= 0 || bytes == 0)
		return false;

	int sectors = (int)((bytes + sector_size - 1) / sector_size);
	off_t_large lba;
	if (!chs_to_lba(cylinder, head, sector, eot, *geometry, &lba))
		return false;

	off_t_large last_lba = lba + sectors - 1;
	if (last_lba < lba)
		return false;
	off_t_large byte_offset = lba * sector_size;
	if (byte_offset < 0 || byte_offset + (off_t_large)bytes > FDISK(drive)->get_byte_size())
		return false;

	return true;
}

size_t CFloppyController::pio_transfer_size(int head, int sector, int sector_size, int eot, const SGeometry& geometry)
{
	int sectors = track_eot(geometry, eot);
	int logical_sector = sector;
	if (sectors > geometry.sectors && sector <= geometry.sectors)
		logical_sector += head * geometry.sectors;
	if (logical_sector < 1 || logical_sector > sectors)
		return 0;
	return (size_t)(sectors - logical_sector + 1) * sector_size;
}

void CFloppyController::prepare_transfer_result(int drive, int head, int cylinder, int sector, int sector_size, int eot, size_t transfer_size, const SGeometry& geometry)
{
	int result_c = cylinder;
	int result_h = head;
	int result_r = sector;
	int sectors = (int)((transfer_size + sector_size - 1) / sector_size);
	bool multi_track = (state.cmd_parms[0] & 0x80) != 0;
	u8 st0_flags = 0;
	advance_chs(&result_c, &result_h, &result_r, sectors, geometry, eot, multi_track);
	if (result_c != cylinder || result_h != head)
		st0_flags |= ST0_SE;

	state.drive[drive].cylinder = result_c;
	state.cmd_res[0] = make_st0(drive, result_h, st0_flags);
	state.cmd_res[1] = 0;
	state.cmd_res[2] = 0;
	state.cmd_res[3] = (u8)result_c;
	state.cmd_res[4] = (u8)result_h;
	state.cmd_res[5] = (u8)result_r;
	state.cmd_res[6] = state.cmd_parms[5];
	printf("FDC: transfer result prepared: ST0=%02x ST1=%02x ST2=%02x C=%02x H=%02x R=%02x N=%02x\n",
		state.cmd_res[0], state.cmd_res[1], state.cmd_res[2], state.cmd_res[3],
		state.cmd_res[4], state.cmd_res[5], state.cmd_res[6]);
}

void CFloppyController::finish_pio_transfer(bool ok)
{
	int drive = state.pio_drive;
	int head = state.pio_head;
	u32 transferred = state.pio_pos;
	u32 requested = state.pio_size;
	bool was_write = state.pio_write;
	bool irq_pending = state.interrupt;

	if (ok && state.pio_write)
	{
		FDISK(drive)->seek_byte(state.pio_offset);
		ok = FDISK(drive)->write_bytes(state.pio_data, state.pio_size) == state.pio_size;
		FDISK(drive)->flush();
	}

	state.pio_active = false;
	state.pio_write = false;
	state.pio_size = 0;
	state.pio_pos = 0;

	if (!ok)
	{
		printf("FDC: non-DMA %s failed: %u/%u bytes\n", was_write ? "write" : "read", transferred, requested);
		command_error(drive, head, 0x40, ST1_ND, 0);
		return;
	}

	printf("FDC: non-DMA %s complete: %u/%u bytes\n", was_write ? "write" : "read", transferred, requested);
	finish_result(7, false);
	if (!irq_pending)
		do_interrupt(state.cmd_res[0], state.cmd_res[3]);
}

void CFloppyController::execute_read_write(bool write)
{
	int drive = selected_drive(state.cmd_parms[1]);
	int head = selected_head(state.cmd_parms[1]);
	int cylinder = state.cmd_parms[2];
	int address_head = state.cmd_parms[3];
	int sector = state.cmd_parms[4];
	int sector_size = bytes_per_sector(state.cmd_parms[5], state.cmd_parms[8]);
	int eot = state.cmd_parms[6];
	SGeometry geometry;

	if (state.dma && !theDMA)
		FAILURE(Runtime, "FDC: DMA controller is not available");
	if (head != address_head)
	{
		command_error(drive, head, 0x40, ST1_ND, ST2_WC);
		return;
	}

	size_t transfer_size = state.dma ? theDMA->get_transfer_size(2) : 0;
	if (!state.dma)
	{
		if (!get_geometry(drive, eot, &geometry))
		{
			command_error(drive, head, 0x40 | ST0_NR, ST1_ND, 0);
			return;
		}
		transfer_size = pio_transfer_size(head, sector, sector_size, eot, geometry);
		if (transfer_size == 0 || transfer_size > sizeof(state.pio_data))
		{
			command_error(drive, head, 0x40, ST1_ND, 0);
			return;
		}
	}

	if (!validate_transfer(drive, head, cylinder, sector, eot, sector_size, transfer_size, &geometry))
	{
		command_error(drive, head, 0x40 | (drive >= 0 && drive < 2 && FDISK(drive) ? 0 : ST0_NR), ST1_ND, 0);
		return;
	}
	if (write && FDISK(drive)->ro())
	{
		command_error(drive, head, 0x40, ST1_WP, 0);
		return;
	}

	off_t_large lba;
	chs_to_lba(cylinder, head, sector, eot, geometry, &lba);
	off_t_large byte_offset = lba * sector_size;

	prepare_transfer_result(drive, head, cylinder, sector, sector_size, eot, transfer_size, geometry);

	if (!state.dma)
	{
		state.pio_active = true;
		state.pio_write = write;
		state.pio_drive = (u8)drive;
		state.pio_head = (u8)head;
		state.pio_offset = byte_offset;
		state.pio_size = (u32)transfer_size;
		state.pio_pos = 0;
		state.cmd_res_ptr = 0;
		state.cmd_res_max = 0;
		state.status.rqm = true;
		state.status.dio = !write;

		do_interrupt(0, drive >= 0 && drive < 2 ? DRIVE(drive).cylinder : 0);

		if (write)
		{
			printf("FDC: non-DMA write data: %zx @ %" PRId64 " (C=%d H=%d R=%d EOT=%d LBA=%" PRId64 ")\n",
				transfer_size, byte_offset, cylinder, head, sector, eot, lba);
			return;
		}

		memset(state.pio_data, 0, transfer_size);
		FDISK(drive)->seek_byte(byte_offset);
		bool ok = FDISK(drive)->read_bytes(state.pio_data, transfer_size) == transfer_size;
		printf("FDC: non-DMA read data: %zx @ %" PRId64 " (C=%d H=%d R=%d EOT=%d LBA=%" PRId64 ")\n",
			transfer_size, byte_offset, cylinder, head, sector, eot, lba);
		if (ok)
		{
			fdc_log_buffer_sample("non-DMA read", state.pio_data, transfer_size);
			fdc_log_fat_probe(FDISK(drive), state.pio_data, transfer_size, lba, sector_size);
		}
		if (!ok)
			finish_pio_transfer(false);
		return;
	}

	char* buffer = (char*)malloc(transfer_size);
	CHECK_ALLOCATION(buffer);

	bool ok = true;

	if (write)
	{
		theDMA->recv_data(2, buffer);
		fdc_log_buffer_sample("DMA write", (u8*)buffer, transfer_size);
		FDISK(drive)->seek_byte(byte_offset);
		ok = FDISK(drive)->write_bytes(buffer, transfer_size) == transfer_size;
		FDISK(drive)->flush();
	}
	else
	{
		memset(buffer, 0, transfer_size);
		FDISK(drive)->seek_byte(byte_offset);
		ok = FDISK(drive)->read_bytes(buffer, transfer_size) == transfer_size;
		if (ok)
		{
			fdc_log_buffer_sample("DMA read", (u8*)buffer, transfer_size);
			fdc_log_fat_probe(FDISK(drive), (u8*)buffer, transfer_size, lba, sector_size);
		}
		if (ok)
			theDMA->send_data(2, buffer);
	}

	free(buffer);
	if (!ok)
	{
		command_error(drive, head, 0x40, ST1_ND, 0);
		return;
	}

	finish_result(7, true);
}

void CFloppyController::execute_verify()
{
	int drive = selected_drive(state.cmd_parms[1]);
	int head = selected_head(state.cmd_parms[1]);
	int cylinder = state.cmd_parms[2];
	int address_head = state.cmd_parms[3];
	int sector = state.cmd_parms[4];
	int sector_size = bytes_per_sector(state.cmd_parms[5], state.cmd_parms[8]);
	int eot = state.cmd_parms[6];
	SGeometry geometry;

	if (head != address_head || !validate_transfer(drive, head, cylinder, sector, eot, sector_size, sector_size, &geometry))
	{
		command_error(drive, head, 0x40 | (drive >= 0 && drive < 2 && FDISK(drive) ? 0 : ST0_NR), ST1_ND, head != address_head ? ST2_WC : 0);
		return;
	}

	advance_chs(&cylinder, &head, &sector, 1, geometry, eot, (state.cmd_parms[0] & 0x80) != 0);
	state.cmd_res[0] = make_st0(drive, selected_head(state.cmd_parms[1]), 0);
	state.cmd_res[1] = 0;
	state.cmd_res[2] = 0;
	state.cmd_res[3] = (u8)cylinder;
	state.cmd_res[4] = (u8)head;
	state.cmd_res[5] = (u8)sector;
	state.cmd_res[6] = state.cmd_parms[5];
	finish_result(7, true);
}

void CFloppyController::execute_read_id()
{
	int drive = selected_drive(state.cmd_parms[1]);
	int head = selected_head(state.cmd_parms[1]);
	SGeometry geometry;

	if (!get_geometry(drive, 0, &geometry))
	{
		command_error(drive, head, 0x40 | ST0_NR, ST1_ND, 0);
		return;
	}

	int cylinder = state.drive[drive].cylinder;
	if (cylinder >= geometry.cylinders)
		cylinder = geometry.cylinders - 1;
	state.cmd_res[0] = make_st0(drive, head, 0);
	state.cmd_res[1] = 0;
	state.cmd_res[2] = 0;
	state.cmd_res[3] = (u8)cylinder;
	state.cmd_res[4] = (u8)head;
	state.cmd_res[5] = 1;
	state.cmd_res[6] = 2;
	finish_result(7, true);
}

void CFloppyController::WriteMem(int index, u64 address, int dsize, u64 data)
{
	if (index == 1537)
		address += 7;

	switch (address)
	{
	case FDC_REG_STATUS_A:
	case FDC_REG_STATUS_B:
		printf("FDC: Read only register %" PRId64 " written.\n", address);
		break;

	case FDC_REG_DOR:
	{
		// bit 4 = drive 0 motor, bit 5 = drive 1 motor
		// bit 3 = dma enable
		// bit 2 = 1: fdc enable (reset), 0: hold at reset
		// bits 1-0: drive select
		bool was_reset = (state.dor & 0x04) != 0;
		state.dor = (u8)data;
		state.drive[0].motor = (data & 0x10) != 0;
		state.drive[1].motor = (data & 0x20) != 0;
		state.dor_dma_irq = (data & 0x08) != 0;
		state.drive_select = data & 0x03;

		printf("FDC:  motor a: %s, motor b: %s, irq/dma: %s, transfer: %s, drive: %s\n",
			state.drive[0].motor ? "on" : "off",
			state.drive[1].motor ? "on" : "off",
			state.dor_dma_irq ? "on" : "off",
			state.dma ? "DMA" : "PIO",
			state.drive_select == 0 ? "A" : "B");

		if (!(state.dor & 0x04))
			reset_controller(false);
		else if (!was_reset)
			reset_controller(true);
		break;
	}

	case FDC_REG_TAPE:
		printf("FDC: Tape register written with %" PRIx64 "\n", data);
		break;

	case FDC_REG_STATUS:  // write = data rate selector
		// bit 7 = software reset (self clearing)
		// bit 6 = power down
		// bit 5 = reserved (0)
		// bit 4-2 = write precomp (000 = default)
		// bit 1-0 = data rate select
		if (data & 0x80)
			reset_controller(true);
		state.datarate = data & 0x03;
		state.write_precomp = (data & 0x1c) >> 2;
		printf("FDC: data rate %s, precomp: %d\n", datarate_name[state.datarate].c_str(), state.write_precomp);
		break;

	case FDC_REG_COMMAND:
		if (dsize > 8)
		{
			int bytes = dsize / 8;
			if (bytes > 8)
				bytes = 8;
			for (int i = 0; i < bytes; i++)
				WriteMem(index, address, 8, (data >> (i * 8)) & 0xff);
			break;
		}

		if (state.pio_active)
		{
			if (!state.pio_write)
			{
				printf("FDC: write to data port during non-DMA read phase.\n");
				break;
			}

			clear_interrupt();
			state.pio_data[state.pio_pos++] = (u8)data;
			if ((state.pio_pos & 0x7f) == 0 || state.pio_pos >= state.pio_size)
				printf("FDC: non-DMA write progress: %u/%u\n", state.pio_pos, state.pio_size);
			if (state.pio_pos >= state.pio_size)
				finish_pio_transfer(true);
			break;
		}

		if (state.status.dio) {
			printf("Unrequested data byte to command port.  Throwing away.\n");
			break;
		}

		if (state.cmd_parms_ptr >= sizeof(state.cmd_parms))
			unsupported_command(state.cmd_parms[0] & 0x1f);
		state.cmd_parms[state.cmd_parms_ptr++] = (u8)data;
		{
			int cmd = state.cmd_parms[0] & 0x1f;
			if (cmdinfo[cmd].parms == 0)
				unsupported_command(cmd);
			state.cmd_res_max = 0;
			state.status.dio = false;

			if (state.cmd_parms_ptr == cmdinfo[cmd].parms)
			{
				printf("FDC: command %s[%02x](", cmdinfo[cmd].name.c_str(), state.cmd_parms[0]);
				for (int i = 1; i < state.cmd_parms_ptr; i++)
					printf("%x ", state.cmd_parms[i]);
				printf(")\n");

				state.status.rqm = false;
				switch (cmd) {
				case 3: // specify
					state.dma = (state.cmd_parms[2] & 0x01) == 0;
					break;

				case 4: // sense drive status
				{
					int drive = selected_drive(state.cmd_parms[1]);
					int head = selected_head(state.cmd_parms[1]);
					state.cmd_res[0] = make_st3(drive, head);
					finish_result(1, false);
				}
				break;

				case 5: // write data
					execute_read_write(true);
					break;

				case 6: // read data
					execute_read_write(false);
					break;

				case 7: // recalibrate
				{
					int drive = selected_drive(state.cmd_parms[1]);
					if (drive >= 0 && drive < 2)
					{
						DRIVE(drive).seeking = 3;
						DRIVE(drive).cylinder = 0;
					}
					do_interrupt(make_st0(drive, 0, ST0_SE | (drive >= 0 && drive < 2 && FDISK(drive) ? 0 : ST0_NR)),
						drive >= 0 && drive < 2 ? DRIVE(drive).cylinder : 0);
				}
				break;

				case 8: // sense interrupt status
					if (!state.interrupt)
					{
						state.cmd_res[0] = 0x80;
						state.cmd_res[1] = 0;
					}
					else
					{
						state.cmd_res[0] = state.interrupt_st0;
						state.cmd_res[1] = state.interrupt_pcn;
					}
					clear_interrupt();
					finish_result(2, false);
					break;

				case 10: // read id
					execute_read_id();
					break;

				case 15: // seek
				{
					int drive = selected_drive(state.cmd_parms[1]);
					int head = selected_head(state.cmd_parms[1]);
					if (drive >= 0 && drive < 2)
					{
						DRIVE(drive).seeking = 3;
						DRIVE(drive).cylinder = state.cmd_parms[2];
					}
					do_interrupt(make_st0(drive, head, ST0_SE | (drive >= 0 && drive < 2 && FDISK(drive) ? 0 : ST0_NR)),
						drive >= 0 && drive < 2 ? DRIVE(drive).cylinder : 0);
				}
				break;

				case 16: // version
					state.cmd_res[0] = 0x90;
					finish_result(1, false);
					break;

				case 18: // perpendicular mode
					break;

				case 19: // configure
					break;

				case 20: // lock
					state.locked = (state.cmd_parms[0] & 0x80) != 0;
					state.cmd_res[0] = state.locked ? 0x10 : 0x00;
					finish_result(1, false);
					break;

				case 22: // verify
					execute_verify();
					break;

				case 24: // NSC / 82078 PartID. Windows 2000 issues this to identify the FDC.
					// Return 0x41 (82078, stepping 1) to match QEMU/Bochs and pair with the 0x90 Version response above.
					state.cmd_res[0] = 0x41;
					finish_result(1, false);
					break;

				default:
					unsupported_command(cmd);
				}

				state.status.rqm = true;
				state.cmd_parms_ptr = 0;
			}
		}
		break;

	case FDC_REG_DIR:
		// Write = configuration control register.
		state.datarate = data & 0x03;
		printf("FDC: data rate %s\n", datarate_name[state.datarate].c_str());
		break;
	}
}

u64 CFloppyController::ReadMem(int index, u64 address, int dsize)
{
	u64 data = 0;
	bool log_read = true;

	if (index == 1537)
		address += 7;

	switch (address)
	{
	case FDC_REG_STATUS_A:
		data = (state.interrupt ? 0x80 : 0x00) |
			(state.drive[0].cylinder == 0 ? 0x00 : 0x10);
		break;

	case FDC_REG_STATUS_B:
		data = 0xc0 |
			(state.drive_select == 1 ? 0x20 : 0x00) |
			(state.drive[1].motor ? 0x02 : 0x00) |
			(state.drive[0].motor ? 0x01 : 0x00);
		break;

	case FDC_REG_DOR:
	case FDC_REG_TAPE:
		printf("FDC: Write only register %" PRId64 " read.", address);
		break;

	case FDC_REG_STATUS:
		data = get_status();
		log_read = false;
		break;

	case FDC_REG_COMMAND:
	{
		int bytes = dsize / 8;
		if (bytes < 1)
			bytes = 1;
		if (bytes > 8)
			bytes = 8;

		for (int i = 0; i < bytes; i++)
		{
			u8 value = 0;

			if (state.pio_active && !state.pio_write)
			{
				bool complete;
				clear_interrupt();
				value = state.pio_data[state.pio_pos++];
				log_read = false;
				complete = state.pio_pos >= state.pio_size;
				if (state.pio_pos == 16 || (complete && state.pio_pos < 16))
					fdc_log_buffer_prefix("non-DMA read delivered", state.pio_data, state.pio_pos);
				if ((state.pio_pos & 0x7f) == 0 || complete)
					printf("FDC: non-DMA read progress: %u/%u\n", state.pio_pos, state.pio_size);
				data |= ((u64)value) << (i * 8);
				if (complete)
				{
					finish_pio_transfer(true);
					break;
				}
				continue;
			}

			if (!state.status.dio || state.cmd_res_ptr >= state.cmd_res_max)
			{
				printf("FDC: Data register read with no result pending.\n");
				data |= ((u64)value) << (i * 8);
				break;
			}

			value = state.cmd_res[state.cmd_res_ptr++];
			data |= ((u64)value) << (i * 8);
			if (state.cmd_res_ptr >= state.cmd_res_max) {
				state.status.rqm = 1;
				state.status.dio = 0;
				state.cmd_res_ptr = 0;
				state.cmd_res_max = 0;
				clear_interrupt();
				break;
			}
		}
		break;
	}

	case FDC_REG_DIR:
		// bit 7 = diskette change, bits 6-3 = 1, bits 2-1 = data rate, bit 0 = high density.
		data = 0x78 | ((state.datarate & 0x03) << 1) | 0x01;
		break;
	}

	if (log_read)
		printf("FDC: Read register %" PRId64 ", value: %" PRIx64 "\n", address, data);

	return data;
}

static u32 fdc_magic1 = 0x0fdc0fdc;
static u32 fdc_magic2 = 0xfdc0fdc0;

int CFloppyController::SaveState(FILE* f) {
	long  ss = sizeof(state);

	fwrite(&fdc_magic1, sizeof(u32), 1, f);
	fwrite(&ss, sizeof(long), 1, f);
	fwrite(&state, sizeof(state), 1, f);
	fwrite(&fdc_magic2, sizeof(u32), 1, f);
	printf("fdc: %ld bytes saved.\n", ss);
	return 0;
}

int CFloppyController::RestoreState(FILE* f)
{
	long    ss;
	u32     m1;
	u32     m2;
	size_t  r;

	r = fread(&m1, sizeof(u32), 1, f);
	if (r != 1)
	{
		printf("fdc: unexpected end of file!\n");
		return -1;
	}

	if (m1 != fdc_magic1)
	{
		printf("fdc: MAGIC 1 does not match!\n");
		return -1;
	}

	r = fread(&ss, sizeof(long), 1, f);
	if (r != 1)
	{
		printf("fdc: unexpected end of file!\n");
		return -1;
	}

	if (ss != sizeof(state))
	{
		printf("fdc: STRUCT SIZE does not match!\n");
		return -1;
	}

	r = fread(&state, sizeof(state), 1, f);
	if (r != 1)
	{
		printf("fdc: unexpected end of file!\n");
		return -1;
	}

	r = fread(&m2, sizeof(u32), 1, f);
	if (r != 1)
	{
		printf("fdc: unexpected end of file!\n");
		return -1;
	}

	if (m2 != fdc_magic2)
	{
		printf("fdc: MAGIC 1 does not match!\n");
		return -1;
	}

	printf("fdc: %ld bytes restored.\n", ss);
	return 0;
}

u8 CFloppyController::get_status() {
	// bit 7 = RQM data register is ready (0: no access is permitted)
	// bit 6 = 1: transfer from controller to system, 0: sys to controller
	// bit 5 = non dma mode
	// bit 4 = diskette controller is busy
	// bit 3-2 reserved
	// bit 1 = drive 1 is busy (seeking)
	// bit 0 = drive 0 is busy (seeking)

	for (int i = 0; i < 2; i++) {
		if (state.drive[i].seeking > 0)
			state.drive[i].seeking--;
		state.status.seeking[i] = state.drive[i].seeking != 0;
	}

	state.status.busy = state.status.seeking[0] || state.status.seeking[1] || state.pio_active ||
		(state.status.dio && state.status.rqm);
	state.status.nondma = state.pio_active;

	if (state.interrupt && state.dor_dma_irq && theAli)
		theAli->pic_interrupt(0, 6);

	u8 data = (state.status.rqm ? 0x80 : 0x00) |
		(state.status.dio ? 0x40 : 0x00) |
		(state.status.nondma ? 0x20 : 0x00) |
		(state.status.busy ? 0x10 : 0x00) |
		(state.status.seeking[1] ? 0x02 : 0x00) |
		(state.status.seeking[0] ? 0x01 : 0x00);
	fdc_log_status_change(data, state.status.rqm, state.status.dio, state.status.nondma,
		state.status.busy, state.status.seeking[0], state.status.seeking[1]);
	return data;
}
