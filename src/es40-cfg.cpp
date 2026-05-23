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
  * Configuration file creator.
  *
  * $Id$
  *
  * X-1.7        Camiel Vanderhoeven                             29-APR-2008
  *      Added floppy configuration.
  *
  * X-1.6        Camiel Vanderhoeven                             29-MAR-2008
  *      Replaced SDL with sdl.
  *
  * X-1.5        Camiel Vanderhoeven                             29-MAR-2008
  *      Fix VGA console value.
  *
  * X-1.4        Camiel Vanderhoeven                             29-MAR-2008
  *      Fill in NIC section.
  *
  * X-1.3        Camiel Vanderhoeven                             28-MAR-2008
  *      Fixed CD-ROM question behaviour.
  *
  * X-1.2        Camiel Vanderhoeven                             28-MAR-2008
  *      Properly capitalized "StdAfx.h".
  *
  * X-1.1        Camiel Vanderhoeven                             28-MAR-2008
  *      File created.
  **/

#include "StdAfx.h"

#ifdef _WIN32
#pragma comment(lib, "winmm.lib")
#include <windows.h>
#include <mmsystem.h>
#endif

  // C++ includes
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#if defined(HAVE_PCAP)
#ifdef _WIN32
#include <winsock.h>
typedef int          bpf_int32;
typedef unsigned int bpf_u_int32;

/*
 * The instruction data structure.
 */
struct bpf_insn {
    unsigned short code;
    unsigned char  jt;
    unsigned char  jf;
    bpf_u_int32    k;
};

/*
 * Structure for "pcap_compile()", "pcap_setfilter()", etc..
 */
struct bpf_program {
    unsigned int     bf_len;
    struct bpf_insn *bf_insns;
};

typedef struct pcap_if pcap_if_t;

#    define PCAP_ERRBUF_SIZE 256

struct pcap_pkthdr {
    struct timeval ts;
    bpf_u_int32    caplen;
    bpf_u_int32    len;
};

struct pcap_if {
    struct pcap_if *next;
    char           *name;
    char           *description;
    void           *addresses;
    bpf_u_int32     flags;
};

struct pcap_send_queue {
    unsigned int maxlen; /* Maximum size of the queue, in bytes. This
             variable contains the size of the buffer field. */
    unsigned int len;    /* Current size of the queue, in bytes. */
    char *buffer; /* Buffer containing the packets to be sent. */
};

typedef struct pcap_send_queue pcap_send_queue;

typedef void (*pcap_handler)(unsigned char *user, const struct pcap_pkthdr *h, const unsigned char *bytes);

typedef struct pcap pcap_t;
typedef struct pcap_dumper pcap_dumper_t;
typedef struct pcap_if pcap_if_t;
typedef struct pcap_addr pcap_addr_t;

/* Pointers to the real functions. */
static const char *(*f_pcap_lib_version)(void);
static int (*f_pcap_findalldevs)(pcap_if_t **, char *);
static void (*f_pcap_freealldevs)(void *);
static pcap_t* (*f_pcap_open_live)(const char *, int, int, int, char *);
static int (*f_pcap_compile)(void *, void *, const char *, int, bpf_u_int32);
static int (*f_pcap_setfilter)(void *, void *);
static const unsigned char
    *(*f_pcap_next)(void *, void *);
static int (*f_pcap_sendpacket)(void *, const unsigned char *, int);
static void (*f_pcap_close)(void *);
static int (*f_pcap_setnonblock)(void *, int, char *);
static int (*f_pcap_set_immediate_mode)(void *, int);
static int (*f_pcap_set_promisc)(void *, int);
static int (*f_pcap_set_snaplen)(void *, int);
static int (*f_pcap_dispatch)(void *, int, pcap_handler callback, unsigned char *user);
static void *(*f_pcap_create)(const char *, char *);
static int (*f_pcap_activate)(void *);
static void *(*f_pcap_geterr)(void *);
static pcap_t* (*f_pcap_open)(const char *source, int snaplen, int flags, int read_timeout, struct pcap_rmtauth *auth, char *errbuf);
static int (*f_pcap_next_ex)(pcap_t *, struct pcap_pkthdr **, const unsigned char **);
#else
#include <pcap.h>
#endif
#endif

using namespace std;

// Question classes

#include "Question.h"
#include "FreeTextQuestion.h"
#include "NumberQuestion.h"
#include "MultipleChoiceQuestion.h"
#include "ShrinkingChoiceQuestion.h"

/**
 * Add disks for a controller to the configuration file.
 *
 * \param disk_q: A ShrinkingChoiceQuestion that contains
 *                all allowed disk names for this controller,
 *                and a special answer "none" with a value
 *                of "".
 * \param os:     Output stream for the configuration file.
 **/
void add_disks(ShrinkingChoiceQuestion* disk_q, ostream* os, bool floppy = false)
{
	/* Loop until there are no more disks to be added.
	 */
	for (;;)
	{
		/* If disk_q has only one answer left, this is the
		 * "none" answer. This controller can contain no
		 * more disks.
		 */
		if (disk_q->countAnswers() == 1)
			break;

		/* If the answer value is "", the answer "none"
		 * was given. Stop adding disks to this controller.
		 */
		if (disk_q->ask() == "")
			break;

		/* Find out if this should be using:
		 *   - a disk image file
		 *   - a raw device
		 *   - a RAM DISK
		 */
		MultipleChoiceQuestion type_q;
		type_q.setQuestion("How should " + disk_q->getAnswer() + " be emulated?");
		type_q.setDefault("file");
		type_q.setExplanation("Disks can be emulated in several ways.");
		type_q.addAnswer("file", "file", "The disk uses a disk image on the host system's disk.");
		type_q.addAnswer("device", "device", "The disk uses one of the host system's raw disks.");
		type_q.addAnswer("ramdisk", "ramdisk", "The disk stores it's data in RAM. Volatile.");

		*os << "    " << disk_q->getAnswer() << " = " << type_q.ask() << "\n";
		*os << "    {\n";

		MultipleChoiceQuestion cdrom_q;
		if (type_q.getAnswer() == "file" || type_q.getAnswer() == "device")
		{
			/* For a file or device, we need to know what
			 * file or device to use.
			 */
			FreeTextQuestion img_q;
			img_q.setQuestion("What " + type_q.getAnswer() + " should " + disk_q->getAnswer() + " use?");
			img_q.setExplanation("Enter the path to the " + type_q.getAnswer() + " to use for this disk.");
			*os << "      " << type_q.getAnswer() << " = \"" << img_q.ask() << "\";\n";

			/* We need to know whether to emulate this as
			 * a cd-rom, or as a hard-disk.
			 */
			cdrom_q.setQuestion("Should " + disk_q->getAnswer() + " be a disk or a cd-rom device?");
			cdrom_q.setExplanation("Do you want the OS to see this " + type_q.getAnswer() + " as a hard-disk, or as a cd-rom?");
			cdrom_q.addAnswer("disk", "false", "Hard-disk");
			cdrom_q.addAnswer("cd-rom", "true", "CD-ROM drive");
			cdrom_q.setDefault("disk");
			if (floppy)
			{
				cdrom_q.setAnswer("false");
				*os << "      cdrom = " << "false" << ";\n";
			}
			else
				*os << "      cdrom = " << cdrom_q.ask() << ";\n";
		}

		if (type_q.getAnswer() == "file" && cdrom_q.getAnswer() != "true")
		{
			/* For a file, we need to know whether to create
			 * it when it doesn't exist or not.
			 */
			MultipleChoiceQuestion create_q;
			create_q.setQuestion("If the file doesn't exist, do you want us to create it?");
			create_q.setExplanation("The file will be created the first time the emulator runs.");
			create_q.addAnswer("no", "no", "Don't create this file");
			create_q.addAnswer("yes", "yes", "Create this file");
			create_q.setDefault("yes");
			create_q.ask();
			if (create_q.getAnswer() == "yes")
			{
				/* If we should create the file, we need to
				 * know it's size.
				 */
				MultipleChoiceQuestion unit_q;
				unit_q.setQuestion("What unit do you want to use to specify the disk size?");
				unit_q.setExplanation("This is needed to create the file if it doesn't exist.");
				unit_q.addAnswer("KB", "K", "Kilobytes");
				unit_q.addAnswer("MB", "M", "Megabytes");
				unit_q.addAnswer("GB", "G", "Gigabytes");
				unit_q.setDefault("MB");
				unit_q.ask();
				NumberQuestion size_q;
				size_q.setQuestion("How many " + unit_q.getAnswer() + "Bytes should the disk be?");
				size_q.setExplanation("This is needed to create the file if it doesn't exist.");
				size_q.setRange(1, 1024);
				size_q.setDefault("10");
				size_q.ask();
				*os << "      autocreate_size = \"" << size_q.getAnswer() << unit_q.getAnswer() << "\";\n";
			}
		}

		if (type_q.getAnswer() == "ramdisk")
		{
			cdrom_q.setQuestion("Should " + disk_q->getAnswer() + " be a disk or a read-only in-memory cd-rom device?");
			cdrom_q.setExplanation("Do you want the OS to see this " + type_q.getAnswer() + " as a hard-disk, or as a read-only in-memory cd-rom?");
			cdrom_q.addAnswer("disk", "false", "Hard-disk");
			cdrom_q.addAnswer("cd-rom", "true", "CD-ROM drive");
			cdrom_q.setDefault("disk");
			if (floppy)
			{
				cdrom_q.setAnswer("false");
				*os << "      cdrom = " << "false" << ";\n";
			}
			else
				*os << "      cdrom = " << cdrom_q.ask() << ";\n";
			if (cdrom_q.getAnswer() == "cd-rom" || cdrom_q.getAnswer() == "true")
			{
				FreeTextQuestion img_q;
				img_q.setQuestion("What file should " + disk_q->getAnswer() + " use?");
				img_q.setExplanation("Enter the path to the file to use for this disk.");
				*os << "      " << "file = \"" << img_q.ask() << "\";\n";
			}
			else
			{
				/* For a RAM DISK, we need to know what
				 * size it should be.
				 */
				MultipleChoiceQuestion unit_q;
				unit_q.setQuestion("What unit do you want to use to specify the disk size?");
				unit_q.setExplanation("This is needed to create the RAMDISK.");
				unit_q.addAnswer("KB", "K", "Kilobytes");
				unit_q.addAnswer("MB", "M", "Megabytes");
				unit_q.addAnswer("GB", "G", "Gigabytes");
				unit_q.setDefault("MB");
				unit_q.ask();
				NumberQuestion size_q;
				size_q.setQuestion("How many " + unit_q.getAnswer() + "Bytes should the disk be?");
				size_q.setExplanation("This is needed to create the RAMDISK.");
				size_q.setRange(1, 1024);
				size_q.setDefault("10");
				size_q.ask();
				*os << "      size = \"" << size_q.getAnswer() << unit_q.getAnswer() << "\";\n";
			}
		}

		/* We also need to know whether this is a
		 * writeable or a read-only device.
		 */
		MultipleChoiceQuestion ro_q;
		ro_q.setQuestion("Should " + disk_q->getAnswer() + " be a read-only disk?");
		ro_q.setExplanation("You might want to write-protect this disk.");
		ro_q.addAnswer("no", "false", "writeable");
		ro_q.addAnswer("yes", "true", "read-only");
		ro_q.setDefault("no");

		if (cdrom_q.getAnswer() == "true")
		{
			/* CD-ROMs are always read-only.
			 */
			ro_q.setAnswer("true");
		}
		else if (type_q.getAnswer() == "ramdisk")
		{
			/* Read-only RAM DISKs don't make any sense.
			 */
			ro_q.setAnswer("false");
		}
		else
		{
			/* Otherwise, ask.
			 */
			ro_q.ask();
		}
		*os << "      read_only = " << ro_q.getAnswer() << ";\n";

		/* The user can define a custom model
		 * number for the device.
		 */
		FreeTextQuestion ft_q;
		ft_q.setQuestion("Would you like to set a disk model number?");
		ft_q.setExplanation("Leave blank to choose the default.");
		if (ft_q.ask() != "")
			*os << "      model_number = \"" << ft_q.getAnswer() << "\";\n";

		/* The user can define a custom revision
		 * number for the device.
		 */
		ft_q.setQuestion("Would you like to set a revision number?");
		if (ft_q.ask() != "")
			*os << "      rev_number = \"" << ft_q.getAnswer() << "\";\n";

		/* The user can define a custom serial
		 * number for the device.
		 */
		ft_q.setQuestion("Would you like to set a serial number?");
		if (ft_q.ask() != "")
			*os << "      serial_number = \"" << ft_q.getAnswer() << "\";\n";

		*os << "    }\n\n";
	}
}

/**
 * Main program entry point.
 **/
int main(int argc, char* argv[])
{
	/* Banner
	 */
#if defined _WIN32 && defined HAVE_PCAP
	    char npcap_dir[512];
    	GetSystemDirectoryA(npcap_dir, 480);
    	strcat(npcap_dir, "\\Npcap");
    	SetDllDirectoryA(npcap_dir);	
		auto libhandle = LoadLibraryA("wpcap.dll");
    	SetDllDirectoryA(NULL); /* reset the DLL search path */
		if (!libhandle) {
			printf("Failed to load wpcap.dll");
			return -1;
		}
		f_pcap_lib_version = (const char *(*)())GetProcAddress(libhandle, "pcap_lib_version");
		f_pcap_findalldevs = (int (*)(pcap_if_t **, char *))GetProcAddress(libhandle, "pcap_findalldevs");
		f_pcap_freealldevs = (void (*)(void *))GetProcAddress(libhandle, "pcap_freealldevs");
		f_pcap_open_live   = (pcap_t *(*)(const char *, int, int, int, char *))GetProcAddress(libhandle, "pcap_open_live");
		f_pcap_open   = (pcap_t* (*)(const char *, int , int , int , pcap_rmtauth *, char *))GetProcAddress(libhandle, "pcap_open");
		f_pcap_compile = (int (*)(void *, void *, const char *, int, bpf_u_int32))GetProcAddress(libhandle, "pcap_compile");
		f_pcap_setfilter = (int (*)(void *, void *))GetProcAddress(libhandle, "pcap_setfilter");
		f_pcap_next = (const unsigned char *(*)(void *, void *))GetProcAddress(libhandle, "pcap_next");
		f_pcap_sendpacket = (int (*)(void *, const unsigned char *, int))GetProcAddress(libhandle, "pcap_sendpacket");
		f_pcap_close = (void (*)(void *))GetProcAddress(libhandle, "pcap_close");
		f_pcap_setnonblock = (int (*)(void *, int, char *))GetProcAddress(libhandle, "pcap_setnonblock");
		f_pcap_set_immediate_mode = (int (*)(void *, int))GetProcAddress(libhandle, "pcap_set_immediate_mode");
		f_pcap_set_promisc = (int (*)(void *, int))GetProcAddress(libhandle, "pcap_set_promisc");
		f_pcap_set_snaplen = (int (*)(void *, int))GetProcAddress(libhandle, "pcap_set_snaplen");
		f_pcap_dispatch = (int (*)(void *, int, pcap_handler callback, unsigned char *user))GetProcAddress(libhandle, "pcap_dispatch");
		f_pcap_create = (void * (*)(const char *, char *))GetProcAddress(libhandle, "pcap_create");
		f_pcap_geterr = (void *(*)(void *))GetProcAddress(libhandle, "pcap_geterr");
		f_pcap_next_ex = (int (*)(pcap *, pcap_pkthdr **, const unsigned char **))GetProcAddress(libhandle, "pcap_next_ex");

		#define pcap_findalldevs f_pcap_findalldevs
		#define pcap_open f_pcap_open
		#define pcap_sendpacket f_pcap_sendpacket
		#define pcap_close f_pcap_close
		#define pcap_setnonblock f_pcap_setnonblock
		#define pcap_setfilter f_pcap_setfilter
		#define pcap_compile f_pcap_compile
		#define pcap_geterr f_pcap_geterr
		#define pcap_next_ex f_pcap_next_ex

		#define PCAP_ERROR -1
		#define PCAP_OPENFLAG_PROMISCUOUS 0x00000001
		#define PCAP_OPENFLAG_NOCAPTURE_LOCAL 0x00000008
#endif
	printf("\n\n");
	printf("   **======================================================================**\n");
	printf("   ||                             ES40  emulator                           ||\n");
	printf("   ||                              Version " VERSION "                            ||\n");
	printf("   ||                                                                      ||\n");
	printf("   ||  Copyright (C) 2007-2008 by the ES40 Emulator Project                ||\n");
	printf("   ||  Website: http://sourceforge.net/projects/es40                       ||\n");
	printf("   ||  E-mail : camiel@camicom.com                                         ||\n");
	printf("   ||                                                                      ||\n");
	printf("   ||  This program is free software; you can redistribute it and/or       ||\n");
	printf("   ||  modify it under the terms of the GNU General Public License         ||\n");
	printf("   ||  as published by the Free Software Foundation; either version 2      ||\n");
	printf("   ||  of the License, or (at your option) any later version.              ||\n");
	printf("   **======================================================================**\n");
	printf("\n\n");

	/* Explanation
	 */
	printf("We are now going to set up an initial configuration file for the Emulator.\n");
	printf("This file will be saved as es40.cfg in the current directory.\n\n");
	printf("For more detailed information to the current question, answer '?'.\n");
	
	/* Check if es40.cfg already exists.
	 */
	ifstream check_file("es40.cfg");
	if (check_file.is_open())
	{
		check_file.close();
		MultipleChoiceQuestion overwrite_q;
		overwrite_q.setQuestion("The file es40.cfg already exists. Do you want to overwrite it?");
		overwrite_q.setExplanation("If you answer 'no', the program will exit without overwriting the existing file.");
		overwrite_q.addAnswer("no", "no", "Keep the existing file and exit.");
		overwrite_q.addAnswer("yes", "yes", "Overwrite the existing configuration file.");
		overwrite_q.setDefault("no");
		
		if (overwrite_q.ask() == "no")
		{
			printf("Exiting without changing es40.cfg.\n");
			return 0;
		}
	}
	
	/* Open es40.cfg for writing.
	 */
	filebuf fb;
	fb.open("es40.cfg", ios::out);
	ostream os(&fb);

	/* **************************** *
	 * GUI Choice                   *
	 * **************************** */

	MultipleChoiceQuestion gui_q;

	gui_q.setQuestion("Do you want to have a GUI?");
	gui_q.setExplanation("You need a GUI if you want to use an emulated graphics card. You don't need this for most OS'es. If you don't need this, we recomment that you answer 'none' to this question.");
	gui_q.setDefault("none");
	gui_q.addAnswer("none", "", "No GUI. Graphics cards are not supported.");

#if defined(HAVE_SDL)
	gui_q.addAnswer("SDL", "sdl", "Simple Directmedia Layer. Preferred GUI mechanism.");
#endif
//#if defined(HAVE_X11)
//	gui_q.addAnswer("X11", "X11", "Unix X-Windows GUI support.");
//#endif
//#if defined(_WIN32)
//	gui_q.addAnswer("win32", "win32", "Windows 32 GUI support.");
//#endif

	if (gui_q.countAnswers() == 1)
	{
		/* The only valid answer is "none".
		 */
		//cout << "\nSorry, the GUI is not available! (no SDL, win32 or X11 support found).\n";
		cout << "\nSorry, the GUI is not available! (no SDL support found).\n";
		gui_q.setAnswer("");
	}
	else
	{
		/* Ask what GUI to use?
		 */
		gui_q.ask();
	}

	if (gui_q.getAnswer() != "")
	{
		/* Yes, we have a GUI.
		 */
		os << "gui = " << gui_q.getAnswer() << "\n";
		os << "{\n";

		MultipleChoiceQuestion vid_scale_q;
		vid_scale_q.setQuestion("Do you want to set a custom scale ratio for the display output?");
		vid_scale_q.setExplanation("The display output is scaled automatically based on system DPI by default, which can be overrided.");
		vid_scale_q.addAnswer("no", "no", "Set the display scale ratio automatically.");
		vid_scale_q.addAnswer("yes", "yes", "Set a custom display scale ratio.");
		vid_scale_q.setDefault("no");
		vid_scale_q.ask();

		NumberQuestion vid_scale_ratio_q;
		/* If none was answered, we don't need to
		 * ask for arguments.
		 */
		if (vid_scale_q.getAnswer() != "no")
		{
			vid_scale_ratio_q.setQuestion("How many times should the display be scaled?");
			vid_scale_ratio_q.setExplanation("Type an integer to set the scale ratio for the display output.");
			vid_scale_ratio_q.setRange(1, 10);
			vid_scale_ratio_q.setDefault("1");
			
			os << "  video.scale_ratio = " << vid_scale_ratio_q.ask() << ";\n";
		}

		MultipleChoiceQuestion vid_linear_q;
		vid_linear_q.setQuestion("Should the display output be nearest or bilinear?");
		vid_linear_q.setExplanation("This affects the resized display output. Nearest looks pixel-y but harsh, while linear does not look as harsh.");
		vid_linear_q.addAnswer("nearest", "false", "Nearest display output.");
		vid_linear_q.addAnswer("bilinear", "true", "Bilinear display output");
		vid_linear_q.setDefault("bilinear");

		os << "  video.linear = " << vid_linear_q.ask() << ";\n";

		MultipleChoiceQuestion vid_scale_change_enable_q;
		vid_scale_change_enable_q.setQuestion("Enable runtime display scale changes via hotkeys?");
		vid_scale_change_enable_q.setExplanation(
			"If enabled, the display scale ratio can be adjusted on the fly "
			"while the emulator is running, without restarting. The change is "
			"not persisted back to this config file.\n"
			"Currently the assigned keys are fixed:\n"
			"  Ctrl+PageUp   - increase scale by 1 (clamped at 8x)\n"
			"  Ctrl+PageDown - decrease scale by 1 (clamped at 1x)\n"
			"These key assignments may become user-configurable in the future.");
		vid_scale_change_enable_q.addAnswer("no", "false", "Disable runtime scale change hotkeys.");
		vid_scale_change_enable_q.addAnswer("yes", "true", "Enable Ctrl+PageUp / Ctrl+PageDown to adjust scaling at runtime.");
		vid_scale_change_enable_q.setDefault("no");

		os << "  video.scale_change_enable = " << vid_scale_change_enable_q.ask() << ";\n";

		os << "}\n\n";
	}

	/* **************************** *
	 * Memory Size                  *
	 * **************************** */

	MultipleChoiceQuestion mem_q;

	mem_q.setQuestion("How much RAM memory do you want to emulate?");
	mem_q.setExplanation("Your system should have enough free memory to emulate the amount you choose here.");
	mem_q.setDefault("256M");

	/* Add memory sizes from 32 MB to 8 GB
	 * (25 to 34 bits).
	 */
	for (int i = 25; i < 34; i++)
	{
		string a;
		int j = i;
		if (i < 30)
		{
			/* Megabyte-range.
			 */
			j -= 20;
			a = "M";
		}
		else
		{
			/* Gigabyte range.
			 */
			j -= 30;
			a = "G";
		}
		mem_q.addAnswer(i2s(1 << j) + a, i2s(i), i2s(1 << j) + a + "Bytes of memory.");
	}

	os << "sys0 = tsunami\n";
	os << "{\n";
	os << "  memory.bits = " << mem_q.ask() << ";\n";

	/* **************************** *
	 * ROM Files                    *
	 * **************************** */

	FreeTextQuestion rom_q;
	rom_q.setQuestion("Where can the SRM ROM image be found?");
	rom_q.setExplanation("This file is required.");
#if defined(_WIN32)
	rom_q.setDefault("rom\\cl67srmrom.exe");
#elif defined(__VMS)
	rom_q.setDefault("[.ROM]CL67SRMROM.EXE");
#else
	rom_q.setDefault("rom/cl67srmrom.exe");
#endif

	os << "  rom.srm = \"" << rom_q.ask() << "\";\n";

	rom_q.setQuestion("Where should the Flash ROM image be saved?");
#if defined(_WIN32)
	rom_q.setDefault("rom\\flash.rom");
#elif defined(__VMS)
	rom_q.setDefault("[.ROM]FLASH.ROM");
#else
	rom_q.setDefault("rom/flash.rom");
#endif

	os << "  rom.flash = \"" << rom_q.ask() << "\";\n";

	rom_q.setQuestion("Where should the DPR EEPROM image be saved?");
#if defined(_WIN32)
	rom_q.setDefault("rom\\dpr.rom");
#elif defined(__VMS)
	rom_q.setDefault("[.ROM]DPR.ROM");
#else
	rom_q.setDefault("rom/dpr.rom");
#endif

	os << "  rom.dpr = \"" << rom_q.ask() << "\";\n\n";

	/* **************************** *
	 * CPU's                        *
	 * **************************** */

	NumberQuestion cpu_q;

	cpu_q.setQuestion("How many CPU's do you want in the system (Currently, only one is supported)?");
	cpu_q.setRange(1, 1);
	cpu_q.setDefault("1");
	cpu_q.setExplanation("The normal value for the number of CPU's is 1. More CPU's are very experimental, and currently doesn't work.");

	cpu_q.ask();

	//MultipleChoiceQuestion icache_q;

	//icache_q.setQuestion("Do you want the ICACHE on the CPU's enabled?");
	//icache_q.setExplanation("The ICACHE makes the CPU emulation more accurate, but also slows down the emulator. Decent operating systems shouldn't depend on this.");
	//icache_q.setDefault("yes");
	//icache_q.addAnswer("yes", "true", "ICACHE enabled. Performance hit, but may be necessary for some software.");
	//icache_q.addAnswer("no", "false", "ICACHE disabled. Better performance, but may not always work.");

	//icache_q.ask();

	NumberQuestion mhz_q;

	mhz_q.setQuestion("What should the reported to guest platform speed of the CPU's be in MHz?");
	mhz_q.setExplanation("This only changes the CPU speed reported to the OS; not the speed of the emulation.");
	mhz_q.setRange(10, 1250);
	mhz_q.setDefault("500");

	mhz_q.ask();

	for (int i = 0; i < cpu_q.getNum(); i++)
	{
		/* Repeat the CPU configuration for each
		 * CPU. Differing CPU specs are not supported.
		 */
		os << "  cpu" << i << " = ev68cb\n";
		os << "  {\n";
		os << "    speed = " << mhz_q.getAnswer() << "M;\n";
		//os << "    icache = " << icache_q.getAnswer() << ";\n";
		os << "    icache = true;\n";
		os << "  }\n\n";
	}

	/* **************************** *
	 * Serial Ports                 *
	 * **************************** */

	 /* There are two serial ports (0 and 1).
	  */
	for (int i = 0; i < 2; i++)
	{
		/* Port number — accepts a numeric port or "none" for a bit-bucket
		 * (null_attach) UART.
		 */
		FreeTextQuestion port_q;
		port_q.setQuestion("What telnet port should serial " + i2s(i) + " listen?");
		port_q.setOptions("1-65535 or 'none'");
		/* The default ports are 21264 and 21265. */
		port_q.setDefault(i2s(21264 + i));
		port_q.setExplanation(
			"You will telnet to this port to establish a connection with emulated serial port "
			+ i2s(i) + ".\n"
			"Answer 'none' to make this port a bit-bucket: the UART still exists on the bus and "
			"presents itself to the guest as a healthy idle 16550 (THRE/TSRE, CTS/DSR), but no "
			"telnet listener is opened and any bytes the guest transmits are silently discarded. "
			"Useful since two are required by the platform firmwares in case you don't need them.");

		bool is_null = false;
		for (;;)
		{
			port_q.ask();
			if (port_q.getAnswer() == "none")
			{
				is_null = true;
				break;
			}
			try
			{
				int v = s2i(port_q.getAnswer());
				if (v >= 1 && v <= 65535)
					break;
				cout << "\nPlease enter a port in 1..65535, or 'none'.\n\n";
			}
			catch (CLogicException)
			{
				cout << "\nPlease enter an integer port number, or 'none'.\n\n";
			}
		}

		if (is_null)
		{
			/* Bit-bucket port — skip the action/args prompts entirely. */
			os << "  serial" << i << " = serial\n";
			os << "  {\n";
			os << "    null_attach = true;\n";
			os << "  }\n\n";
			continue;
		}

		FreeTextQuestion exec_q;
		exec_q.setQuestion("What program should be started automatically for serial " + i2s(i) + "?");
#if defined(_WIN32)
		/* On windows, default to
		 * C:\Program Files\Putty\Putty.exe
		 */
		exec_q.setDefault("C:\\Program Files\\Putty\\Putty.exe");
#else
		/* On other OS'es, default to
		 * putty
		 */
		exec_q.setDefault("putty");
#endif
		exec_q.setExplanation("Enter the path to a program to start this to create an automatic connection with the serial port. Set to 'none' to establish the connection manually.");

		exec_q.ask();
		FreeTextQuestion arg_q;

		/* If none was answered, we don't need to
		 * ask for arguments.
		 */
		if (exec_q.getAnswer() != "none")
		{
			arg_q.setQuestion("What arguments should the program use to connect to the serial port?");
			arg_q.setExplanation("Enter the arguments the program needs.");
			/* This is the argument format for PuTTy.
			 */
			arg_q.setDefault("telnet://localhost:" + port_q.getAnswer());

			arg_q.ask();
		}

		os << "  serial" << i << " = serial\n";
		os << "  {\n";
		os << "    port = " << port_q.getAnswer() << ";\n";
		if (exec_q.getAnswer() != "none")
		{
#if defined(_WIN32)
			/* Quote the program path/name in "",
			 * as it may contain spaces.
			 */

			string exec = exec_q.getAnswer();
			if (exec.size() >= 2 && exec.front() == '"' && exec.back() == '"')
				exec = exec.substr(1, exec.size() - 2);

			os << "    action = \"\"\"" << exec << "\"\" " << arg_q.getAnswer() << "\";\n";
#else
			os << "    action = \"" << exec_q.getAnswer() << " " << arg_q.getAnswer() << "\";\n";
#endif

		}
		os << "  }\n\n";
	}

	/* **************************** *
	 * Floppy Disks                *
	 * **************************** */

	/* The floppy controller is always present. The user only chooses
	 * whether to attach any drives to it.
	 */
	ShrinkingChoiceQuestion fd_q;
	fd_q.setQuestion("Do you want to add any disks to the Floppy controller?");
	fd_q.setDefault("none");
	fd_q.setExplanation("Here, you can add floppy drives to your system.");
	fd_q.addAnswer("none", "", "stop adding disks");
	fd_q.addAnswer("0", "disk0.0", "A:");
	fd_q.addAnswer("1", "disk0.1", "B:");

	os << "  fdc0 = floppy\n";
	os << "  {\n";
	/* Ask what disks to add. */
	add_disks(&fd_q, &os, true);
	os << "  }\n\n";

	/* **************************** *
	 * ALi IDE Disks                *
	 * **************************** */

	 /* Use a ShrinkingChoiceQuestion; once
	  * a disk position has been used, it
	  * can't be used again.
	  */
	ShrinkingChoiceQuestion ide_q;
	ide_q.setQuestion("Do you want to add any disks to the IDE controller?");
	ide_q.setDefault("none");
	ide_q.setExplanation("The IDE controller is mandatory. You can skip this, and set up a SCSI controller, too.");
	ide_q.addAnswer("none", "", "stop adding disks");
	ide_q.addAnswer("0.0", "disk0.0", "primary master");
	ide_q.addAnswer("0.1", "disk0.1", "primary slave");
	ide_q.addAnswer("1.0", "disk1.0", "secondary master");
	ide_q.addAnswer("1.1", "disk1.1", "secondary slave");

	os << "  pci0.15 = ali_ide\n";
	os << "  {\n";
	/* Ask what disks to add.
	 */
	add_disks(&ide_q, &os);
	os << "  }\n\n";

	/* **************************** *
	 * VGA Card                     *
	 * **************************** */

	 /* Use a ShrinkingChoiceQuestion. Once a
	  * card is using a specific PCI slot, it
	  * can't be used by another card.
	  */
	ShrinkingChoiceQuestion pci_q;

	/* Only add the PCI bus 0 slots, as the
	 * VGA card is only supported on hose 0.
	 */
	for (int i = 1; i < 5; i++)
	{
		pci_q.addAnswer("0." + i2s(i), "pci0." + i2s(i), "Bus 0, Slot " + i2s(i));
	}
	pci_q.setExplanation("Only free PCI slots are listed.");

	MultipleChoiceQuestion vga_q;

	if (gui_q.getAnswer() != "")
	{
		vga_q.setQuestion("What (if any) VGA card do you wish to add to the system?");
		vga_q.setExplanation("Functionality of the different cards is pretty much the same; some OS'es seem to have a preference, though.");
		vga_q.setDefault("S3");
		vga_q.addAnswer("none", "", "No graphics card");
		//vga_q.addAnswer("Cirrus", "cirrus", "Cirrus CL-GD something");
		vga_q.addAnswer("S3", "s3", "S3 Trio 64");
#if defined(HAVE_RADEON)
		/* Radeon support is optional, and currently
		 * unreleased, because the specs are only
		 * available under an NDA with AMD. Once AMD
		 * has publicly released the Radeon 7500 (RV200)
		 * specs, the emulated Radeon card will be
		 * released.
		 */
		vga_q.addAnswer("Radeon", "radeon", "Radeon 7500");
#endif

		vga_q.ask();
	}

	if (vga_q.getAnswer() != "")
	{
		pci_q.setQuestion("What PCI slot should the " + vga_q.getAnswer() + " card be on?");
		pci_q.setDefault("0.1");

		rom_q.setQuestion("Where can the VGA BIOS ROM image be found?");
		rom_q.setExplanation("This file is required.");
#if defined(_WIN32)
		rom_q.setDefault("rom\\CHANGE_ME_TO_CORRECT_VGA_BIOS_FOR_SELECTED_CARD_NOT_VGABIOS-0.6a.bin");
#elif defined(__VMS)
		rom_q.setDefault("[.ROM]CHANGE_ME_TO_CORRECT_VGA_BIOS_FOR_SELECTED_CARD_NOT_VGABIOS-0.6a.bin");
#else
		rom_q.setDefault("rom/CHANGE_ME_TO_CORRECT_VGA_BIOS_FOR_SELECTED_CARD_NOT_VGABIOS-0.6a.bin");
#endif

		os << "  " << pci_q.ask() << " = " << vga_q.getAnswer() << "\n";
		os << "  {\n";
		os << "    rom = \"" << rom_q.ask() << "\";\n";
		os << "  }\n\n";
	}

	/* **************************** *
	 * Free Form PCI Cards          *
	 * **************************** */

	 /* Add the PCI bus 1 slots. All non-VGA
	  * PCI cards can be on either hose 0 or
	  * hose 1.
	  */
	for (int i = 1; i < 7; i++)
	{
		pci_q.addAnswer("1." + i2s(i), "pci1." + i2s(i), "Bus 1, Slot " + i2s(i));
	}

	MultipleChoiceQuestion card_q;

	card_q.setQuestion("Would you like to add another PCI card to the system?");
	card_q.setDefault("none");
	card_q.setExplanation("Choose what PCI card you'd like to add. Choose none if you have no more cards to add.");
	card_q.addAnswer("none", "", "No more cards to add");
#if defined(HAVE_PCAP)
	card_q.addAnswer("nic", "dec21143", "DEC 21143 Network Interface (1 max)");
#endif
	card_q.addAnswer("scsi", "sym53c810", "Symbios 53C810 narrow SCSI controller");
	card_q.addAnswer("wide scsi", "sym53c895", "Symbios 53C895 wide SCSI controller (doesn't work with OpenVMS)");
	card_q.addAnswer("es1370 audio", "es1370", "ES1370 Audio card (works only with Windows NT 4.0)");

	/* Loop until there are no more PCI
	 * cards to add.
	 */
	for (;;)
	{
		/* If there are no more free PCI slots,
		 * stop adding PCI cards.
		 */
		if (pci_q.countAnswers() == 0)
			break;

		/* Default to the first available free
		 * PCI slot.
		 */
		pci_q.setDefault(pci_q.getFirstChoice());

		/* If this answer has been answered with
		 * "none", stop adding PCI cards.
		 */
		if (card_q.ask() == "")
			break;

		/* Determine where to put this card.
		 */
		pci_q.setQuestion("In what PCI slot would you like to put the " + card_q.getAnswer() + " card?");
		os << "  " << pci_q.ask() << " = " << card_q.getAnswer() << "\n";
		os << "  {\n";

		if (card_q.getAnswer() == "dec21143")
		{
			/* Due to limitations in our network
			 * emulation, only one NIC is allowed.
			 * Remove it from the list of choices.
			 */
			card_q.dropChoice("nic");

#if defined(HAVE_PCAP)
			MultipleChoiceQuestion if_q;
			if_q.setQuestion("What host network interface should we connect to (answer ? for a list)?");
			if_q.setExplanation("Choose 'list' to get a list at run-time.");
			if_q.addAnswer("list", "", "Get a list at run-time");

			/* Get a list of network interfaces and
			 * add them to the list.
			 */
			pcap_if_t* alldevs;
			pcap_if_t* d;
			char        errbuf[PCAP_ERRBUF_SIZE];

			if (pcap_findalldevs(&alldevs, errbuf) == -1)
			{
				/* No devices to add.
				 */
				printf("Error in pcap_findalldevs_ex: %s", errbuf);
			}
			else
			{
				int i = 1;
				for (d = alldevs; d; d = d->next)
				{
					const char* description = d->description ? d->description : "No description available";
					if_q.addAnswer(i2s(i), d->name, string(d->name) + " (" + description + ")");
					i++;
				}
			}

			if (if_q.ask() != "")
				os << "    adapter = \"" << if_q.getAnswer() << "\";\n";

			FreeTextQuestion mac_q;
			mac_q.setQuestion("What should the NIC's MAC address be?");
			mac_q.setExplanation("This should be unique on your network.");
			mac_q.setDefault("08-00-2B-E5-40-00");
			os << "    mac = \"" << mac_q.ask() << "\";\n";
#endif
		}
		else if (card_q.getAnswer() == "sym53c810")
		{
			/* Use a ShrinkingChoiceQuestion; once
			 * a disk position has been used, it
			 * can't be used again.
			 */
			ShrinkingChoiceQuestion disk_q;
			disk_q.setQuestion("Do you want to add any disks to the Sym53C810 controller?");
			disk_q.setDefault("none");
			disk_q.setExplanation("Add disks. Select 'none' if you have no more disks to add.");
			disk_q.addAnswer("none", "", "stop adding disks");
			/* The narrow SCSI controller supports
			 * devices at targets 0..6.
			 */
			for (int i = 0; i < 7; i++)
			{
				disk_q.addAnswer(i2s(i), "disk0." + i2s(i), "Target " + i2s(i));
			}
			/* Ask what disks to add.
			 */
			add_disks(&disk_q, &os);
		}
		else if (card_q.getAnswer() == "sym53c895")
		{
			/* Use a ShrinkingChoiceQuestion; once
			 * a disk position has been used, it
			 * can't be used again.
			 */
			ShrinkingChoiceQuestion disk_q;
			disk_q.setQuestion("Do you want to add any disks to the Sym53C895 controller?");
			disk_q.setDefault("none");
			disk_q.setExplanation("Add disks. Select 'none' if you have no more disks to add.");
			disk_q.addAnswer("none", "", "stop adding disks");
			/* The wide SCSI controller supports
			 * devices at targets 0..6 and 8..15.
			 */
			for (int i = 0; i < 16; i++)
			{
				if (i != 7)
					disk_q.addAnswer(i2s(i), "disk0." + i2s(i), "Target " + i2s(i));
			}
			/* Ask what disks to add.
			 */
			add_disks(&disk_q, &os);
		}
		os << "  }\n\n";
	}

#ifdef _WIN32
	MultipleChoiceQuestion mpu_q;
	mpu_q.setQuestion("Would you like to emulate the MPU-401 MIDI device?");
	mpu_q.addAnswer("no", "", "Disable the MPU-401");
	mpu_q.addAnswer("yes", "yes", "Enable the MPU-401");
	mpu_q.setDefault("no");
	mpu_q.ask();

	if (mpu_q.getAnswer() != "")
	{
		MultipleChoiceQuestion midi_q;
		midi_q.setQuestion("What MIDI out device should we connect to (answer ? for a list)?");
		midi_q.setExplanation("Choose 'list' to get a list at run-time.");
		midi_q.addAnswer("list", "", "Get a list at run-time");
		
		MIDIOUTCAPSA caps;
		for (UINT i = 0; i < midiOutGetNumDevs(); i++)
		{
			midiOutGetDevCapsA(i, &caps, sizeof(caps));
			midi_q.addAnswer(i2s(i + 1), i2s(i), string(caps.szPname));
		}

		os << "  mpu = mpu401\n";
		os << "  {\n";
		os << "     midi_out = " << midi_q.ask() << ";\n";
		os << "  }\n\n";
	}

#endif

	MultipleChoiceQuestion mouse_q;
	mouse_q.setQuestion("Would you like to emulate the mouse?");
	mouse_q.setExplanation("The mouse is not really working yet... :-(");
	mouse_q.addAnswer("no", "false", "Disable the mouse");
	mouse_q.addAnswer("yes", "true", "Enable the mouse");
	mouse_q.setDefault("yes");

	MultipleChoiceQuestion vgacons_q;
	vgacons_q.setQuestion("Where would you like console output to go?");
	vgacons_q.setExplanation("This is the SRM console option");
	vgacons_q.addAnswer("serial", "false", "Console on serial port 0");
	vgacons_q.addAnswer("graphics", "true", "Console on graphics controller");
	vgacons_q.setDefault("graphics");

	if (vga_q.getAnswer() != "")
	{
		/* If a VGA card is present, ask about
		 * the mouse and the console.
		 */
		mouse_q.ask();
		vgacons_q.ask();
	}
	else
	{
		/* No VGA card present, mouse support
		 * is disabled, and the console goes
		 * to serial port 0.
		 */
		mouse_q.setAnswer("false");
		vgacons_q.setAnswer("false");
	}

	FreeTextQuestion lpt_q;
	lpt_q.setQuestion("Where would you like printer output to go?");
	lpt_q.setExplanation("Output from the printer port will be saved to this file. Leave blank if not wanted.");
	lpt_q.ask();


	MultipleChoiceQuestion pmu_q;
	pmu_q.setQuestion("Enable the M7101 power-management / ACPI device at PCI 0:17?");
	pmu_q.setExplanation("AliM1543C chipset power management unit");
	pmu_q.addAnswer("yes", "true", "Enable the PMU");
	pmu_q.addAnswer("no", "false", "Disable the PMU");
	pmu_q.setDefault("yes");
	pmu_q.ask();

	MultipleChoiceQuestion usb_q;
	usb_q.setQuestion("Enable the USB OHCI controller at PCI 0:19?");
	usb_q.setExplanation("AliM1543C chipset USB controller");
	usb_q.addAnswer("yes", "true", "Enable the USB controller");
	usb_q.addAnswer("no", "false", "Disable the USB controller");
	usb_q.setDefault("yes");
	usb_q.ask();

	MultipleChoiceQuestion arc_compat_q;
	arc_compat_q.setQuestion("Which OS the reported year should be compatible with?");
	arc_compat_q.setExplanation("This only affects the year reported to the guest. Use \"nt\" if planning to run Windows OSes.");
	arc_compat_q.addAnswer("nt", "true", "Report the year as an offset from 1980, as expected by Windows OSes and AlphaBIOS.");
	arc_compat_q.addAnswer("vms", "false", "Report the year as expected by OpenVMS, Tru64 UNIX, Linux and BSDs.");
	arc_compat_q.setDefault("vms");
	arc_compat_q.ask();
	os << "\n  arc_year_compat = " << arc_compat_q.ask() << ";\n\n";

	MultipleChoiceQuestion time_q;
	time_q.setQuestion("Do you want to set a fixed date and time when the VM starts?");
	time_q.setExplanation("By default, the VM's date and time is initialized to the current host date and time at startup. If you want to set a fixed date and time instead, you can do so here.");
	time_q.addAnswer("yes", "true", "Set a fixed date and time");
	time_q.addAnswer("no", "false", "Initialize the VM's date and time to the current host date and time at startup");
	time_q.setDefault("no");
	time_q.ask();

	if (time_q.getAnswer() == "yes")
	{
		for (;;)
		{
			FreeTextQuestion datetime_q;
			datetime_q.setQuestion("What date and time should the VM have at startup? (format: YYYY-MM-DD HH:MM:SS)");
			datetime_q.setExplanation("Enter the date and time in the format shown above. For example, '2000-01-01 12:00:00' for January 1st, 2000 at noon.");
			datetime_q.ask();
			struct tm ft = {};
			int n = sscanf(datetime_q.getAnswer().c_str(), "%d-%d-%d %d:%d:%d",
				&ft.tm_year, &ft.tm_mon, &ft.tm_mday,
				&ft.tm_hour, &ft.tm_min, &ft.tm_sec);
			if (n < 3) {
				cout << "\nInvalid date/time format.\n";
				continue;
			}
			std::string datestr = i2s(ft.tm_year) + "-" + i2s(ft.tm_mon) + "-" + i2s(ft.tm_mday);
			if (n >= 4) datestr += " " + i2s(ft.tm_hour);
			if (n >= 5) datestr += ":" + i2s(ft.tm_min);
			if (n >= 6) datestr += ":" + i2s(ft.tm_sec);
			os << "  time = \"" << datestr << "\";\n\n";
			break;
		}
	}


	os << "  pci0.7 = ali\n";
	os << "  {\n";
	os << "    mouse.enabled = " << mouse_q.getAnswer() << ";\n";
	os << "    vga_console = " << vgacons_q.getAnswer() << ";\n";
	if (lpt_q.getAnswer() != "")
		os << "    lpt.outfile = \"" << lpt_q.getAnswer() << "\"\n";
	os << "  }\n\n";

	if (usb_q.getAnswer() == "true")
	{
		os << "  pci0.19 = ali_usb\n";
		os << "  {\n";
		os << "  }\n";
	}

	if (pmu_q.getAnswer() == "true")
	{
		os << "  pci0.17 = ali_pmu\n";
		os << "  {\n";
		os << "  }\n";
	}

	os << "}\n";

	/* Close es40.cfg.
	 */
	fb.close();

	/* All is well.
	 */
	return 0;
}

