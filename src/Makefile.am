################################################################################
# ES40 emulator.
# Copyright (C) 2007-2008 by the ES40 Emulator Project
#
# Website: http://www.es40.org
# E-mail : camiel@es40.org
# 
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 
# 02110-1301, USA.
# 
# Although this is not required, the author would appreciate being notified of, 
# and receiving any modifications you may make to the source code that might serve
# the general public.
#
################################################################################
#
# $Id$
#
# X-1.6      Caolan McNamara                          12-JUN-2008
#      Fixes to build on Fedora 9 / gcc 4.3.0.
#
# X-1.5      Camiel Vanderhoeven                      31-MAY-2008
#      Add parts of Poco.
#
# X-1.4      Camiel Vanderhoeven                      04-APR-2008
#      Added comments.
#
# X-1.3      Camiel Vanderhoeven                      29-MAR-2008
#      Added es40_cfg.
#
# X-1.1	     Camiel Vanderhoeven                      20-MAR-2008
#      File Created.
#
################################################################################

bin_PROGRAMS = es40 es40_idb es40_lss es40_lsm es40_cfg

es40_SOURCES = AliM1543C.cpp \
       AliM1543C_ide.cpp \
       AliM1543C_usb.cpp \
       AlphaCPU.cpp \
       AlphaCPU_ieeefloat.cpp \
       AlphaCPU_vaxfloat.cpp \
       AlphaCPU_vmspal.cpp \
       AlphaSim.cpp \
#       Cirrus.cpp \
       Configurator.cpp \
       DEC21143.cpp \
       Disk.cpp \
       DiskController.cpp \
       DiskDevice.cpp \
       DiskFile.cpp \
       DiskRam.cpp \
       ibm8514a.cpp \
       DMA.cpp \
       DPR.cpp \
       es40_debug.cpp \
       Ethernet.cpp \
       Flash.cpp \
       FloppyController.cpp \
       Keyboard.cpp \
       lockstep.cpp \
       PCIDevice.cpp \
       Port80.cpp \
       S3Trio64.cpp \
       SCSIBus.cpp \
       SCSIDevice.cpp \
       Serial.cpp \
       StdAfx.cpp \
       Sym53C810.cpp \
       Sym53C895.cpp \
       SystemComponent.cpp \
       System.cpp \
       TraceEngine.cpp \
       VGA.cpp \
       i2c_spd.cpp \
       gui/gui.cpp \
       gui/gui_x11.cpp \
       gui/keymap.cpp \
       gui/scancodes.cpp \
       gui/sdl.cpp \
        base/Bugcheck.cpp \
	base/ErrorHandler.cpp \
	base/Event.cpp \
	base/Exception.cpp \
	base/Mutex.cpp \
	base/NumberFormatter.cpp \
	base/RefCountedObject.cpp \
	base/Runnable.cpp \
	base/RWLock.cpp \
	base/Semaphore.cpp \
	base/SignalHandler.cpp \
	base/ThreadLocal.cpp \
	base/Thread.cpp \
	base/Timestamp.cpp

es40_idb_SOURCES = $(es40_SOURCES)
es40_lss_SOURCES = $(es40_SOURCES)
es40_lsm_SOURCES = $(es40_SOURCES)

es40_idb_CXXFLAGS = -DIDB
es40_lss_CXXFLAGS = -DIDB -DLSS
es40_lsm_CXXFLAGS = -DIDB -DLSM

es40_cfg_SOURCES = es40-cfg.cpp \
    base/Exception.cpp

