$ SET VERIFY
$!
$! ES40 emulator.
$! Copyright (C) 2007 by Camiel Vanderhoeven
$!
$! Website: www.camicom.com
$! E-mail : camiel@camicom.com
$! 
$! This program is free software; you can redistribute it and/or
$! modify it under the terms of the GNU General Public License
$! as published by the Free Software Foundation; either version 2
$! of the License, or (at your option) any later version.
$! 
$! This program is distributed in the hope that it will be useful,
$! but WITHOUT ANY WARRANTY; without even the implied warranty of
$! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
$! GNU General Public License for more details.
$! 
$! You should have received a copy of the GNU General Public License
$! along with this program; if not, write to the Free Software
$! Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
$! 
$! Although this is not required, the author would appreciate being notified of, 
$! and receiving any modifications you may make to the source code that might serve
$! the general public.
$!
$! MAKEFILE FOR OPENVMS
$!
$!==========================================================================================
$! START
$!
$! Edit the folloing line to point to your UNIX porting library include files:
$!
$ ASSIGN "DRA0:[PORTING.DIST.INCLUDE]" PORT_INC
$!
$! Errors (file not found) can be ignored for this delete action:
$!
$ DEL *.OBJ;*
$!
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) ALIM1543C.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) ALPHACPU.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) ALPHASIM.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) DPR.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) FLASH.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) FLOPPYCONTROLLER.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) PORT80.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) SERIAL.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) STDAFX.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) SYSTEMCOMPONENT.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) SYSTEM.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) TRACEENGINE.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) TRANSLATIONBUFFER.CPP
$!
$ CXXLINK ALIM1543C.OBJ,ALPHACPU.OBJ,ALPHASIM.OBJ,DPR.OBJ,FLASH.OBJ,FLOPPYCONTROLLER.OBJ,PORT80.OBJ,SERIAL.OBJ,STDAFX.OBJ,SYSTEMCOMPONENT.OBJ,SYSTEM.OBJ,TRACEENGINE.OBJ,TRANSLATIONBUFFER.OBJ /EXECUTABLE=ES40.EXE
$!
$ DEL *.OBJ;*
$!
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB ALIM1543C.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB ALPHACPU.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB ALPHASIM.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB DPR.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB FLASH.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB FLOPPYCONTROLLER.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB PORT80.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB SERIAL.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB STDAFX.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB SYSTEMCOMPONENT.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB SYSTEM.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB TRACEENGINE.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=IDB TRANSLATIONBUFFER.CPP
$!
$ CXXLINK ALIM1543C.OBJ,ALPHACPU.OBJ,ALPHASIM.OBJ,DPR.OBJ,FLASH.OBJ,FLOPPYCONTROLLER.OBJ,PORT80.OBJ,SERIAL.OBJ,STDAFX.OBJ,SYSTEMCOMPONENT.OBJ,SYSTEM.OBJ,TRACEENGINE.OBJ,TRANSLATIONBUFFER.OBJ /EXECUTABLE=ES40_IDB.EXE
$!
$ DEL *.OBJ;*
$!
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) ALIM1543C.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) ALPHACPU.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) ALPHASIM.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) DPR.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) FLASH.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) FLOPPYCONTROLLER.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) PORT80.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) SERIAL.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) STDAFX.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) SYSTEMCOMPONENT.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) SYSTEM.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) TRACEENGINE.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) TRANSLATIONBUFFER.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_SLAVE) LOCKSTEP.CPP
$!
$ CXXLINK ALIM1543C.OBJ,ALPHACPU.OBJ,ALPHASIM.OBJ,DPR.OBJ,FLASH.OBJ,FLOPPYCONTROLLER.OBJ,PORT80.OBJ,SERIAL.OBJ,STDAFX.OBJ,SYSTEMCOMPONENT.OBJ,SYSTEM.OBJ,TRACEENGINE.OBJ,TRANSLATIONBUFFER.OBJ,LOCKSTEP.OBJ /EXECUTABLE=ES40_LSS.EXE
$!
$ DEL *.OBJ;*
$!
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) ALIM1543C.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) ALPHACPU.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) ALPHASIM.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) DPR.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) FLASH.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) FLOPPYCONTROLLER.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) PORT80.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) SERIAL.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) STDAFX.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) SYSTEMCOMPONENT.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) SYSTEM.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) TRACEENGINE.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) TRANSLATIONBUFFER.CPP
$ CXX /INCLUDE_DIRECTORY=(PORT_INC,[]) /OPTIMIZE=(LEVEL=4,INLINE=SPEED) /DEFINE=(IDB,LS_MASTER) LOCKSTEP.CPP
$!
$ CXXLINK ALIM1543C.OBJ,ALPHACPU.OBJ,ALPHASIM.OBJ,DPR.OBJ,FLASH.OBJ,FLOPPYCONTROLLER.OBJ,PORT80.OBJ,SERIAL.OBJ,STDAFX.OBJ,SYSTEMCOMPONENT.OBJ,SYSTEM.OBJ,TRACEENGINE.OBJ,TRANSLATIONBUFFER.OBJ,LOCKSTEP.OBJ /EXECUTABLE=ES40_LSM.EXE
$!