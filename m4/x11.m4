################################################################################
# ES40 emulator.
# Copyright (C) 2007-2008 by the ES40 Emulator Project
#
# Website: http://sourceforge.net/projects/es40
# E-mail : camiel@camicom.com
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
# X-1.1	     Camiel Vanderhoeven                      20-MAR-2008
#      File Created.
#
################################################################################
#
# Check if X11 is installed, and configure paths

dnl AM_PATH_X11([ACTION-IF-FOUND [, ACTION-IF-NOT-FOUND]])
dnl test for X11, and define X11_CFLAGS and X11_LIBS
dnl

AC_DEFUN([AM_PATH_X11], 
[dnl
dnl Get the cflags and libraries for X11
dnl

  AC_PATH_X([X11],[X11/Xlib.h],[XOpenDisplay(NULL)])

  not_really_there=""
  if test "X$no_x" = "X"; then
    if test "X$x_includes" = "X"; then
      AC_TRY_CPP([#include <X11/XIntrinsic.h>], , not_really_there="yes")
    else
      if test ! -r $x_includes/X11/Intrinsic.h; then
        not_really_there="yes"
      fi
    fi
  fi

  if test "$no_x" = "yes" -o "$not_really_there" = "yes"; then
    AC_MSG_CHECKING(for X11 header files)
    X11_CFLAGS=""
    AC_TRY_CPP([#include <X11/Intrinsic.h>], , X11_CFLAGS="nope")
    if test "$X11_CFLAGS" = "nope"; then
      dirs="/usr/unsupported/include /usr/local/include /usr/X386/include /usr/include/X11R4 /usr/X11R5/include /usr/include/X11R5 /usr/openwin/include /usr/X11/include /usr/sww/include /usr/X11R6/include /usr/include/X11R6"
      for i in $dirs ; do
        if test -r $i/X11/Intrinsic.h; then
          AC_MSG_RESULT($i)
          X11_CFLAGS=" -I$i"
          break
 	    fi
      done
    fi
  else
    if test "$x_includes" != ""; then
      X11_CFLAGS=-I$x_includes
    else
      X11_CFLAGS=""
    fi
  fi
  if test "$X11_CFLAGS" = "nope"; then
    AC_MSG_RESULT(couldn't find any!)
  fi

  if test "$no_x" = "yes"; then
    AC_MSG_CHECKING(for X11 libraries)
    X11_LIBS="nope"
    dirs="/usr/unsupported/lib /usr/local/lib /usr/X386/lib /usr/lib/X11R4 /usr/X11R5/lib /usr/lib/X11R5 /usr/X11R6/lib /usr/lib/X11R6 /usr/openwin/lib /usr/X11/lib /usr/sww/X11/lib"
    for i in $dirs ; do
      if test -r $i/libX11.a -o -r $i/libX11.so -o -r $i/libX11.sl; then
	AC_MSG_RESULT($i)
	X11_LIBS="-L$i -lX11"
	break
      fi
    done
  else
    if test "$x_libraries" = ""; then
      X11_LIBS=-"lX11"
    else
      X11_LIBS="-L$x_libraries -lX11"
    fi
  fi
  if test "$X11_LIBS" = "nope" ; then
    AC_CHECK_LIB(Xwindow, XCreateWindow, X11_LIBS=-lXwindow)
  fi
  if test "$X11_LIBS" = "nope" ; then
    AC_MSG_RESULT(couldn't find any!)
  fi

  AC_MSG_CHECKING(whether to use X11)
  if test "$X11_FLAGS" = "nope" -o "$X11_LIBS" = "nope"; then
    AC_MSG_RESULT(no)
    X11_FLAGS=""
    X11_LIBS=""
    ifelse([$2], , :, [$2])
  else
    AC_MSG_RESULT(yes)
    ifelse([$1], , :, [$1])
  fi
  AC_SUBST(X11_CFLAGS)
  AC_SUBST(X11_LIBS)
])
