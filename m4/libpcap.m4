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
# Check if libpcap is installed, and configure paths

dnl AM_PATH_PCAP([ACTION-IF-FOUND [, ACTION-IF-NOT-FOUND]])
dnl Test for PCAP, and define PCAP_CFLAGS and PCAP_LIBS
dnl

AC_DEFUN([ES_PCAP_INC_WHERE1], [
  ac_cv_found_pcap_inc=no
  if test -f "$1/pcap.h" ; then
    ac_cv_found_pcap_inc=yes
  fi
])

AC_DEFUN([ES_PCAP_INC_WHERE], [
  for i in $1; do
    AC_MSG_CHECKING(for pcap header in $i)
    ES_PCAP_INC_WHERE1($i)
    if test "$ac_cv_found_pcap_inc" = "yes"; then
      ac_cv_pcap_where_inc=$i
      AC_MSG_RESULT(found)
      break
    else
      AC_MSG_RESULT(not here)
    fi
  done
])

AC_DEFUN([ES_PCAP_LIB_WHERE1], [
  saved_LIBS=$LIBS
  LIBS="$saved_LIBS -L$1 -lpcap"
  AC_TRY_LINK(,
    [pcap_lookupdev("");],
    [ac_cv_found_pcap_lib=yes],
    ac_cv_found_pcap_lib=no)
  LIBS=$saved_LIBS
])

AC_DEFUN([ES_PCAP_LIB_WHERE], [
  for i in $1; do
    AC_MSG_CHECKING(for pcap library in $i)
    ES_PCAP_LIB_WHERE1($i)
    if test "$ac_cv_found_pcap_lib" = "yes"; then
      ac_cv_pcap_where_lib=$i
      AC_MSG_RESULT(found)
      break
    else
      AC_MSG_RESULT(not here)
    fi
  done
])


AC_DEFUN([AM_PATH_PCAP],
[dnl 
dnl Get the cflags and libraries for libpcap
dnl

  ES_PCAP_LIB_WHERE(/usr/ng/lib /usr/lib /usr/local/lib)
  ES_PCAP_INC_WHERE(/usr/ng/include /usr/include /usr/local/include)

  AC_MSG_CHECKING(whether pcap is available)

  if test "X$ac_cv_pcap_where_lib" = "X" -o "X$ac_cv_pcap_where_inc" = "X"; then
    ac_cv_found_pcap=no
    AC_MSG_RESULT(no)
    PCAP_CFLAGS=""
    PCAP_LIBS=""
    ifelse([$2], , :, [$2])
  else
    ac_cv_found_pcap=yes
    AC_MSG_RESULT(yes)
    PCAP_INC_DIR=$ac_cv_pcap_where_inc
    PCAP_LIB_DIR=$ac_cv_pcap_where_lib
    PCAP_CFLAGS="-I${PCAP_INC_DIR}"
    if test "X$PCAP_LIB_DIR" = "X"; then
     PCAP_LIBS="-lpcap"
    else
      PCAP_LIBS="-L${PCAP_LIB_DIR} -lpcap"
    fi
    ifelse([$1], , :, [$1])     
  fi
  AC_SUBST(PCAP_CFLAGS)
  AC_SUBST(PCAP_LIBS)
])
