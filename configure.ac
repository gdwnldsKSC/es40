#                                               -*- Autoconf -*-
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
# X-1.3      Camiel Vanderhoeven                      31-MAY-2008
#      Add parts of Poco.
#
# X-1.2      Camiel Vanderhoeven                      04-APR-2008
#      Version number set to "0.18+"
#
# X-1.1	     Camiel Vanderhoeven                      20-MAR-2008
#      File Created.
#
################################################################################
#
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.71])
AC_INIT([es40],[0.50],[https://github.com/gdwnldsKSC/es40/issues/new],[],[https://github.com/gdwnldsKSC/es40/issues])
AC_CONFIG_SRCDIR([src/Flash.cpp])
AM_CONFIG_HEADER([src/config.h])
AM_INIT_AUTOMAKE([foreign subdir-objects])
AC_CANONICAL_HOST

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

# We need C++17.
AX_CXX_COMPILE_STDCXX([17], [ext], [mandatory])

case "$host_os" in
  darwin*)
    # Xcode 16+ marks sprintf as deprecated and the legacy codebase still uses
    # it in debug/error macros. Keep source unchanged and make macOS builds
    # tolerate these deprecation warnings while we modernize incrementally.
    CXXFLAGS="$CXXFLAGS -Wno-deprecated-declarations"

    # Work around duplicate "thread-local wrapper routine" symbols produced
    # by Apple clang/ld for inline thread_local header variables. Making
    # symbol visibility hidden keeps these internal wrappers out of the global
    # link namespace without touching runtime/source files.
    CXXFLAGS="$CXXFLAGS -fvisibility=hidden -fvisibility-inlines-hidden"

    # Keep CI output focused on actionable issues on macOS where clang can
    # emit noisy override-style warnings for legacy code.
    CXXFLAGS="$CXXFLAGS -Wno-inconsistent-missing-override"
    ;;
esac

# Checks for header files.
m4_warn([obsolete],
[The preprocessor macro 'STDC_HEADERS' is obsolete.
  Except in unusual embedded environments, you can safely include all
  C89 headers unconditionally.])dnl
# Autoupdate added the next two lines to ensure that your configure
# script's behavior did not change.  They are probably safe to remove.
AC_CHECK_INCLUDES_DEFAULT
AC_PROG_EGREP

AC_HEADER_SYS_WAIT
AC_CHECK_HEADERS([arpa/inet.h arpa/telnet.h ctype.h errno.h fcntl.h in.h inet.h inttypes.h malloc.h netinet/in.h process.h pthread.h signal.h socket.h stdint.h stdlib.h string.h sys/param.h sys/socket.h sys/time.h unistd.h windows.h winsock2.h ws2tcpip.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_CONST
AC_C_INLINE
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_TYPE_INT8_T
AC_TYPE_PID_T
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
m4_warn([obsolete],
[Update your code to rely only on HAVE_SYS_TIME_H,
then remove this warning and the obsolete code below it.
All current systems provide time.h; it need not be checked for.
Not all systems provide sys/time.h, but those that do, all allow
you to include it and time.h simultaneously.])dnl
AC_CHECK_HEADERS_ONCE([sys/time.h])
# Obsolete code to be removed.
if test $ac_cv_header_sys_time_h = yes; then
  AC_DEFINE([TIME_WITH_SYS_TIME],[1],[Define to 1 if you can safely include both <sys/time.h>
	     and <time.h>.  This macro is obsolete.])
fi
# End of obsolete code.

AC_STRUCT_TM
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_FUNC_FORK
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_FUNC_SELECT_ARGTYPES
m4_warn([obsolete],
[your code may safely assume C89 semantics that RETSIGTYPE is void.
Remove this warning and the 'AC_CACHE_CHECK' when you adjust the code.])dnl
AC_CACHE_CHECK([return type of signal handlers],[ac_cv_type_signal],[AC_COMPILE_IFELSE(
[AC_LANG_PROGRAM([#include <sys/types.h>
#include <signal.h>
],
		 [return *(signal (0, 0)) (0) == 1;])],
		   [ac_cv_type_signal=int],
		   [ac_cv_type_signal=void])])
AC_DEFINE_UNQUOTED([RETSIGTYPE],[$ac_cv_type_signal],[Define as the return type of signal handlers
		    ('int' or 'void').])

AC_CHECK_FUNCS([alarm atexit fopen fopen64 fseek fseeko fseeko64 _fseeki64 ftell ftello ftello64 _ftelli64 gmtime_s inet_aton isblank memset pow select socket sqrt strcasecmp _stricmp strchr strdup _strdup strncasecmp _stricasecmp strspn])

# Check for pthreads
AX_PTHREAD
CXXFLAGS="$CXXFLAGS $PTHREAD_CFLAGS"
LDFLAGS="$LDFLAGS $PTHREAD_LDFLAGS"
LIBS="$LIBS $PTHREAD_LIBS"

# Allow disabling PCAP for no-network builds.
AC_ARG_ENABLE([pcap],
  [AS_HELP_STRING([--disable-pcap],[Disable pcap support])],
  [],
  [enable_pcap=yes])

AS_IF([test "x$enable_pcap" != "xno"], [
  case "$host_os" in
    darwin*)
      # On macOS, use our own AM_PATH_PCAP (m4/libpcap.m4)
      AM_PATH_PCAP(
        CXXFLAGS="$CXXFLAGS $PCAP_CFLAGS -DHAVE_PCAP"
        LIBS="$LIBS $PCAP_LIBS"
        ,:)
      ;;
    *)
      PKG_CHECK_MODULES([PCAP],[libpcap >= 0.8],
        [
          AC_DEFINE([HAVE_PCAP],[1],[Use PCAP])
          CXXFLAGS="$CXXFLAGS $PCAP_CFLAGS -DHAVE_PCAP"
          LIBS="$LIBS $PCAP_LIBS"
        ],
        [
          AC_MSG_FAILURE([libpcap >= 0.8 is required unless --disable-pcap is used])
        ])
      ;;
  esac
])

# Use our own method to search for X11
AM_PATH_X11([], [:])

if test "x$X11_CFLAGS" != xnope && test "x$X11_LIBS" != xnope; then
  CPPFLAGS="$CPPFLAGS $X11_CFLAGS"
  CXXFLAGS="$CXXFLAGS -DHAVE_X11"
  LIBS="$LIBS $X11_LIBS"
fi

# Allow disabling SDL for no-screen builds.
AC_ARG_ENABLE([sdl],
  [AS_HELP_STRING([--disable-sdl],[Disable SDL support])],
  [],
  [enable_sdl=yes])

AS_IF([test "x$enable_sdl" != "xno"], [
  PKG_CHECK_MODULES([SDL],[sdl3 >= 3.2.0],
    [
      AC_DEFINE([HAVE_SDL],[1],[Use SDL])
      CXXFLAGS="$CXXFLAGS $SDL_CFLAGS -DHAVE_SDL"
      LIBS="$LIBS $SDL_LIBS"
    ],
    [
      AC_MSG_FAILURE([sdl3 >= 3.2.0 is required unless --disable-sdl is used])
    ])
])

# Generate the various makefiles.
AC_CONFIG_FILES([Makefile doc/Makefile m4/Makefile src/Makefile])
AC_OUTPUT
