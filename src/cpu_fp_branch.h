/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * WWW    : http://sourceforge.net/projects/es40
 * E-mail : camiel@camicom.com
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
  * Contains code macros for the processor floating-point branch instructions.
  * Based on ARM chapter 4.9.
  *
  * $Id$
  *
  * X-1.12       Camiel Vanderhoeven                             14-MAR-2008
  *   1. More meaningful exceptions replace throwing (int) 1.
  *   2. U64 macro replaces X64 macro.
  *
  * X-1.11       Brian Wheeler                                   27-FEB-2008
  *      Avoid compiler warnings.
  *
  * X-1.10       Camiel Vanderhoeven                             06-FEB-2008
  *      Check for FPEN in old floating point code.
  *
  * X-1.9        Camiel Vanderhoeven                             05-FEB-2008
  *      Only use new floating-point code when HAVE_NEW_FP has been defined.
  *
  * X-1.8        Camiel Vanderhoeven                             30-JAN-2008
  *      Always use set_pc or add_pc to change the program counter.
  *
  * X-1.7        Camiel Vanderhoeven                             30-JAN-2008
  *      Remember number of instructions left in current memory page, so
  *      that the translation-buffer doens't need to be consulted on every
  *      instruction fetch when the Icache is disabled.
  *
  * X-1.6        Camiel Vanderhoeven                             28-JAN-2008
  *      Better floating-point exception handling.
  *
  * X-1.5        Camiel Vanderhoeven                             22-JAN-2008
  *      Implement new floating-point code.
  *
  * X-1.4        Camiel Vanderhoeven                             11-APR-2007
  *      Moved all data that should be saved to a state file to a structure
  *      "state".
  *
  * X-1.3        Camiel Vanderhoeven                             30-MAR-2007
  *      Added old changelog comments.
  *
  * X-1.2        Camiel Vanderhoeven                             13-MAR-2007
  *      Basic floating point support added.
  *
  * X-1.1        Camiel Vanderhoeven                             18-FEB-2007
  *      File created. Contains code previously found in AlphaCPU.h
  **/
#define FP_IS_ZERO(val) (((val) & ~FPR_SIGN) == 0)
#define FP_IS_NEGATIVE(val) (((val) & FPR_SIGN) != 0)

// Branch if equal to zero (+0 or -0)
#define DO_FBEQ FPSTART; \
  if(FP_IS_ZERO(state.f[FREG_1])) \
    add_pc(DISP_21 * 4);

// Branch if >= 0 (positive or zero, including -0)
#define DO_FBGE FPSTART; \
  if(!FP_IS_NEGATIVE(state.f[FREG_1]) || FP_IS_ZERO(state.f[FREG_1])) \
    add_pc(DISP_21 * 4);

// Branch if > 0 (positive and non-zero)
#define DO_FBGT FPSTART; \
  if(!FP_IS_NEGATIVE(state.f[FREG_1]) && !FP_IS_ZERO(state.f[FREG_1])) \
    add_pc(DISP_21 * 4);

// Branch if <= 0 (negative or zero)
#define DO_FBLE FPSTART; \
  if(FP_IS_NEGATIVE(state.f[FREG_1]) || FP_IS_ZERO(state.f[FREG_1])) \
    add_pc(DISP_21 * 4);

// Branch if < 0 (negative and non-zero)
#define DO_FBLT FPSTART; \
  if(FP_IS_NEGATIVE(state.f[FREG_1]) && !FP_IS_ZERO(state.f[FREG_1])) \
    add_pc(DISP_21 * 4);

// Branch if not equal to zero
#define DO_FBNE FPSTART; \
  if(!FP_IS_ZERO(state.f[FREG_1])) \
    add_pc(DISP_21 * 4);
