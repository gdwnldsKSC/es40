/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * WWW    : https://github.com/gdwnldsKSC/es40/
 *
 *  This file is based upon Bochs.
 *
 *  Copyright (C) 2002  MandrakeSoft S.A.
 *
 *    MandrakeSoft S.A.
 *    43, rue d'Aboukir
 *    75002 Paris - France
 *    http://www.linux-mandrake.com/
 *    http://www.mandrakesoft.com/
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

 /**
  * \file
  * This file is simply a list of SDL key symbols taken from <SDL/SDL_keysym.h>.
  * The order in this file is not important.  In sdl.cc, DEF_SDL_KEY() is
  * defined as a macro and then it includes this file to fill in all the data in
  * its key mapping table.
  *
  * The symbols, such as SDLK_RETURN, are used for two purposes.  They
  * are converted into a string (by the # operator in processor), which is
  * compared to the host key name in the keymap file.  Also, the value of
  * the symbol is inserted into the key mapping table.  Then the value is
  * compared with the keysym field of each key up/down event as it arrives.
  *
  * If you get undefined symbol errors in this file, it must mean that
  * your SDL library version doesn't define those same SDLK_* symbols in
  * <SDL/SDL_keysym.h>.  You can't fix it with #ifdef SDLK_SYM because
  * they are enums, so you'll just have to comment out the offending line.
  * The list was generated using symbols from SDL 1.2.3.
  *
  * $Id$
  * X-1.3        Camiel Vanderhoeven                             02-JAN-2008
  *      Comments.
  *
  * X-1.1        Camiel Vanderhoeven                             6-DEC-2007
  *      Initial version for ES40 emulator.
  *
  **/
DEF_SDL_KEY(SDLK_UNKNOWN)
// SDLK_FIRST -- removed in SDL3
DEF_SDL_KEY(SDLK_BACKSPACE)
DEF_SDL_KEY(SDLK_TAB)
DEF_SDL_KEY(SDLK_CLEAR)
DEF_SDL_KEY(SDLK_RETURN)
DEF_SDL_KEY(SDLK_PAUSE)
DEF_SDL_KEY(SDLK_ESCAPE)
DEF_SDL_KEY(SDLK_SPACE)
DEF_SDL_KEY(SDLK_EXCLAIM)
DEF_SDL_KEY(SDLK_DBLAPOSTROPHE)  // was SDLK_QUOTEDBL
DEF_SDL_KEY(SDLK_HASH)
DEF_SDL_KEY(SDLK_DOLLAR)
DEF_SDL_KEY(SDLK_AMPERSAND)
DEF_SDL_KEY(SDLK_APOSTROPHE)     // was SDLK_QUOTE
DEF_SDL_KEY(SDLK_LEFTPAREN)
DEF_SDL_KEY(SDLK_RIGHTPAREN)
DEF_SDL_KEY(SDLK_ASTERISK)
DEF_SDL_KEY(SDLK_PLUS)
DEF_SDL_KEY(SDLK_COMMA)
DEF_SDL_KEY(SDLK_MINUS)
DEF_SDL_KEY(SDLK_PERIOD)
DEF_SDL_KEY(SDLK_SLASH)
DEF_SDL_KEY(SDLK_0)
DEF_SDL_KEY(SDLK_1)
DEF_SDL_KEY(SDLK_2)
DEF_SDL_KEY(SDLK_3)
DEF_SDL_KEY(SDLK_4)
DEF_SDL_KEY(SDLK_5)
DEF_SDL_KEY(SDLK_6)
DEF_SDL_KEY(SDLK_7)
DEF_SDL_KEY(SDLK_8)
DEF_SDL_KEY(SDLK_9)
DEF_SDL_KEY(SDLK_COLON)
DEF_SDL_KEY(SDLK_SEMICOLON)
DEF_SDL_KEY(SDLK_LESS)
DEF_SDL_KEY(SDLK_EQUALS)
DEF_SDL_KEY(SDLK_GREATER)
DEF_SDL_KEY(SDLK_QUESTION)
DEF_SDL_KEY(SDLK_AT)

//DEF_SDL_KEY( /*  )
//DEF_SDL_KEY( Skip uppercase letters )
//DEF_SDL_KEY( */ )
DEF_SDL_KEY(SDLK_LEFTBRACKET)
DEF_SDL_KEY(SDLK_BACKSLASH)
DEF_SDL_KEY(SDLK_RIGHTBRACKET)
DEF_SDL_KEY(SDLK_CARET)
DEF_SDL_KEY(SDLK_UNDERSCORE)
DEF_SDL_KEY(SDLK_GRAVE)          // was SDLK_BACKQUOTE
DEF_SDL_KEY(SDLK_A)              // was SDLK_a, SDL3 uses uppercase
DEF_SDL_KEY(SDLK_B)
DEF_SDL_KEY(SDLK_C)
DEF_SDL_KEY(SDLK_D)
DEF_SDL_KEY(SDLK_E)
DEF_SDL_KEY(SDLK_F)
DEF_SDL_KEY(SDLK_G)
DEF_SDL_KEY(SDLK_H)
DEF_SDL_KEY(SDLK_I)
DEF_SDL_KEY(SDLK_J)
DEF_SDL_KEY(SDLK_K)
DEF_SDL_KEY(SDLK_L)
DEF_SDL_KEY(SDLK_M)
DEF_SDL_KEY(SDLK_N)
DEF_SDL_KEY(SDLK_O)
DEF_SDL_KEY(SDLK_P)
DEF_SDL_KEY(SDLK_Q)
DEF_SDL_KEY(SDLK_R)
DEF_SDL_KEY(SDLK_S)
DEF_SDL_KEY(SDLK_T)
DEF_SDL_KEY(SDLK_U)
DEF_SDL_KEY(SDLK_V)
DEF_SDL_KEY(SDLK_W)
DEF_SDL_KEY(SDLK_X)
DEF_SDL_KEY(SDLK_Y)
DEF_SDL_KEY(SDLK_Z)
DEF_SDL_KEY(SDLK_DELETE)
// SDLK_WORLD_0 ... SDLK_WORLD_95 -- removed in SDL3
DEF_SDL_KEY(SDLK_KP_0)           // was SDLK_KP0
DEF_SDL_KEY(SDLK_KP_1)           // was SDLK_KP1
DEF_SDL_KEY(SDLK_KP_2)           // was SDLK_KP2
DEF_SDL_KEY(SDLK_KP_3)           // was SDLK_KP3
DEF_SDL_KEY(SDLK_KP_4)           // was SDLK_KP4
DEF_SDL_KEY(SDLK_KP_5)           // was SDLK_KP5
DEF_SDL_KEY(SDLK_KP_6)           // was SDLK_KP6
DEF_SDL_KEY(SDLK_KP_7)           // was SDLK_KP7
DEF_SDL_KEY(SDLK_KP_8)           // was SDLK_KP8
DEF_SDL_KEY(SDLK_KP_9)           // was SDLK_KP9
DEF_SDL_KEY(SDLK_KP_PERIOD)
DEF_SDL_KEY(SDLK_KP_DIVIDE)
DEF_SDL_KEY(SDLK_KP_MULTIPLY)
DEF_SDL_KEY(SDLK_KP_MINUS)
DEF_SDL_KEY(SDLK_KP_PLUS)
DEF_SDL_KEY(SDLK_KP_ENTER)
DEF_SDL_KEY(SDLK_KP_EQUALS)
DEF_SDL_KEY(SDLK_UP)
DEF_SDL_KEY(SDLK_DOWN)
DEF_SDL_KEY(SDLK_RIGHT)
DEF_SDL_KEY(SDLK_LEFT)
DEF_SDL_KEY(SDLK_INSERT)
DEF_SDL_KEY(SDLK_HOME)
DEF_SDL_KEY(SDLK_END)
DEF_SDL_KEY(SDLK_PAGEUP)
DEF_SDL_KEY(SDLK_PAGEDOWN)
DEF_SDL_KEY(SDLK_F1)
DEF_SDL_KEY(SDLK_F2)
DEF_SDL_KEY(SDLK_F3)
DEF_SDL_KEY(SDLK_F4)
DEF_SDL_KEY(SDLK_F5)
DEF_SDL_KEY(SDLK_F6)
DEF_SDL_KEY(SDLK_F7)
DEF_SDL_KEY(SDLK_F8)
DEF_SDL_KEY(SDLK_F9)
DEF_SDL_KEY(SDLK_F10)
DEF_SDL_KEY(SDLK_F11)
DEF_SDL_KEY(SDLK_F12)
DEF_SDL_KEY(SDLK_F13)
DEF_SDL_KEY(SDLK_F14)
DEF_SDL_KEY(SDLK_F15)
DEF_SDL_KEY(SDLK_NUMLOCKCLEAR)   // was SDLK_NUMLOCK
DEF_SDL_KEY(SDLK_CAPSLOCK)
DEF_SDL_KEY(SDLK_SCROLLLOCK)     // was SDLK_SCROLLOCK
DEF_SDL_KEY(SDLK_RSHIFT)
DEF_SDL_KEY(SDLK_LSHIFT)
DEF_SDL_KEY(SDLK_RCTRL)
DEF_SDL_KEY(SDLK_LCTRL)
DEF_SDL_KEY(SDLK_RALT)
DEF_SDL_KEY(SDLK_LALT)
DEF_SDL_KEY(SDLK_RMETA)          // SDL3 extended key
DEF_SDL_KEY(SDLK_LMETA)          // SDL3 extended key
DEF_SDL_KEY(SDLK_LGUI)           // was SDLK_LSUPER
DEF_SDL_KEY(SDLK_RGUI)           // was SDLK_RSUPER
DEF_SDL_KEY(SDLK_MODE)
DEF_SDL_KEY(SDLK_MULTI_KEY_COMPOSE)        // was SDLK_COMPOSE
DEF_SDL_KEY(SDLK_HELP)
DEF_SDL_KEY(SDLK_PRINTSCREEN)    // was SDLK_PRINT
DEF_SDL_KEY(SDLK_SYSREQ)
// SDLK_BREAK -- (SDLK_PAUSE covers it)
DEF_SDL_KEY(SDLK_MENU)
DEF_SDL_KEY(SDLK_POWER)
// SDLK_EURO -- removed in SDL3 
DEF_SDL_KEY(SDLK_UNDO)
