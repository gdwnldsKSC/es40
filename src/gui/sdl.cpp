/* ES40 emulator.
 * Copyright (C) 2007-2008 by the ES40 Emulator Project
 *
 * WWW    : http://es40.org
 * E-mail : camiel@camicom.com
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
  * Contains the code for the bx_sdl_gui_c class used for interfacing with
  * SDL.
  *
  * $Id$
  *
  * X-1.18       Martin Borgman                                  10-APR-2008
  *	    Handle SDL support on OS X through OS_X/SDLMain.m.
  *
  * X-1.15       Camiel Vanderhoeven                             29-FEB-2008
  *      Comments
  *
  * X-1.14       Camiel Vanderhoeven                             12-FEB-2008
  *      Moved keyboard code into it's own class (CKeyboard)
  *
  * X-1.13       Camiel Vanderhoeven                             22-JAN-2008
  *      Minor cleanups.
  *
  * X-1.12       Fang Zhe                                        05-JAN-2008
  *      Last patch was applied incompletely.
  *
  * X-1.11       Fang Zhe                                        04-JAN-2008
  *      Improved compatibility with Apple OS X; keyboard works now.
  *
  * X-1.10       Fang Zhe                                        03-JAN-2008
  *      Compatibility with Apple OS X.
  *
  * X-1.9        Camiel Vanderhoeven                             02-JAN-2008
  *      Comments.
  *
  * X-1.4        Camiel Vanderhoeven                             10-DEC-2007
  *      Use Configurator.
  *
  * X-1.3        Camiel Vanderhoeven                             7-DEC-2007
  *      Made keyboard messages conditional.
  *
  * X-1.2        Camiel Vanderhoeven                             7-DEC-2007
  *      Code cleanup.
  *
  * X-1.1        Camiel Vanderhoeven                             6-DEC-2007
  *      Initial version for ES40 emulator.
  *
  **/
#include "../StdAfx.h"

#if defined(HAVE_SDL)
#include "gui.h"
#include "keymap.h"
#include "../VGA.h"
#include "../System.h"

  //#include "../AliM1543C.h"
#include "../Keyboard.h"
#include "../Configurator.h"

#define _MULTI_THREAD

// Define BX_PLUGGABLE in files that can be compiled into plugins.  For
// platforms that require a special tag on exported symbols, BX_PLUGGABLE
// is used to know when we are exporting symbols and when we are importing.
#define BX_PLUGGABLE

#include <stdlib.h>
#include <string.h>
#include <SDL3/SDL.h>

#include "sdl_fonts.h"

/**
 * \brief GUI implementation using SDL3.
 **/
class bx_sdl_gui_c : public bx_gui_c
{
public:
	bx_sdl_gui_c(CConfigurator* cfg);
	virtual void    specific_init(unsigned x_tilesize, unsigned y_tilesize);
	virtual void    text_update(u8* old_text, u8* new_text, unsigned long cursor_x, unsigned long cursor_y, bx_vga_tminfo_t tm_info, unsigned rows) {};
	virtual void    graphics_tile_update(u8* snapshot, unsigned x, unsigned y);
	virtual void    handle_events(void);
	virtual void    flush(void);
	virtual void    clear_screen(void);
	virtual bool    palette_change(unsigned index, unsigned red, unsigned green, unsigned blue);
	virtual void    dimension_update(unsigned x, unsigned y, unsigned fheight = 0, unsigned fwidth = 0, unsigned bpp = 8);
	virtual void    mouse_enabled_changed_specific(bool val);
	virtual void    exit(void);
	virtual			bx_svga_tileinfo_t* graphics_tile_info(bx_svga_tileinfo_t* info);
	virtual			u8* graphics_tile_get(unsigned x, unsigned y, unsigned* w, unsigned* h);
	virtual void    graphics_tile_update_in_place(unsigned x, unsigned y, unsigned w, unsigned h);
	void			graphics_frame_update(const u32* pixels, unsigned w, unsigned h) override;
private:
	CConfigurator* myCfg;
};

// declare one instance of the gui object and call macro to insert the
// plugin code
static bx_sdl_gui_c* theGui = NULL;
IMPLEMENT_GUI_PLUGIN_CODE(sdl)
static unsigned     prev_cursor_x = 0;
static unsigned     prev_cursor_y = 0;
static u32          convertStringToSDLKey(const char* string);

static SDL_Window*   sdl_window = NULL;
static SDL_Renderer* sdl_renderer = NULL;
static SDL_Texture*  sdl_texture = NULL;


SDL_Event           sdl_event;
int                 sdl_grab = 0;
unsigned            res_x = 0, res_y = 0;
unsigned            half_res_x, half_res_y;
u8                  old_mousebuttons = 0, new_mousebuttons = 0;
int                 old_mousex = 0, new_mousex = 0;
int                 old_mousey = 0, new_mousey = 0;
bool                just_warped = false;

bx_sdl_gui_c::bx_sdl_gui_c(CConfigurator* cfg)
{
	myCfg = cfg;
	bx_keymap = new bx_keymap_c(cfg);
}

void bx_sdl_gui_c::specific_init(unsigned x_tilesize, unsigned y_tilesize)
{
	if (!SDL_Init(SDL_INIT_VIDEO))
	{
		FAILURE(SDL, "Unable to initialize SDL3 video subsystem");
	}

	// Create the initial window + renderer + texture at 640x480.
	// dimension_update() will recreate the texture if the resolution changes.
	dimension_update(640, 480);

	// SDL3: key repeat is handled by the OS; no SDL_EnableKeyRepeat().

	// Warp mouse to center of window
	if (sdl_window) {
		SDL_WarpMouseInWindow(sdl_window, (float)half_res_x, (float)half_res_y);
	}

	// load keymap for sdl
	if (myCfg->get_bool_value("keyboard.use_mapping", false))
	{
		bx_keymap->loadKeymap(convertStringToSDLKey);
	}

	new_gfx_api = 1;
}

void bx_sdl_gui_c::graphics_frame_update(const u32* pixels, unsigned width, unsigned height)
{
	if (!sdl_texture || !sdl_renderer)
		return;

	// Upload the ARGB32 pixels directly to the streaming texture.
	// pitch = width * 4 bytes per pixel
	SDL_UpdateTexture(sdl_texture, NULL, pixels, (int)(width * sizeof(u32)));

	// Present: clear -> draw texture -> flip
	SDL_RenderClear(sdl_renderer);
	SDL_RenderTexture(sdl_renderer, sdl_texture, NULL, NULL);
	SDL_RenderPresent(sdl_renderer);
}

void bx_sdl_gui_c::graphics_tile_update(u8* snapshot, unsigned x, unsigned y)
{
//
}

bx_svga_tileinfo_t* bx_sdl_gui_c::graphics_tile_info(bx_svga_tileinfo_t* info)
{
	return NULL;
}

u8* bx_sdl_gui_c::graphics_tile_get(unsigned x0, unsigned y0, unsigned* w,
	unsigned* h)
{
	return NULL;
}

void bx_sdl_gui_c::graphics_tile_update_in_place(unsigned x0, unsigned y0,
	unsigned w, unsigned h)
{
	//
}

static u32 sdl_sym_to_bx_key(SDL_Keycode sym)
{
	switch (sym)
	{
	case SDLK_BACKSPACE:    return BX_KEY_BACKSPACE;
	case SDLK_TAB:          return BX_KEY_TAB;
	case SDLK_RETURN:       return BX_KEY_ENTER;
	case SDLK_PAUSE:        return BX_KEY_PAUSE;
	case SDLK_ESCAPE:       return BX_KEY_ESC;
	case SDLK_SPACE:        return BX_KEY_SPACE;
	case SDLK_APOSTROPHE:   return BX_KEY_SINGLE_QUOTE;
	case SDLK_COMMA:        return BX_KEY_COMMA;
	case SDLK_MINUS:        return BX_KEY_MINUS;
	case SDLK_PERIOD:       return BX_KEY_PERIOD;
	case SDLK_SLASH:        return BX_KEY_SLASH;

	case SDLK_0:            return BX_KEY_0;
	case SDLK_1:            return BX_KEY_1;
	case SDLK_2:            return BX_KEY_2;
	case SDLK_3:            return BX_KEY_3;
	case SDLK_4:            return BX_KEY_4;
	case SDLK_5:            return BX_KEY_5;
	case SDLK_6:            return BX_KEY_6;
	case SDLK_7:            return BX_KEY_7;
	case SDLK_8:            return BX_KEY_8;
	case SDLK_9:            return BX_KEY_9;

	case SDLK_SEMICOLON:    return BX_KEY_SEMICOLON;
	case SDLK_EQUALS:       return BX_KEY_EQUALS;

	case SDLK_LEFTBRACKET:  return BX_KEY_LEFT_BRACKET;
	case SDLK_BACKSLASH:    return BX_KEY_BACKSLASH;
	case SDLK_RIGHTBRACKET: return BX_KEY_RIGHT_BRACKET;
	case SDLK_GRAVE:        return BX_KEY_GRAVE;

	case SDLK_A:            return BX_KEY_A;
	case SDLK_B:            return BX_KEY_B;
	case SDLK_C:            return BX_KEY_C;
	case SDLK_D:            return BX_KEY_D;
	case SDLK_E:            return BX_KEY_E;
	case SDLK_F:            return BX_KEY_F;
	case SDLK_G:            return BX_KEY_G;
	case SDLK_H:            return BX_KEY_H;
	case SDLK_I:            return BX_KEY_I;
	case SDLK_J:            return BX_KEY_J;
	case SDLK_K:            return BX_KEY_K;
	case SDLK_L:            return BX_KEY_L;
	case SDLK_M:            return BX_KEY_M;
	case SDLK_N:            return BX_KEY_N;
	case SDLK_O:            return BX_KEY_O;
	case SDLK_P:            return BX_KEY_P;
	case SDLK_Q:            return BX_KEY_Q;
	case SDLK_R:            return BX_KEY_R;
	case SDLK_S:            return BX_KEY_S;
	case SDLK_T:            return BX_KEY_T;
	case SDLK_U:            return BX_KEY_U;
	case SDLK_V:            return BX_KEY_V;
	case SDLK_W:            return BX_KEY_W;
	case SDLK_X:            return BX_KEY_X;
	case SDLK_Y:            return BX_KEY_Y;
	case SDLK_Z:            return BX_KEY_Z;

	case SDLK_DELETE:       return BX_KEY_DELETE;

		// Keypad
	case SDLK_KP_0:         return BX_KEY_KP_INSERT;
	case SDLK_KP_1:         return BX_KEY_KP_END;
	case SDLK_KP_2:         return BX_KEY_KP_DOWN;
	case SDLK_KP_3:         return BX_KEY_KP_PAGE_DOWN;
	case SDLK_KP_4:         return BX_KEY_KP_LEFT;
	case SDLK_KP_5:         return BX_KEY_KP_5;
	case SDLK_KP_6:         return BX_KEY_KP_RIGHT;
	case SDLK_KP_7:         return BX_KEY_KP_HOME;
	case SDLK_KP_8:         return BX_KEY_KP_UP;
	case SDLK_KP_9:         return BX_KEY_KP_PAGE_UP;
	case SDLK_KP_PERIOD:    return BX_KEY_KP_DELETE;
	case SDLK_KP_DIVIDE:    return BX_KEY_KP_DIVIDE;
	case SDLK_KP_MULTIPLY:  return BX_KEY_KP_MULTIPLY;
	case SDLK_KP_MINUS:     return BX_KEY_KP_SUBTRACT;
	case SDLK_KP_PLUS:      return BX_KEY_KP_ADD;
	case SDLK_KP_ENTER:     return BX_KEY_KP_ENTER;

		// Arrows + Home/End pad
	case SDLK_UP:           return BX_KEY_UP;
	case SDLK_DOWN:         return BX_KEY_DOWN;
	case SDLK_RIGHT:        return BX_KEY_RIGHT;
	case SDLK_LEFT:         return BX_KEY_LEFT;
	case SDLK_INSERT:       return BX_KEY_INSERT;
	case SDLK_HOME:         return BX_KEY_HOME;
	case SDLK_END:          return BX_KEY_END;
	case SDLK_PAGEUP:       return BX_KEY_PAGE_UP;
	case SDLK_PAGEDOWN:     return BX_KEY_PAGE_DOWN;

		// Function keys
	case SDLK_F1:           return BX_KEY_F1;
	case SDLK_F2:           return BX_KEY_F2;
	case SDLK_F3:           return BX_KEY_F3;
	case SDLK_F4:           return BX_KEY_F4;
	case SDLK_F5:           return BX_KEY_F5;
	case SDLK_F6:           return BX_KEY_F6;
	case SDLK_F7:           return BX_KEY_F7;
	case SDLK_F8:           return BX_KEY_F8;
	case SDLK_F9:           return BX_KEY_F9;
	case SDLK_F10:          return BX_KEY_F10;
	case SDLK_F11:          return BX_KEY_F11;
	case SDLK_F12:          return BX_KEY_F12;

		// Modifier keys
	case SDLK_NUMLOCKCLEAR: return BX_KEY_NUM_LOCK;
	case SDLK_CAPSLOCK:     return BX_KEY_CAPS_LOCK;
	case SDLK_SCROLLLOCK:   return BX_KEY_SCRL_LOCK;
	case SDLK_RSHIFT:       return BX_KEY_SHIFT_R;
	case SDLK_LSHIFT:       return BX_KEY_SHIFT_L;
	case SDLK_RCTRL:        return BX_KEY_CTRL_R;
	case SDLK_LCTRL:        return BX_KEY_CTRL_L;
	case SDLK_RALT:         return BX_KEY_ALT_R;
	case SDLK_LALT:         return BX_KEY_ALT_L;
	case SDLK_LGUI:         return BX_KEY_WIN_L;
	case SDLK_RGUI:         return BX_KEY_WIN_R;

		// Misc function keys
	case SDLK_PRINTSCREEN:  return BX_KEY_PRINT;
	case SDLK_MENU:         return BX_KEY_MENU;

	default:
		BX_ERROR(("sdl3 keycode 0x%x not mapped", (unsigned)sym));
		return BX_KEY_UNHANDLED;
	}
}

void bx_sdl_gui_c::handle_events(void)
{
	u32 key_event;

	while (SDL_PollEvent(&sdl_event))
	{
		switch (sdl_event.type)
		{
		case SDL_EVENT_WINDOW_EXPOSED:
			// Window needs redraw — re-present the current texture
			if (sdl_renderer && sdl_texture)
			{
				SDL_RenderClear(sdl_renderer);
				SDL_RenderTexture(sdl_renderer, sdl_texture, NULL, NULL);
				SDL_RenderPresent(sdl_renderer);
			}
			break;

		case SDL_EVENT_MOUSE_MOTION:
			// Mouse handling — placeholder for future integration.
			break;

		case SDL_EVENT_MOUSE_BUTTON_DOWN:
		case SDL_EVENT_MOUSE_BUTTON_UP:
			// Mouse button handling — placeholder
			break;

		case SDL_EVENT_KEY_DOWN:
			// Filter out ScrollLock (fullscreen toggle prev.) and invalid keys
			if (sdl_event.key.key == SDLK_SCROLLLOCK)
				break;

			// convert sym -> bochs code
			if (!myCfg->get_bool_value("keyboard.use_mapping", false))
			{
				key_event = sdl_sym_to_bx_key(sdl_event.key.key);
			}
			else
			{
				/* use mapping */
				BXKeyEntry* entry = bx_keymap->findHostKey(sdl_event.key.key);
				if (!entry)
				{
					BX_ERROR(("host key 0x%x not mapped!",
						(unsigned)sdl_event.key.key));
					break;
				}
				key_event = entry->baseKey;
			}

			if (key_event == BX_KEY_UNHANDLED)
				break;

			theKeyboard->gen_scancode(key_event);

			// Locks: generate immediate press+release pair
			if ((key_event == BX_KEY_NUM_LOCK) || (key_event == BX_KEY_CAPS_LOCK))
			{
				theKeyboard->gen_scancode(key_event | BX_KEY_RELEASED);
			}
			break;

		case SDL_EVENT_KEY_UP:
			if (sdl_event.key.key == SDLK_SCROLLLOCK)
				break;

			if (!myCfg->get_bool_value("keyboard.use_mapping", false))
			{
				key_event = sdl_sym_to_bx_key(sdl_event.key.key);
			}
			else
			{
				BXKeyEntry* entry = bx_keymap->findHostKey(sdl_event.key.key);
				if (!entry)
				{
					BX_ERROR(("host key 0x%x not mapped!",
						(unsigned)sdl_event.key.key));
					break;
				}
				key_event = entry->baseKey;
			}

			if (key_event == BX_KEY_UNHANDLED)
				break;

			if ((key_event == BX_KEY_NUM_LOCK) || (key_event == BX_KEY_CAPS_LOCK))
			{
				theKeyboard->gen_scancode(key_event);
			}

			theKeyboard->gen_scancode(key_event | BX_KEY_RELEASED);
			break;

		case SDL_EVENT_QUIT:
			FAILURE(Graceful, "User requested shutdown");
		}
	}
}

/**
 * Flush any changes to sdl_screen to the actual window.
 **/
void bx_sdl_gui_c::flush(void)
{
	//
}

/**
 * Clear sdl_screen display, and flush it.
 **/
void bx_sdl_gui_c::clear_screen(void)
{
	if (!sdl_renderer)
		return;

	SDL_SetRenderDrawColor(sdl_renderer, 0, 0, 0, 255);
	SDL_RenderClear(sdl_renderer);
	SDL_RenderPresent(sdl_renderer);
}

/**
 * Set palette-entry index to the desired value.
 *
 * The palette is used in text-mode and in 8bpp VGA mode.
 **/
bool bx_sdl_gui_c::palette_change(unsigned index, unsigned red, unsigned green,
	unsigned blue)
{
	return 1;
}

void bx_sdl_gui_c::dimension_update(unsigned x, unsigned y, unsigned fheight,
	unsigned fwidth, unsigned bpp)
{
	if ((x == res_x) && (y == res_y) && sdl_window)
		return;

	if (sdl_texture)
	{
		SDL_DestroyTexture(sdl_texture);
		sdl_texture = NULL;
	}

	if (!sdl_window)
	{
		sdl_window = SDL_CreateWindow("ES40 Emulator",
			(int)x, (int)y, SDL_WINDOW_RESIZABLE);
		if (!sdl_window)
		{
			FAILURE_3(SDL, "Unable to create SDL3 window: %ix%i: %s\n",
				x, y, SDL_GetError());
		}

		sdl_renderer = SDL_CreateRenderer(sdl_window, NULL);
		if (!sdl_renderer)
		{
			FAILURE_3(SDL, "Unable to create SDL3 renderer: %ix%i: %s\n",
				x, y, SDL_GetError());
		}

		SDL_SetRenderLogicalPresentation(sdl_renderer,
			(int)x, (int)y,
			SDL_LOGICAL_PRESENTATION_LETTERBOX);
	}
	else
	{
		SDL_SetWindowSize(sdl_window, (int)x, (int)y);
		SDL_SetRenderLogicalPresentation(sdl_renderer,
			(int)x, (int)y,
			SDL_LOGICAL_PRESENTATION_LETTERBOX);
	}

	sdl_texture = SDL_CreateTexture(sdl_renderer,
		SDL_PIXELFORMAT_ARGB8888,
		SDL_TEXTUREACCESS_STREAMING,
		(int)x, (int)y);
	if (!sdl_texture)
	{
		FAILURE_3(SDL, "Unable to create SDL3 texture: %ix%i: %s\n",
			x, y, SDL_GetError());
	}

	SDL_SetTextureScaleMode(sdl_texture, SDL_SCALEMODE_NEAREST);

	res_x = x;
	res_y = y;
	half_res_x = x / 2;
	half_res_y = y / 2;
}

void bx_sdl_gui_c::mouse_enabled_changed_specific(bool val)
{
	if (val)
	{
		SDL_HideCursor();
		if (sdl_window)
			SDL_SetWindowRelativeMouseMode(sdl_window, true);
	}
	else
	{
		SDL_ShowCursor();
		if (sdl_window)
			SDL_SetWindowRelativeMouseMode(sdl_window, false);
	}

	sdl_grab = val;
}

void bx_sdl_gui_c::exit(void)
{
	if (sdl_texture) {
		SDL_DestroyTexture(sdl_texture);
		sdl_texture = NULL;
	}
	if (sdl_renderer) {
		SDL_DestroyRenderer(sdl_renderer);
		sdl_renderer = NULL;
	}
	if (sdl_window) {
		SDL_DestroyWindow(sdl_window);
		sdl_window = NULL;
	}
}

/// key mapping for SDL
typedef struct
{
	const char* name;
	u32           value;
} keyTableEntry;

#define DEF_SDL_KEY(key) \
  {                      \
    #key, key            \
  },

keyTableEntry keytable[] = {
	// this include provides all the entries.
  #include "sdlkeys.h"
	// one final entry to mark the end
	{ NULL, 0}
};

// function to convert key names into SDLKey values.
// This first try will be horribly inefficient, but it only has
// to be done while loading a keymap.  Once the simulation starts,

// this function won't be called.
static u32 convertStringToSDLKey(const char* string)
{
	keyTableEntry* ptr;
	for (ptr = &keytable[0]; ptr->name != NULL; ptr++)
	{
		if (!strcmp(string, ptr->name))
			return ptr->value;
	}
	return 0;
}
#endif //defined(HAVE_SDL)
