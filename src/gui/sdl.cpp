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

#include "../Disk.h"
#include "../DiskFile.h"

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
	virtual void    specific_init(unsigned x_tilesize, unsigned y_tilesize) override;
	virtual void    text_update(u8* old_text, u8* new_text, unsigned long cursor_x, unsigned long cursor_y, bx_vga_tminfo_t tm_info, unsigned rows) override {}
	virtual void    graphics_tile_update(u8* snapshot, unsigned x, unsigned y) override;
	virtual void    handle_events(void) override;
	virtual void    flush(void) override;
	virtual void    clear_screen(void) override;
	virtual bool    palette_change(unsigned index, unsigned red, unsigned green, unsigned blue) override;
	virtual void    dimension_update(unsigned x, unsigned y, unsigned fheight = 0, unsigned fwidth = 0, unsigned bpp = 8) override;
	virtual void    mouse_enabled_changed_specific(bool val) override;
	virtual void    exit(void) override;
	virtual			bx_svga_tileinfo_t* graphics_tile_info(bx_svga_tileinfo_t* info) override;
	virtual			u8* graphics_tile_get(unsigned x, unsigned y, unsigned* w, unsigned* h) override;
	virtual void    graphics_tile_update_in_place(unsigned x, unsigned y, unsigned w, unsigned h) override;
	void			graphics_frame_update(const u32* pixels, unsigned w, unsigned h) override;
private:
	CConfigurator* myCfg;
	unsigned int   vid_scale = 0;
	bool           vid_linear = true;
	bool           vid_scale_change_enable = false;
	void           reset_window_size();
	void           adjust_window_scale(int delta);
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
static int          last_driven_w = 0, last_driven_h = 0;
static int          runtime_scale_override = 0;  // 0 = inactive; >0 = use this integer scale
static const int    runtime_scale_min = 1;
static const int    runtime_scale_max = 8;
u8                  old_mousebuttons = 0, new_mousebuttons = 0;
int                 old_mousex = 0, new_mousex = 0;
int                 old_mousey = 0, new_mousey = 0;
static int          sdl_mouse_button_state = 0;
static bool         sdl_swallow_keys = false;
static bool         sdl_swallow_end_release = false;
static bool         sdl_swallow_home_release = false;
static bool         sdl_swallow_pageup_release = false;
static bool         sdl_swallow_pagedown_release = false;
static const char*  sdl_title = "ES40 Emulator - Ctrl+Alt+End sends Ctrl+Alt+Del - Ctrl+Alt+Home resets window";
static const char*  sdl_title_grabbed = "ES40 Emulator - Ctrl+F10 releases mouse - Ctrl+Alt+End sends Ctrl+Alt+Del - Ctrl+Alt+Home resets window";

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

	// load keymap for sdl
	if (myCfg->get_bool_value("keyboard.use_mapping", false))
	{
		bx_keymap->loadKeymap(convertStringToSDLKey);
	}

	this->vid_linear = myCfg->get_bool_value("video.linear", true);
	this->vid_scale = (int)myCfg->get_num_value("video.scale_ratio", true, 0);
	this->vid_scale_change_enable = myCfg->get_bool_value("video.scale_change_enable", false);

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

static u32 sdl_scan_to_bx_key(SDL_Scancode sym)
{
	switch (sym)
	{
	case SDL_SCANCODE_BACKSPACE:    return BX_KEY_BACKSPACE;
	case SDL_SCANCODE_TAB:          return BX_KEY_TAB;
	case SDL_SCANCODE_RETURN:       return BX_KEY_ENTER;
	case SDL_SCANCODE_PAUSE:        return BX_KEY_PAUSE;
	case SDL_SCANCODE_ESCAPE:       return BX_KEY_ESC;
	case SDL_SCANCODE_SPACE:        return BX_KEY_SPACE;
	case SDL_SCANCODE_APOSTROPHE:   return BX_KEY_SINGLE_QUOTE;
	case SDL_SCANCODE_COMMA:        return BX_KEY_COMMA;
	case SDL_SCANCODE_MINUS:        return BX_KEY_MINUS;
	case SDL_SCANCODE_PERIOD:       return BX_KEY_PERIOD;
	case SDL_SCANCODE_SLASH:        return BX_KEY_SLASH;

	case SDL_SCANCODE_0:            return BX_KEY_0;
	case SDL_SCANCODE_1:            return BX_KEY_1;
	case SDL_SCANCODE_2:            return BX_KEY_2;
	case SDL_SCANCODE_3:            return BX_KEY_3;
	case SDL_SCANCODE_4:            return BX_KEY_4;
	case SDL_SCANCODE_5:            return BX_KEY_5;
	case SDL_SCANCODE_6:            return BX_KEY_6;
	case SDL_SCANCODE_7:            return BX_KEY_7;
	case SDL_SCANCODE_8:            return BX_KEY_8;
	case SDL_SCANCODE_9:            return BX_KEY_9;

	case SDL_SCANCODE_SEMICOLON:    return BX_KEY_SEMICOLON;
	case SDL_SCANCODE_EQUALS:       return BX_KEY_EQUALS;

	case SDL_SCANCODE_LEFTBRACKET:  return BX_KEY_LEFT_BRACKET;
	case SDL_SCANCODE_BACKSLASH:    return BX_KEY_BACKSLASH;
	case SDL_SCANCODE_RIGHTBRACKET: return BX_KEY_RIGHT_BRACKET;
	case SDL_SCANCODE_GRAVE:        return BX_KEY_GRAVE;

	case SDL_SCANCODE_A:            return BX_KEY_A;
	case SDL_SCANCODE_B:            return BX_KEY_B;
	case SDL_SCANCODE_C:            return BX_KEY_C;
	case SDL_SCANCODE_D:            return BX_KEY_D;
	case SDL_SCANCODE_E:            return BX_KEY_E;
	case SDL_SCANCODE_F:            return BX_KEY_F;
	case SDL_SCANCODE_G:            return BX_KEY_G;
	case SDL_SCANCODE_H:            return BX_KEY_H;
	case SDL_SCANCODE_I:            return BX_KEY_I;
	case SDL_SCANCODE_J:            return BX_KEY_J;
	case SDL_SCANCODE_K:            return BX_KEY_K;
	case SDL_SCANCODE_L:            return BX_KEY_L;
	case SDL_SCANCODE_M:            return BX_KEY_M;
	case SDL_SCANCODE_N:            return BX_KEY_N;
	case SDL_SCANCODE_O:            return BX_KEY_O;
	case SDL_SCANCODE_P:            return BX_KEY_P;
	case SDL_SCANCODE_Q:            return BX_KEY_Q;
	case SDL_SCANCODE_R:            return BX_KEY_R;
	case SDL_SCANCODE_S:            return BX_KEY_S;
	case SDL_SCANCODE_T:            return BX_KEY_T;
	case SDL_SCANCODE_U:            return BX_KEY_U;
	case SDL_SCANCODE_V:            return BX_KEY_V;
	case SDL_SCANCODE_W:            return BX_KEY_W;
	case SDL_SCANCODE_X:            return BX_KEY_X;
	case SDL_SCANCODE_Y:            return BX_KEY_Y;
	case SDL_SCANCODE_Z:            return BX_KEY_Z;

	case SDL_SCANCODE_DELETE:       return BX_KEY_DELETE;

		// Keypad
	case SDL_SCANCODE_KP_0:         return BX_KEY_KP_INSERT;
	case SDL_SCANCODE_KP_1:         return BX_KEY_KP_END;
	case SDL_SCANCODE_KP_2:         return BX_KEY_KP_DOWN;
	case SDL_SCANCODE_KP_3:         return BX_KEY_KP_PAGE_DOWN;
	case SDL_SCANCODE_KP_4:         return BX_KEY_KP_LEFT;
	case SDL_SCANCODE_KP_5:         return BX_KEY_KP_5;
	case SDL_SCANCODE_KP_6:         return BX_KEY_KP_RIGHT;
	case SDL_SCANCODE_KP_7:         return BX_KEY_KP_HOME;
	case SDL_SCANCODE_KP_8:         return BX_KEY_KP_UP;
	case SDL_SCANCODE_KP_9:         return BX_KEY_KP_PAGE_UP;
	case SDL_SCANCODE_KP_PERIOD:    return BX_KEY_KP_DELETE;
	case SDL_SCANCODE_KP_DIVIDE:    return BX_KEY_KP_DIVIDE;
	case SDL_SCANCODE_KP_MULTIPLY:  return BX_KEY_KP_MULTIPLY;
	case SDL_SCANCODE_KP_MINUS:     return BX_KEY_KP_SUBTRACT;
	case SDL_SCANCODE_KP_PLUS:      return BX_KEY_KP_ADD;
	case SDL_SCANCODE_KP_ENTER:     return BX_KEY_KP_ENTER;

		// Arrows + Home/End pad
	case SDL_SCANCODE_UP:           return BX_KEY_UP;
	case SDL_SCANCODE_DOWN:         return BX_KEY_DOWN;
	case SDL_SCANCODE_RIGHT:        return BX_KEY_RIGHT;
	case SDL_SCANCODE_LEFT:         return BX_KEY_LEFT;
	case SDL_SCANCODE_INSERT:       return BX_KEY_INSERT;
	case SDL_SCANCODE_HOME:         return BX_KEY_HOME;
	case SDL_SCANCODE_END:          return BX_KEY_END;
	case SDL_SCANCODE_PAGEUP:       return BX_KEY_PAGE_UP;
	case SDL_SCANCODE_PAGEDOWN:     return BX_KEY_PAGE_DOWN;

		// Function keys
	case SDL_SCANCODE_F1:           return BX_KEY_F1;
	case SDL_SCANCODE_F2:           return BX_KEY_F2;
	case SDL_SCANCODE_F3:           return BX_KEY_F3;
	case SDL_SCANCODE_F4:           return BX_KEY_F4;
	case SDL_SCANCODE_F5:           return BX_KEY_F5;
	case SDL_SCANCODE_F6:           return BX_KEY_F6;
	case SDL_SCANCODE_F7:           return BX_KEY_F7;
	case SDL_SCANCODE_F8:           return BX_KEY_F8;
	case SDL_SCANCODE_F9:           return BX_KEY_F9;
	case SDL_SCANCODE_F10:          return BX_KEY_F10;
	case SDL_SCANCODE_F11:          return BX_KEY_F11;
	case SDL_SCANCODE_F12:          return BX_KEY_F12;

		// Modifier keys
	case SDL_SCANCODE_NUMLOCKCLEAR: return BX_KEY_NUM_LOCK;
	case SDL_SCANCODE_CAPSLOCK:     return BX_KEY_CAPS_LOCK;
	case SDL_SCANCODE_SCROLLLOCK:   return BX_KEY_SCRL_LOCK;
	case SDL_SCANCODE_RSHIFT:       return BX_KEY_SHIFT_R;
	case SDL_SCANCODE_LSHIFT:       return BX_KEY_SHIFT_L;
	case SDL_SCANCODE_RCTRL:        return BX_KEY_CTRL_R;
	case SDL_SCANCODE_LCTRL:        return BX_KEY_CTRL_L;
	case SDL_SCANCODE_RALT:         return BX_KEY_ALT_R;
	case SDL_SCANCODE_LALT:         return BX_KEY_ALT_L;
	case SDL_SCANCODE_LGUI:         return BX_KEY_WIN_L;
	case SDL_SCANCODE_RGUI:         return BX_KEY_WIN_R;

		// Misc function keys
	case SDL_SCANCODE_PRINTSCREEN:  return BX_KEY_PRINT;
	case SDL_SCANCODE_MENU:         return BX_KEY_MENU;

	default:
		BX_ERROR(("sdl3 scancode 0x%x not mapped", (unsigned)sym));
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

		case SDL_EVENT_WINDOW_RESTORED:
		case SDL_EVENT_WINDOW_DISPLAY_SCALE_CHANGED:
			// System DPI changed — re-scale SDL GUI window
			if (res_x > 0 && res_y > 0)
			{
				dimension_update(res_x, res_y);
			}
			break;

		case SDL_EVENT_MOUSE_MOTION:
			if (sdl_grab)
			{
				int dx = (int)sdl_event.motion.xrel;
				int dy = -(int)sdl_event.motion.yrel;

				if (dx != 0 || dy != 0)
				{
					theKeyboard->mouse_motion(dx, dy, 0, sdl_mouse_button_state);
				}
			}
			break;

		case SDL_EVENT_MOUSE_BUTTON_DOWN:
		case SDL_EVENT_MOUSE_BUTTON_UP:
		{
			if (!sdl_grab)
			{
				if (sdl_event.type == SDL_EVENT_MOUSE_BUTTON_DOWN
					&& sdl_event.button.button == SDL_BUTTON_LEFT)
				{
					bx_gui->mouse_enabled_changed(true);
				}
				break;
			}

			int bitmask = 0;
			switch (sdl_event.button.button)
			{
			case SDL_BUTTON_LEFT:   bitmask = 0x01; break;
			case SDL_BUTTON_RIGHT:  bitmask = 0x02; break;
			case SDL_BUTTON_MIDDLE: bitmask = 0x04; break;
			default: break;
			}

			if (sdl_event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
				sdl_mouse_button_state |= bitmask;
			else
				sdl_mouse_button_state &= ~bitmask;

			theKeyboard->mouse_motion(0, 0, 0, sdl_mouse_button_state);
			break;
		}

		case SDL_EVENT_MOUSE_WHEEL: // disabled for now, isn't working properly yet. 
/*			if (sdl_grab)
			{
				int dz = (int)sdl_event.wheel.y;  // positive = scroll up
				if (dz != 0)
				{
					theKeyboard->mouse_motion(0, 0, dz, sdl_mouse_button_state);
				}
			} */
			break;
		case SDL_EVENT_WINDOW_FOCUS_LOST:
		{
			if (sdl_grab)
				bx_gui->mouse_enabled_changed(false);
			break;
		}
		case SDL_EVENT_KEY_DOWN:
			if (sdl_event.key.key == SDLK_END &&
				(sdl_event.key.mod & SDL_KMOD_CTRL) &&
				(sdl_event.key.mod & SDL_KMOD_ALT))
			{
				theKeyboard->gen_scancode(BX_KEY_DELETE);
				theKeyboard->gen_scancode(BX_KEY_DELETE | BX_KEY_RELEASED);
				sdl_swallow_end_release = true;
				break;
			}

			// Ctrl+Alt+Home: reset window to last GPU-driven size
			if (sdl_event.key.key == SDLK_HOME &&
				(sdl_event.key.mod & SDL_KMOD_CTRL) &&
				(sdl_event.key.mod & SDL_KMOD_ALT))
			{
				reset_window_size();
				sdl_swallow_home_release = true;
				break;
			}

			// Ctrl+PageUp / Ctrl+PageDown: runtime scale adjust (gated by config)
			if (vid_scale_change_enable &&
				(sdl_event.key.mod & SDL_KMOD_CTRL) &&
				!(sdl_event.key.mod & SDL_KMOD_ALT))
			{
				if (sdl_event.key.key == SDLK_PAGEUP)
				{
					adjust_window_scale(+1);
					sdl_swallow_pageup_release = true;
					break;
				}
				if (sdl_event.key.key == SDLK_PAGEDOWN)
				{
					adjust_window_scale(-1);
					sdl_swallow_pagedown_release = true;
					break;
				}
			}

			// Ctrl+F10: toggle mouse capture
			if (sdl_event.key.key == SDLK_F10 && (sdl_event.key.mod & SDL_KMOD_CTRL))
			{
				theKeyboard->gen_scancode(BX_KEY_CTRL_L | BX_KEY_RELEASED);
				theKeyboard->gen_scancode(BX_KEY_CTRL_R | BX_KEY_RELEASED);

				bx_gui->mouse_enabled_changed(!sdl_grab);
				sdl_swallow_keys = true;  // eat subsequent releases
				break;
			}
#ifdef _WIN32
			extern void win32_select_file(HWND hwnd);
			if (sdl_event.key.key == SDLK_F11 && (sdl_event.key.mod & SDL_KMOD_CTRL))
			{
				theKeyboard->gen_scancode(BX_KEY_CTRL_L | BX_KEY_RELEASED);
				theKeyboard->gen_scancode(BX_KEY_CTRL_R | BX_KEY_RELEASED);
				
				if (sdl_grab)
					bx_gui->mouse_enabled_changed(false);

				win32_select_file((HWND)SDL_GetPointerProperty(SDL_GetWindowProperties(sdl_window), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));

				sdl_swallow_keys = true;  // eat subsequent releases
				break;
			}
#endif
			if (sdl_swallow_keys)
				break;  // swallow any key-down during toggle

			// Filter out ScrollLock (fullscreen toggle prev.) and invalid keys
			if (sdl_event.key.key == SDLK_SCROLLLOCK)
				break;

			// convert sym -> bochs code
			if (!myCfg->get_bool_value("keyboard.use_mapping", false))
			{
				key_event = sdl_scan_to_bx_key(sdl_event.key.scancode);
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

			if (sdl_swallow_end_release && sdl_event.key.key == SDLK_END)
			{
				sdl_swallow_end_release = false;
				break;
			}

			if (sdl_swallow_home_release && sdl_event.key.key == SDLK_HOME)
			{
				sdl_swallow_home_release = false;
				break;
			}

			if (sdl_swallow_pageup_release && sdl_event.key.key == SDLK_PAGEUP)
			{
				sdl_swallow_pageup_release = false;
				break;
			}

			if (sdl_swallow_pagedown_release && sdl_event.key.key == SDLK_PAGEDOWN)
			{
				sdl_swallow_pagedown_release = false;
				break;
			}

			if (sdl_swallow_keys)
			{
				// hanlde dealing with ctrl+f10 escape
				if (!(SDL_GetModState() & SDL_KMOD_CTRL))
					sdl_swallow_keys = false;
				break;  
			}

			if (!myCfg->get_bool_value("keyboard.use_mapping", false))
			{
				key_event = sdl_scan_to_bx_key(sdl_event.key.scancode);
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
			if (!sdl_grab)
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
	SDL_DisplayID display;
	float scaled_x, scaled_y;
	float content_scale = 1.0f;

	if (sdl_texture)
	{
		SDL_DestroyTexture(sdl_texture);
		sdl_texture = NULL;
	}

	if (!sdl_window)
		display = SDL_GetPrimaryDisplay();
	else
		display = SDL_GetDisplayForWindow(sdl_window);

	if (runtime_scale_override > 0)
		content_scale = (float)runtime_scale_override;
	else if (vid_scale)
		content_scale = (float)vid_scale;
	else
		content_scale = SDL_GetDisplayContentScale(display);

	scaled_x = x * content_scale;
	scaled_y = y * content_scale;

	if (!sdl_window)
	{
		sdl_window = SDL_CreateWindow(sdl_title,
			(int)scaled_x, (int)scaled_y, SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
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

		SDL_RaiseWindow(sdl_window);
		SDL_SetRenderLogicalPresentation(sdl_renderer,
			(int)x, (int)y,
			SDL_LOGICAL_PRESENTATION_LETTERBOX);
	}
	else
	{
		SDL_SetWindowSize(sdl_window, (int)scaled_x, (int)scaled_y);
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

	SDL_SetTextureScaleMode(sdl_texture, vid_linear ? SDL_SCALEMODE_LINEAR : SDL_SCALEMODE_NEAREST);

	res_x = x;
	res_y = y;
	half_res_x = x / 2;
	half_res_y = y / 2;

	// Remember the actual OS-pixel size we just drove the window to,
	// so Ctrl+Alt+Home can snap back here after a manual resize.
	last_driven_w = (int)scaled_x;
	last_driven_h = (int)scaled_y;
}

void bx_sdl_gui_c::reset_window_size()
{
	if (!sdl_window || last_driven_w == 0 || last_driven_h == 0)
		return;

	SDL_WindowFlags flags = SDL_GetWindowFlags(sdl_window);
	if (flags & (SDL_WINDOW_MAXIMIZED | SDL_WINDOW_FULLSCREEN))
		SDL_RestoreWindow(sdl_window);

	SDL_SetWindowSize(sdl_window, last_driven_w, last_driven_h);
}

void bx_sdl_gui_c::adjust_window_scale(int delta)
{
	// Snapshot the currently-effective integer scale.
	int current;
	if (runtime_scale_override > 0)
		current = runtime_scale_override;
	else if (vid_scale)
		current = (int)vid_scale;
	else if (sdl_window)
		current = (int)(SDL_GetDisplayContentScale(SDL_GetDisplayForWindow(sdl_window)) + 0.5f);
	else
		current = 1;

	int next = current + delta;
	if (next < runtime_scale_min) next = runtime_scale_min;
	if (next > runtime_scale_max) next = runtime_scale_max;
	if (next == runtime_scale_override) return;  // no change

	runtime_scale_override = next;

	if (sdl_window && res_x > 0 && res_y > 0)
	{
		SDL_WindowFlags flags = SDL_GetWindowFlags(sdl_window);
		if (flags & (SDL_WINDOW_MAXIMIZED | SDL_WINDOW_FULLSCREEN))
			SDL_RestoreWindow(sdl_window);
		dimension_update(res_x, res_y);
	}
}

void bx_sdl_gui_c::mouse_enabled_changed_specific(bool val)
{
	if (val)
	{
		SDL_HideCursor();
		if (sdl_window)
		{
			SDL_SetWindowKeyboardGrab(sdl_window, true);
			SDL_SetWindowRelativeMouseMode(sdl_window, true);
			SDL_SetWindowTitle(sdl_window, sdl_title_grabbed);
		}
	}
	else
	{
		SDL_ShowCursor();
		if (sdl_window)
		{
			SDL_SetWindowKeyboardGrab(sdl_window, false);
			SDL_SetWindowRelativeMouseMode(sdl_window, false);
			SDL_SetWindowTitle(sdl_window, sdl_title);
		}
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
