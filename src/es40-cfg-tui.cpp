/* ES40 emulator.
 * Copyright (C) 2026 by the ES40 Emulator Project
 * Copyright (C) 2025 by Kisara Development LLC.
 * All rights reserved.
 *
 * WWW    : https://github.com/ES40-Emu/es40
 *
 * SPDX-License-Identifier: BSD-1-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * Configuration file editor, TUI variant.
 * This still is work in progress!
 *
 * $Id$
 *
 * X-0.2        Martin Vorländer                                17-AUG-2028
 *      Changed header comment and license.
 * X-0.1        Martin Vorländer                                01-JUL-2026
 *      File created.
 **/

// TODO: When config has been read from a file, use those values in the forms instead of the defaults.
// TODO: If user does not enter a form, use the default values for that section (e.g. system settings)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <locale.h>

#include <sstream>
#include <string>
#include <vector>

using namespace std;

#include <pcap/pcap.h>

#ifdef __MINGW32__
// compile with -DNCURSES_STATIC to be able to link
#include <ncurses/ncurses.h>
#include <ncurses/panel.h>
#include << ncurses / menu.h>
#include << ncurses / form.h>
#else
#include <ncurses.h>
#include <panel.h>
#include <menu.h>
#include <form.h>
#endif

#include "StdAfx.h"
#include "Configurator.h"

#define ARRAY_SIZE(a) (int)(sizeof(a) / sizeof(a[0]))

// Low and high values for memory.bits parameter
#define LOW_MEM_BITS 25
#define HI_MEM_BITS 35

// y,x for menus
#define MENU_1ST_LEVEL 1, 4
#define MENU_2ND_LEVEL 4, 7
#define MENU_3RD_LEVEL 7, 10
#define MENU_4TH_LEVEL 10, 13

// Time to wait at the banner for a keypress
#define WAITSEC 5

// Keys
#define KEY_TAB 9
#define KEY_RETURN 10
#define KEY_DEL 127

// Menu item type
typedef void (*SelectFuncPtr)(const char *_title);
typedef struct MenuEntry_t
{
    const char *text;
    const char *description;
    SelectFuncPtr select_callback; // Routine to be called when item is selected
} MenuEntry_t;

// Form field type
typedef void (*ValidationFuncPtr)(FIELD *_field);
typedef struct FormEntry_t
{
    const char *label;
    const char *preset;
    const char *name; // configuration file attribute name
    const char *description;
    ValidationFuncPtr validation_callback; // Routine to be called to set up type and validation for field
} FormEntry_t;

// Type for array of values entered into form
typedef char **FormValues_t;

// The configuration
CConfigurator *theConfig, *sys0;

// (Re-)calculated height and width of screen
int scrh, scrw;

/**
 * Convert an integer to a string.
 **/
inline string i2s(int x)
{
    // From src/NumberQuestion.h
    ostringstream o;
    o << x;
    return o.str();
}

/**
 * Convert a string to an integer.
 *
 * Throws a CLogicException when the input is not numeric.
 **/
inline int s2i(const string x)
{
    // From src/NumberQuestion.h
    istringstream i(x);
    int x1;
    char c;
    if (!(i >> x1) || i.get(c))
        FAILURE(Logic, "invalid conversion");
    return x1;
}

/**
 * Signal handler for SIGWINCH
 **/
RETSIGTYPE resizeHandler(int sig)
{
    getmaxyx(stdscr, scrh, scrw);
    // TODO: recalculate the dimensions of all active windows using scrh, scrw
    //       and redisplay them
    wclear(stdscr);
    wrefresh(stdscr);
}

/**
 * Read theConfig from a file named 'filename'.
 */
void read_configuration(const char *filename)
{
    // From src/AlphaSim.cpp
    FILE *f;

    // Open chosen file (print the path on failure)
    f = fopen(filename, "rb");
    if (!f)
    {
        char buf[1024];
        snprintf(buf, sizeof(buf), "failed to open configuration file: %s (%s)",
                 filename, strerror(errno));
        FAILURE(Configuration, buf);
    }

    // Determine size (Windows: 64-bit length; POSIX: fseek/ftell)
    size_t ll1 = 0;
#if defined(_WIN32)
    if (_fseeki64(f, 0, SEEK_END) != 0)
    {
        char buf[1024];
        fclose(f);
        snprintf(buf, sizeof(buf), "failed to seek end of configuration file: %s", filename);
        FAILURE(Configuration, buf);
    }
    __int64 pos64 = _ftelli64(f);
    if (pos64 < 0)
    {
        char buf[1024];
        fclose(f);
        snprintf(buf, sizeof(buf), "failed to tell size of configuration file: %s", filename);
        FAILURE(Configuration, buf);
    }
    ll1 = (size_t)pos64;
    if (_fseeki64(f, 0, SEEK_SET) != 0)
    {
        char buf[1024];
        fclose(f);
        snprintf(buf, sizeof(buf), "failed to rewind configuration file: %s", filename);
        FAILURE(Configuration, buf);
    }
#else
    if (fseek(f, 0, SEEK_END) != 0)
    {
        char buf[1024];
        fclose(f);
        snprintf(buf, sizeof(buf), "failed to seek end of configuration file: %s", filename);
        FAILURE(Configuration, buf);
    }
    long pos = ftell(f);
    if (pos < 0)
    {
        char buf[1024];
        fclose(f);
        snprintf(buf, sizeof(buf), "failed to tell size of configuration file: %s", filename);
        FAILURE(Configuration, buf);
    }
    ll1 = (size_t)pos;
    if (fseek(f, 0, SEEK_SET) != 0)
    {
        char buf[1024];
        fclose(f);
        snprintf(buf, sizeof(buf), "failed to rewind configuration file: %s", filename);
        FAILURE(Configuration, buf);
    }
#endif

    // Allocate buffer (+1 for NUL), read, and NUL-terminate for safety
    char *ch1 = (char *)calloc(ll1 + 1, 1);
    if (!ch1)
    {
        char buf[1024];
        fclose(f);
        snprintf(buf, sizeof(buf), "out of memory reading configuration file: %s", filename);
        FAILURE(Configuration, buf);
    }

    size_t read_bytes = fread(ch1, 1, ll1, f);
    if (read_bytes != ll1)
    {
        char buf[1024];
        fclose(f);
        free(ch1);
        snprintf(buf, sizeof(buf), "failed to read entire configuration file: %s", filename);
        FAILURE(Configuration, buf);
    }
    ch1[ll1] = '\0';

    theConfig = new CConfigurator(nullptr, NULL, NULL, ch1, ll1);
    fclose(f);
    free(ch1);
}

/**
 * Write theConfig to a file. If filename is '-', write to stdout.
 */
void write_configuration(const char *filename)
{
    FILE *f;

    if (!strcmp(filename, "-"))
    {
        f = stdout;
    }
    else
    {
        f = fopen(filename, "wb");
        if (!f)
            FAILURE_2(Configuration, "failed to open configuration file: %s (%s)",
                      filename, strerror(errno));
    }
    theConfig->write_configuration(f);
    fclose(f);
}

/**
 * Update the columns setting 'c' to be big enough to hold string 's'
 **/
inline void set_min_width(int *c, const char *s)
{
    if (s == NULL)
        return;
    if ((int)strlen(s) + 4 > *c)
        *c = strlen(s) + 4;
}

/**
 * Display 'text' in row 'y' of window 'win'
 * centered to maximum width 'maxw'.
 **/
void mvwprintw_center(WINDOW *win, int y, int maxw, const char *text)
{
    int len = (int)strlen(text);
    if (len > maxw - 4)
        len = maxw - 4; // truncate rather than break the border

    int left_x = (maxw - len + 1) / 2;

    mvwprintw(win, y, left_x, text);
}

/**
 * Create a new window of 'nLines' rows and at least 'nCols' columns
 * with the upper left at 'begin_y', 'begin_x'.
 * Draw a border around it with 'title' in the first row,
 * and 'helptext' in the last row.
 * Widen the window by increasing 'nCols' to accomodate 'title' and 'helptext'.
 * If 'begin_x' and 'begin_y' are both -1, center the window on the screen.
 **/
WINDOW *create_window(int nLines, int nCols, int begin_y, int begin_x, const char *title, const char *helptext)
{
    if (title != NULL)
        set_min_width(&nCols, title);
    if (helptext != NULL)
        set_min_width(&nCols, helptext);

    if (begin_y == -1 && begin_x == -1)
    {
        begin_y = (scrh - nLines - 2) / 2;
        begin_x = (scrw - nCols - 2) / 2;
    }

    WINDOW *my_win = newwin(nLines + 2, nCols + 2, begin_y, begin_x);
    box(my_win, 0, 0);
    keypad(my_win, TRUE);

    if (title != NULL)
        mvwprintw_center(my_win, 0, nCols, title);
    if (helptext != NULL)
        mvwprintw(my_win, nLines + 1, nCols - strlen(helptext), helptext);

    return my_win;
}

/**
 * Display a centered window with a 'title' containing the 'text'.
 * Wait for a key, and then close the window.
 * 'text' may be multiple lines, separated by '\n'.
 **/
void show_text(const char *title, const char *text)
{
    if (text == NULL)
        return;

    const char *helptext = "Any key to close";

    int nLines = 1;
    int nCols = 1;
    char *linebuffer = strdup(text);
    char *p, *q;

    if (strchr(linebuffer, '\n') == NULL)
        set_min_width(&nCols, linebuffer);
    else
        for (p = linebuffer; (q = strchr(p, '\n')) != NULL; p = q + 1)
        {
            ++nLines;
            *q = '\0';
            set_min_width(&nCols, p);
        }

    WINDOW *my_win = create_window(nLines, nCols, -1, -1, title, helptext);
    PANEL *my_panel = new_panel(my_win);

    p = linebuffer;
    for (int i = 1; i <= nLines; ++i)
    {
        mvwprintw(my_win, i, 1, p);
        p += strlen(p) + 1; // Skip NUL terminating part
    }

    update_panels();
    doupdate();

    wgetch(my_win);

    // Clean up
    del_panel(my_panel);
    wclear(my_win);
    wrefresh(my_win);
    delwin(my_win);
    update_panels();
    doupdate();

    free(linebuffer);
}

/**
 * Display a menu of 'num_entries' menu 'entry's in a window
 * with upper left corner at row 'begin_y', column 'begin_x'.
 *
 * React to keys pressed:
 * <UP>, <DOWN>: go to previous/next item
 * <ENTER>: if entry's callback is NULL: exit menu, else call callback
 * <F1>: Display entry's description
 * <F2>: exit menu
 *
 * Returns the index of the last item selected,
 * or -1 if leaving via <F2>.
 **/
int show_menu(const char *title, MenuEntry_t entry[], int num_entries, int begin_y, int begin_x)
{
    const char *helptext = "F1 for help, F2 to Exit";

    int nLines = num_entries;
    int nCols = 0;

    for (int i = 0; i < num_entries; ++i)
        set_min_width(&nCols, entry[i].text);

    WINDOW *my_win = create_window(nLines, nCols, begin_x, begin_y, title, helptext);
    PANEL *my_panel = new_panel(my_win);
    update_panels();

    ITEM **my_item = (ITEM **)calloc(num_entries + 1, sizeof(ITEM *));
    for (int i = 0; i < num_entries; ++i)
        my_item[i] = new_item(entry[i].text, NULL);
    my_item[num_entries] = (ITEM *)NULL;

    MENU *my_menu = new_menu(my_item);
    set_menu_win(my_menu, my_win);
    set_menu_sub(my_menu, derwin(my_win, nLines, nCols, 1, 1));
    post_menu(my_menu);
    update_panels();
    doupdate();

    bool stay_in_loop = TRUE;
    int cur_idx = 0;
    while (stay_in_loop)
    {
        ITEM *cur = current_item(my_menu);
        cur_idx = item_index(cur);
        switch (wgetch(my_win))
        {
        case KEY_F(1):
            show_text(entry[cur_idx].text, entry[cur_idx].description);
            break;
        case KEY_F(2):
            cur_idx = -1;
            stay_in_loop = FALSE;
            break;
        case KEY_DOWN:
            menu_driver(my_menu, REQ_DOWN_ITEM);
            break;
        case KEY_UP:
            menu_driver(my_menu, REQ_UP_ITEM);
            break;
        case KEY_RETURN:
            SelectFuncPtr callback;

            callback = entry[cur_idx].select_callback;
            if (callback == NULL)
                stay_in_loop = FALSE;
            else
            {
                callback(item_name(cur));
                pos_menu_cursor(my_menu);
            }
            break;
        default:
            break;
        }
    }

    // Clean up
    unpost_menu(my_menu);
    free_menu(my_menu);
    del_panel(my_panel);
    wclear(my_win);
    wrefresh(my_win);
    delwin(my_win);
    update_panels();
    doupdate();

    for (int i = 0; i < num_entries; ++i)
        free_item(my_item[i]);
    free(my_item);

    return cur_idx;
}

/**
 * Display a form of 'num_entries' form 'entry's
 * in a centered window with a 'title'.
 *
 * React to keys pressed:
 * <UP>, <DOWN>, <ENTER>: go to previous/next/next field
 * <TAB>, Shift-<TAB>: in enum fields, go to next/previous choice
 * <F1>: Display field help
 * <F2>: exit form
 * any other printable character is put inin the current field
 *
 * The entered values are returned in a 'num_entries' wide array.
 * The calling routine is responsible for freeing values' storage.
 *
 **/
FormValues_t show_form(const char *title, FormEntry_t entry[], int num_entries)
{
    const char *helptext = "F1 for help, F2 to Exit";

    int nLines = num_entries * 2; // label + field
    int nCols = scrw - 20;

    for (int i = 0; i < num_entries; ++i)
    {
        set_min_width(&nCols, entry[i].label);
        set_min_width(&nCols, entry[i].preset);
    }

    WINDOW *my_win = create_window(nLines + 1, nCols + 1, -1, -1, title, helptext);
    PANEL *my_panel = new_panel(my_win);
    update_panels();

    FIELD **my_fld = (FIELD **)calloc(num_entries + 1, sizeof(FIELD *));
    for (int i = 0; i < num_entries; ++i)
    {
        // Coordinates relative to the form's derwin() window!
        my_fld[i] = new_field(1, nCols - 1, 2 * i + 1, 1, FALSE, 0);
        if (entry[i].preset != NULL)
            set_field_buffer(my_fld[i], 0, entry[i].preset);
        set_field_back(my_fld[i], A_UNDERLINE);
        field_opts_off(my_fld[i], O_STATIC);   // Use a dynamic length field ...
        set_max_field(my_fld[i], 256);         // ... of at most 256
        field_opts_off(my_fld[i], O_AUTOSKIP); // Don't skip to next field when running off the end

        ValidationFuncPtr set_field_validation = entry[i].validation_callback;
        if (set_field_validation != NULL)
            set_field_validation(my_fld[i]);
    }
    my_fld[num_entries] = NULL;

    FORM *my_form = new_form(my_fld);
    set_form_win(my_form, my_win);
    set_form_sub(my_form, derwin(my_win, nLines, nCols, 1, 1));
    post_form(my_form);

    for (int i = 0; i < num_entries; ++i)
        mvwprintw(my_win, 2 * i + 1, 1, entry[i].label);

    form_driver(my_form, REQ_FIRST_FIELD);
    form_driver(my_form, REQ_END_FIELD);
    update_panels();
    doupdate();

    bool stay_in_loop = TRUE;
    while (stay_in_loop)
    {
        FIELD *cur_field = current_field(my_form);
        int cur_idx = field_index(cur_field);
        switch (int c = wgetch(my_win))
        {
        case KEY_F(1):
            show_text(entry[cur_idx].label, entry[cur_idx].description);
            break;
        case KEY_F(2):
            stay_in_loop = FALSE;
            break;
        case KEY_TAB:
            if (field_type(cur_field) == TYPE_ENUM)
            {
                form_driver(my_form, REQ_NEXT_CHOICE);
                form_driver(my_form, REQ_END_LINE);
            }
            break;
        case KEY_STAB: // Shift-<TAB> ?
            if (field_type(cur_field) == TYPE_ENUM)
            {
                form_driver(my_form, REQ_PREV_CHOICE);
                form_driver(my_form, REQ_END_LINE);
            }
            break;
        case KEY_DOWN:
        case KEY_RETURN:
            form_driver(my_form, REQ_NEXT_FIELD);
            form_driver(my_form, REQ_END_LINE);
            break;
        case KEY_UP:
            form_driver(my_form, REQ_PREV_FIELD);
            form_driver(my_form, REQ_END_LINE);
            break;
        case KEY_LEFT:
            form_driver(my_form, REQ_PREV_CHAR);
            break;
        case KEY_RIGHT:
            form_driver(my_form, REQ_NEXT_CHAR);
            break;
        case KEY_BACKSPACE: // Ctrl-H
        case KEY_DEL:
            if (field_type(cur_field) != TYPE_ENUM)
                form_driver(my_form, REQ_DEL_PREV);
            break;
        case KEY_UNDO: // AFAIK, this key is not configured in vt100's terminfo
            if (entry[cur_idx].preset != NULL)
            {
                set_field_buffer(cur_field, 0, entry[cur_idx].preset);
                form_driver(my_form, REQ_END_LINE);
            }
            break;
        default:
            if (field_type(cur_field) != TYPE_ENUM && isprint(c))
                form_driver(my_form, c);
            break;
        }
    }

    // Make sure all validation routines have ended before retrieving values
    form_driver(my_form, REQ_VALIDATION);

    FormValues_t values = (FormValues_t)calloc(num_entries, sizeof(char *));
    for (int i = 0; i < num_entries; ++i)
    {
        char *buf = field_buffer(my_fld[i], 0);
        size_t len = strlen(buf);
        while (len > 0 && buf[--len] == field_pad(my_fld[i]))
            ;
        values[i] = (char *)malloc(len + 1);
        strncpy(values[i], buf, len + 1);
    }

    // Clean up
    unpost_form(my_form);
    free_form(my_form);
    del_panel(my_panel);
    wclear(my_win);
    wrefresh(my_win);
    delwin(my_win);
    update_panels();
    doupdate();

    for (int i = 0; i < num_entries; ++i)
        free_field(my_fld[i]);
    free(my_fld);

    return values;
}

/**
 * Display the ES40 banner in a centered window,
 * then wait at most 5 seconds for a keypress.
 **/
void es40_banner(const char *title)
{
    const char *text[] = {
        title,
        "Version " VERSION,
        "",
        "Copyright (C) 2007-2025 by the ES40 Emulator Project & Others",
        "Website: https://github.com/ES40-Emu/es40/",
        "",
        "This program is free software; you can redistribute it and/or",
        "modify it under the terms of the GNU General Public License",
        "as published by the Free Software Foundation; either version 2",
        "of the License, or (at your option) any later version."};
    int num_entries = ARRAY_SIZE(text);
    const char *helptext = "Any key to continue";

    int nLines = num_entries;
    int nCols = 0;

    for (int i = 0; i < num_entries; ++i)
        set_min_width(&nCols, text[i]);

    string helptext_dur = string(helptext) + " (" + i2s(WAITSEC) + ")";
    WINDOW *my_win = create_window(nLines, nCols, -1, -1, NULL, helptext_dur.c_str());
    PANEL *my_panel = new_panel(my_win);

    for (int i = 0; i < num_entries; ++i)
        mvwprintw_center(my_win, i + 1, nCols, text[i]);

    update_panels();
    doupdate();

    // FIXME: Why does this not work as intended (counting down)?
    const int factor = 10;
    const timespec req = {0, 1000L * 1000 * 1000 / factor}; // = 100,000,000 ns = 0.1 seconds
    int count = WAITSEC * factor;
    int maxx, maxy;
    getmaxyx(my_win, maxy, maxx);
    nodelay(my_win, TRUE);
    while (--count > factor && wgetch(my_win) == ERR)
    {
        nanosleep(&req, NULL);
        if (count % factor == 0)
        {
            string dur = string("(") + i2s(count / factor) + ")";
            mvwprintw(my_win, maxy, maxx - strlen(dur.c_str()), dur.c_str());
            update_panels();
            doupdate();
        }
    }
    nodelay(my_win, FALSE);

    // Clean up
    del_panel(my_panel);
    wclear(my_win);
    wrefresh(my_win);
    delwin(my_win);
    update_panels();
    doupdate();
}

CConfigurator *get_pcislot(const char *bus_slot)
{
    return sys0->find_child((string("pci") + bus_slot).c_str());
}

CConfigurator *get_pcislot(int bus, int slot)
{
    return get_pcislot((i2s(bus) + "." + i2s(slot)).c_str());
}

bool is_pcislot_free(int bus, int slot)
{
    return (get_pcislot(bus, slot) == nullptr);
}

const char *find_first_free_pcislot(void)
{
    for (int bus = 0; bus <= 1; ++bus)
        for (int slot = 1; slot < ((bus == 0) ? 4 : 6); ++slot)
            if (is_pcislot_free(bus, slot))
            {
                static string bus_slot = i2s(bus) + "." + i2s(slot);
                return bus_slot.c_str();
            }
    return NULL;
}

char *device_in_pcislot(int bus, int slot)
{
    string pcislot = string("pci") + i2s(bus) + "." + i2s(slot);
    CConfigurator *c = sys0->find_child(pcislot.c_str());
    if (c == nullptr)
        return (char *)"-";
    else
        return c->get_myValue();
}

/**
 * General validation routines
 **/

void validation_bool(FIELD *field)
{
    const char *choices[] = {
        "no",  // "0", "false"
        "yes", // "1", "true"
        NULL};
    set_field_type(field, TYPE_ENUM, choices, FALSE, TRUE);
}

void validation_datetime(FIELD *field)
{
    // Preceeding and trailing optional blanks added for trimming the field,
    // see form_field_validation(3X) manpage
    set_field_type(field, TYPE_REGEXP,
                   "^ *"
                   "[0-8]{4}-[0-8]{2}-[0-8]{2}"
                   "( [0-8]{2}:[0-8]{2}:[0-8]{2})?"
                   " *$");
}

void validation_file(FIELD *field)
{
    /* NYI */
}

void validation_mac(FIELD *field)
{
    set_field_type(field, TYPE_REGEXP,
                   "^ *"
                   "[0-9A-Fa-f]{2}(-[0-9A-Fa-f]{2}){5}"
                   " *$");
}

void validation_pcislot(FIELD *field)
{
    vector<const char *> choices;
    for (int bus = 0; bus <= 1; ++bus)
        for (int slot = 1; slot <= ((bus == 0) ? 4 : 6); ++slot)
            if (is_pcislot_free(bus, slot))
            {
                string o = i2s(bus) + "." + i2s(slot);
                choices.push_back(o.c_str());
            }
    choices.push_back(NULL);

    set_field_type(field, TYPE_ENUM, choices.data(), FALSE, TRUE);
}

/**
 * General helper routines
 **/

int fentry_index(FormEntry_t *entry, int num_entries, const char *name)
{
    for (int i = 0; i < num_entries; ++i)
        if (!strcmp(entry[i].label, name))
            return i;

    FAILURE_1(Configuration, "Entry '%s not found", name);
}

void validation_disk_type(FIELD *field)
{
    const char *choices[] = {
        "file",
        "device",
        "ramdisk",
        NULL};
    set_field_type(field, TYPE_ENUM, choices, FALSE, TRUE);
}

void validation_disk_hd_cd(FIELD *field)
{
    const char *choices[] = {
        "disk",
        "cd-rom",
        NULL};
    set_field_type(field, TYPE_ENUM, choices, FALSE, TRUE);
}

void validation_disk_autocreate_size(FIELD *field)
{
    // Preceeding and trailing optional blanks added for trimming the field,
    // see form_field_validation(3X) manpage
    // (..)? to allow leaving field empty
    set_field_type(field, TYPE_REGEXP, "^ *([0-8]+[KMG] *)?$");
}

/**
 * Add disks for a controller to the configuration file.
 **/
FormValues_t add_disks(const char *title, const char *disk_name, CConfigurator *parent)
{
    FormEntry_t entry[] = {
        {"Type", "file", "type",
         "Disks can be emulated in several ways.",
         validation_disk_type},
        {"File / Device name", "", NULL, // "file" | "device",
         "Enter the path to the file or device to use for this disk.\n"
         "This parameter is ignored for type 'ramdisk'.",
         validation_file},
        {"Harddisk or CD-ROM", "disk", "cdrom",
         "Do you want the OS to see this as a hard-disk, or as a cd-rom?\n"
         "This parameter is ignored for floppy drives, as they always are disks.",
         validation_disk_hd_cd},
        {"Autocreate size", "", NULL, // "autocreate_size" | "size"
         "For type 'file': Do you want the program to create it? And in what size?\n"
         "Leave blank to not autocreate the file.\n"
         "For type 'ramdisk': Required to be non-empty.\n"
         "For type 'device': Ignored."
         "The file/ramdisk will be created the first time the emulator runs.\n"
         "Format: number followed by K (kilo), M (mega), or G (giga)",
         validation_disk_autocreate_size},
        {"Read-only?", "no", "read_only",
         "Should the disk be set to read-only?\n"
         "This parameter is ignoed for CD-ROMs (always true),\n"
         "and for ramdisks (always false)",
         validation_bool},
        {"Disk model number", "", "model_number",
         "Would you like to set a disk model number?",
         NULL},
        {"Revision number", "", "rev_number",
         "Would you like to set a revision number?",
         NULL},
        {"Serial number", "", "serial_number",
         "Would you like to set a serial number?",
         NULL}};
    int num_entries = ARRAY_SIZE(entry);
    int idx;

    FormValues_t values = show_form(title, entry, num_entries);

    char *disk_type = values[fentry_index(entry, num_entries, "type")];
    if ((!strcmp(disk_type, "file") || !strcmp(disk_type, "device")) &&
        !strcmp(values[fentry_index(entry, num_entries, "File / Device name")], ""))
    {
        // TODO: Invalid - show message, return to form
    }
    idx = fentry_index(entry, num_entries, "Harddisk or CD-ROM");
    bool is_cdrom = !strcmp(values[idx], "cd-rom");
    if (!strcmp(disk_type, "floppy") && is_cdrom)
        values[idx] = (char *)"disk";
    idx = fentry_index(entry, num_entries, "Read-only?");
    if (!strcmp(disk_type, "ramdisk"))
        values[idx] = (char *)"no";
    if (is_cdrom)
        values[idx] = (char *)"yes";

    CConfigurator *c = new CConfigurator(parent, (char *)disk_name, (char *)disk_type);
    for (int i = 0; i < num_entries; ++i)
        if (entry[i].name != NULL)
            c->set_value(strdup(entry[i].name), strdup(values[i]));
        else
        {
            if (!strcmp(entry[i].label, "File / Device name") &&
                (!strcmp(disk_type, "file") || !strcmp(disk_type, "device")))
                c->set_value(disk_type, values[i]);
            if (!strcmp(entry[i].label, "Autocreate size") && strcmp(values[i], ""))
            {
                if (!strcmp(disk_type, "ramdisk"))
                    c->set_value(strdup("size"), strdup(values[i]));
                if (!strcmp(disk_type, "file"))
                    c->set_value(strdup("autocreate_size"), strdup(values[i]));
            }
        }

    return values;
}

/**
 * GUI configuration
 **/

#ifdef HAVE_SDL
void validation_gui_sdl_mousespeed(FIELD *field)
{
    // FIXME: Never can get out of ths field after enteing 0.5 ?!
    set_field_type(field, TYPE_NUMERIC, 1, 0.1, 10.0);
}

void validation_gui_sdl_scaleratio(FIELD *field)
{
    const char *choices[11 + 1];
    choices[0] = "auto";
    for (int i = 1; i <= 10; ++i)
        choices[i] = i2s(i).c_str();
    choices[11] = NULL;

    set_field_type(field, TYPE_ENUM, choices, FALSE, TRUE);
}

void validation_gui_sdl_linear(FIELD *field)
{
    const char *choices[] = {
        "nearest",
        "bilinear",
        NULL};
    set_field_type(field, TYPE_ENUM, choices, FALSE, TRUE);
}

// GUI SDL form
void edit_gui_sdl(const char *title)
{
    FormEntry_t entry[] = {
        {"keyboard use mapping?", "no", "keyboard.use_mapping",
         "Use a keymap file to translate host keys to guest scancodes.\n"
         "Enable this if you use a non-US keyboard layout.\n"
         "If set to 'yes', keyboard.map must be filled out.",
         validation_bool},
        {"keyboard map", "keys.map", "keyboard.map",
         "The keymap file to use when keyboard.use_mapping is enabled.",
         validation_file},
        {"mouse.speed favtor", "1.0", "mouse.speed",
         "Multiplier applied to host mouse motion before it is passed to the guest.\n"
         "Use a value below 1.0 to slow the guest pointer down (e.g. 0.5 for half speed),\n"
         "or above 1.0 (up to 10.0) to speed it up.",
         validation_gui_sdl_mousespeed},
        {"mouse invert x?", "no", "mouse.invert_x",
         "Reverse the direction of host mouse motion on the horizontal axis.",
         validation_bool},
        {"mouse invert y?", "no", "mouse.invert_y",
         "Reverse the direction of host mouse motion on the vertical axis.",
         validation_bool},
        {"video linear", "bilinear", "video.linear",
         "This affects the resized display output. 'nearest' looks pixel-y but harsh,\n"
         "while 'bilinear' does not look as harsh.",
         validation_gui_sdl_linear},
        {"video scale ratio", "auto", "video.scale_ratio",
         "The display output is scaled automatically based on system DPI by default.\n"
         "Select 'auto' or how many times the display should be scaled (1..10).",
         validation_gui_sdl_scaleratio},
        {"video scale change enable?", "no", "video.scale_change_enable",
         "If enabled, the display scale ratio can be adjusted on the fly\n"
         "while the emulator is running, without restarting.\n"
         "The change is not persisted back to the config file.\n"
         "The runtime defaults are:\n"
         "  Ctrl+PageUp   - increase scale by 1 (clamped at 8x)\n"
         "  Ctrl+PageDown - decrease scale by 1 (clamped at 1x)\n"
         "Optional overrides to the keys used can be configured directly in the config file.\n"
         "These hotkey bindings take effect only\n"
         "when runtime display scale changes are enabled.",
         validation_bool}
        // TODO: Implement editing of hotkey.*
    };
    const int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    if (!strcmp(values[fentry_index(entry, num_entries, "keyboard use mapping?")], "yes") &&
        !strcmp(values[fentry_index(entry, num_entries, "keyboard map")], ""))
    {
        // TODO: Invalid - show message, return to form
    }

    CConfigurator *c = theConfig->find_child("gui");
    if (c != nullptr)
    {
        // TODO: remove old gui config from theConfig
    }
    c = new CConfigurator(theConfig, (char *)"gui", (char *)"sdl");

    for (int i = 0; i < num_entries; ++i)
        c->set_value(strdup(entry[i].name), strdup(values[i]));

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}
#endif // HAVE_SDL

#ifdef HAVE_X11
void edit_gui_x11(const char *title)
{
    FormEntry_t entry[] = {
        {"keyboard use mapping?", "no", "keyboard.use_mapping?",
         "Use a keymap file to translate host keys to guest scancodes.\n"
         "Enable this if you use a non-US keyboard layout.\n"
         "If set to 'yes', keyboard.map must be filled out.",
         validation_bool},
        {"keyboard map", "keys.map", "keyboard.map",
         "The keymap file to use when keyboard.use_mapping is enabled.",
         validation_file},
        {"private colormap?", "no", "private_colormap?", "Use a private colormap?", validation_bool}};
    const int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    if (!strcmp(values[fentry_index(entry, num_entries, "keyboard use mapping?")], "yes") &&
        !strcmp(values[fentry_index(entry, num_entries, "keyboard map")], ""))
    {
        // TODO: Invalid - show message, return to form
    }

    CConfigurator *c = theConfig->find_child("gui");
    if (c != nullptr)
    {
        // TODO: remove old gui config from theConfig
    }
    c = new CConfigurator(theConfig, (char *)"gui", (char *)"x11");

    for (int i = 0; i < num_entries; ++i)
    {
        c->set_value(strdup(entry[i].name), strdup(values[i]));
    }

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}
#endif

// GUI menu
void edit_gui(const char *title)
{
    MenuEntry_t entry[] = {
        {"none", "No GUI. Graphics cards are not supported.", NULL}
#if defined(HAVE_SDL)
        ,
        {"sdl", "Simple Directmedia Layer. Preferred GUI mechanism.", edit_gui_sdl}
#endif
#if 0 && defined(HAVE_X11)
        ,
        {"X11", "Unix X-Windows GUI support.", edit_gui_x11}
#endif
#if 0 && defined(_WIN32)
        ,
        {"win32", "Windows 32 GUI support.", edit_gui_win32}
#endif
    };
    int num_entries = ARRAY_SIZE(entry);

    if (num_entries == 1)
    {
        show_text(title, "Sorry, the GUI is not available! (no SDL support found).");
        return;
    }

    int sel = show_menu(title, entry, num_entries, MENU_2ND_LEVEL);

    if (sel == 0)
    {
        CConfigurator *c = theConfig->find_child("gui");
        if (c != nullptr)
        {
            // TODO: remove gui config from theConfig
        }
    }
}

/**
 * Tsunami system settings configuration
 **/

void validation_tsunami_memory(FIELD *field)
{
    char *choices[HI_MEM_BITS + 1 - LOW_MEM_BITS + 1];
    for (int i = LOW_MEM_BITS; i <= HI_MEM_BITS; ++i)
    {
        char a = '\0';
        int j = i;
        if (i < 30)
        {
            // Megabyte-range.
            j -= 20;
            a = 'M';
        }
        else
        {
            // Gigabyte range.
            j -= 30;
            a = 'G';
        }
        string o = i2s(1 << j) + a;
        choices[i - LOW_MEM_BITS] = strdup(o.c_str());
    }
    choices[HI_MEM_BITS + 1 - LOW_MEM_BITS] = NULL;

    set_field_type(field, TYPE_ENUM, choices, FALSE, TRUE);
}

// Tsunami form
void edit_tsunami(const char *title)
{
    FormEntry_t entry[] = {
        {"rom.srm file",
#if defined(_WIN32)
         "rom\\cl67srmrom.exe",
#elif defined(__VMS)
         "[.ROM]CL67SRMROM.EXE",
#else
         "rom/cl67srmrom.exe",
#endif
         "rom.srm",
         "Location of the SRM ROM image.\n"
         "This file is required.",
         validation_file},
        {"rom.flash file",
#if defined(_WIN32)
         "rom\\flash.rom",
#elif defined(__VMS)
         "[.ROM]FLASH.ROM",
#else
         "rom/flash.rom",
#endif
         "rom.flash",
         "Location of the Flash ROM image",
         validation_file},
        {"rom.dpr file",
#if defined(_WIN32)
         "rom\\dpr.rom",
#elif defined(__VMS)
         "[.ROM]DPR.ROM",
#else
         "rom/dpr.rom",
#endif
         "rom.dpr",
         "Location of the DPR EEPROM image.",
         validation_file},
        {"rom.toy file",
#if defined(_WIN32)
         "rom\\toy.rom",
#elif defined(__VMS)
         "[.ROM]TOY.ROM",
#else
         "rom/toy.rom",
#endif
         "rom.toy",
         "Location of the CMOS/TOY NVRAM image.\n"
         "Preserves SRM CMOS settings (such as heap_expand)\n"
         "across emulator restarts, like the battery-backed\n"
         "CMOS on real hardware.",
         validation_file},
        {"memory size", "256M", NULL, // "memory.bits"
         "Amount of RAM memory.\n"
         "Your system should have enough free memory\n"
         "to emulate the amount you choose here.",
         validation_tsunami_memory},
        {"fixed date", "", "time",
         "Set a fixed date and time when the VM starts.\n"
         "By default (empty value), the VM's date and time is initialized\n"
         "to the current host date and time at startup.\n"
         "You can set a fixed date and time instead,\n"
         "Format: 'YYYY-MM-DD' or 'YYYY-MM-DD HH:MM:SS'",
         validation_datetime},
        {"arc_year_compat?", "no", "arc_year_compat",
         "Should the reported year be compatible with Windows?\n"
         "This only affects the year reported to the guest.\n"
         "Select 'yes' if you are planning to run Windows OSes.",
         validation_bool},
        {"Exit on PAL_halt?", "no", "exit_on_pal_halt",
         "Should the VM power off on the guest's request?",
         validation_bool}};
    const int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    // Convert memory size to memory bits (assumes power of 2)
    int idx = fentry_index(entry, num_entries, "memory size");
    char unit = values[idx][strlen(values[idx]) - 1];
    u64 amount = atoi(values[idx]);
    switch (unit)
    {
    case 'M':
        amount *= 1024 * 1024;
        break;
    case 'G':
        amount *= 1024 * 1024 * 1024;
        break;
    default:
        FAILURE_1(Configuration, "Unrecognized memory ammout: '%s", values[idx]);
        break;
    }
    int mem_bits = 0;
    while ((amount & 1) == 0 && amount > 0)
    {
        ++mem_bits;
        amount >>= 1;
    }
    sys0->set_value(strdup("memory.bits"), strdup(i2s(mem_bits).c_str()));

    for (int i = 0; i < num_entries; ++i)
        if (entry[i].name != NULL)
            sys0->set_value(strdup(entry[i].name), strdup(values[i]));

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}

/**
 * EV68 CPU configuration
 **/

void validation_ev68cb_cpu0_enabled(FIELD *field)
{
    const char *choices[] = {"yes", NULL};
    set_field_type(field, TYPE_ENUM, choices, FALSE, TRUE);
}

void validation_ev68cb_cpuspeed(FIELD *field)
{
    set_field_type(field, TYPE_INTEGER, 2, 10, 1250);
}

// EV68CB form
void edit_ev68cb(const char *title)
{
    vector<FormEntry_t> entry; // (enabled, speed [,nohle]) * 4 CPUs

    for (int i = 0; i < 4; ++i)
    {
        const string enabled = "cpu" + i2s(i) + " enabled?";
        if (i == 0)
            entry.push_back({strdup(enabled.c_str()), "yes", NULL,
                             "CPU 0 is always enabled.",
                             validation_ev68cb_cpu0_enabled});
        else
            entry.push_back({strdup(enabled.c_str()), "no", NULL,
                             "Enable CPU?\n"
                             "If not enabled, all parameters refering to this CPU will be ignored.",
                             validation_bool});

        const string speed = "cpu" + i2s(i) + ".speed";
        entry.push_back({strdup(speed.c_str()), "500", "speed",
                         "The CPU speed reported to the guest platform (in MHz, ranging from 10 to 1250).\n"
                         "This does not affect the speed of the emulation.",
                         validation_ev68cb_cpuspeed});
#ifndef ASM_JIT
        const string nohle = "cpu" + i2s(i) + ".palcode.vms.nohle?";
        entry.push_back({strdup(nohle.c_str()), "false", "palcode.vms.nohle",
                         "Disable the high-level emulation (HLE) of the OpenVMS PALcode\n"
                         "and run the real SRM PALcode instead.",
                         validation_bool});
#endif
    }
    const int num_entries = entry.size();

    FormValues_t values = show_form(title, entry.data(), num_entries);

    // TODO: Process values

    // Clean up
    for (int i = 0; i < num_entries; ++i)
    {
        free((void *)entry[i].label);
        free(values[i]);
    }
    free(values);
}

/**
 * ALI 1543 configuration
 **/

void validation_ali_console(FIELD *field)
{
    const char *choices[] = {
        "serial",
        "graphics",
        NULL};
    set_field_type(field, TYPE_ENUM, choices, FALSE, TRUE);
}

// ALI 1543 form
void edit_ali(const char *title)
{
    FormEntry_t entry[] = {
        {"Console Output", "serial", "vga_console",
         "Where would you like console output to go?\n"
         "This is the SRM 'console' variable.\n"
         "WARNING: for the 'graphics' option to work,\n"
         "you need to configure a VGA card.",
         validation_ali_console},
        {"LPT Output", "", "lpt.outfile",
         "Where would you like printer output to go?\n"
         "Output from the printer port will be saved to this file.\n"
         "Leave blank if not wanted.",
         validation_file},
        {"M7101 PMU enabled?", "yes", NULL,
         "Enable the M7101 power-management / ACPI device at PCI 0:17?",
         validation_bool},
        {"USB controller enabled?", "yes", NULL,
         "Enable the USB OHCI controller at PCI 0:19?",
         validation_bool}};
    const int num_entries = ARRAY_SIZE(entry);
    int idx;

    FormValues_t values = show_form(title, entry, num_entries);

    CConfigurator *c = sys0->find_child("pci0.7");
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)"pci0.7", (char *)"ali");

    idx = fentry_index(entry, num_entries, "Console Output");
    c->set_value(strdup(entry[idx].name), strdup(values[idx]));
    idx = fentry_index(entry, num_entries, "LPT Output");
    if (strcmp(values[idx], ""))
        c->set_value(strdup(entry[idx].name), strdup(values[idx]));

    idx = fentry_index(entry, num_entries, "M7101 PMU enabled?");
    if (!strcmp(values[idx], "yes"))
    {
        c = sys0->find_child("pci0.17");
        if (c == nullptr)
            c = new CConfigurator(sys0, (char *)"pci0.17", (char *)"ali_pmu");
    }

    idx = fentry_index(entry, num_entries, "USB controller enabled?");
    if (!strcmp(values[idx], "yes"))
    {
        c = sys0->find_child("pci0.19");
        if (c == nullptr)
            c = new CConfigurator(sys0, (char *)"pci0.19", (char *)"ali_usb");
    }

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}

/**
 * PCI bus configuration
 **/

void show_pcislots(const char *title)
{
    string lines;
    for (int bus = 0; bus <= 1; ++bus)
        for (int slot = 1; slot <= ((bus == 0) ? 4 : 6); ++slot)
        {
            string o = string("pci") + i2s(bus) + "." + i2s(slot) + ": " + device_in_pcislot(bus, slot);
            lines.append(o);
            if (bus == 0 || slot < 6)
                lines.append("\n");
        }

    show_text(title, lines.c_str());
}

void edit_pci_vga_s3(const char *title)
{
    const char *first_free_pcislot = find_first_free_pcislot();
    if (first_free_pcislot == NULL)
    {
        show_text(title, "No free PCI slots");
        return;
    }

    FormEntry_t entry[] = {
        {"PCI slot", first_free_pcislot, NULL,
         "Which PCI slot should the VGA card be on?\n"
         // "Only free PCI slots are listed.\n"
         "AFAIK, VGA should always be on pci0.x",
         validation_pcislot},
        {"rom file",
#if defined(_WIN32)
         "rom\\VGABIOSFILE.bin",
#elif defined(__VMS)
         "[.ROM]VGABIOSFILE.bin",
#else
         "rom/VGABIOSFILE.bin",
#endif
         "rom",
         "Where can the VGA BIOS ROM image be found?\n"
         "Real S3 Trio64 VGA BIOS images work, GNU vgabios project also works,\n"
         "but may not enable or support all S3 features.\n"
         "This file is required.",
         validation_file}};
    int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    int idx = fentry_index(entry, num_entries, "PCI slot");
    CConfigurator *c = get_pcislot(values[idx]);
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)(string("pci") + values[idx]).c_str(), (char *)"s3");

    for (int i = 0; i < num_entries; ++i)
        if (i != idx && entry[i].name != NULL)
            c->set_value(strdup(entry[i].name), strdup(values[i]));

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}

void edit_pci_vga(const char *title)
{
    MenuEntry_t entry[] = {
        {"none", "No graphics card", NULL},
        {"s3", "S3 Trio 64", edit_pci_vga_s3}
#if 0 && defined(HAVE_CIRRUS)
        ,
        {"cirrus", "Cirrus CL-GD542x", edit_pci_vga_cirrus}
#endif
#if 0 && defined(HAVE_RADEON)
        ,
        {"radeon", "ATI Radeon 7500", edit_pci_vga_radeon}
#endif
    };
    int num_entries = ARRAY_SIZE(entry);

    int sel = show_menu(title, entry, num_entries, MENU_3RD_LEVEL);

    if (sel == 0)
    {
        // Remove any VGA cards that may have been added
        for (int bus = 0; bus <= 1; ++bus)
            for (int slot = 1; slot < ((bus == 0) ? 4 : 6); ++slot)
            {
                CConfigurator *c = get_pcislot(bus, slot);
                if (c == nullptr)
                    continue;
                for (int i = 1; i < num_entries; ++i)
                    if (!strcmp(c->get_myValue(), entry[i].text))
                    {
                        // TODO: Remove c from config
                    }
            }
    }
}

void validation_pci_dec21143_type(FIELD *field)
{
    vector<const char *> choices;

#ifdef HAVE_PCAP
    choices.push_back("pcap");
#endif
#ifdef HAVE_TAP_NET
    choices.push_back("tap");
#endif
#ifdef HAVE_VMNET
    choices.push_back("vmnet");
#endif
    // choices.push_back("none");
    choices.push_back(NULL);

    set_field_type(field, TYPE_ENUM, choices.data(), FALSE, TRUE);
}

void validation_pci_dec21143_adapter(FIELD *field)
{
    vector<const char *> choices;

    choices.push_back("list");

#ifdef HAVE_PCAP
    /* Get a list of network interfaces and
     * add them to the list.
     */
    pcap_if_t *alldevs;
    pcap_if_t *d;
    char errbuf[PCAP_ERRBUF_SIZE];

    if (pcap_findalldevs(&alldevs, errbuf) == -1)
    {
        /* No devices to add. */
        string msg = "Error in pcap_findalldevs:\n" + string(errbuf);
        show_text("Error", msg.c_str());
    }
    else
    {
        for (d = alldevs; d; d = d->next)
            choices.push_back(d->name);
    }
#endif
    choices.push_back(NULL);

    set_field_type(field, TYPE_ENUM, choices.data(), FALSE, TRUE);
}

void validation_pci_dec21143_queue(FIELD *field)
{
    set_field_type(field, TYPE_INTEGER, 1, 1, INT_MAX);
}

void edit_pci_dec21143(const char *title)
{
#if !defined(HAVE_PCAP) && !defined(HAVE_TAP_NET) && !defined(HAVE_VMNET)
    show_description("Error", "This program has been compiled with no network support.");
#else
    const char *first_free_pcislot = find_first_free_pcislot();
    if (first_free_pcislot == NULL)
    {
        show_text(title, "No free PCI slots");
        return;
    }

    FormEntry_t entry[] = {
        {"PCI slot", first_free_pcislot, NULL,
         "Which PCI slot should the NIC be on?\n"
         "Only free PCI slots are listed.",
         validation_pcislot},
        {"type",
#if defined(HAVE_PCAP)
         "pcap",
#elif defined(HAVE_TAP_NET)
         "tap0",
#elif defined(HAVE_VMNET)
         "vmnet",
#else
         "none",
#endif
         "type",
         "How should this NIC connect to the host network?"
#ifdef HAVE_PCAP
         "\n"
         "'pcap' captures/injects packets on an existing host adapter\n"
         "(works everywhere, but usually requires a bridge/promiscuous\n"
         "setup on the host to reach a wired LAN)."
#endif
#ifdef HAVE_TAP_NET
         "\n"
         "'tap' uses a TAP virtual network device (Linux/FreeBSD/NetBSD only),\n"
         "which plays more nicely with host firewalling and bridging."
#endif
#ifdef HAVE_VMNET
         "\n"
         "'vmnet' (MacOS only) Use the macOS vmnet framework which uses\n"
         "a host's network interface as a bridge to effectively place\n"
         "the Alpha system on the same subnet as the host."
#endif
         ,
         validation_pci_dec21143_type},
#if defined(HAVE_PCAP)
        {"adapter", "list", "adapter",
         "What host network interface should we connect to?\n"
         "Choose 'list' to get a list at run-time.",
         validation_pci_dec21143_adapter},
#endif
        {"mac", "08-00-2B-E5-40-00", "mac",
         "What should the NIC's MAC address be?\n"
         "This should be unique on your network.",
         validation_mac},
        {"queue", "1024", "queue",
         "The number of frames the receive queue can hold between the host\n"
         "capture interface and the emulated NIC. Frames that arrive while the\n"
         "queue is full are dropped (and reported on the console).",
         validation_pci_dec21143_queue},
        {"crc?", "false", "crc",
         "Calculate a real ethernet CRC (FCS) for frames delivered to the\n"
         "guest. When false, a zeroed placeholder CRC is appended instead,\n"
         "which saves host CPU; guests normally never check it",
         validation_bool},
        {"trace packets?", "false", "trace_packets",
         "Dump every transmitted and received packet to the console,\n"
         "for network debugging. Very noisy.",
         validation_bool}
        /*
        ,{"autonegotiate_delay?", "false", "autonegotiate_delay",
            "Hidden option: defer SIA autoneg completion ~50ms.",
            validation_bool}
        */
        /*
        #ifdef HAVE_VMNET
        ,{"drop_privileges?", "true", "drop_privileges",
            "For vmnet interfaces, controls whether privileges are dropped after the\n"
            "interface is initialized. (ES40 starts with privileges enabled but\n"
            "doesn't need privileges once the network interfaces are initialized.)\n"
            "If you configure multiple network interfaces with vmnet, set this\n"
            "variable to false on all but the interface in the highest PCI slot.",
            validation_bool}
        #endif
        */
    };
    int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    int idx = fentry_index(entry, num_entries, "PCI slot");
    CConfigurator *c = get_pcislot(values[idx]);
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)(string("pci") + values[idx]).c_str(), (char *)"dec21143");

    for (int i = 0; i < num_entries; ++i)
        if (i != idx && entry[i].name != NULL)
            c->set_value(strdup(entry[i].name), strdup(values[i]));

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
#endif // !defined(HAVE_PCAP) && !defined(HAVE_TAP_NET) && !defined(HAVE_VMNET)
}

void edit_pci_sym53c810_settings(const char *title)
{
    const char *first_free_pcislot = find_first_free_pcislot();
    if (first_free_pcislot == NULL)
    {
        show_text(title, "No free PCI slots");
        return;
    }

    FormEntry_t entry[] = {
        {"PCI slot", first_free_pcislot, NULL,
         "Which PCI slot should the Symbios 53C810 narrow SCSI controller be on?\n"
         "Only free PCI slots are listed.",
         validation_pcislot}};
    int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    CConfigurator *c = sys0->find_child("pci0.99");

    int idx = fentry_index(entry, num_entries, "PCI slot");
    c->set_value(strdup("pci_slot"), strdup(values[idx]));

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}

void edit_pci_sym53c810_disks(const char *title)
{
    vector<MenuEntry_t> entry;

    entry.push_back({"none", "Stop adding disks", NULL});

    for (int i = 0; i < 7; ++i)
    {
        string text = "disk0." + i2s(i), desc = "Target " + i2s(i);
        entry.push_back({strdup(text.c_str()), strdup(desc.c_str()), NULL});
    }
    int num_entries = entry.size();

    CConfigurator *c = sys0->find_child("pci0.99");

    while (TRUE)
    {
        int sel = show_menu(title, entry.data(), num_entries, MENU_4TH_LEVEL);
        if (sel <= 0)
            break;

        string subtitle = string(title) + ": " + entry[sel].text;
        FormValues_t values = add_disks(subtitle.c_str(), entry[sel].text, c);

        // TODO: Process values

        // Clean up
        for (int i = 0; i < num_entries; ++i)
            free(values[i]);
        free(values);
    }

    // Clean up
    for (int i = 1; i <= 7; ++i)
    {
        free((void *)entry[i].text);
        free((void *)entry[i].description);
    }
}

void edit_pci_sym53c810(const char *title)
{
    MenuEntry_t entry[] = {
        {"Settings", "Edit the Symbios 53C810 narrow SCSI controller settings.", edit_pci_sym53c810_settings},
        {"Add disks", "Add disks to the Symbios 53C810 narrow SCSI controller.", edit_pci_sym53c810_disks}};
    int num_entries = ARRAY_SIZE(entry);

    CConfigurator *c = sys0->find_child("pci0.99");
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)"pci0.99", (char *)"sym53c810");

    show_menu(title, entry, num_entries, MENU_3RD_LEVEL);

    char *pcislot = c->get_text_value("pci_slot", NULL);
    if (pcislot == NULL)
    {
        // TODO: Invalid - show message, return to menu
    }

    // TODO: Copy c to the real thing, remove pci_slot value, and remove c from the config
}

void edit_pci_lsi53c1020_settings(const char *title)
{
    const char *first_free_pcislot = find_first_free_pcislot();
    if (first_free_pcislot == NULL)
    {
        show_text(title, "No free PCI slots");
        return;
    }

    FormEntry_t entry[] = {
        {"PCI slot", first_free_pcislot, NULL,
         "Which PCI slot should the LSI 53C1020 Fusion-MPT Ultra320 SCSI controller be on?\n"
         "Only free PCI slots are listed.",
         validation_pcislot},
        {"persistant flash?", "no", NULL,
         "Do you want the LSI card flash to persist between emulator runs?\n"
         "The optional backing file is a raw 512 KiB image of the card's flash.\n"
         "If it does not exist, ES40 creates it when possible. Without a flash backing\n"
         "file, BIOS and IOC firmware changes remain volatile.",
         validation_bool},
        {"flash file",
#if defined(_WIN32)
         "rom\\lsi53c1020.flash",
#elif defined(__VMS)
         "[.ROM]LSI53C1020.FLASH",
#else
         "rom/lsi53c1020.flash",
#endif
         "flash",
         "Where should the LSI card flash image be stored?\n"
         "Use a different 512 KiB backing file for each emulated controller.\n"
         "An existing image always takes precedence over seed files.",
         validation_file},
        {"initial LSI BIOS and IOC firmware images?", "no", NULL,
         "These files seed the card only when no persistent flash imagehas been loaded.\n"
         "They are not reapplied on later starts, so changes made by firmware or\n"
         "an operating system remain intact. The behavioral controller starts without either seed.",
         validation_bool},
        {"rom file",
#if defined(_WIN32)
         "rom\\mptps.rom",
#elif defined(__VMS)
         "[.ROM]MPTPS.ROM",
#else
         "rom/mptps.rom",
#endif
         "rom",
         "Where can the LSI PCI BIOS image be found?\n"
         "Use the unmodified mptps.rom file from a compatible LSI firmware package.\n"
         "It is used only when initializing card flash. AlphaBIOS can execute this\n"
         "x86 ROM and expose attached disks through SCSIBIOS; SRM does not use it.",
         validation_file},
        {"firmware file",
#if defined(_WIN32)
         "rom\\it_1030.fw",
#elif defined(__VMS)
         "[.ROM]IT_1030.FW",
#else
         "rom/it_1030.fw",
#endif
         "firmware",
         "Where can the LSI IOC firmware image be found?\n"
         "Use the supplied it_1030.fw compatibility image for the plain 53C1020;\n"
         "the T image is for 53C1020A/1030T controllers. The image is used only\n"
         "when initializing flash, and does not change the emulated card's 53C1020 identity.",
         validation_file}};
    int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    CConfigurator *c = sys0->find_child("pci0.99");

    int idx = fentry_index(entry, num_entries, "PCI slot");
    c->set_value(strdup("pci_slot"), strdup(values[idx]));

    // TODO: Process values

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}

void edit_pci_lsi53c1020_disks(const char *title)
{
    vector<MenuEntry_t> entry;

    entry.push_back({"none", "Stop adding disks", NULL});

    for (int i = 0; i < 16; ++i)
    {
        string text = "disk0." + i2s(i), desc = "Target " + i2s(i);
        entry.push_back({strdup(text.c_str()), strdup(desc.c_str()), NULL});
    }
    int num_entries = entry.size();

    CConfigurator *c = sys0->find_child("pci0.99");

    while (TRUE)
    {
        int sel = show_menu(title, entry.data(), num_entries, MENU_4TH_LEVEL);
        if (sel <= 0)
            break;

        string subtitle = string(title) + ": " + entry[sel].text;
        FormValues_t values = add_disks(subtitle.c_str(), entry[sel].text, c);

        // TODO: Process values

        // Clean up
        for (int i = 0; i < num_entries; ++i)
            free(values[i]);
        free(values);
    }

    // Clean up
    for (int i = 1; i <= 16; ++i)
    {
        free((void *)entry[i].text);
        free((void *)entry[i].description);
    }
}

void edit_pci_lsi53c1020(const char *title)
{
    MenuEntry_t entry[] = {
        {"Settings", "Edit the LSI 53C1020 Fusion-MPT Ultra320 SCSI controller settings.", edit_pci_lsi53c1020_settings},
        {"Add disks", "Add disks to the LSI 53C1020 Fusion-MPT Ultra320 SCSI controller.", edit_pci_lsi53c1020_disks}};
    int num_entries = ARRAY_SIZE(entry);

    CConfigurator *c = sys0->find_child("pci0.99");
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)"pci0.99", (char *)"lsi53c1020");

    show_menu(title, entry, num_entries, MENU_3RD_LEVEL);
}

void edit_pci_es1370(const char *title)
{
    const char *first_free_pcislot = find_first_free_pcislot();
    if (first_free_pcislot == NULL)
    {
        show_text(title, "No free PCI slots");
        return;
    }

    FormEntry_t entry[] = {
        {"PCI slot", first_free_pcislot, NULL,
         "Which PCI slot should the Ensoniq AudioPCI ES1370 sound card be on?\n"
         "Only free PCI slots are listed.",
         validation_pcislot}};
    int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    int idx = fentry_index(entry, num_entries, "PCI slot");
    CConfigurator *c = get_pcislot(values[idx]);
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)(string("pci") + values[idx]).c_str(), (char *)"es1370");

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}

void edit_pci(const char *title)
{
    MenuEntry_t entry[] = {
        {"List PCI bus", "Show a list of devices on the PCI bus.", show_pcislots},
        {"VGA", "Add a VGA card", edit_pci_vga},
        {"NIC", "Add a DEC 21143 Network Interface Card", edit_pci_dec21143},
        {"SCSI", "Add a Symbios 53C810 narrow SCSI controller (most tested/proven for OS boot/install)", edit_pci_sym53c810},
        {"LSI SCSI", "Add a LSI 53C1020 Fusion-MPT Ultra320 SCSI controller (NOT SRM BOOT CAPABLE!)", edit_pci_lsi53c1020}
#ifdef HAVE_SDL
        ,
        {"ES1370 Audio", "Add an Ensoniq AudioPCI ES1370 sound card (works only with Windows NT 4.0 guest)", edit_pci_es1370}};
#endif
    int num_entries = ARRAY_SIZE(entry);

    int sel = show_menu(title, entry, num_entries, MENU_2ND_LEVEL);
}

/**
 * Serial lines configuration
 **/

void validation_serial_port(FIELD *field)
{
    set_field_type(field, TYPE_INTEGER, 1, 1, 65535);
}

void edit_serial(const char *title)
{
    const int num_attr = 6;
    FormEntry_t entry[num_attr * 2]; // (disabled, null_attach, port, raw_mode, program, arguments) * 2 serial ports

    for (int i = 0; i < 2; ++i)
    {
        string serial_name = "serial" + i2s(i);
        entry[num_attr * i] = {
            strdup((serial_name + ".disabled?").c_str()), "no", "disabled",
            "Make the guest see no UART at this address at all, so drivers skip\n"
            "the port completely. No Telnet port is opened. Unlike null_attach,\n"
            "the UART appears absent rather than present-but-idle.\n"
            "If enabled, all parameters refering to this serial port will be ignored.",
            validation_bool};

        entry[num_attr * i + 1] = {
            strdup((serial_name + ".null_attach?").c_str()), "no", "null_attach",
            "If 'yes' is selected, the UART exists on the bus and presents itself\n"
            "to the guest as a healthy idle 16550 (THRE/TSRE, CTS/DSR), but no telnet listener\n"
            "is opened and any bytes the guest transmits are silently discarded.\n"
            "Useful since two are required by the platform firmwares in case you don't need them.\n"
            "If enabled, all parameters refering to this serial port will be ignored.",
            validation_bool};

        const string port_value = i2s(21264 + i);
        entry[num_attr * i + 2] = {
            strdup((serial_name + ".port").c_str()), strdup(port_value.c_str()), "port",
            "The telnet port the serial device will listen on (in the range 1..65535).",
            validation_serial_port};

        entry[num_attr * i + 3] = {
            strdup((serial_name + ".raw_mode?").c_str()), "no", "raw_mode",
            "Pass the byte stream through unmodified: no Telnet protocol (IAC)\n"
            "processing and no throttling to emulated line speed. Required when\n"
            "the port carries a binary protocol such as windbg (KD) or kgdb\n"
            "instead of a terminal session.",
            validation_bool};

        entry[num_attr * i + 4] = {
            strdup((serial_name + ".action").c_str()),
#if defined(_WIN32)
            "C:\\Program Files\\Putty\\Putty.exe",
#else
            "putty",
#endif
            "action",
            "The program that should be started automatically to connect to the serial device listener.\n"
            "Enter the path to a program to start this to create an automatic connection with the serial port.\n"
            "Set to an empty value to establish the connection manually.\n"
            "In that case, the arguments parameter is ignored.",
            validation_file};

        const string arguments_value = "telnet://localhost:" + port_value;

        entry[num_attr * i + 5] = {
            strdup((serial_name + ".arguments").c_str()), strdup(arguments_value.c_str()), "arguments",
            "Arguments the program should use to connect to the serial port.",
            NULL};
    }
    const int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    for (int i = 0; i < 2; ++i)
    {
        string serial_name = "serial" + i2s(i);
        CConfigurator *c = sys0->find_child(serial_name.c_str());
        if (c == nullptr)
            c = new CConfigurator(sys0, (char *)serial_name.c_str(), (char *)"serial");

        int idx = fentry_index(entry, num_entries, (serial_name + ".disabled?").c_str());
        if (!strcmp(values[idx], "yes"))
        {
            c->set_value(strdup(entry[idx].name), strdup(values[idx]));
            continue;
        }

        idx = fentry_index(entry, num_entries, (serial_name + ".null_attach?").c_str());
        if (!strcmp(values[idx], "yes"))
        {
            c->set_value(strdup(entry[idx].name), strdup(values[idx]));
            continue;
        }

        idx = fentry_index(entry, num_entries, (serial_name + ".port").c_str());
        c->set_value(strdup(entry[idx].name), strdup(values[idx]));
        idx = fentry_index(entry, num_entries, (serial_name + ".raw_mode?").c_str());
        c->set_value(strdup(entry[idx].name), strdup(values[idx]));
        idx = fentry_index(entry, num_entries, (serial_name + ".action").c_str());
        c->set_value(strdup(entry[idx].name), strdup(values[idx]));
        idx = fentry_index(entry, num_entries, (serial_name + ".arguments").c_str());
        c->set_value(strdup(entry[idx].name), strdup(values[idx]));
    }

    // Clean up
    for (int i = 0; i < num_entries; ++i)
    {
        free(values[i]);

        free((void *)entry[i].label);
        if (i % num_attr == 2 || i % num_attr == 5)
            free((void *)entry[i].preset);
    }
    free(values);
}

/**
 * Floppy configuration
 **/
void edit_floppy(const char *title)
{
    MenuEntry_t entry[] = {
        {"empty", "Don't add drives to the floppy controller", NULL},
        {"disk0.0", "Drive A:", NULL},
        {"disk0.1", "Drive B:", NULL}};
    int num_entries = ARRAY_SIZE(entry);

    CConfigurator *c = sys0->find_child("fdc0");
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)"fdc0", (char *)"floppy");

    while (TRUE)
    {
        int sel = show_menu(title, entry, num_entries, MENU_2ND_LEVEL);
        if (sel <= 0)
            break;

        string subtitle = string(title) + ": " + entry[sel].text;
        FormValues_t values = add_disks(subtitle.c_str(), entry[sel].text, c);

        // Clean up
        for (int i = 0; i < num_entries; ++i)
            free(values[i]);
        free(values);
    }
}

/**
 * IDE bus configuration
 **/

void edit_ide_settings(const char *title)
{
    FormEntry_t entry[] = {
        {"dma?", "yes", "dma",
         "Allow the guest to use (busmaster) DMA transfers on this\n"
         "IDE controller. Set to false to force PIO-only operation.",
         validation_bool}};
    int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    CConfigurator *c = sys0->find_child("pci0.15");
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)"pci0.15", (char *)"ali_ide");

    int idx = fentry_index(entry, num_entries, "dma?");
    c->set_value(strdup(entry[idx].name), strdup(values[idx]));

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}

void edit_ide_disks(const char *title)
{
    MenuEntry_t entry[] = {
        {"none", "Stop adding disks", NULL},
        {"disk0.0", "primary master", NULL},
        {"disk0.1", "primary slave", NULL},
        {"disk1.0", "secondary master", NULL},
        {"disk1.1", "secondary slave", NULL}};
    int num_entries = ARRAY_SIZE(entry);

    CConfigurator *c = sys0->find_child("pci0.15");
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)"pci0.15", (char *)"ali_ide");

    while (TRUE)
    {
        int sel = show_menu(title, entry, num_entries, MENU_3RD_LEVEL);
        if (sel <= 0)
            break;

        string subtitle = string(title) + ": " + entry[sel].text;
        FormValues_t values = add_disks(subtitle.c_str(), entry[sel].text, c);

        // Clean up
        for (int i = 0; i < num_entries; ++i)
            free(values[i]);
        free(values);
    }
}

void edit_ide(const char *title)
{
    MenuEntry_t entry[] = {
        {"empty", "Don't add disks to the IDE controller (recommended).", NULL},
        {"Settings", "Edit the IDE controller settings.", edit_ide_settings},
        {"Add disks", "Add disks to the IDE controller.", edit_ide_disks}};
    int num_entries = ARRAY_SIZE(entry);

    int sel = show_menu(title, entry, num_entries, MENU_2ND_LEVEL);

    if (sel == 0)
    {
        // TODO: Remove ide controller (node pci0.15) from theConfig
    }
}

/**
 * MPU-401 configuration (Windows only)
 **/

// #ifdef _WIN32
void validation_mpu401_midiout(FIELD *field)
{
    set_field_type(field, TYPE_INTEGER, 1, 0, INT_MAX);
}

void edit_mpu401(const char *title)
{
    FormEntry_t entry[] = {
        {"midi_out", "0", "midi_out",
         "The host MIDI output device number to play on.\n"
         "The default is 0 (the Windows default MIDI device).",
         validation_mpu401_midiout}};
    const int num_entries = ARRAY_SIZE(entry);

    FormValues_t values = show_form(title, entry, num_entries);

    CConfigurator *c = sys0->find_child("mpu0");
    if (c == nullptr)
        c = new CConfigurator(sys0, (char *)"mpu0", (char *)"mpu401");

    int idx = fentry_index(entry, num_entries, "midi_out");
    c->set_value(strdup(entry[idx].name), strdup(values[idx]));

    // Clean up
    for (int i = 0; i < num_entries; ++i)
        free(values[i]);
    free(values);
}
// #endif

/**
 * Main menu
 *
 * Returns TRUE is results are to be saved.
 **/
bool main_menu(void)
{
    MenuEntry_t entry[] = {
        {"GUI settings",
         "You need a GUI if you want to use an emulated graphics card.\n"
         "You don't need this for most OS'es. If you don't need this,\n"
         "we recommend that you select 'none'.",
         edit_gui},
        {"System settings", "", edit_tsunami},
        {"CPU settings", "", edit_ev68cb},
        {"ALI 1543 settings", "", edit_ali},
        {"PCI devices", "", edit_pci},
        {"Serial devices", "", edit_serial},
        {"Floppy devices", "", edit_floppy},
        {"IDE devices", "", edit_ide}
#ifdef _WIN32
        ,
        {"MPU-401 device", "", edit_mpu401}
#endif
        ,
        {"Quit without saving", "Quit the program without writing to es40.cfg", NULL}};
    int num_entries = ARRAY_SIZE(entry);

    int sel = show_menu("Main menu", entry, num_entries, MENU_1ST_LEVEL);

    bool save_results = (sel != num_entries - 1);
    if (save_results)
    {
        // TODO: If GUI has been enabled, check existence of a graphics card
        // TODO: if a graphics card has been configured, check that GUI has been enabled
        // TODO: If serial.console=graphics, check that GUI has been enabled and existence of a graphics card
    }

    return save_results;
}

int main(int argc, char **argv)
{
    const char *out_filename = NULL;

    if (argc >= 2 && argv[1][0] != '\0')
    {
        if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "-?"))
        {
            printf("Usage: %s [input-filename] [output-filename]\n", argv[0]);
            return 0;
        }
        read_configuration(argv[1]); // initializes theConfig
    }
    else
    {
        static char minimal_config[] = "sys0 = tsunami { cpu0 = ev68cb {} }";
        theConfig = new CConfigurator(nullptr, NULL, NULL, minimal_config, strlen(minimal_config));
    }
    sys0 = theConfig->find_child("sys0");

    if (argc >= 3)
    {
        if (argv[2][0] == '\0')
            FAILURE(Configuration, "Output filename is empty");
        out_filename = argv[2];
    }
    else
        out_filename = "es40.cfg";

    // see ncurses(3X) manpage
    setlocale(LC_ALL, "");

    // see https://invisible-island.net/ncurses/ncurses.faq.html#no_padding
    setenv("NCURSES_NO_PADDING", "1", FALSE);

    atexit(reinterpret_cast<void (*)()>(endwin));
    initscr();
    cbreak();
    noecho();
    noqiflush();
    keypad(stdscr, TRUE);
    // start_color();

    getmaxyx(stdscr, scrh, scrw);
    // signal(SIGWINCH, resizeHandler);

    es40_banner("AlphaServer ES40 emulator configuration utility");

    bool save_results = main_menu();
    endwin();

    if (save_results)
        write_configuration(out_filename);

    return 0;
}