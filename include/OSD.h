/*
ESPeccy - Sinclair ZX Spectrum emulator for the Espressif ESP32 SoC

Copyright (c) 2024 Juan José Ponteprino [SplinterGU]
https://github.com/SplinterGU/ESPeccy

This file is part of ESPeccy.

Based on previous work by:
- Víctor Iborra [Eremus] and David Crespo [dcrespo3d] (ESPectrum)
  https://github.com/EremusOne/ZX-ESPectrum-IDF
- David Crespo [dcrespo3d] (ZX-ESPectrum-Wiimote)
  https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote
- Ramón Martinez and Jorge Fuertes (ZX-ESPectrum)
  https://github.com/rampa069/ZX-ESPectrum
- Pete Todd (paseVGA)
  https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#ifndef ESPECCY_OSD_H
#define ESPECCY_OSD_H

#include "fabgl.h"
#include <string>
#include <vector>
#include <algorithm>

#include "Cheat.h"

using namespace std;

// Defines

// Line type
#define IS_TITLE                0
#define IS_FOCUSED              1
#define IS_NORMAL               2
#define IS_INFO                 3
#define IS_SELECTED             4
#define IS_SELECTED_FOCUSED     5

#define OSD_FONT_W          5
#define OSD_FONT_H          8

#define OSD_COLS            46

#define LEVEL_INFO  0
#define LEVEL_OK    1
#define LEVEL_WARN  2
#define LEVEL_ERROR 3

#define DLG_CANCEL  0
#define DLG_YES     1
#define DLG_NO      2

// States

#define SLOTNAME_LEN 23

// Input filter behavior

#define FILTER_FORBIDDEN    0
#define FILTER_ALLOWED      1

// Scrollbar

#define SCROLL_SEP_CHAR '\xFA'

#define MAXSEARCHLEN 8 // Max visible search len in status bar

// Screen preview

#define RENDER_PREVIEW_OK               0
#define RENDER_PREVIEW_ERROR            1
#define RENDER_PREVIEW_OK_MORE          2
#define RENDER_PREVIEW_REQUEST_NO_FOUND 3

// Input

#define INPUT_CANCELED      1

// Menu states (no tiene relacion con los estados de emulacion, sino con el estado de la ventana de menu)

typedef struct MenuState
{
    unsigned short begin_row;       // First real displayed row
    uint8_t focus;                  // Focused virtual row
    uint8_t last_focus;             // To check for changes
    unsigned short last_begin_row;  // To check for changes
};

// Estructura para almacenar el contexto del scroll
typedef struct RowScrollContext {
    int rowTimeStartScroll; // Contador actual del tiempo para iniciar el scroll
    int rowTimeScroll;      // Contador actual de tiempo entre scrolls
    int rowScrollPos;       // Posición de scroll actual
    bool rowScrollStatus;   // Estado del scroll (incremento o decremento)
};

// OSD Interface
class OSD {

public:

    // Screen size to be set at initialization
    static unsigned short scrW;
    static unsigned short scrH;

    // Calc
    static unsigned short scrAlignCenterX(unsigned short pixel_width);
    static unsigned short scrAlignCenterY(unsigned short pixel_height);
    static uint8_t osdMaxRows();
    static uint8_t osdMaxCols();
    static unsigned short osdInsideX();
    static unsigned short osdInsideY();

    // OSD
    static void osdHome();
    static void osdAt(uint8_t row, uint8_t col);
    static void drawOSD(bool bottom_info);
    static void drawWindow(uint16_t width, uint16_t height,string top, string bottom, bool clear);
    static void drawKbdLayout(uint8_t layout);
    static void drawStats();
    static int  prepare_checkbox_menu(string &menu, string curopt);
    static void pref_rom_menu();
    static void do_OSD(fabgl::VirtualKey KeytoESP, bool CTRL, bool SHIFT);
    static void do_OSD_MenuUpdateROM(uint8_t arch);
    static void do_OSD_MenuUpdateKBDLayout();
    static void HWInfo();
    // static void UART_test();

    //static void render_screen_scaled(int x0, int y0, const uint8_t *bitmap, int divisor, bool monocrome);
    static void renderScreenScaled(int x0, int y0, const uint32_t *bitmap, int divisor, bool monocrome);
    //static void loadCompressedScreen(FILE *f, unsigned char *buffer);
    static void loadCompressedScreen(FILE *f, uint32_t *buffer);
    static int renderScreen(int x, int y, const char* filename, int screen_number, off_t* screen_offset = nullptr);
    static void saveSCR(const std::string& absolutePath, const uint32_t *bitmap);

    // Error
    static void osdCenteredMsg(string msg, uint8_t warn_level);
    static void osdCenteredMsg(string msg, uint8_t warn_level, uint16_t millispause);

    static void restoreBackbufferData(bool force = false);
    static void saveBackbufferData(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool force = false);
    static void saveBackbufferData(bool force = false);
    static void flushBackbufferData();

    // Menu
    static void LoadState();
    static void SaveState();
    static void FileBrowser();
    static void FileEject();

    static unsigned short menuRealRowFor(uint8_t virtual_row_num);
    static void menuPrintRow(uint8_t virtual_row_num, uint8_t line_type);
    static void menuRedraw();
    static void WindowDraw();
    static short menuRun(const string& new_menu, const string& statusbar = "", int (*proc_cb)(fabgl::VirtualKeyItem Menukey) = nullptr);
    static short menuSlotsWithPreview(const string& new_menu, const string& statusbar, int (*proc_cb)(fabgl::VirtualKeyItem Menukey) = nullptr);
    static unsigned short simpleMenuRun(const string& new_menu, uint16_t posx, uint16_t posy, uint8_t max_rows, uint8_t max_cols);
    static string getStatusBar(uint8_t ftype, bool thumb_funcs_enabled);
    static string fileDialog(string &fdir, const string& title, uint8_t ftype, uint8_t mfcols, uint8_t mfrows);
    static int menuTape(string title);
    static void menuScroll(bool up);
    static void fd_Redraw(const string& title, const string& fdir, uint8_t ftype);
    static void fd_PrintRow(uint8_t virtual_row_num, uint8_t line_type);
    static void fd_StatusbarDraw(const string& statusbar, bool fdMode);
    static void tapemenuStatusbarRedraw();
    static void tapemenuRedraw(string title, bool force = true);
    static void PrintRow(uint8_t virtual_row_num, uint8_t line_type, bool is_menu = false);
    static void menuAt(short int row, short int col);
    static void menuScrollBar(unsigned short br);
    static void click();
    static void statusbarDraw(const string& statusbar);

    static void drawCompressedBMP(int x, int y, const uint8_t * bmp);

    static short menuGenericRun(const string& title, const string& statusbar = "", void *user_data = nullptr, size_t (*rowCount)(void *) = nullptr, size_t (*colsCount)(void *) = nullptr, void (*menuRedraw)(const string&, bool) = nullptr, int (*proc_cb)(fabgl::VirtualKeyItem Menukey) = nullptr);

    // menu callbacks
    static int menuProcessSnapshot(fabgl::VirtualKeyItem Menukey);
    static int menuProcessSnapshotSave(fabgl::VirtualKeyItem Menukey);

    static size_t rowCountCheat(void *data);
    static size_t colsCountCheat(void *data);
    static void menuRedrawCheat(const string& title, bool force = false);
    static int menuProcessCheat(fabgl::VirtualKeyItem Menukey);

    static size_t rowCountPoke(void *data);
    static size_t colsCountPoke(void *data);
    static void menuRedrawPoke(const string& title, bool force = false);
    static int menuProcessPokeInput(fabgl::VirtualKeyItem Menukey);

    static bool browseCheatFiles();
    static void LoadCheatFile(const string& snapfile);
    static void showCheatDialog();

    // Estados de menu
    static void menuSaveState(MenuState& state);
    static void menuRestoreState(const MenuState& state);

    // Statusbar scroll
    static void ResetRowScrollContext(RowScrollContext &context);
    static string RotateLine(const std::string &line, RowScrollContext *context, int maxLength, int startThreshold, int scrollInterval);

    static uint8_t menu_level;
    static bool menu_saverect;
    static unsigned short menu_curopt;
    static unsigned int SaveRectpos;

    static unsigned int elements;
    static unsigned int ndirs;

    static uint8_t msgDialog(const string& title, const string& msg);

    static void progressDialog(const string& title, const string& msg, int percent, int action, bool noprogressbar = false);
    string inputBox(int x, int y, string text);
    static void joyDialog(uint8_t joynum);
    static void pokeDialog();

    static int VirtualKey2ASCII(fabgl::VirtualKeyItem Nextkey, bool * mode_E);
    static string input(int x, int y, string inputLabel, int maxSize, int maxDisplaySize, uint16_t ink_color, uint16_t paper_color, const string& default_value = "", const string& filterchars = "", uint8_t * result_flags = nullptr, int filterbehavior = FILTER_FORBIDDEN );

    // Rows
    static unsigned short rowCount(string& menu);
    static string rowGet(string& menu, unsigned short row_number);
    static string rowReplace(string& menu, unsigned short row, const string& newRowContent);

    static void esp_hard_reset();

    static esp_err_t updateFirmware(FILE *firmware);
    static esp_err_t updateFirmwareContent(void *content, uint8_t type);

    static char stats_lin1[25]; // "CPU: 00000 / IDL: 00000 ";
    static char stats_lin2[25]; // "FPS:000.00 / FND:000.00 ";

    static uint8_t cols;                    // Maximum columns
    static uint8_t rows;                    // File menu maximum rows
    static unsigned short real_rows;        // Real row count
    static uint8_t virtual_rows;            // Virtual maximum rows on screen
    static uint16_t w;                      // Width in pixels
    static uint16_t h;                      // Height in pixels
    static uint16_t x;                      // X vertical position
    static uint16_t y;                      // Y horizontal position
    static uint16_t prev_y[5];              // Y prev. position
    static unsigned short menu_prevopt;
    static string menu;                     // Menu string

    static unsigned short begin_row;        // First real displayed row
    static uint8_t focus;                   // Focused virtual row
    static uint8_t last_focus;              // To check for changes
    static unsigned short last_begin_row;   // To check for changes

    static bool use_current_menu_state;

    static uint8_t fdCursorFlash;
    static bool fdSearchRefresh;
    static unsigned int fdSearchElements;

    static Cheat currentCheat;

    static RowScrollContext rowScrollCTX;
    static RowScrollContext fdRowScrollCTX;
    static RowScrollContext statusBarScrollCTX;

    static string lastPreviewFile;

    static uint8_t menuBGColor;
    static uint8_t menuFGColor;

};

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    rtrim(s);
    ltrim(s);
}

// trim from start (copying)
static inline std::string ltrim_copy(std::string s) {
    ltrim(s);
    return s;
}

// trim from end (copying)
static inline std::string rtrim_copy(std::string s) {
    rtrim(s);
    return s;
}

// trim from both ends (copying)
static inline std::string trim_copy(std::string s) {
    trim(s);
    return s;
}

#endif // ESPECCY_OSD_H
