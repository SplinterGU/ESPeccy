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


#ifndef VIDEO_h
#define VIDEO_h

#include <inttypes.h>
#include "ESPeccy.h"
#include "ESP32Lib/VGA/VGA6Bit.h"

#define SPEC_W 256
#define SPEC_H 192

#define TSTATES_PER_LINE 224
#define TSTATES_PER_LINE_TK_50 228
#define TSTATES_PER_LINE_TK_60 228
#define TSTATES_PER_LINE_128 228
#define TSTATES_PER_LINE_PENTAGON 224

#define TS_SCREEN_48           14335  // START OF ULA DRAW PAPER 48K
#define TS_SCREEN_TK_50        14687  // START OF ULA DRAW PAPER TK 50HZ
#define TS_SCREEN_TK_60        8759   // START OF ULA DRAW PAPER TK 60HZ
#define TS_SCREEN_128          14361  // START OF ULA DRAW PAPER 128K
#define TS_SCREEN_PENTAGON     17983  // START OF ULA DRAW PAPER PENTAGON

#define TS_BORDER_320x240 8948  // START OF BORDER 48
#define TS_BORDER_320x240_TK_50  9201 // START OF BORDER TK 50HZ
#define TS_BORDER_320x240_TK_60  3273 // START OF BORDER TK 60HZ
#define TS_BORDER_320x240_128 8878  // START OF BORDER 128
#define TS_BORDER_320x240_PENTAGON 12595  // START OF BORDER PENTAGON

// // TS_BORDER_320X240 + (TSTATES_PER_LINE * 20)
// #define TS_BORDER_360x200 13428  // START OF BORDER 48
// #define TS_BORDER_360x200_TK_50 13761  // START OF BORDER TK 50HZ
// #define TS_BORDER_360x200_TK_60 7833  // START OF BORDER TK 60HZ
// #define TS_BORDER_360x200_128 13438  // START OF BORDER 128
// #define TS_BORDER_360x200_PENTAGON 17075  // START OF BORDER PENTAGON

// TS_BORDER_320X240 + (TSTATES_PER_LINE * 20)
#define TS_BORDER_360x200 13420  // START OF BORDER 48
#define TS_BORDER_360x200_TK_50 13753  // START OF BORDER TK 50HZ
#define TS_BORDER_360x200_TK_60 7825  // START OF BORDER TK 60HZ
#define TS_BORDER_360x200_128 13430  // START OF BORDER 128
#define TS_BORDER_360x200_PENTAGON 17067  // START OF BORDER PENTAGON

// TS_BORDER_320X240 - (TSTATES_PER_LINE * 16) - 8
#define TS_BORDER_352x272 5356  // START OF BORDER 48
#define TS_BORDER_352x272_TK_50  5545 // START OF BORDER TK 50HZ
#define TS_BORDER_352x224_TK_60  5089 // START OF BORDER TK 60HZ TS_BORDER_320X240 + (TSTATES_PER_LINE * 8) - 8
#define TS_BORDER_352x272_128 5222  // START OF BORDER 128
#define TS_BORDER_352x272_PENTAGON 9003  // START OF BORDER PENTAGON

// Colors for 6 bit mode
//                  //  BBGGRR
#define BLACK       0b00000000 // 0
#define BLUE        0b00100000 // 20
#define RED         0b00000010 // 2
#define MAGENTA     0b00100010 // 34
#define GREEN       0b00001000 // 8
#define CYAN        0b00101000 // 40
#define YELLOW      0b00001010 // 10
#define WHITE       0b00101010 // 42
#define BRI_BLACK   0b00000000 // 0
#define BRI_BLUE    0b00110000 // 48
#define BRI_RED     0b00000011 // 3
#define BRI_MAGENTA 0b00110011 // 51
#define BRI_GREEN   0b00001100 // 12
#define BRI_CYAN    0b00111100 // 60
#define BRI_YELLOW  0b00001111 // 15
#define BRI_WHITE   0b00111111 // 63

#define NUM_SPECTRUM_COLORS 16

const int redPins[] = {RED_PINS_6B};
const int grePins[] = {GRE_PINS_6B};
const int bluPins[] = {BLU_PINS_6B};

#define zxColor(color,bright) VIDEO::spectrum_colors[bright ? color + 8 : color]

#define USE_FULLCOLOR_BACKGROUND_BACKUP 0

class VIDEO
{
    public:

        // Initialize video
        static void Init();

        // Reset video
        static void Reset();

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // Video draw functions
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        static void EndFrame();
        static void Blank(unsigned int statestoadd, bool contended);
        static void Blank_Opcode(bool contended);
        static void Blank_Snow(unsigned int statestoadd, bool contended);
        static void Blank_Snow_Opcode(bool contended);
        // 48 / 128
        static void MainScreen_Blank(unsigned int statestoadd, bool contended);
        static void MainScreen_Blank_2A3(unsigned int statestoadd, bool contended);
        static void MainScreen_Blank_Opcode(bool contended);
        static void MainScreen_Blank_Opcode_2A3(bool contended);
        static void MainScreen(unsigned int statestoadd, bool contended);
        static void MainScreen_2A3(unsigned int statestoadd, bool contended);
        static void MainScreen_OSD(unsigned int statestoadd, bool contended);
        static void MainScreen_OSD_2A3(unsigned int statestoadd, bool contended);
        static void MainScreen_Opcode(bool contended);
        static void MainScreen_Blank_Snow(unsigned int statestoadd, bool contended);
        static void MainScreen_Blank_Snow_Opcode(bool contended);
        static void MainScreen_Snow(unsigned int statestoadd, bool contended);
        static void MainScreen_Snow_Opcode(bool contended);

        static void TopBorder_Blank();
        static void TopBorder();
        static void MiddleBorder();
        static void BottomBorder();
        static void BottomBorder_OSD();
        static void Border_Blank();

        static void TopBorder_Blank_Pentagon();
        static void TopBorder_Pentagon();
        static void MiddleBorder_Pentagon();
        static void BottomBorder_Pentagon();
        static void BottomBorder_OSD_Pentagon();

        static void (*Draw)(unsigned int, bool);
        static void (*Draw_Opcode)(bool);
        static void (*Draw_OSD169)(unsigned int, bool);
        static void (*Draw_OSD43)();

        static void (*DrawBorder)();

        static void vgataskinit(void *unused);

        static uint8_t* grmem;  // ZX screen memory

        static uint16_t spectrum_colors[NUM_SPECTRUM_COLORS];

        // precalculate ULA SWAP values
        static const uint16_t offBmp[];
        static const uint16_t offAtt[];

        static VGA6Bit vga;

        static uint8_t borderColor;
        static uint32_t border32[8];
        static uint32_t brd;
        static bool brdChange;
        static bool brdnextframe;
        static uint32_t lastBrdTstate;
        static uint8_t brdnextline;
        static uint8_t brdlin_osdstart;
        static uint8_t brdlin_osdend;

        static uint8_t tStatesPerLine;
        static int tStatesScreen;
        static int tStatesBorder;

        static uint8_t flashing;
        static uint8_t flash_ctr;

        static uint8_t att1;
        static uint8_t bmp1;
        static uint8_t att2;
        static uint8_t bmp2;

        static uint8_t dispUpdCycle;
        static bool snow_att;
        static bool dbl_att;
        static uint8_t lastbmp;
        static uint8_t lastatt;
        static uint8_t snowpage;
        static uint8_t snowR;
        static bool snow_toggle;

        static uint8_t OSD;

        static uint32_t* SaveRect;

        static TaskHandle_t videoTaskHandle;

        static int VsyncFinetune[2];

        static uint32_t framecnt; // Frames elapsed


        static int CenterH;
        static int CenterV;


        // -------------------
        // Basic functions
        // -------------------

        using Color = unsigned char;

        static Graphics<unsigned char>* gfx;

        static inline int &xres() {
            return gfx->xres;
        };

        static inline int &yres() {
            return gfx->yres;
        };

        static inline Color ** &frameBuffer() {
            return gfx->frameBuffer;
        };

        static void dotFast(int x, int y, Color color) {
            gfx->dotFast(x, y, color);
        };

        static void setResolution(int xres, int yres) {
            gfx->setResolution(xres, yres);
        };

        static void setTextColor(long front, long back = 0) {
            gfx->setTextColor(front, back);
        };

        static void setFont(Font &font) {
            gfx->setFont(font);
        };

        static void setCodepage(int cp) {
            gfx->setCodepage(cp);
        };

        static void setCursor(int x, int y) {
            gfx->setCursor(x,  y);
        };

        static void drawChar(int x, int y, int ch) {
            gfx->drawChar(x, y, ch);
        }

        static void print(const char ch) {
            gfx->print(ch);
        }

        static void print(const char* str) {
            gfx->print(str);
        }

        static void print(long number, int base = 10, int minCharacters = 1) {
            gfx->print(number, base, minCharacters);
        }

        static void print(unsigned long number, int base = 10, int minCharacters = 1) {
            gfx->print(number, base, minCharacters);
        }

        static void print(int number, int base = 10, int minCharacters = 1) {
            gfx->print(number, base, minCharacters);
        }

        static void print(unsigned int number, int base = 10, int minCharacters = 1) {
            gfx->print(number, base, minCharacters);
        }

        static void print(short number, int base = 10, int minCharacters = 1) {
            gfx->print(number, base, minCharacters);
        }

        static void print(unsigned short number, int base = 10, int minCharacters = 1) {
            gfx->print(number, base, minCharacters);
        }

        static void print(unsigned char number, int base = 10, int minCharacters = 1) {
            gfx->print(number, base, minCharacters);
        }

        static void print(double number, int fractionalDigits = 2, int minCharacters = 1) {
            gfx->print(number, fractionalDigits, minCharacters);
        }

        static void println() {
            gfx->println();
        }

        static void println(const char ch) {
            gfx->println(ch);
        }

        static void println(const char* str) {
            gfx->println(str);
        }

        static void println(long number, int base = 10, int minCharacters = 1) {
            gfx->println(number, base, minCharacters);
        }

        static void println(unsigned long number, int base = 10, int minCharacters = 1) {
            gfx->println(number, base, minCharacters);
        }

        static void println(int number, int base = 10, int minCharacters = 1) {
            gfx->println(number, base, minCharacters);
        }

        static void println(unsigned int number, int base = 10, int minCharacters = 1) {
            gfx->println(number, base, minCharacters);
        }

        static void println(short number, int base = 10, int minCharacters = 1) {
            gfx->println(number, base, minCharacters);
        }

        static void println(unsigned short number, int base = 10, int minCharacters = 1) {
            gfx->println(number, base, minCharacters);
        }

        static void println(unsigned char number, int base = 10, int minCharacters = 1) {
            gfx->println(number, base, minCharacters);
        }

        static void println(double number, int fractionalDigits = 2, int minCharacters = 1) {
            gfx->println(number, fractionalDigits, minCharacters);
        }

        static void clear(Color color = 0) {
            gfx->clear(color);
        }

        static void xLine(int x0, int x1, int y, Color color) {
            gfx->xLine(x0, x1, y, color);
        }

        static void line(int x1, int y1, int x2, int y2, Color color) {
            gfx->line(x1, y1, x2, y2, color);
        }

        static void fillRect(int x, int y, int w, int h, Color color) {
            gfx->fillRect(x, y, w, h, color);
        }

        static void rect(int x, int y, int w, int h, Color color) {
            gfx->rect(x, y, w, h, color);
        }

        static void circle(int x, int y, int r, Color color) {
            gfx->circle(x, y, r, color);
        }

        static void fillCircle(int x, int y, int r, Color color) {
            gfx->fillCircle(x, y, r, color);
        }

/*
vga.CenterH
vga.CenterV
vga.init
vga.SBits
vga.VGA6Bit_useinterrupt
*/


};

#endif // VIDEO_h
