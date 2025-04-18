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

#include "Video.h"
#include "VidPrecalc.h"
#include "cpuESP.h"
#include "MemESP.h"
#include "ZXKeyb.h"
#include "Config.h"
#include "OSD.h"
#include "messages.h"
#include "hardpins.h"
#include "Z80_JLS/z80.h"
#include "Z80_JLS/z80operations.h"

#pragma GCC optimize("O3")

extern Font SystemFont;

Graphics<unsigned char>* VIDEO::gfx = nullptr;

VGA6Bit VIDEO::vga;

uint16_t VIDEO::spectrum_colors[NUM_SPECTRUM_COLORS] = {
    BLACK,     BLUE,     RED,     MAGENTA,     GREEN,     CYAN,     YELLOW,     WHITE,
    BRI_BLACK, BRI_BLUE, BRI_RED, BRI_MAGENTA, BRI_GREEN, BRI_CYAN, BRI_YELLOW, BRI_WHITE, ORANGE
};

uint8_t VIDEO::borderColor = 0;
uint32_t VIDEO::brd;
uint32_t VIDEO::border32[8];
uint8_t VIDEO::flashing = 0;
uint8_t VIDEO::flash_ctr= 0;
uint8_t VIDEO::OSD = 0;
uint8_t VIDEO::tStatesPerLine;
int VIDEO::tStatesScreen;
int VIDEO::tStatesBorder;
uint8_t* VIDEO::grmem;

// precalculate ULA SWAP values
const uint16_t VIDEO::offBmp[SPEC_H] = {
    0x0000, 0x0100, 0x0200, 0x0300, 0x0400, 0x0500, 0x0600, 0x0700,
    0x0020, 0x0120, 0x0220, 0x0320, 0x0420, 0x0520, 0x0620, 0x0720,
    0x0040, 0x0140, 0x0240, 0x0340, 0x0440, 0x0540, 0x0640, 0x0740,
    0x0060, 0x0160, 0x0260, 0x0360, 0x0460, 0x0560, 0x0660, 0x0760,
    0x0080, 0x0180, 0x0280, 0x0380, 0x0480, 0x0580, 0x0680, 0x0780,
    0x00a0, 0x01a0, 0x02a0, 0x03a0, 0x04a0, 0x05a0, 0x06a0, 0x07a0,
    0x00c0, 0x01c0, 0x02c0, 0x03c0, 0x04c0, 0x05c0, 0x06c0, 0x07c0,
    0x00e0, 0x01e0, 0x02e0, 0x03e0, 0x04e0, 0x05e0, 0x06e0, 0x07e0,
    0x0800, 0x0900, 0x0a00, 0x0b00, 0x0c00, 0x0d00, 0x0e00, 0x0f00,
    0x0820, 0x0920, 0x0a20, 0x0b20, 0x0c20, 0x0d20, 0x0e20, 0x0f20,
    0x0840, 0x0940, 0x0a40, 0x0b40, 0x0c40, 0x0d40, 0x0e40, 0x0f40,
    0x0860, 0x0960, 0x0a60, 0x0b60, 0x0c60, 0x0d60, 0x0e60, 0x0f60,
    0x0880, 0x0980, 0x0a80, 0x0b80, 0x0c80, 0x0d80, 0x0e80, 0x0f80,
    0x08a0, 0x09a0, 0x0aa0, 0x0ba0, 0x0ca0, 0x0da0, 0x0ea0, 0x0fa0,
    0x08c0, 0x09c0, 0x0ac0, 0x0bc0, 0x0cc0, 0x0dc0, 0x0ec0, 0x0fc0,
    0x08e0, 0x09e0, 0x0ae0, 0x0be0, 0x0ce0, 0x0de0, 0x0ee0, 0x0fe0,
    0x1000, 0x1100, 0x1200, 0x1300, 0x1400, 0x1500, 0x1600, 0x1700,
    0x1020, 0x1120, 0x1220, 0x1320, 0x1420, 0x1520, 0x1620, 0x1720,
    0x1040, 0x1140, 0x1240, 0x1340, 0x1440, 0x1540, 0x1640, 0x1740,
    0x1060, 0x1160, 0x1260, 0x1360, 0x1460, 0x1560, 0x1660, 0x1760,
    0x1080, 0x1180, 0x1280, 0x1380, 0x1480, 0x1580, 0x1680, 0x1780,
    0x10a0, 0x11a0, 0x12a0, 0x13a0, 0x14a0, 0x15a0, 0x16a0, 0x17a0,
    0x10c0, 0x11c0, 0x12c0, 0x13c0, 0x14c0, 0x15c0, 0x16c0, 0x17c0,
    0x10e0, 0x11e0, 0x12e0, 0x13e0, 0x14e0, 0x15e0, 0x16e0, 0x17e0
};
const uint16_t VIDEO::offAtt[SPEC_H] = {
    0x1800, 0x1800, 0x1800, 0x1800, 0x1800, 0x1800, 0x1800, 0x1800,
    0x1820, 0x1820, 0x1820, 0x1820, 0x1820, 0x1820, 0x1820, 0x1820,
    0x1840, 0x1840, 0x1840, 0x1840, 0x1840, 0x1840, 0x1840, 0x1840,
    0x1860, 0x1860, 0x1860, 0x1860, 0x1860, 0x1860, 0x1860, 0x1860,
    0x1880, 0x1880, 0x1880, 0x1880, 0x1880, 0x1880, 0x1880, 0x1880,
    0x18a0, 0x18a0, 0x18a0, 0x18a0, 0x18a0, 0x18a0, 0x18a0, 0x18a0,
    0x18c0, 0x18c0, 0x18c0, 0x18c0, 0x18c0, 0x18c0, 0x18c0, 0x18c0,
    0x18e0, 0x18e0, 0x18e0, 0x18e0, 0x18e0, 0x18e0, 0x18e0, 0x18e0,
    0x1900, 0x1900, 0x1900, 0x1900, 0x1900, 0x1900, 0x1900, 0x1900,
    0x1920, 0x1920, 0x1920, 0x1920, 0x1920, 0x1920, 0x1920, 0x1920,
    0x1940, 0x1940, 0x1940, 0x1940, 0x1940, 0x1940, 0x1940, 0x1940,
    0x1960, 0x1960, 0x1960, 0x1960, 0x1960, 0x1960, 0x1960, 0x1960,
    0x1980, 0x1980, 0x1980, 0x1980, 0x1980, 0x1980, 0x1980, 0x1980,
    0x19a0, 0x19a0, 0x19a0, 0x19a0, 0x19a0, 0x19a0, 0x19a0, 0x19a0,
    0x19c0, 0x19c0, 0x19c0, 0x19c0, 0x19c0, 0x19c0, 0x19c0, 0x19c0,
    0x19e0, 0x19e0, 0x19e0, 0x19e0, 0x19e0, 0x19e0, 0x19e0, 0x19e0,
    0x1a00, 0x1a00, 0x1a00, 0x1a00, 0x1a00, 0x1a00, 0x1a00, 0x1a00,
    0x1a20, 0x1a20, 0x1a20, 0x1a20, 0x1a20, 0x1a20, 0x1a20, 0x1a20,
    0x1a40, 0x1a40, 0x1a40, 0x1a40, 0x1a40, 0x1a40, 0x1a40, 0x1a40,
    0x1a60, 0x1a60, 0x1a60, 0x1a60, 0x1a60, 0x1a60, 0x1a60, 0x1a60,
    0x1a80, 0x1a80, 0x1a80, 0x1a80, 0x1a80, 0x1a80, 0x1a80, 0x1a80,
    0x1aa0, 0x1aa0, 0x1aa0, 0x1aa0, 0x1aa0, 0x1aa0, 0x1aa0, 0x1aa0,
    0x1ac0, 0x1ac0, 0x1ac0, 0x1ac0, 0x1ac0, 0x1ac0, 0x1ac0, 0x1ac0,
    0x1ae0, 0x1ae0, 0x1ae0, 0x1ae0, 0x1ae0, 0x1ae0, 0x1ae0, 0x1ae0
};

uint32_t* VIDEO::SaveRect;
int VIDEO::VsyncFinetune[2];
uint32_t VIDEO::framecnt = 0;
uint8_t VIDEO::dispUpdCycle;
uint8_t VIDEO::att1;
uint8_t VIDEO::bmp1;
uint8_t VIDEO::att2;
uint8_t VIDEO::bmp2;
bool VIDEO::snow_att = false;
bool VIDEO::dbl_att = false;
// bool VIDEO::opCodeFetch;
uint8_t VIDEO::lastbmp;
uint8_t VIDEO::lastatt;
uint8_t VIDEO::snowpage;
uint8_t VIDEO::snowR;
bool VIDEO::snow_toggle = false;

static unsigned int is169;

static uint32_t* lineptr32;

static unsigned int tstateDraw; // Drawing start point (in Tstates)
static unsigned int linedraw_cnt;
static unsigned int lin_end, lin_end2 /*, lin_end3*/;
static unsigned int coldraw_cnt;
static unsigned int col_end;
static unsigned int video_rest;
static unsigned int video_opcode_rest;
static unsigned int curline;

static unsigned int bmpOffset;  // offset for bitmap in graphic memory
static unsigned int attOffset;  // offset for attrib in graphic memory

static const uint8_t* wait_st;

static const uint8_t wait_st_std[131] = {
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    0, 0, 0
}; // sequence of wait states

static const uint8_t wait_st_2a3[131] = {
    1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2,
    1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2,
    1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2,
    1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2,
    1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2,
    1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2,
    1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2,
    1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2,
    1, 0, 0
}; // sequence of wait states

IRAM_ATTR void VGA6Bit::interrupt(void *arg) {

    static int64_t prevmicros = 0;
    static int64_t elapsedmicros = 0;
    static int cntvsync = 0;

    if (Config::load_monitor) {
        ESPeccy::vsync = true;
        return;
    }

    int64_t currentmicros = esp_timer_get_time();

    if (prevmicros) {

        elapsedmicros += currentmicros - prevmicros;

        if (elapsedmicros >= ESPeccy::target[0]) {

            ESPeccy::vsync = true;

            // This code is needed to "finetune" the sync. Without it, vsync and emu video gets slowly desynced.
            if (VIDEO::VsyncFinetune[0]) {
                if (cntvsync++ == VIDEO::VsyncFinetune[1]) {
                    elapsedmicros += VIDEO::VsyncFinetune[0];
                    cntvsync = 0;
                }
            }

            elapsedmicros -= ESPeccy::target[0];

        } else ESPeccy::vsync = false;

    } else {

        elapsedmicros = 0;
        ESPeccy::vsync = false;

    }

    prevmicros = currentmicros;

}

void (*VIDEO::Draw)(unsigned int, bool) = &VIDEO::Blank;
void (*VIDEO::Draw_Opcode)(bool) = &VIDEO::Blank_Opcode;
void (*VIDEO::Draw_OSD169)(unsigned int, bool) = &VIDEO::MainScreen;
void (*VIDEO::Draw_OSD43)() = &VIDEO::BottomBorder;

void (*VIDEO::DrawBorder)() = &VIDEO::TopBorder_Blank;

static uint32_t* brdptr32;
static uint16_t* brdptr16;

uint32_t VIDEO::lastBrdTstate;
uint8_t VIDEO::brdnextline;
uint8_t VIDEO::brdlin_osdstart;
uint8_t VIDEO::brdlin_osdend;
bool VIDEO::brdChange = false;
bool VIDEO::brdnextframe = true;

void precalcborder32() {
    for (int i = 0; i < 8; i++) {
        uint8_t border = zxColor(i,0);
        VIDEO::border32[i] = border | (border << 8) | (border << 16) | (border << 24);
    }
}

void VIDEO::vgataskinit(void *unused) {

    uint8_t Mode;

    if (Config::videomode == 1) {

        char c_arch = Config::arch[0];

        switch (c_arch) {
        case '4':
            Mode = 4;
            break;
        case 'T':
            Mode = Config::ALUTK == 0 ? 4 : Config::ALUTK == 1 ? 8 : 12; // Video mode depends on ULA chosen
            break;
        case '1':
        case '+':
            Mode = 16;
            break;
        case 'P':
            Mode = 20;
            break;
        }

        Mode += (Config::aspect_16_9 ? 2 : 0) + Config::scanlines;

        OSD::scrW = vidmodes[Mode][vmodeproperties::hRes];
        OSD::scrH = (vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv]) >> Config::scanlines;

    } else {

        char c_arch = Config::arch[0];

        switch (c_arch) {
        case '4':
            Mode = 24;
            break;
        case 'T':
            Mode = Config::ALUTK == 0 ? 24 : Config::ALUTK == 1 ? 25 : 26; // Video mode depends on ULA chosen
            break;
        case '1':
        case '+':
            Mode = 27;
            break;
        case 'P':
            Mode = 28;
            break;
        }

        OSD::scrW = vidmodes[Mode][vmodeproperties::hRes];
        OSD::scrH = vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv];

        // CRT Centering
        vga.CenterH = Config::CenterH;
        vga.CenterV = Config::CenterV;

    }

    vga.VGA6Bit_useinterrupt=true;

    // Init mode
    vga.init(Mode, redPins, grePins, bluPins, HSYNC_PIN, VSYNC_PIN);

    // This 'for' is needed for video mode with use_interruption = true.
    for (;;){}

}

TaskHandle_t VIDEO::videoTaskHandle;

void VIDEO::Init() {

    gfx = &vga;

    if (Config::videomode) {

        // xTaskCreatePinnedToCore(&VIDEO::vgataskinit, "videoTask", 1024, NULL, configMAX_PRIORITIES - 2, &videoTaskHandle, 1);
        xTaskCreatePinnedToCore(&VIDEO::vgataskinit, "videoTask", 1024, NULL, 5, &videoTaskHandle, 1);

        // Wait for vertical sync to ensure vga.init is done
        for (;;) {
            if (ESPeccy::vsync) break;
        }

    } else {

        int Mode = Config::aspect_16_9 ? 2 : 0;

        Mode += Config::scanlines;

        OSD::scrW = vidmodes[Mode][vmodeproperties::hRes];
        OSD::scrH = (vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv]) >> Config::scanlines;

        vga.VGA6Bit_useinterrupt=false;

        vga.init( Mode, redPins, grePins, bluPins, HSYNC_PIN, VSYNC_PIN);

    }

    // Precalculate colors for current VGA mode
    for (int i = 0; i < NUM_SPECTRUM_COLORS; i++)
        spectrum_colors[i] = (spectrum_colors[i] & VIDEO::vga.RGBAXMask) | VIDEO::vga.SBits;

    for (int n = 0; n < 16; n++)
        AluByte[n] = (unsigned int *)AluBytes[bitRead(VIDEO::vga.SBits,7)][n];

    precalcborder32();  // Precalc border 32 bits values

    // 0x9000 for full color
    // 0x4800 for 16 colors
#if USE_FULLCOLOR_BACKGROUND_BACKUP
    SaveRect = (uint32_t *) heap_caps_malloc(0x9000, MALLOC_CAP_32BIT);
#else
    SaveRect = (uint32_t *) heap_caps_malloc(0x4800, MALLOC_CAP_32BIT);
#endif

    if (!SaveRect) {
        printf("ERROR can't alloc SaveRect!!!\n");
    }

    // Set font & Codepage
    setFont(SystemFont);
    setCodepage(LANGCODEPAGE[Config::lang]);

}

void VIDEO::Reset() {

    borderColor = 7;
    brd = border32[7];

    is169 = Config::aspect_16_9 ? 1 : 0;

    OSD = 0;

    Draw_OSD169 = Config::arch == "+2A" || Config::arch == "+3" ? MainScreen_2A3 : MainScreen;
    Draw_OSD43 = BottomBorder;
    DrawBorder = TopBorder_Blank;

    wait_st = (uint8_t *) wait_st_std;

    if (Config::arch == "48K") {
        tStatesPerLine = TSTATES_PER_LINE;
        tStatesScreen = TS_SCREEN_48;
        tStatesBorder = Config::videomode == 2 ? TS_BORDER_352x272 : (is169 ? TS_BORDER_360x200 : TS_BORDER_320x240);
        if (Config::videomode == 1) {
            VsyncFinetune[0] = is169 ? -1 : 0;
            VsyncFinetune[1] = is169 ? 152 : 0;
        } else {
            VsyncFinetune[0] = is169 ? 0 : 0;
            VsyncFinetune[1] = is169 ? 0 : 0;
        }

    } else if (Config::arch == "TK90X" || Config::arch == "TK95" ) {

        switch (Config::ALUTK) {
        case 0: // Ferranti
            tStatesPerLine = TSTATES_PER_LINE;
            tStatesScreen = TS_SCREEN_48;
            tStatesBorder = Config::videomode == 2 ? TS_BORDER_352x272 : (is169 ? TS_BORDER_360x200 : TS_BORDER_320x240);
            break;
        case 1: // Microdigital 50hz
            tStatesPerLine = TSTATES_PER_LINE_TK_50;
            tStatesScreen = TS_SCREEN_TK_50;
            tStatesBorder = Config::videomode == 2 ? TS_BORDER_352x272_TK_50 : (is169 ? TS_BORDER_360x200_TK_50 : TS_BORDER_320x240_TK_50);
            break;
        case 2: // Microdigital 60hz
            tStatesPerLine = TSTATES_PER_LINE_TK_60;
            tStatesScreen = TS_SCREEN_TK_60;
            tStatesBorder = Config::videomode == 2 ? TS_BORDER_352x224_TK_60 : (is169 ? TS_BORDER_360x200_TK_60 : TS_BORDER_320x240_TK_60);
        }

        if (Config::videomode == 1) {
            VsyncFinetune[0] = is169 ? -1 : 0;
            VsyncFinetune[1] = is169 ? 152 : 0;
        } else {
            VsyncFinetune[0] = is169 ? 0 : 0;
            VsyncFinetune[1] = is169 ? 0 : 0;
        }

    } else if (Config::arch == "128K") {

        tStatesPerLine = TSTATES_PER_LINE_128;
        tStatesScreen = TS_SCREEN_128;
        tStatesBorder = Config::videomode == 2 ? TS_BORDER_352x272_128 : (is169 ? TS_BORDER_360x200_128 : TS_BORDER_320x240_128);
        if (Config::videomode == 1) {
            VsyncFinetune[0] = is169 ? 1 : 1;
            VsyncFinetune[1] = is169 ? 123 : 123;
        } else {
            VsyncFinetune[0] = is169 ? 0 : 0;
            VsyncFinetune[1] = is169 ? 0 : 0;
        }

    } else if (Config::arch == "+2A") {

        wait_st = (uint8_t *) wait_st_2a3;

        tStatesPerLine = TSTATES_PER_LINE_128;
        tStatesScreen = TS_SCREEN_128 + 3;
        tStatesBorder = Config::videomode == 2 ? TS_BORDER_352x272_128 : (is169 ? TS_BORDER_360x200_128 : TS_BORDER_320x240_128);
        tStatesBorder += 3;
        if (Config::videomode == 1) {
            VsyncFinetune[0] = is169 ? 1 : 1;
            VsyncFinetune[1] = is169 ? 123 : 123;
        } else {
            VsyncFinetune[0] = is169 ? 0 : 0;
            VsyncFinetune[1] = is169 ? 0 : 0;
        }

    } else if (Config::arch == "Pentagon") {

        tStatesPerLine = TSTATES_PER_LINE_PENTAGON;
        tStatesScreen = TS_SCREEN_PENTAGON;
        tStatesBorder = Config::videomode == 2 ? TS_BORDER_352x272_PENTAGON : (is169 ? TS_BORDER_360x200_PENTAGON : TS_BORDER_320x240_PENTAGON);
        // TODO: ADJUST THESE VALUES FOR PENTAGON
        if (Config::videomode == 1) {
            VsyncFinetune[0] = is169 ? 1 : 1;
            VsyncFinetune[1] = is169 ? 123 : 123;
        } else {
            VsyncFinetune[0] = is169 ? 0 : 0;
            VsyncFinetune[1] = is169 ? 0 : 0;
        }

        Draw_OSD43 = BottomBorder_Pentagon;
        DrawBorder = TopBorder_Blank_Pentagon;

    }

    brdnextline = tStatesPerLine - 160; // T-states to advance to next border line

    if (Config::videomode == 2) {

        brdnextline -= 16;

        if (Config::arch[0] == 'T' && Config::ALUTK == 2) {
            lin_end = 16;
            lin_end2 = 208;
            brdlin_osdstart = 208;
            brdlin_osdend = 223;
        } else {
            lin_end = 40;
            lin_end2 = 232;
            brdlin_osdstart = 236;
            brdlin_osdend = 251;
        }

    } else {

        if (is169) {
            brdnextline -= 16;
            lin_end = 4;
            lin_end2 = 196;
        } else {
            lin_end = 24;
            lin_end2 = 216;
            brdlin_osdstart = 220;
            brdlin_osdend = 235;
        }

    }

    grmem = MemESP::videoLatch ? MemESP::ram[7] : MemESP::ram[5];

    VIDEO::snow_toggle = Config::render;

    if (VIDEO::snow_toggle) {
        Draw = &Blank_Snow;
        Draw_Opcode = &Blank_Snow_Opcode;
    } else {
        Draw = &Blank;
        Draw_Opcode = &Blank_Opcode;
    }

    // Restart border drawing
    lastBrdTstate = tStatesBorder;
    brdChange = false;
    brdnextframe = true;

}

///////////////////////////////////////////////////////////////////////////////
//  VIDEO DRAW FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

IRAM_ATTR void VIDEO::MainScreen_Blank(unsigned int statestoadd, bool contended) {

    CPU::tstates += statestoadd;

    if (CPU::tstates >= tstateDraw) {

        if (brdChange) DrawBorder(); // Needed to avoid tearing in demos like Gabba (Pentagon)

        lineptr32 = (uint32_t *)(VIDEO::frameBuffer()[linedraw_cnt]) + (Config::videomode == 2 ? 12 : is169 ? 13 : 8);

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
        bmpOffset = offBmp[curline];
        attOffset = offAtt[curline];

        Draw = linedraw_cnt >= 176 && linedraw_cnt <= 191 ? Draw_OSD169 : MainScreen;
        Draw_Opcode = MainScreen_Opcode;

        video_rest = CPU::tstates - tstateDraw;
        Draw(0,false);

    }

}

IRAM_ATTR void VIDEO::MainScreen_Blank_2A3(unsigned int statestoadd, bool contended) {

    if (contended && (CPU::tstates >= (tstateDraw - 3))) {
        statestoadd += wait_st[CPU::tstates - (tstateDraw - 3)];
    }

    CPU::tstates += statestoadd;

    if (CPU::tstates >= tstateDraw) {

        if (brdChange) DrawBorder(); // Needed to avoid tearing in demos like Gabba (Pentagon)

        lineptr32 = (uint32_t *)(VIDEO::frameBuffer()[linedraw_cnt]) + (Config::videomode == 2 ? 12 : is169 ? 13 : 8);

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
        bmpOffset = offBmp[curline];
        attOffset = offAtt[curline];

        Draw = linedraw_cnt >= 176 && linedraw_cnt <= 191 ? Draw_OSD169 : MainScreen_2A3;
        Draw_Opcode = MainScreen_Opcode;

        video_rest = CPU::tstates - tstateDraw;
        Draw(0,false);

    }

}

IRAM_ATTR void VIDEO::MainScreen_Blank_Opcode(bool contended) { MainScreen_Blank(4, contended); }
IRAM_ATTR void VIDEO::MainScreen_Blank_Opcode_2A3(bool contended) { MainScreen_Blank_2A3(4, contended); }

IRAM_ATTR void VIDEO::MainScreen_Blank_Snow(unsigned int statestoadd, bool contended) {

    if (Z80Ops::is2a3 && contended && (CPU::tstates >= (tstateDraw - 3))) {
        statestoadd += wait_st[CPU::tstates - (tstateDraw - 3)];
    }

    CPU::tstates += statestoadd;

    if (CPU::tstates >= tstateDraw) {

        if (brdChange) DrawBorder();

        lineptr32 = (uint32_t *)(VIDEO::frameBuffer()[linedraw_cnt]) + (Config::videomode == 2 ? 12 : is169 ? 13 : 8);

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
        bmpOffset = offBmp[curline];
        attOffset = offAtt[curline];

        snowpage = MemESP::videoLatch ? 7 : 5;

        dispUpdCycle = 0; // For ULA cycle perfect emulation

        Draw = &MainScreen_Snow;
        Draw_Opcode = &MainScreen_Snow_Opcode;

        // For ULA cycle perfect emulation
        int vid_rest = CPU::tstates - tstateDraw;
        if (vid_rest) {
            CPU::tstates = tstateDraw;
            Draw(vid_rest,false);
        }

    }

}

IRAM_ATTR void VIDEO::MainScreen_Blank_Snow_Opcode(bool contended) {

    if (Z80Ops::is2a3 && contended && (CPU::tstates >= (tstateDraw - 3))) {
        CPU::tstates += 4 + wait_st[CPU::tstates - (tstateDraw - 3)];
    } else
    	CPU::tstates += 4;

    if (CPU::tstates >= tstateDraw) {

        if (brdChange) DrawBorder();

        lineptr32 = (uint32_t *)(VIDEO::frameBuffer()[linedraw_cnt]) + (Config::videomode == 2 ? 12 : is169 ? 13: 8);

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
        bmpOffset = offBmp[curline];
        attOffset = offAtt[curline];

        snowpage = MemESP::videoLatch ? 7 : 5;

        dispUpdCycle = 0; // For ptime-128 compliant version

        Draw = &MainScreen_Snow;
        Draw_Opcode = &MainScreen_Snow_Opcode;

        // For ULA cycle perfect emulation
        video_opcode_rest = CPU::tstates - tstateDraw;
        if (video_opcode_rest) {
            CPU::tstates = tstateDraw;
            Draw_Opcode(false);
            video_opcode_rest = 0;
        }

    }

}

// ----------------------------------------------------------------------------------
// Fast video emulation with no ULA cycle emulation and no snow effect support
// ----------------------------------------------------------------------------------
IRAM_ATTR void VIDEO::MainScreen(unsigned int statestoadd, bool contended) {

    if (contended) statestoadd += wait_st[CPU::tstates - tstateDraw];

    CPU::tstates += statestoadd;
    statestoadd += video_rest;
    video_rest = statestoadd & 0x03;
    unsigned int loopCount = statestoadd >> 2;
    coldraw_cnt += loopCount;

    if (coldraw_cnt >= 32) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw = &Blank;
            Draw_Opcode = &Blank_Opcode;
        } else {
            Draw = &MainScreen_Blank;
            Draw_Opcode = &MainScreen_Blank_Opcode;
        }
        loopCount -= coldraw_cnt - 32;
    }

    for (;loopCount--;) {
        uint8_t att = grmem[attOffset++];
        uint8_t bmp = att & flashing ? ~grmem[bmpOffset++] : grmem[bmpOffset++];
        *lineptr32++ = AluByte[bmp >> 4][att];
        *lineptr32++ = AluByte[bmp & 0xF][att];
    }

}

IRAM_ATTR void VIDEO::MainScreen_2A3(unsigned int statestoadd, bool contended) {

    uint8_t att;

    if (contended) statestoadd += wait_st[CPU::tstates - (tstateDraw - 3)];

    CPU::tstates += statestoadd;
    statestoadd += video_rest;
    video_rest = statestoadd & 0x03;
    unsigned int loopCount = statestoadd >> 2;
    coldraw_cnt += loopCount;

    if (coldraw_cnt >= 32) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw = &Blank;
            Draw_Opcode = &Blank_Opcode;
        } else {
            Draw = &MainScreen_Blank_2A3;
            Draw_Opcode = &MainScreen_Blank_Opcode_2A3;
        }
        loopCount -= coldraw_cnt - 32;
    }

    if (loopCount == 0) return;

    for (;loopCount--;) {
        att = grmem[attOffset++];
        uint8_t bmp = att & flashing ? ~grmem[bmpOffset++] : grmem[bmpOffset++];
        *lineptr32++ = AluByte[bmp >> 4][att];
        *lineptr32++ = AluByte[bmp & 0xF][att];
    }

    if ((attOffset & 1) == 0) MemESP::lastContendedMemReadWrite = att;

}

IRAM_ATTR void VIDEO::MainScreen_OSD_2A3(unsigned int statestoadd, bool contended) {

    uint8_t att;

    if (contended) statestoadd += wait_st[CPU::tstates - (tstateDraw - 3)];

    CPU::tstates += statestoadd;
    statestoadd += video_rest;
    video_rest = statestoadd & 0x03;
    unsigned int loopCount = statestoadd >> 2;
    unsigned int coldraw_osd = coldraw_cnt;
    coldraw_cnt += loopCount;

    if (coldraw_cnt >= 32) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw = &Blank;
            Draw_Opcode = &Blank_Opcode;
        } else {
            Draw = &MainScreen_Blank_2A3;
            Draw_Opcode = &MainScreen_Blank_Opcode_2A3;
        }
        loopCount -= coldraw_cnt - 32;
    }

    if (loopCount == 0) return;

    for (;loopCount--;) {
        if (coldraw_osd >= 13 && coldraw_osd <= 30) {
            lineptr32+=2;
            attOffset++;
            bmpOffset++;
        } else {
            att = grmem[attOffset++];
            uint8_t bmp = (att & flashing) ? ~grmem[bmpOffset++] : grmem[bmpOffset++];
            *lineptr32++ = AluByte[bmp >> 4][att];
            *lineptr32++ = AluByte[bmp & 0xF][att];
        }
        coldraw_osd++;
    }

    if ((attOffset & 1) == 0) MemESP::lastContendedMemReadWrite = att;

}

IRAM_ATTR void VIDEO::MainScreen_OSD(unsigned int statestoadd, bool contended) {

    if (contended) statestoadd += wait_st[CPU::tstates - tstateDraw];

    CPU::tstates += statestoadd;
    statestoadd += video_rest;
    video_rest = statestoadd & 0x03;
    unsigned int loopCount = statestoadd >> 2;
    unsigned int coldraw_osd = coldraw_cnt;
    coldraw_cnt += loopCount;

    if (coldraw_cnt >= 32) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw = &Blank;
            Draw_Opcode = &Blank_Opcode;
        } else {
            Draw = &MainScreen_Blank;
            Draw_Opcode = &MainScreen_Blank_Opcode;
        }
        loopCount -= coldraw_cnt - 32;
    }

    for (;loopCount--;) {
        if (coldraw_osd >= 13 && coldraw_osd <= 13 - 1 + 24 * OSD_FONT_W / 8) { // 27 for font 5x8, 30 for font 6x8
            lineptr32+=2;
            attOffset++;
            bmpOffset++;
        } else {
            uint8_t att = grmem[attOffset++];
            uint8_t bmp = (att & flashing) ? ~grmem[bmpOffset++] : grmem[bmpOffset++];
            *lineptr32++ = AluByte[bmp >> 4][att];
            *lineptr32++ = AluByte[bmp & 0xF][att];
        }
        coldraw_osd++;
    }

}

IRAM_ATTR void VIDEO::MainScreen_Opcode(bool contended) { Draw(4,contended); }

// ----------------------------------------------------------------------------------
// ULA cycle perfect emulation with snow effect support
// ----------------------------------------------------------------------------------
IRAM_ATTR void VIDEO::MainScreen_Snow(unsigned int statestoadd, bool contended) {

    bool do_stats = false;

    if (contended) statestoadd += wait_st[coldraw_cnt + (Z80Ops::is2a3 ? 3 : 0)];

    CPU::tstates += statestoadd;

    unsigned int col_osd = coldraw_cnt >> 2;
    if (linedraw_cnt >= 176 && linedraw_cnt <= 191) do_stats = Draw_OSD169 == MainScreen_OSD || Draw_OSD169 == MainScreen_OSD_2A3;

    coldraw_cnt += statestoadd;

    if (coldraw_cnt >= 128) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw = &Blank_Snow;
            Draw_Opcode = &Blank_Snow_Opcode;
        } else {
            Draw = &MainScreen_Blank_Snow;
            Draw_Opcode = &MainScreen_Blank_Snow_Opcode;
        }
        statestoadd -= coldraw_cnt - 128;
    }

    for (;statestoadd--;) {

        switch(dispUpdCycle) {

            // In Weiv's Spectramine cycle starts in 2 and half black strip shows at 14349 in ptime-128.tap (early timings).
            // In SpecEmu cycle starts in 3, black strip at 14350. Will use Weiv's data for now.
            case 2:
                bmp1 = grmem[bmpOffset++];
                lastbmp = bmp1;
                break;
            case 3:
                if (snow_att) {
                    att1 = MemESP::ram[snowpage][(attOffset++ & 0xff80) | snowR];  // get attribute byte
                    snow_att = false;
                } else
                    att1 = grmem[attOffset++];  // get attribute byte

                lastatt = att1;

                if (do_stats && (col_osd >= 13 && col_osd <= 30)) {
                    lineptr32 += 2;
                } else {
                    if (att1 & flashing) bmp1 = ~bmp1;
                    *lineptr32++ = AluByte[bmp1 >> 4][att1];
                    *lineptr32++ = AluByte[bmp1 & 0xF][att1];
                }

                col_osd++;

                break;
            case 4:
                bmp2 = grmem[bmpOffset++];
                break;
            case 5:
                if (dbl_att) {
                    att2 = lastatt;
                    attOffset++;
                    dbl_att = false;
                } else
                    att2 = grmem[attOffset++];  // get attribute byte

                if (do_stats && (col_osd >= 13 && col_osd <= 30)) {
                    lineptr32 += 2;
                } else {
                    if (att2 & flashing) bmp2 = ~bmp2;
                    *lineptr32++ = AluByte[bmp2 >> 4][att2];
                    *lineptr32++ = AluByte[bmp2 & 0xF][att2];

                }

                col_osd++;

                break;

            case 6:

                MemESP::lastContendedMemReadWrite = att2;

        }

        ++dispUpdCycle &= 0x07; // Update the cycle counter.

    }

}

// ----------------------------------------------------------------------------------
// ULA cycle perfect emulation with snow effect support
// ----------------------------------------------------------------------------------
IRAM_ATTR void VIDEO::MainScreen_Snow_Opcode(bool contended) {

    int snow_effect = 0;
    unsigned int addr;
    bool do_stats = false;

    unsigned int statestoadd = video_opcode_rest ? video_opcode_rest : 4;

    if (contended) statestoadd += wait_st[coldraw_cnt + (Z80Ops::is2a3 ? 3 : 0)];

    CPU::tstates += statestoadd;

    unsigned int col_osd = coldraw_cnt >> 2;
    if (linedraw_cnt >= 176 && linedraw_cnt <= 191) do_stats = Draw_OSD169 == MainScreen_OSD || Draw_OSD169 == MainScreen_OSD_2A3;

    coldraw_cnt += statestoadd;

    if (coldraw_cnt >= 128) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw =&Blank_Snow;
            Draw_Opcode = &Blank_Snow_Opcode;
        } else {
            Draw = &MainScreen_Blank_Snow;
            Draw_Opcode = &MainScreen_Blank_Snow_Opcode;
        }

        statestoadd -= coldraw_cnt - 128;

    }

    // +2A/3 floating bus compatibility
    if (!Z80Ops::is2a3 && dispUpdCycle == 6) {
        dispUpdCycle = 2;
        return;
    }

    // Determine if snow effect can be applied
    uint8_t page = Z80::getRegI() & 0xc0;
    if (page == 0x40) { // Snow 48K, 128K
        snow_effect = Z80Ops::is48 || Z80Ops::is128;
        snowpage = MemESP::videoLatch ? 7 : 5;
    } else if (Z80Ops::is128 && (MemESP::bankLatch & 0x01) && page == 0xc0) {  // Snow 128K
        snow_effect = Z80Ops::is48 || Z80Ops::is128;
        if (MemESP::bankLatch == 1 || MemESP::bankLatch == 3)
            snowpage = MemESP::videoLatch ? 3 : 1;
        else
            snowpage = MemESP::videoLatch ? 7 : 5;
    }

    for (;statestoadd--;) {

        switch(dispUpdCycle) {

            // In Weiv's Spectramine cycle starts in 2 and half black strip shows at 14349 in ptime-128.tap (early timings).
            // In SpecEmu cycle starts in 3, black strip at 14350. Will use Weiv's data for now.

            case 2:

                if (snow_effect && statestoadd == 0) {
                    snowR = Z80::getRegR() & 0x7f;
                    bmp1 = MemESP::ram[snowpage][(bmpOffset++ & 0xff80) | snowR];
                    snow_att = true;
                } else
                    bmp1 = grmem[bmpOffset++];

                lastbmp = bmp1;

                break;

            case 3:

                if (snow_att) {
                    att1 = MemESP::ram[snowpage][(attOffset++ & 0xff80) | snowR];  // get attribute byte
                    snow_att = false;
                } else
                    att1 = grmem[attOffset++];  // get attribute byte

                lastatt = att1;

                if (do_stats && (col_osd >= 13 && col_osd <= 30)) {
                    lineptr32 += 2;
                } else {
                    if (att1 & flashing) bmp1 = ~bmp1;
                    *lineptr32++ = AluByte[bmp1 >> 4][att1];
                    *lineptr32++ = AluByte[bmp1 & 0xF][att1];
                }

                col_osd++;

                break;

            case 4:

                if (snow_effect && statestoadd == 0) {
                    bmp2 = lastbmp;
                    bmpOffset++;
                    dbl_att = true;
                } else
                    bmp2 = grmem[bmpOffset++];

                break;

            case 5:

                if (dbl_att) {
                    att2 = lastatt;
                    attOffset++;
                    dbl_att = false;
                } else
                    att2 = grmem[attOffset++];  // get attribute byte

                if (do_stats && (col_osd >= 13 && col_osd <= 30)) {
                    lineptr32 += 2;
                } else {
                    if (att2 & flashing) bmp2 = ~bmp2;
                    *lineptr32++ = AluByte[bmp2 >> 4][att2];
                    *lineptr32++ = AluByte[bmp2 & 0xF][att2];
                }

                col_osd++;

                break;

            case 6:

                MemESP::lastContendedMemReadWrite = att2;

        }

        ++dispUpdCycle &= 0x07; // Update the cycle counter.

    }

}

IRAM_ATTR void VIDEO::Blank(unsigned int statestoadd, bool contended) { CPU::tstates += statestoadd; }
IRAM_ATTR void VIDEO::Blank_Opcode(bool contended) { CPU::tstates += 4; }
IRAM_ATTR void VIDEO::Blank_Snow(unsigned int statestoadd, bool contended) { CPU::tstates += statestoadd; }
IRAM_ATTR void VIDEO::Blank_Snow_Opcode(bool contended) { CPU::tstates += 4; }

IRAM_ATTR void VIDEO::EndFrame() {

    linedraw_cnt = lin_end;

    tstateDraw = tStatesScreen;

    if (VIDEO::snow_toggle) {
        Draw = &MainScreen_Blank_Snow;
        Draw_Opcode = &MainScreen_Blank_Snow_Opcode;
    } else {
        Draw = Z80Ops::is2a3 ? &MainScreen_Blank_2A3 : &MainScreen_Blank;
        Draw_Opcode = Z80Ops::is2a3 ? &MainScreen_Blank_Opcode_2A3 : &MainScreen_Blank_Opcode;
    }

    if (brdChange || brdnextframe) {
        DrawBorder();
        brdnextframe = brdChange;
    }

    // Restart border drawing
    DrawBorder = Z80Ops::isPentagon ? &TopBorder_Blank_Pentagon : &TopBorder_Blank;

    lastBrdTstate = tStatesBorder;
    brdChange = false;

    framecnt++;

}

//----------------------------------------------------------------------------------------------------------------
// Border Drawing
//----------------------------------------------------------------------------------------------------------------

static int brdcol_cnt = 0;
static int brdlin_cnt = 0;

IRAM_ATTR void VIDEO::TopBorder_Blank() {

    if (CPU::tstates >= tStatesBorder) {
        brdcol_cnt = 0;
        brdlin_cnt = 0;
        brdptr32 = (uint32_t *)(VIDEO::frameBuffer()[brdlin_cnt]) + (Config::videomode == 2 ? 0 : (is169 ? 1 : 0));
        DrawBorder = &TopBorder;
        DrawBorder();
    }

}

IRAM_ATTR void VIDEO::TopBorder() {

    while (lastBrdTstate <= CPU::tstates) {

        *brdptr32++ = brd;
        *brdptr32++ = brd;

        lastBrdTstate += 4;

        brdcol_cnt++;

        if (brdcol_cnt == (Config::videomode == 2 || is169 ? 44 : 40)) {
            brdlin_cnt++;
            brdptr32 = (uint32_t *)(VIDEO::frameBuffer()[brdlin_cnt]) + (Config::videomode == 2 ? 0 : (is169 ? 1 : 0));
            brdcol_cnt = 0;
            lastBrdTstate += brdnextline;
            if (brdlin_cnt == lin_end) {
                DrawBorder = &MiddleBorder;
                MiddleBorder();
                return;
            }
        }

    }

}

IRAM_ATTR void VIDEO::MiddleBorder() {

    while (lastBrdTstate <= CPU::tstates) {

        *brdptr32++ = brd;
        *brdptr32++ = brd;

        lastBrdTstate+=4;

        brdcol_cnt++;

        if (brdcol_cnt == (Config::videomode == 2 || is169 ? 6 : 4)) {
            lastBrdTstate += 128;
            brdptr32 += 64;
            brdcol_cnt += 32;
        } else if (brdcol_cnt == (Config::videomode == 2 || is169 ? 44 : 40)) {
            brdlin_cnt++;
            brdptr32 = (uint32_t *)(VIDEO::frameBuffer()[brdlin_cnt]) + (Config::videomode == 2 ? 0 : (is169 ? 1 : 0));
            brdcol_cnt = 0;
            lastBrdTstate += brdnextline;
            if (brdlin_cnt == lin_end2) {
                DrawBorder = Draw_OSD43;
                DrawBorder();
                return;
            }
        }

    }

}

IRAM_ATTR void VIDEO::BottomBorder() {

    while (lastBrdTstate <= CPU::tstates) {

        *brdptr32++ = brd;
        *brdptr32++ = brd;

        lastBrdTstate+=4;

        brdcol_cnt++;

        if (brdcol_cnt == (Config::videomode == 2 || is169 ? 44 : 40)) {
            brdlin_cnt++;
            brdptr32 = (uint32_t *)(VIDEO::frameBuffer()[brdlin_cnt]) + (Config::videomode == 2 ? 0 : (is169 ? 1 : 0));
            brdcol_cnt = 0;
            lastBrdTstate += brdnextline;
            if (brdlin_cnt == OSD::scrH) {
                DrawBorder = &Border_Blank;
                return;
            }
        }

    }

}

IRAM_ATTR void VIDEO::BottomBorder_OSD() {

    while (lastBrdTstate <= CPU::tstates) {

        if (brdlin_cnt < brdlin_osdstart || brdlin_cnt > brdlin_osdend) {
            *brdptr32++ = brd;
            *brdptr32++ = brd;
        } else if (brdcol_cnt < 21 || brdcol_cnt > 21 - 1 + 24 * OSD_FONT_W / 8) { // 35 for font 5x8, 38 for font 6x8
            *brdptr32++ = brd;
            *brdptr32++ = brd;
        } else {
            brdptr32 += 2;
        }

        lastBrdTstate+=4;

        brdcol_cnt++;

        if (brdcol_cnt == (Config::videomode == 2 || is169 ? 44 : 40)) {
            brdlin_cnt++;
            brdptr32 = (uint32_t *)(VIDEO::frameBuffer()[brdlin_cnt]) + (Config::videomode == 2 ? 0 : (is169 ? 1 : 0));
            brdcol_cnt = 0;
            lastBrdTstate += brdnextline;
            if (brdlin_cnt == OSD::scrH) {
                DrawBorder = &Border_Blank;
                return;
            }
        }

    }

}

IRAM_ATTR void VIDEO::Border_Blank() {

}

static int brdcol_end = 0;
static int brdcol_end1 = 0;

IRAM_ATTR void VIDEO::TopBorder_Blank_Pentagon() {

    if (CPU::tstates >= tStatesBorder) {

        brdcol_cnt = Config::videomode == 2 ? 0 : (is169 ? 2 : 0);
        brdcol_end = Config::videomode == 2 ? 176 : (is169 ? 178 : 160);
        brdcol_end1 = Config::videomode == 2 ? 24 : (is169 ? 26 : 16);

        brdlin_cnt = 0;
        brdptr16 = (uint16_t *)(VIDEO::frameBuffer()[brdlin_cnt]);
        DrawBorder = &TopBorder_Pentagon;
        DrawBorder();
    }

}

IRAM_ATTR void VIDEO::TopBorder_Pentagon() {

    while (lastBrdTstate <= CPU::tstates) {

        brdptr16[brdcol_cnt ^ 1] = (uint16_t) brd;

        lastBrdTstate++;

        brdcol_cnt++;

        if (brdcol_cnt == brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(VIDEO::frameBuffer()[brdlin_cnt]);
            brdcol_cnt = Config::videomode == 2 ? 0 : (is169 ? 2 : 0);
            lastBrdTstate += brdnextline;
            if (brdlin_cnt == lin_end) {
                DrawBorder = &MiddleBorder_Pentagon;
                MiddleBorder_Pentagon();
                return;
            }
        }

    }

}

IRAM_ATTR void VIDEO::MiddleBorder_Pentagon() {

    while (lastBrdTstate <= CPU::tstates) {

        brdptr16[brdcol_cnt ^ 1] = (uint16_t) brd;

        lastBrdTstate++;

        brdcol_cnt++;

        if (brdcol_cnt == brdcol_end1) {
            lastBrdTstate += 128;
            brdcol_cnt = Config::videomode == 2 ? 152 : (is169 ? 154 : 144);
        } else if (brdcol_cnt == brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(VIDEO::frameBuffer()[brdlin_cnt]);
            brdcol_cnt = Config::videomode == 2 ? 0 : (is169 ? 2 : 0);
            lastBrdTstate += brdnextline;
            if (brdlin_cnt == lin_end2) {
                DrawBorder = Draw_OSD43;
                DrawBorder();
                return;
            }
        }

    }

}

IRAM_ATTR void VIDEO::BottomBorder_Pentagon() {

    while (lastBrdTstate <= CPU::tstates) {

        brdptr16[brdcol_cnt ^ 1] = (uint16_t) brd;

        lastBrdTstate++;

        brdcol_cnt++;

        if (brdcol_cnt == brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(VIDEO::frameBuffer()[brdlin_cnt]);
            brdcol_cnt = Config::videomode == 2 ? 0 : (is169 ? 2 : 0);
            lastBrdTstate += brdnextline;
            if (brdlin_cnt == OSD::scrH) {
                DrawBorder = &Border_Blank;
                return;
            }
        }

    }

}

IRAM_ATTR void VIDEO::BottomBorder_OSD_Pentagon() {

    while (lastBrdTstate <= CPU::tstates) {

        if (brdlin_cnt < brdlin_osdstart || brdlin_cnt > brdlin_osdend)
            brdptr16[brdcol_cnt ^ 1] = (uint16_t) brd;
        else if (brdcol_cnt < 84 || brdcol_cnt > 155)
            brdptr16[brdcol_cnt ^ 1] = (uint16_t) brd;

        lastBrdTstate++;

        brdcol_cnt++;

        if (brdcol_cnt == brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(VIDEO::frameBuffer()[brdlin_cnt]);
            brdcol_cnt = 0;
            lastBrdTstate += brdnextline;
            if (brdlin_cnt == OSD::scrH) {
                DrawBorder = &Border_Blank;
                return;
            }
        }

    }

}

