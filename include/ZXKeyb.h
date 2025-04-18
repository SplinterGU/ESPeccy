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


#ifndef ZXKEYB_h
#define ZXKEYB_h

#include <inttypes.h>

#include "ESPeccy.h"

//#define ZXKBDREAD_MODENORMAL 0
//#define ZXKBDREAD_MODEINTERACTIVE 1
//#define ZXKBDREAD_MODEINPUT 2
//#define ZXKBDREAD_MODEKBDLAYOUT 3

#define ZXKBD_CS        (!bitRead(ZXKeyb::ZXcols[0], 0))
#define ZXKBD_Z         (!bitRead(ZXKeyb::ZXcols[0], 1))
#define ZXKBD_X         (!bitRead(ZXKeyb::ZXcols[0], 2))
#define ZXKBD_C         (!bitRead(ZXKeyb::ZXcols[0], 3))
#define ZXKBD_V         (!bitRead(ZXKeyb::ZXcols[0], 4))
#define ZXKBD_A         (!bitRead(ZXKeyb::ZXcols[1], 0))
#define ZXKBD_S         (!bitRead(ZXKeyb::ZXcols[1], 1))
#define ZXKBD_D         (!bitRead(ZXKeyb::ZXcols[1], 2))
#define ZXKBD_F         (!bitRead(ZXKeyb::ZXcols[1], 3))
#define ZXKBD_G         (!bitRead(ZXKeyb::ZXcols[1], 4))
#define ZXKBD_Q         (!bitRead(ZXKeyb::ZXcols[2], 0))
#define ZXKBD_W         (!bitRead(ZXKeyb::ZXcols[2], 1))
#define ZXKBD_E         (!bitRead(ZXKeyb::ZXcols[2], 2))
#define ZXKBD_R         (!bitRead(ZXKeyb::ZXcols[2], 3))
#define ZXKBD_T         (!bitRead(ZXKeyb::ZXcols[2], 4))
#define ZXKBD_1         (!bitRead(ZXKeyb::ZXcols[3], 0))
#define ZXKBD_2         (!bitRead(ZXKeyb::ZXcols[3], 1))
#define ZXKBD_3         (!bitRead(ZXKeyb::ZXcols[3], 2))
#define ZXKBD_4         (!bitRead(ZXKeyb::ZXcols[3], 3))
#define ZXKBD_5         (!bitRead(ZXKeyb::ZXcols[3], 4))
#define ZXKBD_0         (!bitRead(ZXKeyb::ZXcols[4], 0))
#define ZXKBD_9         (!bitRead(ZXKeyb::ZXcols[4], 1))
#define ZXKBD_8         (!bitRead(ZXKeyb::ZXcols[4], 2))
#define ZXKBD_7         (!bitRead(ZXKeyb::ZXcols[4], 3))
#define ZXKBD_6         (!bitRead(ZXKeyb::ZXcols[4], 4))
#define ZXKBD_P         (!bitRead(ZXKeyb::ZXcols[5], 0))
#define ZXKBD_O         (!bitRead(ZXKeyb::ZXcols[5], 1))
#define ZXKBD_I         (!bitRead(ZXKeyb::ZXcols[5], 2))
#define ZXKBD_U         (!bitRead(ZXKeyb::ZXcols[5], 3))
#define ZXKBD_Y         (!bitRead(ZXKeyb::ZXcols[5], 4))
#define ZXKBD_ENTER     (!bitRead(ZXKeyb::ZXcols[6], 0))
#define ZXKBD_L         (!bitRead(ZXKeyb::ZXcols[6], 1))
#define ZXKBD_K         (!bitRead(ZXKeyb::ZXcols[6], 2))
#define ZXKBD_J         (!bitRead(ZXKeyb::ZXcols[6], 3))
#define ZXKBD_H         (!bitRead(ZXKeyb::ZXcols[6], 4))
#define ZXKBD_SPACE     (!bitRead(ZXKeyb::ZXcols[7], 0))
#define ZXKBD_SS        (!bitRead(ZXKeyb::ZXcols[7], 1))
#define ZXKBD_M         (!bitRead(ZXKeyb::ZXcols[7], 2))
#define ZXKBD_N         (!bitRead(ZXKeyb::ZXcols[7], 3))
#define ZXKBD_B         (!bitRead(ZXKeyb::ZXcols[7], 4))

class ZXKeyb {

public:

    static void setup();    // setup pins for physical keyboard
    static void process();  // process physical keyboard
    static void ZXKbdRead(uint8_t mode = KBDREAD_MODENORMAL);

    static uint8_t ZXcols[8];
    static uint8_t Exists;

private:

    static void putRows(uint8_t row_pattern);
    static uint8_t getCols();

};

#endif // ZXKEYB_h
