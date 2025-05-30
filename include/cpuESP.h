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


#ifndef CPU_h
#define CPU_h

#include <inttypes.h>
#include "ESPeccy.h"

#define TSTATES_PER_FRAME_48 69888
#define TSTATES_PER_FRAME_TK_50 71136
#define TSTATES_PER_FRAME_TK_60 59736
#define TSTATES_PER_FRAME_128 70908
#define TSTATES_PER_FRAME_PENTAGON 71680

#define MICROS_PER_FRAME_48 19968
#define MICROS_PER_FRAME_48_125SPEED 15974
#define MICROS_PER_FRAME_48_150SPEED 13312

#define MICROS_PER_FRAME_TK_50 19895
#define MICROS_PER_FRAME_TK_50_125SPEED 15916
#define MICROS_PER_FRAME_TK_50_150SPEED 13263

#define MICROS_PER_FRAME_TK_60 16707
#define MICROS_PER_FRAME_TK_60_125SPEED 13366
#define MICROS_PER_FRAME_TK_60_150SPEED 11138

#define MICROS_PER_FRAME_128 19992
#define MICROS_PER_FRAME_128_125SPEED 15994
#define MICROS_PER_FRAME_128_150SPEED 13328

#define MICROS_PER_FRAME_PENTAGON 20480
#define MICROS_PER_FRAME_PENTAGON_125SPEED 16384
#define MICROS_PER_FRAME_PENTAGON_150SPEED 13653

#define INT_START48 0
#define INT_END48 32
#define INT_STARTTK 0
#define INT_ENDTK 32
#define INT_START128 0
#define INT_END128 36 // 35 in real +2 and Weiv's Spectramine. I'll have to check those numbers
#define INT_STARTPLUS2A3 0
#define INT_ENDPLUS2A3 32
#define INT_START_PENTAGON 0
#define INT_END_PENTAGON 36

class CPU
{
public:

    // call this for executing a frame's worth of instructions
    static void loop();

    // call this for resetting the CPU
    static void reset();

    // Flush screen
    static void FlushOnHalt();

    // CPU Tstates elapsed in current frame
    static uint32_t tstates;

    static int32_t prev_tstates;

    static uint32_t tstates_diff;

    // CPU Tstates elapsed since reset
    static uint64_t global_tstates;

    // CPU Tstates in frame
    static uint32_t statesInFrame;

    // Late timing
    static uint8_t latetiming;

    // INT signal lenght
    static uint8_t IntStart;
    static uint8_t IntEnd;

    // CPU Tstates in frame - IntEnd
    static uint32_t stFrame;

};

#endif // CPU_h
