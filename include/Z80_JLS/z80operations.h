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

///////////////////////////////////////////////////////////////////////////////
//
// z80cpp - Z80 emulator core
//
// Copyright (c) 2017, 2018, 2019, 2020 jsanchezv - https://github.com/jsanchezv
//
// Heretic optimizations and minor adaptations
// Copyright (c) 2021 dcrespo3d - https://github.com/dcrespo3d
//

#ifndef Z80OPERATIONS_H
#define Z80OPERATIONS_H

#include <stdint.h>
#include "esp_attr.h"

// Class grouping callbacks for Z80 operations
// methods declared here should be defined elsewhere... for example, in CPU.cpp

class Z80Ops
{
public:
    // Fetch opcode from RAM
    static uint8_t (*fetchOpcode)();
    static uint8_t fetchOpcode_2A3();
    static uint8_t fetchOpcode_std();

    // Read byte from RAM
    static uint8_t (*peek8)(uint16_t address);
    static uint8_t peek8_2A3(uint16_t address);
    static uint8_t peek8_std(uint16_t address);

    // Write byte to RAM
    static void (*poke8)(uint16_t address, uint8_t value);
    static void poke8_2A3(uint16_t address, uint8_t value);
    static void poke8_std(uint16_t address, uint8_t value);

    // Read word from RAM
    static uint16_t (*peek16)(uint16_t address);
    static uint16_t peek16_2A3(uint16_t adddress);
    static uint16_t peek16_std(uint16_t adddress);

    // Write word to RAM
    static void (*poke16)(uint16_t address, RegisterPair word);
    static void poke16_2A3(uint16_t address, RegisterPair word);
    static void poke16_std(uint16_t address, RegisterPair word);

    // Put an address on bus lasting 'tstates' cycles
    static void (*addressOnBus)(uint16_t address, int32_t wstates);
    static void addressOnBus_2A3(uint16_t address, int32_t wstates);
    static void addressOnBus_std(uint16_t address, int32_t wstates);

    // Callback to know when the INT signal is active
    static bool isActiveINT(void);

    static bool is48;
    static bool is128;
    static bool isPentagon;
    static bool is2a3;

};

#endif // Z80OPERATIONS_H
