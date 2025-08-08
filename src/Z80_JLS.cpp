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
// ESPectrum specific optimizations and code
// Copyright (c) 2023-24 Víctor Iborra [Eremus] - https://github.com/EremusOne
//

// Converted to C++ from Java at
//... https://github.com/jsanchezv/Z80Core
//... commit c4f267e3564fa89bd88fd2d1d322f4d6b0069dbd
//... GPL 3
//... v1.0.0 (13/02/2017)
//    quick & dirty conversion by dddddd (AKA deesix)

#include <stdio.h>
#include <algorithm>
#include <sys/stat.h>

#include "Z80_JLS/z80.h"
#include "Ports.h"
#include "Video.h"
#include "MemESP.h"
#include "cpuESP.h"
#include "Tape.h"
#include "Config.h"
#include "FileUtils.h"
#include "OSD.h"
#include "messages.h"

#include "RealTape.h"

#include <functional>

// #pragma GCC optimize("O3")

//uint8_t page;
// #define FETCH_OPCODE(result,address) page = address >> 14; VIDEO::Draw(4,MemESP::ramContended[page]); result = MemESP::ramCurrent[page][address & 0x3fff];
// #define PEEK8(result,address) page = address >> 14; VIDEO::Draw(3,MemESP::ramContended[page]); result = MemESP::ramCurrent[page][address & 0x3fff];

///////////////////////////////////////////////////////////////////////////////
// miembros estáticos

uint8_t Z80::opCode;
uint8_t Z80::prefixOpcode = { 0x00 };
//bool Z80::execDone;
uint8_t Z80::regA;
uint8_t Z80::sz5h3pnFlags;
bool Z80::carryFlag;
RegisterPair Z80::regBC, Z80::regBCx, Z80::regDE, Z80::regDEx, Z80::regHL, Z80::regHLx;
bool Z80::flagQ;
bool Z80::lastFlagQ;
RegisterPair Z80::regAFx;
RegisterPair Z80::regPC;
RegisterPair Z80::regIX;
RegisterPair Z80::regIY;
RegisterPair Z80::regSP;
uint8_t Z80::regI;
uint8_t Z80::regR;
bool Z80::regRbit7;
bool Z80::ffIFF1 = false;
bool Z80::ffIFF2 = false;
bool Z80::pendingEI = false;
bool Z80::activeNMI = false;
Z80::IntMode Z80::modeINT = Z80::IntMode::IM0;
bool Z80::halted = false;
bool Z80::pinReset = false;
RegisterPair Z80::memptr;

const uint8_t Z80::sz53n_addTable[256] = {
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8,
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8,
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8,
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8, 0xA8
};

const uint8_t Z80::sz53pn_addTable[256] = {
    0x44, 0x00, 0x00, 0x04, 0x00, 0x04, 0x04, 0x00, 0x08, 0x0C, 0x0C, 0x08, 0x0C, 0x08, 0x08, 0x0C,
    0x00, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x04, 0x0C, 0x08, 0x08, 0x0C, 0x08, 0x0C, 0x0C, 0x08,
    0x20, 0x24, 0x24, 0x20, 0x24, 0x20, 0x20, 0x24, 0x2C, 0x28, 0x28, 0x2C, 0x28, 0x2C, 0x2C, 0x28,
    0x24, 0x20, 0x20, 0x24, 0x20, 0x24, 0x24, 0x20, 0x28, 0x2C, 0x2C, 0x28, 0x2C, 0x28, 0x28, 0x2C,
    0x00, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x04, 0x0C, 0x08, 0x08, 0x0C, 0x08, 0x0C, 0x0C, 0x08,
    0x04, 0x00, 0x00, 0x04, 0x00, 0x04, 0x04, 0x00, 0x08, 0x0C, 0x0C, 0x08, 0x0C, 0x08, 0x08, 0x0C,
    0x24, 0x20, 0x20, 0x24, 0x20, 0x24, 0x24, 0x20, 0x28, 0x2C, 0x2C, 0x28, 0x2C, 0x28, 0x28, 0x2C,
    0x20, 0x24, 0x24, 0x20, 0x24, 0x20, 0x20, 0x24, 0x2C, 0x28, 0x28, 0x2C, 0x28, 0x2C, 0x2C, 0x28,
    0x80, 0x84, 0x84, 0x80, 0x84, 0x80, 0x80, 0x84, 0x8C, 0x88, 0x88, 0x8C, 0x88, 0x8C, 0x8C, 0x88,
    0x84, 0x80, 0x80, 0x84, 0x80, 0x84, 0x84, 0x80, 0x88, 0x8C, 0x8C, 0x88, 0x8C, 0x88, 0x88, 0x8C,
    0xA4, 0xA0, 0xA0, 0xA4, 0xA0, 0xA4, 0xA4, 0xA0, 0xA8, 0xAC, 0xAC, 0xA8, 0xAC, 0xA8, 0xA8, 0xAC,
    0xA0, 0xA4, 0xA4, 0xA0, 0xA4, 0xA0, 0xA0, 0xA4, 0xAC, 0xA8, 0xA8, 0xAC, 0xA8, 0xAC, 0xAC, 0xA8,
    0x84, 0x80, 0x80, 0x84, 0x80, 0x84, 0x84, 0x80, 0x88, 0x8C, 0x8C, 0x88, 0x8C, 0x88, 0x88, 0x8C,
    0x80, 0x84, 0x84, 0x80, 0x84, 0x80, 0x80, 0x84, 0x8C, 0x88, 0x88, 0x8C, 0x88, 0x8C, 0x8C, 0x88,
    0xA0, 0xA4, 0xA4, 0xA0, 0xA4, 0xA0, 0xA0, 0xA4, 0xAC, 0xA8, 0xA8, 0xAC, 0xA8, 0xAC, 0xAC, 0xA8,
    0xA4, 0xA0, 0xA0, 0xA4, 0xA0, 0xA4, 0xA4, 0xA0, 0xA8, 0xAC, 0xAC, 0xA8, 0xAC, 0xA8, 0xA8, 0xAC
};

const uint8_t Z80::sz53n_subTable[256] = {
    0x42, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
    0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A,
    0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A,
    0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A,
    0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A, 0x8A,
    0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
    0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xA2, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA
};

const uint8_t Z80::sz53pn_subTable[256] = {
    0x46, 0x02, 0x02, 0x06, 0x02, 0x06, 0x06, 0x02, 0x0A, 0x0E, 0x0E, 0x0A, 0x0E, 0x0A, 0x0A, 0x0E,
    0x02, 0x06, 0x06, 0x02, 0x06, 0x02, 0x02, 0x06, 0x0E, 0x0A, 0x0A, 0x0E, 0x0A, 0x0E, 0x0E, 0x0A,
    0x22, 0x26, 0x26, 0x22, 0x26, 0x22, 0x22, 0x26, 0x2E, 0x2A, 0x2A, 0x2E, 0x2A, 0x2E, 0x2E, 0x2A,
    0x26, 0x22, 0x22, 0x26, 0x22, 0x26, 0x26, 0x22, 0x2A, 0x2E, 0x2E, 0x2A, 0x2E, 0x2A, 0x2A, 0x2E,
    0x02, 0x06, 0x06, 0x02, 0x06, 0x02, 0x02, 0x06, 0x0E, 0x0A, 0x0A, 0x0E, 0x0A, 0x0E, 0x0E, 0x0A,
    0x06, 0x02, 0x02, 0x06, 0x02, 0x06, 0x06, 0x02, 0x0A, 0x0E, 0x0E, 0x0A, 0x0E, 0x0A, 0x0A, 0x0E,
    0x26, 0x22, 0x22, 0x26, 0x22, 0x26, 0x26, 0x22, 0x2A, 0x2E, 0x2E, 0x2A, 0x2E, 0x2A, 0x2A, 0x2E,
    0x22, 0x26, 0x26, 0x22, 0x26, 0x22, 0x22, 0x26, 0x2E, 0x2A, 0x2A, 0x2E, 0x2A, 0x2E, 0x2E, 0x2A,
    0x82, 0x86, 0x86, 0x82, 0x86, 0x82, 0x82, 0x86, 0x8E, 0x8A, 0x8A, 0x8E, 0x8A, 0x8E, 0x8E, 0x8A,
    0x86, 0x82, 0x82, 0x86, 0x82, 0x86, 0x86, 0x82, 0x8A, 0x8E, 0x8E, 0x8A, 0x8E, 0x8A, 0x8A, 0x8E,
    0xA6, 0xA2, 0xA2, 0xA6, 0xA2, 0xA6, 0xA6, 0xA2, 0xAA, 0xAE, 0xAE, 0xAA, 0xAE, 0xAA, 0xAA, 0xAE,
    0xA2, 0xA6, 0xA6, 0xA2, 0xA6, 0xA2, 0xA2, 0xA6, 0xAE, 0xAA, 0xAA, 0xAE, 0xAA, 0xAE, 0xAE, 0xAA,
    0x86, 0x82, 0x82, 0x86, 0x82, 0x86, 0x86, 0x82, 0x8A, 0x8E, 0x8E, 0x8A, 0x8E, 0x8A, 0x8A, 0x8E,
    0x82, 0x86, 0x86, 0x82, 0x86, 0x82, 0x82, 0x86, 0x8E, 0x8A, 0x8A, 0x8E, 0x8A, 0x8E, 0x8E, 0x8A,
    0xA2, 0xA6, 0xA6, 0xA2, 0xA6, 0xA2, 0xA2, 0xA6, 0xAE, 0xAA, 0xAA, 0xAE, 0xAA, 0xAE, 0xAE, 0xAA,
    0xA6, 0xA2, 0xA2, 0xA6, 0xA2, 0xA6, 0xA6, 0xA2, 0xAA, 0xAE, 0xAE, 0xAA, 0xAE, 0xAA, 0xAA, 0xAE
};


///////////////////////////////////////////////////////////////////////////////

// Constructor de la clase
void Z80::create() {
#if 0
    bool evenBits;

    for (uint32_t idx = 0; idx < 256; idx++) {
		sz53n_addTable[idx] = 0;
		sz53pn_addTable[idx] = 0;
		sz53n_subTable[idx] = 0;
		sz53pn_subTable[idx] = 0;

		if (idx > 0x7f) {
            sz53n_addTable[idx] |= SIGN_MASK;
        }

        evenBits = true;
        for (uint8_t mask = 0x01; mask != 0; mask <<= 1) {
            if ((idx & mask) != 0) {
                evenBits = !evenBits;
            }
        }

        sz53n_addTable[idx] |= (idx & FLAG_53_MASK);
        sz53n_subTable[idx] = sz53n_addTable[idx] | ADDSUB_MASK;

        if (evenBits) {
            sz53pn_addTable[idx] = sz53n_addTable[idx] | PARITY_MASK;
            sz53pn_subTable[idx] = sz53n_subTable[idx] | PARITY_MASK;
        } else {
            sz53pn_addTable[idx] = sz53n_addTable[idx];
            sz53pn_subTable[idx] = sz53n_subTable[idx];
        }
    }

    sz53n_addTable[0] |= ZERO_MASK;
    sz53pn_addTable[0] |= ZERO_MASK;
    sz53n_subTable[0] |= ZERO_MASK;
    sz53pn_subTable[0] |= ZERO_MASK;

    execDone = false;

    // reset();
#endif
}

void Z80::destroy(void)
{
}

RegisterPair Z80::getPairIR(void) {
    RegisterPair IR;
    IR.byte8.hi = regI;
    IR.byte8.lo = regR & 0x7f;
    if (regRbit7) {
        IR.byte8.lo |= SIGN_MASK;
    }
    return IR;
}

void Z80::setAddSubFlag(bool state) {
    if (state) {
        sz5h3pnFlags |= ADDSUB_MASK;
    } else {
        sz5h3pnFlags &= ~ADDSUB_MASK;
    }
}

void Z80::setParOverFlag(bool state) {
    if (state) {
        sz5h3pnFlags |= PARITY_MASK;
    } else {
        sz5h3pnFlags &= ~PARITY_MASK;
    }
}

void Z80::setBit3Fag(bool state) {
    if (state) {
        sz5h3pnFlags |= BIT3_MASK;
    } else {
        sz5h3pnFlags &= ~BIT3_MASK;
    }
}

void Z80::setHalfCarryFlag(bool state) {
    if (state) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    } else {
        sz5h3pnFlags &= ~HALFCARRY_MASK;
    }
}

void Z80::setBit5Flag(bool state) {
    if (state) {
        sz5h3pnFlags |= BIT5_MASK;
    } else {
        sz5h3pnFlags &= ~BIT5_MASK;
    }
}

void Z80::setZeroFlag(bool state) {
    if (state) {
        sz5h3pnFlags |= ZERO_MASK;
    } else {
        sz5h3pnFlags &= ~ZERO_MASK;
    }
}

void Z80::setSignFlag(bool state) {
    if (state) {
        sz5h3pnFlags |= SIGN_MASK;
    } else {
        sz5h3pnFlags &= ~SIGN_MASK;
    }
}

// Reset
/* Según el documento de Sean Young, que se encuentra en
 * [http://www.myquest.com/z80undocumented], la mejor manera de emular el
 * reset es poniendo PC, IFF1, IFF2, R e IM0 a 0 y todos los demás registros
 * a 0xFFFF.
 *
 * 29/05/2011: cuando la CPU recibe alimentación por primera vez, los
 *             registros PC e IR se inicializan a cero y el resto a 0xFF.
 *             Si se produce un reset a través de la patilla correspondiente,
 *             los registros PC e IR se inicializan a 0 y el resto se preservan.
 *             En cualquier caso, todo parece depender bastante del modelo
 *             concreto de Z80, así que se escoge el comportamiento del
 *             modelo Zilog Z8400APS. Z80A CPU.
 *             http://www.worldofspectrum.org/forums/showthread.php?t=34574
 */
void Z80::reset(void) {
    if (pinReset) {
        pinReset = false;
    } else {
        regA = 0xff;

        setFlags(0xfd); // The only one flag reset at cold start is the add/sub flag

        REG_AFx = 0xffff;
        REG_BC = REG_BCx = 0xffff;
        REG_DE = REG_DEx = 0xffff;
        REG_HL = REG_HLx = 0xffff;

        REG_IX = REG_IY = 0xffff;

        REG_SP = 0xffff;

        REG_WZ = 0xffff;
    }

    REG_PC = 0;
    regI = regR = 0;
    regRbit7 = false;
    ffIFF1 = false;
    ffIFF2 = false;
    pendingEI = false;
    activeNMI = false;
    halted = false;
    setIM(IntMode::IM0);
    lastFlagQ = false;
    prefixOpcode = 0x00;
}

// Rota a la izquierda el valor del argumento
// El bit 0 y el flag C toman el valor del bit 7 antes de la operación
void Z80::rlc(uint8_t &oper8) {
    carryFlag = (oper8 > 0x7f);
    oper8 <<= 1;
    if (carryFlag) {
        oper8 |= CARRY_MASK;
    }
    sz5h3pnFlags = sz53pn_addTable[oper8];
    flagQ = true;
}
// Rota a la izquierda el valor del argumento
// El bit 7 va al carry flag
// El bit 0 toma el valor del flag C antes de la operación
void Z80::rl(uint8_t &oper8) {
    bool carry = carryFlag;
    carryFlag = (oper8 > 0x7f);
    oper8 <<= 1;
    if (carry) {
        oper8 |= CARRY_MASK;
    }
    sz5h3pnFlags = sz53pn_addTable[oper8];
    flagQ = true;
}

// Rota a la izquierda el valor del argumento
// El bit 7 va al carry flag
// El bit 0 toma el valor 0
void Z80::sla(uint8_t &oper8) {
    carryFlag = (oper8 > 0x7f);
    oper8 <<= 1;
    sz5h3pnFlags = sz53pn_addTable[oper8];
    flagQ = true;
}

// Rota a la izquierda el valor del argumento (como sla salvo por el bit 0)
// El bit 7 va al carry flag
// El bit 0 toma el valor 1
// Instrucción indocumentada
void Z80::sll(uint8_t &oper8) {
    carryFlag = (oper8 > 0x7f);
    oper8 <<= 1;
    oper8 |= CARRY_MASK;
    sz5h3pnFlags = sz53pn_addTable[oper8];
    flagQ = true;
}

// Rota a la derecha el valor del argumento
// El bit 7 y el flag C toman el valor del bit 0 antes de la operación
void Z80::rrc(uint8_t &oper8) {
    carryFlag = (oper8 & CARRY_MASK) != 0;
    oper8 >>= 1;
    if (carryFlag) {
        oper8 |= SIGN_MASK;
    }
    sz5h3pnFlags = sz53pn_addTable[oper8];
    flagQ = true;
}

// Rota a la derecha el valor del argumento
// El bit 0 va al carry flag
// El bit 7 toma el valor del flag C antes de la operación
void Z80::rr(uint8_t &oper8) {
    bool carry = carryFlag;
    carryFlag = (oper8 & CARRY_MASK) != 0;
    oper8 >>= 1;
    if (carry) {
        oper8 |= SIGN_MASK;
    }
    sz5h3pnFlags = sz53pn_addTable[oper8];
    flagQ = true;
}

// Rota a la derecha 1 bit el valor del argumento
// El bit 0 pasa al carry.
// El bit 7 conserva el valor que tenga
void Z80::sra(uint8_t &oper8) {
    uint8_t sign = oper8 & SIGN_MASK;
    carryFlag = (oper8 & CARRY_MASK) != 0;
    oper8 = (oper8 >> 1) | sign;
    sz5h3pnFlags = sz53pn_addTable[oper8];
    flagQ = true;
}

// Rota a la derecha 1 bit el valor del argumento
// El bit 0 pasa al carry.
// El bit 7 toma el valor 0
void Z80::srl(uint8_t &oper8) {
    carryFlag = (oper8 & CARRY_MASK) != 0;
    oper8 >>= 1;
    sz5h3pnFlags = sz53pn_addTable[oper8];
    flagQ = true;
}

/*
 * Half-carry flag:
 *
 * FLAG = (A ^ B ^ RESULT) & 0x10  for any operation
 *
 * Overflow flag:
 *
 * FLAG = ~(A ^ B) & (B ^ RESULT) & 0x80 for addition [ADD/ADC]
 * FLAG = (A ^ B) & (A ^ RESULT) &0x80 for subtraction [SUB/SBC]
 *
 * For INC/DEC, you can use following simplifications:
 *
 * INC:
 * H_FLAG = (RESULT & 0x0F) == 0x00
 * V_FLAG = RESULT == 0x80
 *
 * DEC:
 * H_FLAG = (RESULT & 0x0F) == 0x0F
 * V_FLAG = RESULT == 0x7F
 */
// Incrementa un valor de 8 bits modificando los flags oportunos
void Z80::inc8(uint8_t &oper8) {
    ++oper8;

    sz5h3pnFlags = sz53n_addTable[oper8];

    if ((oper8 & 0x0f) == 0) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (oper8 == 0x80) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }

    flagQ = true;
    return;
}

// Decrementa un valor de 8 bits modificando los flags oportunos
void Z80::dec8(uint8_t &oper8) {
    --oper8;

    sz5h3pnFlags = sz53n_subTable[oper8];

    if ((oper8 & 0x0f) == 0x0f) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (oper8 == 0x7f) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }

    flagQ = true;
    return;
}

// Suma de 8 bits afectando a los flags
void Z80::add(uint8_t oper8) {
    uint16_t res = regA + oper8;

    carryFlag = res > 0xff;
    res &= 0xff;
    sz5h3pnFlags = sz53n_addTable[res];

    /* El módulo 16 del resultado será menor que el módulo 16 del registro A
     * si ha habido HalfCarry. Sucede lo mismo para todos los métodos suma
     * SIN carry */
    if ((res & 0x0f) < (regA & 0x0f)) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (((regA ^ ~oper8) & (regA ^ res)) > 0x7f) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }

    regA = res;
    flagQ = true;
}

// Suma con acarreo de 8 bits
void Z80::adc(uint8_t oper8) {
    uint16_t res = regA + oper8;

    if (carryFlag) {
        ++res;
    }

    carryFlag = res > 0xff;
    res &= 0xff;
    sz5h3pnFlags = sz53n_addTable[res];

    if (((regA ^ oper8 ^ res) & 0x10) != 0) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (((regA ^ ~oper8) & (regA ^ res)) > 0x7f) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }

    regA = res;
    flagQ = true;
}

// Suma dos operandos de 16 bits sin carry afectando a los flags
void Z80::add16(RegisterPair &reg16, uint16_t oper16) {
    uint32_t tmp = oper16 + reg16.word;

    REG_WZ = reg16.word + 1;
    carryFlag = tmp > 0xffff;
    reg16.word = tmp;
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZP_MASK) | ((reg16.word >> 8) & FLAG_53_MASK);

    if ((reg16.word & 0x0fff) < (oper16 & 0x0fff)) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    flagQ = true;
    return;
}

// Suma con acarreo de 16 bits
void Z80::adc16(uint16_t reg16) {
    uint16_t tmpHL = REG_HL;
    REG_WZ = REG_HL + 1;

    uint32_t res = REG_HL + reg16;
    if (carryFlag) {
        ++res;
    }

    carryFlag = res > 0xffff;
    res &= 0xffff;
    REG_HL = (uint16_t) res;

    sz5h3pnFlags = sz53n_addTable[REG_H];
    if (res != 0) {
        sz5h3pnFlags &= ~ZERO_MASK;
    }

    if (((res ^ tmpHL ^ reg16) & 0x1000) != 0) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (((tmpHL ^ ~reg16) & (tmpHL ^ res)) > 0x7fff) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }

    flagQ = true;
}

// Resta de 8 bits
void Z80::sub(uint8_t oper8) {
    int16_t res = regA - oper8;

    carryFlag = res < 0;
    res &= 0xff;
    sz5h3pnFlags = sz53n_subTable[res];

    /* El módulo 16 del resultado será mayor que el módulo 16 del registro A
     * si ha habido HalfCarry. Sucede lo mismo para todos los métodos resta
     * SIN carry, incluido cp */
    if ((res & 0x0f) > (regA & 0x0f)) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (((regA ^ oper8) & (regA ^ res)) > 0x7f) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }

    regA = res;
    flagQ = true;
}

// Resta con acarreo de 8 bits
void Z80::sbc(uint8_t oper8) {
    int16_t res = regA - oper8;

    if (carryFlag) {
        --res;
    }

    carryFlag = res < 0;
    res &= 0xff;
    sz5h3pnFlags = sz53n_subTable[res];

    if (((regA ^ oper8 ^ res) & 0x10) != 0) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (((regA ^ oper8) & (regA ^ res)) > 0x7f) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }

    regA = res;
    flagQ = true;
}

// Resta con acarreo de 16 bits
void Z80::sbc16(uint16_t reg16) {
    uint16_t tmpHL = REG_HL;
    REG_WZ = REG_HL + 1;

    int32_t res = REG_HL - reg16;
    if (carryFlag) {
        --res;
    }

    carryFlag = res < 0;
    res &= 0xffff;
    REG_HL = (uint16_t) res;

    sz5h3pnFlags = sz53n_subTable[REG_H];
    if (res != 0) {
        sz5h3pnFlags &= ~ZERO_MASK;
    }

    if (((res ^ tmpHL ^ reg16) & 0x1000) != 0) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (((tmpHL ^ reg16) & (tmpHL ^ res)) > 0x7fff) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }
    flagQ = true;
}

// Operación AND lógica
inline void Z80::and_(uint8_t oper8) {
    regA &= oper8;
    carryFlag = false;
    sz5h3pnFlags = sz53pn_addTable[regA] | HALFCARRY_MASK;
    flagQ = true;
}

// Operación XOR lógica
inline void Z80::xor_(uint8_t oper8) {
    regA ^= oper8;
    carryFlag = false;
    sz5h3pnFlags = sz53pn_addTable[regA];
    flagQ = true;
}

void Z80::Xor(uint8_t oper8) {
    xor_(oper8);
}

// Operación OR lógica
void Z80::or_(uint8_t oper8) {
    regA |= oper8;
    carryFlag = false;
    sz5h3pnFlags = sz53pn_addTable[regA];
    flagQ = true;
}

// Operación de comparación con el registro A
// es como SUB, pero solo afecta a los flags
// Los flags SIGN y ZERO se calculan a partir del resultado
// Los flags 3 y 5 se copian desde el operando (sigh!)
inline void Z80::cp(uint8_t oper8) {
    int16_t res = regA - oper8;

    carryFlag = res < 0;
    res &= 0xff;

    sz5h3pnFlags = (sz53n_addTable[oper8] & FLAG_53_MASK)
            | // No necesito preservar H, pero está a 0 en la tabla de todas formas
            (sz53n_subTable[res] & FLAG_SZHN_MASK);

    if ((res & 0x0f) > (regA & 0x0f)) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }

    if (((regA ^ oper8) & (regA ^ res)) > 0x7f) {
        sz5h3pnFlags |= OVERFLOW_MASK;
    }

    flagQ = true;
}

void Z80::Cp(uint8_t oper8) {
    cp(oper8);
}

// DAA
void Z80::daa(void) {
    uint8_t suma = 0;
    bool carry = carryFlag;

    if ((sz5h3pnFlags & HALFCARRY_MASK) != 0 || (regA & 0x0f) > 0x09) {
        suma = 6;
    }

    if (carry || (regA > 0x99)) {
        suma |= 0x60;
    }

    if (regA > 0x99) {
        carry = true;
    }

    if ((sz5h3pnFlags & ADDSUB_MASK) != 0) {
        sub(suma);
        sz5h3pnFlags = (sz5h3pnFlags & HALFCARRY_MASK) | sz53pn_subTable[regA];
    } else {
        add(suma);
        sz5h3pnFlags = (sz5h3pnFlags & HALFCARRY_MASK) | sz53pn_addTable[regA];
    }

    carryFlag = carry;
    // Los add/sub ya ponen el resto de los flags
    flagQ = true;
}

// POP
IRAM_ATTR uint16_t Z80::pop(void) {
    uint16_t word = Z80Ops::peek16(REG_SP);
    REG_SP = REG_SP + 2;
    return word;
}

// PUSH
IRAM_ATTR void Z80::push(uint16_t word) {
    Z80Ops::poke8(--REG_SP, word >> 8);
    Z80Ops::poke8(--REG_SP, word);
}

// LDI
IRAM_ATTR void Z80::ldi(void) {

    uint8_t work8 = Z80Ops::peek8(REG_HL);

    Z80Ops::poke8(REG_DE, work8);
    Z80Ops::addressOnBus(REG_DE, 2);
    ++REG_HL;
    ++REG_DE;
    --REG_BC;
    work8 += regA;

    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZ_MASK) | (work8 & BIT3_MASK);

    if ((work8 & ADDSUB_MASK) != 0) {
        sz5h3pnFlags |= BIT5_MASK;
    }

    if (REG_BC != 0) {
        sz5h3pnFlags |= PARITY_MASK;
    }
    flagQ = true;
}

// LDD
IRAM_ATTR void Z80::ldd(void) {

    uint8_t work8 = Z80Ops::peek8(REG_HL);

    Z80Ops::poke8(REG_DE, work8);
    Z80Ops::addressOnBus(REG_DE, 2);
    --REG_HL;
    --REG_DE;
    --REG_BC;
    work8 += regA;

    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZ_MASK) | (work8 & BIT3_MASK);

    if ((work8 & ADDSUB_MASK) != 0) {
        sz5h3pnFlags |= BIT5_MASK;
    }

    if (REG_BC != 0) {
        sz5h3pnFlags |= PARITY_MASK;
    }
    flagQ = true;
}

// CPI
IRAM_ATTR void Z80::cpi(void) {

    uint8_t memHL = Z80Ops::peek8(REG_HL);

    bool carry = carryFlag; // lo guardo porque cp lo toca
    cp(memHL);
    carryFlag = carry;
    Z80Ops::addressOnBus(REG_HL, 5);
    ++REG_HL;
    --REG_BC;
    memHL = regA - memHL - ((sz5h3pnFlags & HALFCARRY_MASK) != 0 ? 1 : 0);
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHN_MASK) | (memHL & BIT3_MASK);

    if ((memHL & ADDSUB_MASK) != 0) {
        sz5h3pnFlags |= BIT5_MASK;
    }

    if (REG_BC != 0) {
        sz5h3pnFlags |= PARITY_MASK;
    }

    ++REG_WZ;
    flagQ = true;
}

// CPD
IRAM_ATTR void Z80::cpd(void) {

    uint8_t memHL = Z80Ops::peek8(REG_HL);

    bool carry = carryFlag; // lo guardo porque cp lo toca
    cp(memHL);
    carryFlag = carry;
    Z80Ops::addressOnBus(REG_HL, 5);
    --REG_HL;
    --REG_BC;
    memHL = regA - memHL - ((sz5h3pnFlags & HALFCARRY_MASK) != 0 ? 1 : 0);
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHN_MASK) | (memHL & BIT3_MASK);

    if ((memHL & ADDSUB_MASK) != 0) {
        sz5h3pnFlags |= BIT5_MASK;
    }

    if (REG_BC != 0) {
        sz5h3pnFlags |= PARITY_MASK;
    }

    --REG_WZ;
    flagQ = true;
}

// INI
IRAM_ATTR void Z80::ini(void) {
    REG_WZ = REG_BC;
    Z80Ops::addressOnBus(getPairIR().word, 1);
    uint8_t work8 = Ports::input(REG_WZ++);
    Z80Ops::poke8(REG_HL, work8);

    --REG_B;
    ++REG_HL;

    sz5h3pnFlags = sz53pn_addTable[REG_B];
    if (work8 > 0x7f) {
        sz5h3pnFlags |= ADDSUB_MASK;
    }

    carryFlag = false;
    uint16_t tmp = work8 + ((REG_C + 1) & 255);
    if (tmp > 0xff) {
        sz5h3pnFlags |= HALFCARRY_MASK;
        carryFlag = true;
    }

    if ((sz53pn_addTable[((tmp & 0x07) ^ REG_B)] & PARITY_MASK) == PARITY_MASK) {
        sz5h3pnFlags |= PARITY_MASK;
    } else {
        sz5h3pnFlags &= ~PARITY_MASK;
    }
    flagQ = true;
}

// IND
IRAM_ATTR void Z80::ind(void) {
    REG_WZ = REG_BC;
    Z80Ops::addressOnBus(getPairIR().word, 1);
    uint8_t work8 = Ports::input(REG_WZ--);
    Z80Ops::poke8(REG_HL, work8);

    --REG_B;
    --REG_HL;

    sz5h3pnFlags = sz53pn_addTable[REG_B];
    if (work8 > 0x7f) {
        sz5h3pnFlags |= ADDSUB_MASK;
    }

    carryFlag = false;
    uint16_t tmp = work8 + ((REG_C - 1) & 255);
    if (tmp > 0xff) {
        sz5h3pnFlags |= HALFCARRY_MASK;
        carryFlag = true;
    }

    if ((sz53pn_addTable[((tmp & 0x07) ^ REG_B)] & PARITY_MASK) == PARITY_MASK) {
        sz5h3pnFlags |= PARITY_MASK;
    } else {
        sz5h3pnFlags &= ~PARITY_MASK;
    }
    flagQ = true;
}

// OUTI
IRAM_ATTR void Z80::outi(void) {

    Z80Ops::addressOnBus(getPairIR().word, 1);

    --REG_B;
    REG_WZ = REG_BC;

    uint8_t work8 = Z80Ops::peek8(REG_HL);

    Ports::output(REG_WZ++, work8);

    ++REG_HL;

    carryFlag = false;
    if (work8 > 0x7f) {
        sz5h3pnFlags = sz53n_subTable[REG_B];
    } else {
        sz5h3pnFlags = sz53n_addTable[REG_B];
    }

    if ((REG_L + work8) > 0xff) {
        sz5h3pnFlags |= HALFCARRY_MASK;
        carryFlag = true;
    }

    if ((sz53pn_addTable[(((REG_L + work8) & 0x07) ^ REG_B)] & PARITY_MASK) == PARITY_MASK) {
        sz5h3pnFlags |= PARITY_MASK;
    }
    flagQ = true;
}

// OUTD
IRAM_ATTR void Z80::outd(void) {

    Z80Ops::addressOnBus(getPairIR().word, 1);

    --REG_B;
    REG_WZ = REG_BC;

    uint8_t work8 = Z80Ops::peek8(REG_HL);

    Ports::output(REG_WZ--, work8);

    --REG_HL;

    carryFlag = false;
    if (work8 > 0x7f) {
        sz5h3pnFlags = sz53n_subTable[REG_B];
    } else {
        sz5h3pnFlags = sz53n_addTable[REG_B];
    }

    if ((REG_L + work8) > 0xff) {
        sz5h3pnFlags |= HALFCARRY_MASK;
        carryFlag = true;
    }

    if ((sz53pn_addTable[(((REG_L + work8) & 0x07) ^ REG_B)] & PARITY_MASK) == PARITY_MASK) {
        sz5h3pnFlags |= PARITY_MASK;
    }
    flagQ = true;
}

// Pone a 1 el Flag Z si el bit b del registro
// r es igual a 0
/*
 * En contra de lo que afirma el Z80-Undocumented, los bits 3 y 5 toman
 * SIEMPRE el valor de los bits correspondientes del valor a comparar para
 * las instrucciones BIT n,r. Para BIT n,(HL) toman el valor del registro
 * escondido (REG_WZ), y para las BIT n, (IX/IY+n) toman el valor de los
 * bits superiores de la dirección indicada por IX/IY+n.
 *
 * 04/12/08 Confirmado el comentario anterior:
 *          http://scratchpad.wikia.com/wiki/Z80
 */
void Z80::bitTest(uint8_t mask, uint8_t reg) {
    bool zeroFlag = !(mask & reg);

    sz5h3pnFlags = (sz53n_addTable[reg] & ~FLAG_SZP_MASK) | HALFCARRY_MASK;

    if (zeroFlag) {
        sz5h3pnFlags |= (PARITY_MASK | ZERO_MASK);
    } else
    if (mask == SIGN_MASK /*&& !zeroFlag*/) {
        sz5h3pnFlags |= SIGN_MASK;
    }
    flagQ = true;
}

inline void Z80::check_trdos() {
    if (!Z80Ops::is2a3 && (Config::DiskCtrl || ESPeccy::trdos || Z80Ops::isPentagon)) {
        if (!ESPeccy::trdos) {
            if (REG_PCh == 0x3D) {
                // TR-DOS Rom can be accessed from 48K machines and from Spectrum 128/+2 and Pentagon if the currently mapped ROM is bank 1.
                if (Z80Ops::is48 && (MemESP::romInUse == 0) || ((!Z80Ops::is48) && MemESP::romInUse == 1)) {
                    MemESP::romInUse = 4;
                    MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
                    ESPeccy::trdos = true;
                }
            }
        } else {
            if (REG_PCh >= 0x40) {
                if (Z80Ops::is48) MemESP::romInUse = 0;
                else              MemESP::romInUse = MemESP::romLatch;

                MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
                ESPeccy::trdos = false;
            }
        }
    }
}

//Interrupción
/* Desglose de la interrupción, según el modo:
 * IM0:
 *      M1: 7 T-Estados -> reconocer INT y decSP
 *      M2: 3 T-Estados -> escribir byte alto y decSP
 *      M3: 3 T-Estados -> escribir byte bajo y salto a N
 * IM1:
 *      M1: 7 T-Estados -> reconocer INT y decSP
 *      M2: 3 T-Estados -> escribir byte alto PC y decSP
 *      M3: 3 T-Estados -> escribir byte bajo PC y PC=0x0038
 * IM2:
 *      M1: 7 T-Estados -> reconocer INT y decSP
 *      M2: 3 T-Estados -> escribir byte alto y decSP
 *      M3: 3 T-Estados -> escribir byte bajo
 *      M4: 3 T-Estados -> leer byte bajo del vector de INT
 *      M5: 3 T-Estados -> leer byte alto y saltar a la rutina de INT
 */
void Z80::interrupt(void) {

    halted = false;

    // Z80Ops::interruptHandlingTime(7);
    VIDEO::Draw(7, false);

    ++regR;

    ffIFF1 = ffIFF2 = false;
    push(REG_PC); // el push añadirá 6 t-estados (+contended si toca)
    if (modeINT == IntMode::IM2) {
        REG_PC = Z80Ops::peek16((regI << 8) | 0xff); // +6 t-estados
        check_trdos();
    } else {
        REG_PC = 0x0038;
    }
    REG_WZ = REG_PC;
}

//Interrupción NMI, no utilizado por ahora
/* Desglose de ciclos de máquina y T-Estados
 * M1: 5 T-Estados -> extraer opcode (pá ná, es tontería) y decSP
 * M2: 3 T-Estados -> escribe byte alto de PC y decSP
 * M3: 3 T-Estados -> escribe byte bajo de PC y PC=0x0066
 */
void Z80::nmi(void) {

    halted = false;

    // Esta lectura consigue dos cosas:
    //      1.- La lectura del opcode del M1 que se descarta
    //      2.- Si estaba en un HALT esperando una INT, lo saca de la espera
    Z80Ops::fetchOpcode();

    VIDEO::Draw(1, false);

    ++regR;
    ffIFF1 = false;
    push(REG_PC); // 3+3 t-estados + contended si procede
    REG_PC = REG_WZ = 0x0066;

}

void Z80::doNMI(void) {

    activeNMI = false;
    lastFlagQ = false;
    nmi();
    // printf("NMI!\n");

}

IRAM_ATTR void Z80::checkINT(void) {

    // Comprueba si está activada la señal INT
    if (ffIFF1 && !pendingEI && Z80Ops::isActiveINT()) {
        lastFlagQ = false;
        interrupt();
    }

}

IRAM_ATTR void Z80::incRegR(uint8_t inc) {

    regR += inc;

}

IRAM_ATTR void Z80::execute() {
    opCode = Z80Ops::fetchOpcode();

    ++regR;

    if (!halted) {
        ++REG_PC;
        if (prefixOpcode == 0) {
            flagQ = pendingEI = false;
            dcOpcode[opCode]();
        } else if (prefixOpcode == 0xDD) {
            prefixOpcode = 0;
            dcDDFD[opCode](regIX);
        } else if (prefixOpcode == 0xED) {
            prefixOpcode = 0;
            decodeED();
        } else if (prefixOpcode == 0xFD) {
            prefixOpcode = 0;
            dcDDFD[opCode](regIY);
        } else return;

        if (prefixOpcode != 0) return;

        lastFlagQ = flagQ;

    }

    // Ahora se comprueba si está activada la señal INT
    checkINT();

}

IRAM_ATTR void Z80::exec_nocheck() {

    while (CPU::tstates < CPU::stFrame) {

        uint8_t pg = REG_PCh >> 6;
        VIDEO::Draw_Opcode(MemESP::ramContended[pg]);
        opCode = MemESP::ramCurrent[pg][REG_PC & 0x3fff];

        ++regR;
        ++REG_PC;

        if (prefixOpcode == 0) {
            flagQ = pendingEI = false;
            dcOpcode[opCode]();
            lastFlagQ = flagQ;
            continue;
        }

        if (prefixOpcode == 0xDD) {
            prefixOpcode = 0;
            dcDDFD[opCode](regIX);
        } else if (prefixOpcode == 0xED) {
            prefixOpcode = 0;
            decodeED();
        } else if (prefixOpcode == 0xFD) {
            prefixOpcode = 0;
            dcDDFD[opCode](regIY);
        } else continue;

        if (prefixOpcode == 0) lastFlagQ = flagQ;

    }

}

IRAM_ATTR void Z80::exec_nocheck_2A3() {

    while (CPU::tstates < CPU::stFrame) {

        uint8_t pg = REG_PCh >> 6;
        opCode = MemESP::ramCurrent[pg][REG_PC & 0x3fff];
        if (MemESP::ramContended[pg]) {
            MemESP::lastContendedMemReadWrite = opCode;
            VIDEO::Draw_Opcode(true);
        } else {
            VIDEO::Draw_Opcode(false);
        };

        ++regR;
        ++REG_PC;

        if (prefixOpcode == 0) {
            flagQ = pendingEI = false;
            dcOpcode[opCode]();
            lastFlagQ = flagQ;
            continue;
        }

        if (prefixOpcode == 0xDD) {
            prefixOpcode = 0;
            dcDDFD[opCode](regIX);
        } else if (prefixOpcode == 0xED) {
            prefixOpcode = 0;
            decodeED();
        } else if (prefixOpcode == 0xFD) {
            prefixOpcode = 0;
            dcDDFD[opCode](regIY);
        } else continue;

        if (prefixOpcode == 0) lastFlagQ = flagQ;

    }

}

void Z80::decodeOpcode00()
{ /* NOP */
}

void Z80::decodeOpcode01()
{ /* LD BC,nn */
    REG_BC = Z80Ops::peek16(REG_PC);
    REG_PC += 2;
}

void Z80::decodeOpcode02()
{ /* LD (BC),A */
    Z80Ops::poke8(REG_BC, regA);
    REG_W = regA;
    REG_Z = REG_C + 1;
    //REG_WZ = (regA << 8) | (REG_C + 1);
}

void Z80::decodeOpcode03()
{ /* INC BC */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    ++REG_BC;
}

void Z80::decodeOpcode04()
{ /* INC B */
    inc8(REG_B);
}

void Z80::decodeOpcode05()
{ /* DEC B */
    dec8(REG_B);
}

void Z80::decodeOpcode06()
{ /* LD B,n */
    REG_B = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}

void Z80::decodeOpcode07()
{ /* RLCA */
    carryFlag = (regA > 0x7f);
    regA <<= 1;
    if (carryFlag) {
        regA |= CARRY_MASK;
    }
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZP_MASK) | (regA & FLAG_53_MASK);
    flagQ = true;
}

void Z80::decodeOpcode08()
{ /* EX AF,AF' */
    uint8_t work8 = regA;
    regA = REG_Ax;
    REG_Ax = work8;

    work8 = getFlags();
    setFlags(REG_Fx);
    REG_Fx = work8;
}

void Z80::decodeOpcode09()
{ /* ADD HL,BC */
    Z80Ops::addressOnBus(getPairIR().word, 7);
    add16(regHL, REG_BC);
}

void Z80::decodeOpcode0A()
{ /* LD A,(BC) */
    regA = Z80Ops::peek8(REG_BC);
    REG_WZ = REG_BC + 1;
}

void Z80::decodeOpcode0B()
{ /* DEC BC */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    --REG_BC;

}

void Z80::decodeOpcode0C()
{ /* INC C */
    inc8(REG_C);
}

void Z80::decodeOpcode0D()
{ /* DEC C */
    dec8(REG_C);
}

void Z80::decodeOpcode0E()
{ /* LD C,n */
    REG_C = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}

void Z80::decodeOpcode0F()
{ /* RRCA */
    carryFlag = (regA & CARRY_MASK) != 0;
    regA >>= 1;
    if (carryFlag) {
        regA |= SIGN_MASK;
    }
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZP_MASK) | (regA & FLAG_53_MASK);
    flagQ = true;
}

void Z80::decodeOpcode10()
{ /* DJNZ e */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    int8_t offset = Z80Ops::peek8(REG_PC);
    if (--REG_B != 0) {
        Z80Ops::addressOnBus(REG_PC, 5);
        REG_PC = REG_WZ = REG_PC + offset + 1;
    } else {
        ++REG_PC;
    }
}

void Z80::decodeOpcode11()
{ /* LD DE,nn */
    REG_DE = Z80Ops::peek16(REG_PC);
    REG_PC += 2;
}

void Z80::decodeOpcode12()
{ /* LD (DE),A */
    Z80Ops::poke8(REG_DE, regA);
    REG_W = regA;
    REG_Z = REG_E + 1;
    //REG_WZ = (regA << 8) | (REG_E + 1);
}

void Z80::decodeOpcode13()
{ /* INC DE */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    ++REG_DE;
}

void Z80::decodeOpcode14()
{ /* INC D */
    inc8(REG_D);
}

void Z80::decodeOpcode15()
{ /* DEC D */
    dec8(REG_D);
}

void Z80::decodeOpcode16()
{ /* LD D,n */
    REG_D = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}

void Z80::decodeOpcode17()
{ /* RLA */
    bool oldCarry = carryFlag;
    carryFlag = regA > 0x7f;
    regA <<= 1;
    if (oldCarry) {
        regA |= CARRY_MASK;
    }
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZP_MASK) | (regA & FLAG_53_MASK);
    flagQ = true;
}

void Z80::decodeOpcode18()
{ /* JR e */
    int8_t offset = Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    REG_PC = REG_WZ = REG_PC + offset + 1;

}

void Z80::decodeOpcode19()
{ /* ADD HL,DE */
    Z80Ops::addressOnBus(getPairIR().word, 7);
    add16(regHL, REG_DE);
}

void Z80::decodeOpcode1A()
{ /* LD A,(DE) */
    regA = Z80Ops::peek8(REG_DE);
    REG_WZ = REG_DE + 1;
}

void Z80::decodeOpcode1B()
{ /* DEC DE */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    --REG_DE;
}

void Z80::decodeOpcode1C()
{ /* INC E */
    inc8(REG_E);
}

void Z80::decodeOpcode1D()
{ /* DEC E */
    dec8(REG_E);
}

void Z80::decodeOpcode1E()
{ /* LD E,n */
    REG_E = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}

void Z80::decodeOpcode1F()
{ /* RRA */
    bool oldCarry = carryFlag;
    carryFlag = (regA & CARRY_MASK) != 0;
    regA >>= 1;
    if (oldCarry) {
        regA |= SIGN_MASK;
    }
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZP_MASK) | (regA & FLAG_53_MASK);
    flagQ = true;
}

void Z80::decodeOpcode20()
{ /* JR NZ,e */
    int8_t offset = Z80Ops::peek8(REG_PC);
    if ((sz5h3pnFlags & ZERO_MASK) == 0) {
        Z80Ops::addressOnBus(REG_PC, 5);
        REG_PC += offset;
        REG_WZ = REG_PC + 1;
    }
    ++REG_PC;
}

void Z80::decodeOpcode21()
{ /* LD HL,nn */
    REG_HL = Z80Ops::peek16(REG_PC);
    REG_PC += 2;
}

void Z80::decodeOpcode22()
{ /* LD (nn),HL */
    REG_WZ = Z80Ops::peek16(REG_PC);
    Z80Ops::poke16(REG_WZ, regHL);
    ++REG_WZ;
    REG_PC += 2;
}

void Z80::decodeOpcode23()
{ /* INC HL */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    ++REG_HL;
}

void Z80::decodeOpcode24()
{ /* INC H */
    inc8(REG_H);
}

void Z80::decodeOpcode25()
{ /* DEC H */
    dec8(REG_H);
}

void Z80::decodeOpcode26()
{ /* LD H,n */
    REG_H = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}

void Z80::decodeOpcode27()
{ /* DAA */
    daa();
}

void Z80::decodeOpcode28()
{ /* JR Z,e */
    int8_t offset = Z80Ops::peek8(REG_PC);
    if ((sz5h3pnFlags & ZERO_MASK) != 0) {
        Z80Ops::addressOnBus(REG_PC, 5);
        REG_PC += offset;
        REG_WZ = REG_PC + 1;
    }
    ++REG_PC;
}

void Z80::decodeOpcode29()
{ /* ADD HL,HL */
    Z80Ops::addressOnBus(getPairIR().word, 7);
    add16(regHL, REG_HL);
}

void Z80::decodeOpcode2A()
{ /* LD HL,(nn) */
    REG_WZ = Z80Ops::peek16(REG_PC);
    REG_HL = Z80Ops::peek16(REG_WZ);
    ++REG_WZ;
    REG_PC += 2;
}

void Z80::decodeOpcode2B()
{ /* DEC HL */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    --REG_HL;
}

void Z80::decodeOpcode2C()
{ /* INC L */
    inc8(REG_L);
}

void Z80::decodeOpcode2D()
{ /* DEC L */
    dec8(REG_L);
}

void Z80::decodeOpcode2E()
{ /* LD L,n */
    REG_L = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}

void Z80::decodeOpcode2F()
{ /* CPL */
    regA ^= 0xff;
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZP_MASK) | HALFCARRY_MASK
            | (regA & FLAG_53_MASK) | ADDSUB_MASK;
    flagQ = true;
}

void Z80::decodeOpcode30()
{ /* JR NC,e */
    int8_t offset = Z80Ops::peek8(REG_PC);
    if (!carryFlag) {
        Z80Ops::addressOnBus(REG_PC, 5);
        REG_PC += offset;
        REG_WZ = REG_PC + 1;
    }
    ++REG_PC;
}

void Z80::decodeOpcode31()
{ /* LD SP,nn */
    REG_SP = Z80Ops::peek16(REG_PC);
    REG_PC += 2;
}

void Z80::decodeOpcode32()
{ /* LD (nn),A */
    REG_WZ = Z80Ops::peek16(REG_PC);
    Z80Ops::poke8(REG_WZ, regA);
    REG_WZ = (regA << 8) | ((REG_WZ + 1) & 0xff);
    REG_PC += 2;
}

void Z80::decodeOpcode33()
{ /* INC SP */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    ++REG_SP;
}

void Z80::decodeOpcode34()
{ /* INC (HL) */
    uint8_t work8 = Z80Ops::peek8(REG_HL);
    inc8(work8);
    Z80Ops::addressOnBus(REG_HL, 1);
    Z80Ops::poke8(REG_HL, work8);
}

void Z80::decodeOpcode35()
{ /* DEC (HL) */
    uint8_t work8 = Z80Ops::peek8(REG_HL);
    dec8(work8);
    Z80Ops::addressOnBus(REG_HL, 1);
    Z80Ops::poke8(REG_HL, work8);
}

void Z80::decodeOpcode36()
{ /* LD (HL),n */
    Z80Ops::poke8(REG_HL, Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcode37()
{ /* SCF */
    uint8_t regQ = lastFlagQ ? sz5h3pnFlags : 0;
    carryFlag = true;
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZP_MASK) | (((regQ ^ sz5h3pnFlags) | regA) & FLAG_53_MASK);
    flagQ = true;
}

void Z80::decodeOpcode38()
{ /* JR C,e */
    int8_t offset = Z80Ops::peek8(REG_PC);
    if (carryFlag) {
        Z80Ops::addressOnBus(REG_PC, 5);
        REG_PC += offset;
        REG_WZ = REG_PC + 1;
    }
    ++REG_PC;
}

void Z80::decodeOpcode39()
{ /* ADD HL,SP */
    Z80Ops::addressOnBus(getPairIR().word, 7);
    add16(regHL, REG_SP);
}

void Z80::decodeOpcode3A()
{ /* LD A,(nn) */
    REG_WZ = Z80Ops::peek16(REG_PC);
    regA = Z80Ops::peek8(REG_WZ);
    ++REG_WZ;
    REG_PC += 2;
}

void Z80::decodeOpcode3B()
{ /* DEC SP */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    --REG_SP;
}

void Z80::decodeOpcode3C()
{ /* INC A */
    inc8(regA);
}

void Z80::decodeOpcode3D()
{ /* DEC A */
    dec8(regA);
}

void Z80::decodeOpcode3E()
{ /* LD A,n */
    regA = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}

void Z80::decodeOpcode3F()
{ /* CCF */
    uint8_t regQ = lastFlagQ ? sz5h3pnFlags : 0;
    sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZP_MASK) | (((regQ ^ sz5h3pnFlags) | regA) & FLAG_53_MASK);
    if (carryFlag) {
        sz5h3pnFlags |= HALFCARRY_MASK;
    }
    carryFlag = !carryFlag;
    flagQ = true;
}

void Z80::decodeOpcode40() { /* LD B,B */ }
void Z80::decodeOpcode41() { /* LD B,C */ REG_B = REG_C; }
void Z80::decodeOpcode42() { /* LD B,D */ REG_B = REG_D; }
void Z80::decodeOpcode43() { /* LD B,E */ REG_B = REG_E; }
void Z80::decodeOpcode44() { /* LD B,H */ REG_B = REG_H; }
void Z80::decodeOpcode45() { /* LD B,L */ REG_B = REG_L; }
void Z80::decodeOpcode46() { /* LD B,(HL) */ REG_B = Z80Ops::peek8(REG_HL); }
void Z80::decodeOpcode47() { /* LD B,A */ REG_B = regA; }

void Z80::decodeOpcode48() { /* LD C,B */ REG_C = REG_B; }
void Z80::decodeOpcode49() { /* LD C,C */ }
void Z80::decodeOpcode4A() { /* LD C,D */ REG_C = REG_D; }
void Z80::decodeOpcode4B() { /* LD C,E */ REG_C = REG_E; }
void Z80::decodeOpcode4C() { /* LD C,H */ REG_C = REG_H; }
void Z80::decodeOpcode4D() { /* LD C,L */ REG_C = REG_L; }
void Z80::decodeOpcode4E() { /* LD C,(HL) */ REG_C = Z80Ops::peek8(REG_HL); }
void Z80::decodeOpcode4F() { /* LD C,A */ REG_C = regA; }

void Z80::decodeOpcode50() { /* LD D,B */ REG_D = REG_B; }
void Z80::decodeOpcode51() { /* LD D,C */ REG_D = REG_C; }
void Z80::decodeOpcode52() { /* LD D,D */ }
void Z80::decodeOpcode53() { /* LD D,E */ REG_D = REG_E; }
void Z80::decodeOpcode54() { /* LD D,H */ REG_D = REG_H; }
void Z80::decodeOpcode55() { /* LD D,L */ REG_D = REG_L; }
void Z80::decodeOpcode56() { /* LD D,(HL) */ REG_D = Z80Ops::peek8(REG_HL); }
void Z80::decodeOpcode57() { /* LD D,A */ REG_D = regA; }

void Z80::decodeOpcode58() { /* LD E,B */ REG_E = REG_B; }
void Z80::decodeOpcode59() { /* LD E,C */ REG_E = REG_C; }
void Z80::decodeOpcode5A() { /* LD E,D */ REG_E = REG_D; }
void Z80::decodeOpcode5B() { /* LD E,E */ }
void Z80::decodeOpcode5C() { /* LD E,H */ REG_E = REG_H; }
void Z80::decodeOpcode5D() { /* LD E,L */ REG_E = REG_L; }
void Z80::decodeOpcode5E() { /* LD E,(HL) */ REG_E = Z80Ops::peek8(REG_HL); }
void Z80::decodeOpcode5F() { /* LD E,A */ REG_E = regA; }

void Z80::decodeOpcode60() { /* LD H,B */ REG_H = REG_B; }
void Z80::decodeOpcode61() { /* LD H,C */ REG_H = REG_C; }
void Z80::decodeOpcode62() { /* LD H,D */ REG_H = REG_D; }
void Z80::decodeOpcode63() { /* LD H,E */ REG_H = REG_E; }
void Z80::decodeOpcode64() { /* LD H,H */ }
void Z80::decodeOpcode65() { /* LD H,L */ REG_H = REG_L; }
void Z80::decodeOpcode66() { /* LD H,(HL) */ REG_H = Z80Ops::peek8(REG_HL); }
void Z80::decodeOpcode67() { /* LD H,A */ REG_H = regA; }

void Z80::decodeOpcode68() { /* LD L,B */ REG_L = REG_B; }
void Z80::decodeOpcode69() { /* LD L,C */ REG_L = REG_C; }
void Z80::decodeOpcode6A() { /* LD L,D */ REG_L = REG_D; }
void Z80::decodeOpcode6B() { /* LD L,E */ REG_L = REG_E; }
void Z80::decodeOpcode6C() { /* LD L,H */ REG_L = REG_H; }
void Z80::decodeOpcode6D() { /* LD L,L */ }
void Z80::decodeOpcode6E() { /* LD L,(HL) */ REG_L = Z80Ops::peek8(REG_HL); }
void Z80::decodeOpcode6F() { /* LD L,A */ REG_L = regA; }

void Z80::decodeOpcode70() { /* LD (HL),B */ Z80Ops::poke8(REG_HL, REG_B); }
void Z80::decodeOpcode71() { /* LD (HL),C */ Z80Ops::poke8(REG_HL, REG_C); }
void Z80::decodeOpcode72() { /* LD (HL),D */ Z80Ops::poke8(REG_HL, REG_D); }
void Z80::decodeOpcode73() { /* LD (HL),E */ Z80Ops::poke8(REG_HL, REG_E); }
void Z80::decodeOpcode74() { /* LD (HL),H */ Z80Ops::poke8(REG_HL, REG_H); }
void Z80::decodeOpcode75() { /* LD (HL),L */ Z80Ops::poke8(REG_HL, REG_L); }
void Z80::decodeOpcode76() { /* HALT */ halted = true; /* Signal HALT to CPU Loop */ CPU::stFrame = 0; }
void Z80::decodeOpcode77() { /* LD (HL),A */ Z80Ops::poke8(REG_HL, regA); }

void Z80::decodeOpcode78() { /* LD A,B */ regA = REG_B; }
void Z80::decodeOpcode79() { /* LD A,C */ regA = REG_C; }
void Z80::decodeOpcode7A() { /* LD A,D */ regA = REG_D; }
void Z80::decodeOpcode7B() { /* LD A,E */ regA = REG_E; }
void Z80::decodeOpcode7C() { /* LD A,H */ regA = REG_H; }
void Z80::decodeOpcode7D() { /* LD A,L */ regA = REG_L; }
void Z80::decodeOpcode7E() { /* LD A,(HL) */ regA = Z80Ops::peek8(REG_HL); }
void Z80::decodeOpcode7F() { /* LD A,A */ }

void Z80::decodeOpcode80() { /* ADD A,B */ add(REG_B); }
void Z80::decodeOpcode81() { /* ADD A,C */ add(REG_C); }
void Z80::decodeOpcode82() { /* ADD A,D */ add(REG_D); }
void Z80::decodeOpcode83() { /* ADD A,E */ add(REG_E); }
void Z80::decodeOpcode84() { /* ADD A,H */ add(REG_H); }
void Z80::decodeOpcode85() { /* ADD A,L */ add(REG_L); }
void Z80::decodeOpcode86() { /* ADD A,(HL) */ add(Z80Ops::peek8(REG_HL)); }
void Z80::decodeOpcode87() { /* ADD A,A */ add(regA); }

void Z80::decodeOpcode88() { /* ADC A,B */ adc(REG_B); }
void Z80::decodeOpcode89() { /* ADC A,C */ adc(REG_C); }
void Z80::decodeOpcode8A() { /* ADC A,D */ adc(REG_D); }
void Z80::decodeOpcode8B() { /* ADC A,E */ adc(REG_E); }
void Z80::decodeOpcode8C() { /* ADC A,H */ adc(REG_H); }
void Z80::decodeOpcode8D() { /* ADC A,L */ adc(REG_L); }
void Z80::decodeOpcode8E() { /* ADC A,(HL) */ adc(Z80Ops::peek8(REG_HL)); }
void Z80::decodeOpcode8F() { /* ADC A,A */ adc(regA); }

void Z80::decodeOpcode90() { /* SUB B */ sub(REG_B); }
void Z80::decodeOpcode91() { /* SUB C */ sub(REG_C); }
void Z80::decodeOpcode92() { /* SUB D */ sub(REG_D); }
void Z80::decodeOpcode93() { /* SUB E */ sub(REG_E); }
void Z80::decodeOpcode94() { /* SUB H */ sub(REG_H); }
void Z80::decodeOpcode95() { /* SUB L */ sub(REG_L); }
void Z80::decodeOpcode96() { /* SUB (HL) */ sub(Z80Ops::peek8(REG_HL)); }
void Z80::decodeOpcode97() { /* SUB A */ sub(regA); }

void Z80::decodeOpcode98() { /* SBC A,B */ sbc(REG_B); }
void Z80::decodeOpcode99() { /* SBC A,C */ sbc(REG_C); }
void Z80::decodeOpcode9A() { /* SBC A,D */ sbc(REG_D); }
void Z80::decodeOpcode9B() { /* SBC A,E */ sbc(REG_E); }
void Z80::decodeOpcode9C() { /* SBC A,H */ sbc(REG_H); }
void Z80::decodeOpcode9D() { /* SBC A,L */ sbc(REG_L); }
void Z80::decodeOpcode9E() { /* SBC A,(HL) */ sbc(Z80Ops::peek8(REG_HL)); }
void Z80::decodeOpcode9F() { /* SBC A,A */ sbc(regA); }

void Z80::decodeOpcodeA0() { /* AND B */ and_(REG_B); }
void Z80::decodeOpcodeA1() { /* AND C */ and_(REG_C); }
void Z80::decodeOpcodeA2() { /* AND D */ and_(REG_D); }
void Z80::decodeOpcodeA3() { /* AND E */ and_(REG_E); }
void Z80::decodeOpcodeA4() { /* AND H */ and_(REG_H); }
void Z80::decodeOpcodeA5() { /* AND L */ and_(REG_L); }
void Z80::decodeOpcodeA6() { /* AND (HL) */ and_(Z80Ops::peek8(REG_HL)); }
void Z80::decodeOpcodeA7() { /* AND A */ and_(regA); }

void Z80::decodeOpcodeA8() { /* XOR B */ xor_(REG_B); }
void Z80::decodeOpcodeA9() { /* XOR C */ xor_(REG_C); }
void Z80::decodeOpcodeAA() { /* XOR D */ xor_(REG_D); }
void Z80::decodeOpcodeAB() { /* XOR E */ xor_(REG_E); }
void Z80::decodeOpcodeAC() { /* XOR H */ xor_(REG_H); }
void Z80::decodeOpcodeAD() { /* XOR L */ xor_(REG_L); }
void Z80::decodeOpcodeAE() { /* XOR (HL) */ xor_(Z80Ops::peek8(REG_HL)); }
void Z80::decodeOpcodeAF() { /* XOR A */ xor_(regA); }

void Z80::decodeOpcodeB0() { /* OR B */ or_(REG_B); }
void Z80::decodeOpcodeB1() { /* OR C */ or_(REG_C); }
void Z80::decodeOpcodeB2() { /* OR D */ or_(REG_D); }
void Z80::decodeOpcodeB3() { /* OR E */ or_(REG_E); }
void Z80::decodeOpcodeB4() { /* OR H */ or_(REG_H); }
void Z80::decodeOpcodeB5() { /* OR L */ or_(REG_L); }
void Z80::decodeOpcodeB6() { /* OR (HL) */ or_(Z80Ops::peek8(REG_HL)); }
void Z80::decodeOpcodeB7() { /* OR A */ or_(regA); }

void Z80::decodeOpcodeB8() { /* CP B */ cp(REG_B); }
void Z80::decodeOpcodeB9() { /* CP C */ cp(REG_C); }
void Z80::decodeOpcodeBA() { /* CP D */ cp(REG_D); }
void Z80::decodeOpcodeBB() { /* CP E */ cp(REG_E); }
void Z80::decodeOpcodeBC() { /* CP H */ cp(REG_H); }
void Z80::decodeOpcodeBD() { /* CP L */ cp(REG_L); }
void Z80::decodeOpcodeBE() { /* CP (HL) */ cp(Z80Ops::peek8(REG_HL)); }

IRAM_ATTR void Z80::decodeOpcodeBF()
{ /* CP A */
    cp(regA);

    if (REG_PC == 0x056b && Config::realtape_mode != REALTAPE_FORCE_LOAD) { // LOAD trap
        //printf("Trap Load FL:%d NAME:%s STAT:%s TYPE:%s\n", Config::flashload, Tape::tapeFileName.c_str(), Tape::tapeStatus == TAPE_STOPPED ? "STOPPED" : Tape::tapeStatus == TAPE_LOADING ? "LOADING" : "STOPPED_FORCED", Tape::tapeFileType == TAPE_FTYPE_TAP ? "TAP" : Tape::tapeFileType == TAPE_FTYPE_TZX ? "TZX" : "EMPTY" );
        if (Config::flashload &&
            Tape::tapeFileType == TAPE_FTYPE_TAP &&
            Tape::tapeStatus != TAPE_LOADING) {
            //printf("Loading tape %s\n",Tape::tapeFileName.c_str());
            if (Tape::FlashLoad()) {
                REG_PC = 0x5e2;
                Tape::Stop(); // Force Stop
            }
        }
    }
}

void Z80::decodeOpcodeC0()
{ /* RET NZ */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    if ((sz5h3pnFlags & ZERO_MASK) == 0) {
        REG_PC = REG_WZ = pop();
        check_trdos();
    }
}

void Z80::decodeOpcodeC1()
{ /* POP BC */
    REG_BC = pop();
}

void Z80::decodeOpcodeC2()
{ /* JP NZ,nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    if ((sz5h3pnFlags & ZERO_MASK) == 0) {
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeC3()
{ /* JP nn */
    REG_WZ = REG_PC = Z80Ops::peek16(REG_PC);
    check_trdos();
}

void Z80::decodeOpcodeC4()
{ /* CALL NZ,nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    if ((sz5h3pnFlags & ZERO_MASK) == 0) {
        Z80Ops::addressOnBus(REG_PC + 1, 1);
        push(REG_PC + 2);
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeC5()
{ /* PUSH BC */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_BC);
}

void Z80::decodeOpcodeC6()
{ /* ADD A,n */
    add(Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcodeC7()
{ /* RST 00H */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_PC);
    REG_PC = REG_WZ = 0x00;
}

void Z80::decodeOpcodeC8()
{ /* RET Z */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    if ((sz5h3pnFlags & ZERO_MASK) != 0) {
        REG_PC = REG_WZ = pop();
        check_trdos();
    }
}

void Z80::decodeOpcodeC9()
{ /* RET */
    REG_PC = REG_WZ = pop();
    check_trdos();
}

void Z80::decodeOpcodeCA()
{ /* JP Z,nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    if ((sz5h3pnFlags & ZERO_MASK) != 0) {
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeCB()
{ /* Subconjunto de instrucciones */
    opCode = Z80Ops::fetchOpcode();

    ++REG_PC;
    ++regR;

    dcCB[opCode]();

}

void Z80::decodeOpcodeCC()
{ /* CALL Z,nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    if ((sz5h3pnFlags & ZERO_MASK) != 0) {
        Z80Ops::addressOnBus(REG_PC + 1, 1);
        push(REG_PC + 2);
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeCD()
{ /* CALL nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    Z80Ops::addressOnBus(REG_PC + 1, 1);
    push(REG_PC + 2);
    REG_PC = REG_WZ;
    check_trdos();
}

void Z80::decodeOpcodeCE()
{ /* ADC A,n */
    adc(Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcodeCF()
{ /* RST 08H */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_PC);
    REG_PC = REG_WZ = 0x08;
}

void Z80::decodeOpcodeD0()
{ /* RET NC */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    if (!carryFlag) {
        REG_PC = REG_WZ = pop();
        check_trdos();
    }
}

void Z80::decodeOpcodeD1()
{ /* POP DE */
    REG_DE = pop();
}

void Z80::decodeOpcodeD2()
{ /* JP NC,nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    if (!carryFlag) {
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeD3()
{ /* OUT (n),A */
    uint8_t work8 = Z80Ops::peek8(REG_PC);
    ++REG_PC;
    REG_WZ = regA << 8;
    Ports::output(REG_WZ | work8, regA);
    REG_WZ |= (work8 + 1);
}

void Z80::decodeOpcodeD4()
{ /* CALL NC,nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    if (!carryFlag) {
        Z80Ops::addressOnBus(REG_PC + 1, 1);
        push(REG_PC + 2);
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeD5()
{ /* PUSH DE */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_DE);
}

void Z80::decodeOpcodeD6()
{ /* SUB n */
    sub(Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcodeD7()
{ /* RST 10H */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_PC);
    REG_PC = REG_WZ = 0x10;
}

void Z80::decodeOpcodeD8()
{ /* RET C */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    if (carryFlag) {
        REG_PC = REG_WZ = pop();
        check_trdos();
    }
}

void Z80::decodeOpcodeD9()
{ /* EXX */
    uint16_t tmp;
    tmp = REG_BC;
    REG_BC = REG_BCx;
    REG_BCx = tmp;

    tmp = REG_DE;
    REG_DE = REG_DEx;
    REG_DEx = tmp;

    tmp = REG_HL;
    REG_HL = REG_HLx;
    REG_HLx = tmp;
}

void Z80::decodeOpcodeDA()
{ /* JP C,nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    if (carryFlag) {
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeDB()
{ /* IN A,(n) */
    REG_W = regA;
    REG_Z = Z80Ops::peek8(REG_PC);
    ++REG_PC;
    regA = Ports::input(REG_WZ);
    ++REG_WZ;
}

void Z80::decodeOpcodeDC()
{ /* CALL C,nn */
    REG_WZ = Z80Ops::peek16(REG_PC);
    if (carryFlag) {
        Z80Ops::addressOnBus(REG_PC + 1, 1);
        push(REG_PC + 2);
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeDD()
{ /* Subconjunto de instrucciones */
    opCode = Z80Ops::fetchOpcode();

    ++REG_PC;
    ++regR;
    dcDDFD[opCode](regIX);
}

void Z80::decodeOpcodeDE()
{ /* SBC A,n */
    sbc(Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcodeDF()
{ /* RST 18H */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_PC);
    REG_PC = REG_WZ = 0x18;
}

void Z80::decodeOpcodeE0() /* RET PO */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    if ((sz5h3pnFlags & PARITY_MASK) == 0) {
        REG_PC = REG_WZ = pop();
        check_trdos();
    }
}

void Z80::decodeOpcodeE1() /* POP HL */
{
    REG_HL = pop();
}

void Z80::decodeOpcodeE2() /* JP PO,nn */
{
    REG_WZ = Z80Ops::peek16(REG_PC);
    if ((sz5h3pnFlags & PARITY_MASK) == 0) {
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeE3()
{ /* EX (SP),HL */
    // Instrucción de ejecución sutil.
    RegisterPair work = regHL;
    REG_HL = Z80Ops::peek16(REG_SP);
    Z80Ops::addressOnBus(REG_SP + 1, 1);
    // No se usa poke16 porque el Z80 escribe los bytes AL REVES
    Z80Ops::poke8(REG_SP + 1, work.byte8.hi);
    Z80Ops::poke8(REG_SP, work.byte8.lo);
    Z80Ops::addressOnBus(REG_SP, 2);
    REG_WZ = REG_HL;
}

void Z80::decodeOpcodeE4() /* CALL PO,nn */
{
    REG_WZ = Z80Ops::peek16(REG_PC);
    if ((sz5h3pnFlags & PARITY_MASK) == 0) {
        Z80Ops::addressOnBus(REG_PC + 1, 1);
        push(REG_PC + 2);
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeE5() /* PUSH HL */
{
    if (REG_PC == 0x0762
        && Config::realtape_mode != REALTAPE_FORCE_LOAD
        && Tape::tapeFileType == TAPE_FTYPE_TAP
    ) { // LOAD trap for automatic seek
        int16_t address = Z80::getRegIX();
        char blockName[11];
        for (int i = 0; i < 10; ++i) blockName[i] = MemESP::readbyte(++address);
        blockName[10] = '\0';
        //printf( "search block %s\n", blockName);
        if (blockName[0] && blockName[0] != 0xff) {
            Tape::findBlockByName(blockName);
        }
    }

    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_HL);
}

void Z80::decodeOpcodeE6() /* AND n */
{
    and_(Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcodeE7() /* RST 20H */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_PC);
    REG_PC = REG_WZ = 0x20;
}

void Z80::decodeOpcodeE8() /* RET PE */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    if ((sz5h3pnFlags & PARITY_MASK) != 0) {
        REG_PC = REG_WZ = pop();
        check_trdos();
    }
}

void Z80::decodeOpcodeE9() /* JP (HL) */
{
    REG_PC = REG_HL;
    check_trdos();
}

void Z80::decodeOpcodeEA() /* JP PE,nn */
{
    REG_WZ = Z80Ops::peek16(REG_PC);
    if ((sz5h3pnFlags & PARITY_MASK) != 0) {
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeEB()
{ /* EX DE,HL */
    uint16_t tmp = REG_HL;
    REG_HL = REG_DE;
    REG_DE = tmp;
}

void Z80::decodeOpcodeEC() /* CALL PE,nn */
{
    REG_WZ = Z80Ops::peek16(REG_PC);
    if ((sz5h3pnFlags & PARITY_MASK) != 0) {
        Z80Ops::addressOnBus(REG_PC + 1, 1);
        push(REG_PC + 2);
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeED()
{ /*Subconjunto de instrucciones*/
    opCode = Z80Ops::fetchOpcode();
    ++REG_PC;
    ++regR;
    decodeED();

}

void Z80::decodeOpcodeEE() /* XOR n */
{
    xor_(Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcodeEF() /* RST 28H */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_PC);
    REG_PC = REG_WZ = 0x28;
}

void Z80::decodeOpcodeF0() /* RET P */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    if (sz5h3pnFlags < SIGN_MASK) {
        REG_PC = REG_WZ = pop();
        check_trdos();
    }
}

void Z80::decodeOpcodeF1() /* POP AF */
{
    setRegAF(pop());
}

void Z80::decodeOpcodeF2() /* JP P,nn */
{
    REG_WZ = Z80Ops::peek16(REG_PC);
    if (sz5h3pnFlags < SIGN_MASK) {
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeF3() /* DI */
{
    ffIFF1 = ffIFF2 = false;
}

void Z80::decodeOpcodeF4() /* CALL P,nn */
{
    REG_WZ = Z80Ops::peek16(REG_PC);
    if (sz5h3pnFlags < SIGN_MASK) {
        Z80Ops::addressOnBus(REG_PC + 1, 1);
        push(REG_PC + 2);
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeF5() /* PUSH AF */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(getRegAF());
}

void Z80::decodeOpcodeF6() /* OR n */
{
    or_(Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcodeF7() /* RST 30H */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_PC);
    REG_PC = REG_WZ = 0x30;
}

void Z80::decodeOpcodeF8() /* RET M */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    if (sz5h3pnFlags > 0x7f) {
        REG_PC = REG_WZ = pop();
        check_trdos();
    }
}

void Z80::decodeOpcodeF9() /* LD SP,HL */
{
    Z80Ops::addressOnBus(getPairIR().word, 2);
    REG_SP = REG_HL;
}

void Z80::decodeOpcodeFA() /* JP M,nn */
{
    REG_WZ = Z80Ops::peek16(REG_PC);
    if (sz5h3pnFlags > 0x7f) {
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeFB() /* EI */
{
    ffIFF1 = ffIFF2 = true;
    pendingEI = true;
}

void Z80::decodeOpcodeFC() /* CALL M,nn */
{
    REG_WZ = Z80Ops::peek16(REG_PC);
    if (sz5h3pnFlags > 0x7f) {
        Z80Ops::addressOnBus(REG_PC + 1, 1);
        push(REG_PC + 2);
        REG_PC = REG_WZ;
        check_trdos();
        return;
    }
    REG_PC += 2;
}

void Z80::decodeOpcodeFD()
{ /* Subconjunto de instrucciones */
    opCode = Z80Ops::fetchOpcode();

    ++REG_PC;
    ++regR;
    dcDDFD[opCode](regIY);
}

void Z80::decodeOpcodeFE() /* CP n */
{
    cp(Z80Ops::peek8(REG_PC));
    ++REG_PC;
}

void Z80::decodeOpcodeFF() /* RST 38H */
{
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(REG_PC);
    REG_PC = REG_WZ = 0x38;
}

void (*Z80::dcOpcode[256])() = {
    &decodeOpcode00, &decodeOpcode01, &decodeOpcode02, &decodeOpcode03,
    &decodeOpcode04, &decodeOpcode05, &decodeOpcode06, &decodeOpcode07,
    &decodeOpcode08, &decodeOpcode09, &decodeOpcode0A, &decodeOpcode0B,
    &decodeOpcode0C, &decodeOpcode0D, &decodeOpcode0E, &decodeOpcode0F,

    &decodeOpcode10, &decodeOpcode11, &decodeOpcode12, &decodeOpcode13,
    &decodeOpcode14, &decodeOpcode15, &decodeOpcode16, &decodeOpcode17,
    &decodeOpcode18, &decodeOpcode19, &decodeOpcode1A, &decodeOpcode1B,
    &decodeOpcode1C, &decodeOpcode1D, &decodeOpcode1E, &decodeOpcode1F,

    &decodeOpcode20, &decodeOpcode21, &decodeOpcode22, &decodeOpcode23,
    &decodeOpcode24, &decodeOpcode25, &decodeOpcode26, &decodeOpcode27,
    &decodeOpcode28, &decodeOpcode29, &decodeOpcode2A, &decodeOpcode2B,
    &decodeOpcode2C, &decodeOpcode2D, &decodeOpcode2E, &decodeOpcode2F,

    &decodeOpcode30, &decodeOpcode31, &decodeOpcode32, &decodeOpcode33,
    &decodeOpcode34, &decodeOpcode35, &decodeOpcode36, &decodeOpcode37,
    &decodeOpcode38, &decodeOpcode39, &decodeOpcode3A, &decodeOpcode3B,
    &decodeOpcode3C, &decodeOpcode3D, &decodeOpcode3E, &decodeOpcode3F,

    &decodeOpcode40, &decodeOpcode41, &decodeOpcode42, &decodeOpcode43,
    &decodeOpcode44, &decodeOpcode45, &decodeOpcode46, &decodeOpcode47,
    &decodeOpcode48, &decodeOpcode49, &decodeOpcode4A, &decodeOpcode4B,
    &decodeOpcode4C, &decodeOpcode4D, &decodeOpcode4E, &decodeOpcode4F,

    &decodeOpcode50, &decodeOpcode51, &decodeOpcode52, &decodeOpcode53,
    &decodeOpcode54, &decodeOpcode55, &decodeOpcode56, &decodeOpcode57,
    &decodeOpcode58, &decodeOpcode59, &decodeOpcode5A, &decodeOpcode5B,
    &decodeOpcode5C, &decodeOpcode5D, &decodeOpcode5E, &decodeOpcode5F,

    &decodeOpcode60, &decodeOpcode61, &decodeOpcode62, &decodeOpcode63,
    &decodeOpcode64, &decodeOpcode65, &decodeOpcode66, &decodeOpcode67,
    &decodeOpcode68, &decodeOpcode69, &decodeOpcode6A, &decodeOpcode6B,
    &decodeOpcode6C, &decodeOpcode6D, &decodeOpcode6E, &decodeOpcode6F,

    &decodeOpcode70, &decodeOpcode71, &decodeOpcode72, &decodeOpcode73,
    &decodeOpcode74, &decodeOpcode75, &decodeOpcode76, &decodeOpcode77,
    &decodeOpcode78, &decodeOpcode79, &decodeOpcode7A, &decodeOpcode7B,
    &decodeOpcode7C, &decodeOpcode7D, &decodeOpcode7E, &decodeOpcode7F,

    &decodeOpcode80, &decodeOpcode81, &decodeOpcode82, &decodeOpcode83,
    &decodeOpcode84, &decodeOpcode85, &decodeOpcode86, &decodeOpcode87,
    &decodeOpcode88, &decodeOpcode89, &decodeOpcode8A, &decodeOpcode8B,
    &decodeOpcode8C, &decodeOpcode8D, &decodeOpcode8E, &decodeOpcode8F,

    &decodeOpcode90, &decodeOpcode91, &decodeOpcode92, &decodeOpcode93,
    &decodeOpcode94, &decodeOpcode95, &decodeOpcode96, &decodeOpcode97,
    &decodeOpcode98, &decodeOpcode99, &decodeOpcode9A, &decodeOpcode9B,
    &decodeOpcode9C, &decodeOpcode9D, &decodeOpcode9E, &decodeOpcode9F,

    &decodeOpcodeA0, &decodeOpcodeA1, &decodeOpcodeA2, &decodeOpcodeA3,
    &decodeOpcodeA4, &decodeOpcodeA5, &decodeOpcodeA6, &decodeOpcodeA7,
    &decodeOpcodeA8, &decodeOpcodeA9, &decodeOpcodeAA, &decodeOpcodeAB,
    &decodeOpcodeAC, &decodeOpcodeAD, &decodeOpcodeAE, &decodeOpcodeAF,

    &decodeOpcodeB0, &decodeOpcodeB1, &decodeOpcodeB2, &decodeOpcodeB3,
    &decodeOpcodeB4, &decodeOpcodeB5, &decodeOpcodeB6, &decodeOpcodeB7,
    &decodeOpcodeB8, &decodeOpcodeB9, &decodeOpcodeBA, &decodeOpcodeBB,
    &decodeOpcodeBC, &decodeOpcodeBD, &decodeOpcodeBE, &decodeOpcodeBF,

    &decodeOpcodeC0, &decodeOpcodeC1, &decodeOpcodeC2, &decodeOpcodeC3,
    &decodeOpcodeC4, &decodeOpcodeC5, &decodeOpcodeC6, &decodeOpcodeC7,
    &decodeOpcodeC8, &decodeOpcodeC9, &decodeOpcodeCA, &decodeOpcodeCB,
    &decodeOpcodeCC, &decodeOpcodeCD, &decodeOpcodeCE, &decodeOpcodeCF,

    &decodeOpcodeD0, &decodeOpcodeD1, &decodeOpcodeD2, &decodeOpcodeD3,
    &decodeOpcodeD4, &decodeOpcodeD5, &decodeOpcodeD6, &decodeOpcodeD7,
    &decodeOpcodeD8, &decodeOpcodeD9, &decodeOpcodeDA, &decodeOpcodeDB,
    &decodeOpcodeDC, &decodeOpcodeDD, &decodeOpcodeDE, &decodeOpcodeDF,

    &decodeOpcodeE0, &decodeOpcodeE1, &decodeOpcodeE2, &decodeOpcodeE3,
    &decodeOpcodeE4, &decodeOpcodeE5, &decodeOpcodeE6, &decodeOpcodeE7,
    &decodeOpcodeE8, &decodeOpcodeE9, &decodeOpcodeEA, &decodeOpcodeEB,
    &decodeOpcodeEC, &decodeOpcodeED, &decodeOpcodeEE, &decodeOpcodeEF,

    &decodeOpcodeF0, &decodeOpcodeF1, &decodeOpcodeF2, &decodeOpcodeF3,
    &decodeOpcodeF4, &decodeOpcodeF5, &decodeOpcodeF6, &decodeOpcodeF7,
    &decodeOpcodeF8, &decodeOpcodeF9, &decodeOpcodeFA, &decodeOpcodeFB,
    &decodeOpcodeFC, &decodeOpcodeFD, &decodeOpcodeFE, &decodeOpcodeFF

};

//Subconjunto de instrucciones 0xCB

void Z80::dcCB00() { /* RLC B */ rlc(REG_B); }
void Z80::dcCB01() { /* RLC C */ rlc(REG_C); }
void Z80::dcCB02() { /* RLC D */ rlc(REG_D); }
void Z80::dcCB03() { /* RLC E */ rlc(REG_E); }
void Z80::dcCB04() { /* RLC H */ rlc(REG_H); }
void Z80::dcCB05() { /* RLC L */ rlc(REG_L); }
void Z80::dcCB06() { /* RLC (HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL); rlc(work8); Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB07() { /* RLC A */ rlc(regA); }

void Z80::dcCB08() { /* RRC B */ rrc(REG_B); }
void Z80::dcCB09() { /* RRC C */ rrc(REG_C); }
void Z80::dcCB0A() { /* RRC D */ rrc(REG_D); }
void Z80::dcCB0B() { /* RRC E */ rrc(REG_E); }
void Z80::dcCB0C() { /* RRC H */ rrc(REG_H); }
void Z80::dcCB0D() { /* RRC L */ rrc(REG_L); }
void Z80::dcCB0E() { /* RRC (HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL); rrc(work8); Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB0F() { /* RRC A */ rrc(regA); }

void Z80::dcCB10() { /* RL B */ rl(REG_B); }
void Z80::dcCB11() { /* RL C */ rl(REG_C); }
void Z80::dcCB12() { /* RL D */ rl(REG_D); }
void Z80::dcCB13() { /* RL E */ rl(REG_E); }
void Z80::dcCB14() { /* RL H */ rl(REG_H); }
void Z80::dcCB15() { /* RL L */ rl(REG_L); }
void Z80::dcCB16() { /* RL (HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL); rl(work8); Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB17() { /* RL A */ rl(regA); }

void Z80::dcCB18() { /* RR B */ rr(REG_B); }
void Z80::dcCB19() { /* RR C */ rr(REG_C); }
void Z80::dcCB1A() { /* RR D */ rr(REG_D); }
void Z80::dcCB1B() { /* RR E */ rr(REG_E); }
void Z80::dcCB1C() { /* RR H */ rr(REG_H); }
void Z80::dcCB1D() { /* RR L */ rr(REG_L); }
void Z80::dcCB1E() { /* RR (HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL); rr(work8); Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB1F() { /* RR A */ rr(regA); }

void Z80::dcCB20() { /* SLA B */ sla(REG_B); }
void Z80::dcCB21() { /* SLA C */ sla(REG_C); }
void Z80::dcCB22() { /* SLA D */ sla(REG_D); }
void Z80::dcCB23() { /* SLA E */ sla(REG_E); }
void Z80::dcCB24() { /* SLA H */ sla(REG_H); }
void Z80::dcCB25() { /* SLA L */ sla(REG_L); }
void Z80::dcCB26() { /* SLA (HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL); sla(work8); Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB27() { /* SLA A */ sla(regA); }

void Z80::dcCB28() { /* SRA B */ sra(REG_B); }
void Z80::dcCB29() { /* SRA C */ sra(REG_C); }
void Z80::dcCB2A() { /* SRA D */ sra(REG_D); }
void Z80::dcCB2B() { /* SRA E */ sra(REG_E); }
void Z80::dcCB2C() { /* SRA H */ sra(REG_H); }
void Z80::dcCB2D() { /* SRA L */ sra(REG_L); }
void Z80::dcCB2E() { /* SRA (HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL); sra(work8); Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB2F() { /* SRA A */ sra(regA); }

void Z80::dcCB30() { /* SLL B */ sll(REG_B); }
void Z80::dcCB31() { /* SLL C */ sll(REG_C); }
void Z80::dcCB32() { /* SLL D */ sll(REG_D); }
void Z80::dcCB33() { /* SLL E */ sll(REG_E); }
void Z80::dcCB34() { /* SLL H */ sll(REG_H); }
void Z80::dcCB35() { /* SLL L */ sll(REG_L); }
void Z80::dcCB36() { /* SLL (HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL); sll(work8); Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB37() { /* SLL A */ sll(regA); }

void Z80::dcCB38() { /* SRL B */ srl(REG_B); }
void Z80::dcCB39() { /* SRL C */ srl(REG_C); }
void Z80::dcCB3A() { /* SRL D */ srl(REG_D); }
void Z80::dcCB3B() { /* SRL E */ srl(REG_E); }
void Z80::dcCB3C() { /* SRL H */ srl(REG_H); }
void Z80::dcCB3D() { /* SRL L */ srl(REG_L); }
void Z80::dcCB3E() { /* SRL (HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL); srl(work8); Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB3F() { /* SRL A */ srl(regA); }

void Z80::dcCB40() { /* BIT 0,B */ bitTest(0x01, REG_B); }
void Z80::dcCB41() { /* BIT 0,C */ bitTest(0x01, REG_C); }
void Z80::dcCB42() { /* BIT 0,D */ bitTest(0x01, REG_D); }
void Z80::dcCB43() { /* BIT 0,E */ bitTest(0x01, REG_E); }
void Z80::dcCB44() { /* BIT 0,H */ bitTest(0x01, REG_H); }
void Z80::dcCB45() { /* BIT 0,L */ bitTest(0x01, REG_L); }
void Z80::dcCB46() { /* BIT 0,(HL) */ bitTest(0x01, Z80Ops::peek8(REG_HL)); sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | (REG_W & FLAG_53_MASK); Z80Ops::addressOnBus(REG_HL, 1); }
void Z80::dcCB47() { /* BIT 0,A */ bitTest(0x01, regA); }

void Z80::dcCB48() { /* BIT 1,B */ bitTest(0x02, REG_B); }
void Z80::dcCB49() { /* BIT 1,C */ bitTest(0x02, REG_C); }
void Z80::dcCB4A() { /* BIT 1,D */ bitTest(0x02, REG_D); }
void Z80::dcCB4B() { /* BIT 1,E */ bitTest(0x02, REG_E); }
void Z80::dcCB4C() { /* BIT 1,H */ bitTest(0x02, REG_H); }
void Z80::dcCB4D() { /* BIT 1,L */ bitTest(0x02, REG_L); }
void Z80::dcCB4E() { /* BIT 1,(HL) */ bitTest(0x02, Z80Ops::peek8(REG_HL)); sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | (REG_W & FLAG_53_MASK); Z80Ops::addressOnBus(REG_HL, 1); }
void Z80::dcCB4F() { /* BIT 1,A */ bitTest(0x02, regA); }

void Z80::dcCB50() { /* BIT 2,B */ bitTest(0x04, REG_B); }
void Z80::dcCB51() { /* BIT 2,C */ bitTest(0x04, REG_C); }
void Z80::dcCB52() { /* BIT 2,D */ bitTest(0x04, REG_D); }
void Z80::dcCB53() { /* BIT 2,E */ bitTest(0x04, REG_E); }
void Z80::dcCB54() { /* BIT 2,H */ bitTest(0x04, REG_H); }
void Z80::dcCB55() { /* BIT 2,L */ bitTest(0x04, REG_L); }
void Z80::dcCB56() { /* BIT 2,(HL) */ bitTest(0x04, Z80Ops::peek8(REG_HL)); sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | (REG_W & FLAG_53_MASK); Z80Ops::addressOnBus(REG_HL, 1); }
void Z80::dcCB57() { /* BIT 2,A */ bitTest(0x04, regA); }

void Z80::dcCB58() { /* BIT 3,B */ bitTest(0x08, REG_B); }
void Z80::dcCB59() { /* BIT 3,C */ bitTest(0x08, REG_C); }
void Z80::dcCB5A() { /* BIT 3,D */ bitTest(0x08, REG_D); }
void Z80::dcCB5B() { /* BIT 3,E */ bitTest(0x08, REG_E); }
void Z80::dcCB5C() { /* BIT 3,H */ bitTest(0x08, REG_H); }
void Z80::dcCB5D() { /* BIT 3,L */ bitTest(0x08, REG_L); }
void Z80::dcCB5E() { /* BIT 3,(HL) */ bitTest(0x08, Z80Ops::peek8(REG_HL)); sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | (REG_W & FLAG_53_MASK); Z80Ops::addressOnBus(REG_HL, 1); }
void Z80::dcCB5F() { /* BIT 3,A */ bitTest(0x08, regA); }

void Z80::dcCB60() { /* BIT 4,B */ bitTest(0x10, REG_B); }
void Z80::dcCB61() { /* BIT 4,C */ bitTest(0x10, REG_C); }
void Z80::dcCB62() { /* BIT 4,D */ bitTest(0x10, REG_D); }
void Z80::dcCB63() { /* BIT 4,E */ bitTest(0x10, REG_E); }
void Z80::dcCB64() { /* BIT 4,H */ bitTest(0x10, REG_H); }
void Z80::dcCB65() { /* BIT 4,L */ bitTest(0x10, REG_L); }
void Z80::dcCB66() { /* BIT 4,(HL) */ bitTest(0x10, Z80Ops::peek8(REG_HL)); sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | (REG_W & FLAG_53_MASK); Z80Ops::addressOnBus(REG_HL, 1); }
void Z80::dcCB67() { /* BIT 4,A */ bitTest(0x10, regA); }

void Z80::dcCB68() { /* BIT 5,B */ bitTest(0x20, REG_B); }
void Z80::dcCB69() { /* BIT 5,C */ bitTest(0x20, REG_C); }
void Z80::dcCB6A() { /* BIT 5,D */ bitTest(0x20, REG_D); }
void Z80::dcCB6B() { /* BIT 5,E */ bitTest(0x20, REG_E); }
void Z80::dcCB6C() { /* BIT 5,H */ bitTest(0x20, REG_H); }
void Z80::dcCB6D() { /* BIT 5,L */ bitTest(0x20, REG_L); }
void Z80::dcCB6E() { /* BIT 5,(HL) */ bitTest(0x20, Z80Ops::peek8(REG_HL)); sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | (REG_W & FLAG_53_MASK); Z80Ops::addressOnBus(REG_HL, 1); }
void Z80::dcCB6F() { /* BIT 5,A */ bitTest(0x20, regA); }

void Z80::dcCB70() { /* BIT 6,B */ bitTest(0x40, REG_B); }
void Z80::dcCB71() { /* BIT 6,C */ bitTest(0x40, REG_C); }
void Z80::dcCB72() { /* BIT 6,D */ bitTest(0x40, REG_D); }
void Z80::dcCB73() { /* BIT 6,E */ bitTest(0x40, REG_E); }
void Z80::dcCB74() { /* BIT 6,H */ bitTest(0x40, REG_H); }
void Z80::dcCB75() { /* BIT 6,L */ bitTest(0x40, REG_L); }
void Z80::dcCB76() { /* BIT 6,(HL) */ bitTest(0x40, Z80Ops::peek8(REG_HL)); sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | (REG_W & FLAG_53_MASK); Z80Ops::addressOnBus(REG_HL, 1); }
void Z80::dcCB77() { /* BIT 6,A */ bitTest(0x40, regA); }

void Z80::dcCB78() { /* BIT 7,B */ bitTest(0x80, REG_B); }
void Z80::dcCB79() { /* BIT 7,C */ bitTest(0x80, REG_C); }
void Z80::dcCB7A() { /* BIT 7,D */ bitTest(0x80, REG_D); }
void Z80::dcCB7B() { /* BIT 7,E */ bitTest(0x80, REG_E); }
void Z80::dcCB7C() { /* BIT 7,H */ bitTest(0x80, REG_H); }
void Z80::dcCB7D() { /* BIT 7,L */ bitTest(0x80, REG_L); }
void Z80::dcCB7E() { /* BIT 7,(HL) */ bitTest(0x80, Z80Ops::peek8(REG_HL)); sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | (REG_W & FLAG_53_MASK); Z80Ops::addressOnBus(REG_HL, 1); }
void Z80::dcCB7F() { /* BIT 7,A */ bitTest(0x80, regA); }

void Z80::dcCB80() { /* RES 0,B */ REG_B &= 0xFE; }
void Z80::dcCB81() { /* RES 0,C */ REG_C &= 0xFE; }
void Z80::dcCB82() { /* RES 0,D */ REG_D &= 0xFE; }
void Z80::dcCB83() { /* RES 0,E */ REG_E &= 0xFE; }
void Z80::dcCB84() { /* RES 0,H */ REG_H &= 0xFE; }
void Z80::dcCB85() { /* RES 0,L */ REG_L &= 0xFE; }
void Z80::dcCB86() { /* RES 0,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) & 0xFE; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB87() { /* RES 0,A */ regA &= 0xFE; }
void Z80::dcCB88() { /* RES 1,B */ REG_B &= 0xFD; }
void Z80::dcCB89() { /* RES 1,C */ REG_C &= 0xFD; }
void Z80::dcCB8A() { /* RES 1,D */ REG_D &= 0xFD; }
void Z80::dcCB8B() { /* RES 1,E */ REG_E &= 0xFD; }
void Z80::dcCB8C() { /* RES 1,H */ REG_H &= 0xFD; }
void Z80::dcCB8D() { /* RES 1,L */ REG_L &= 0xFD; }
void Z80::dcCB8E() { /* RES 1,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) & 0xFD; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB8F() { /* RES 1,A */ regA &= 0xFD; }
void Z80::dcCB90() { /* RES 2,B */ REG_B &= 0xFB; }
void Z80::dcCB91() { /* RES 2,C */ REG_C &= 0xFB; }
void Z80::dcCB92() { /* RES 2,D */ REG_D &= 0xFB; }
void Z80::dcCB93() { /* RES 2,E */ REG_E &= 0xFB; }
void Z80::dcCB94() { /* RES 2,H */ REG_H &= 0xFB; }
void Z80::dcCB95() { /* RES 2,L */ REG_L &= 0xFB; }
void Z80::dcCB96() { /* RES 2,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) & 0xFB; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB97() { /* RES 2,A */ regA &= 0xFB; }
void Z80::dcCB98() { /* RES 3,B */ REG_B &= 0xF7; }
void Z80::dcCB99() { /* RES 3,C */ REG_C &= 0xF7; }
void Z80::dcCB9A() { /* RES 3,D */ REG_D &= 0xF7; }
void Z80::dcCB9B() { /* RES 3,E */ REG_E &= 0xF7; }
void Z80::dcCB9C() { /* RES 3,H */ REG_H &= 0xF7; }
void Z80::dcCB9D() { /* RES 3,L */ REG_L &= 0xF7; }
void Z80::dcCB9E() { /* RES 3,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) & 0xF7; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCB9F() { /* RES 3,A */ regA &= 0xF7; }
void Z80::dcCBA0() { /* RES 4,B */ REG_B &= 0xEF; }
void Z80::dcCBA1() { /* RES 4,C */ REG_C &= 0xEF; }
void Z80::dcCBA2() { /* RES 4,D */ REG_D &= 0xEF; }
void Z80::dcCBA3() { /* RES 4,E */ REG_E &= 0xEF; }
void Z80::dcCBA4() { /* RES 4,H */ REG_H &= 0xEF; }
void Z80::dcCBA5() { /* RES 4,L */ REG_L &= 0xEF; }
void Z80::dcCBA6() { /* RES 4,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) & 0xEF; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBA7() { /* RES 4,A */ regA &= 0xEF; }
void Z80::dcCBA8() { /* RES 5,B */ REG_B &= 0xDF; }
void Z80::dcCBA9() { /* RES 5,C */ REG_C &= 0xDF; }
void Z80::dcCBAA() { /* RES 5,D */ REG_D &= 0xDF; }
void Z80::dcCBAB() { /* RES 5,E */ REG_E &= 0xDF; }
void Z80::dcCBAC() { /* RES 5,H */ REG_H &= 0xDF; }
void Z80::dcCBAD() { /* RES 5,L */ REG_L &= 0xDF; }
void Z80::dcCBAE() { /* RES 5,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) & 0xDF; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBAF() { /* RES 5,A */ regA &= 0xDF; }
void Z80::dcCBB0() { /* RES 6,B */ REG_B &= 0xBF; }
void Z80::dcCBB1() { /* RES 6,C */ REG_C &= 0xBF; }
void Z80::dcCBB2() { /* RES 6,D */ REG_D &= 0xBF; }
void Z80::dcCBB3() { /* RES 6,E */ REG_E &= 0xBF; }
void Z80::dcCBB4() { /* RES 6,H */ REG_H &= 0xBF; }
void Z80::dcCBB5() { /* RES 6,L */ REG_L &= 0xBF; }
void Z80::dcCBB6() { /* RES 6,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) & 0xBF; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBB7() { /* RES 6,A */ regA &= 0xBF; }
void Z80::dcCBB8() { /* RES 7,B */ REG_B &= 0x7F; }
void Z80::dcCBB9() { /* RES 7,C */ REG_C &= 0x7F; }
void Z80::dcCBBA() { /* RES 7,D */ REG_D &= 0x7F; }
void Z80::dcCBBB() { /* RES 7,E */ REG_E &= 0x7F; }
void Z80::dcCBBC() { /* RES 7,H */ REG_H &= 0x7F; }
void Z80::dcCBBD() { /* RES 7,L */ REG_L &= 0x7F; }
void Z80::dcCBBE() { /* RES 7,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) & 0x7F; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBBF() { /* RES 7,A */ regA &= 0x7F; }
void Z80::dcCBC0() { /* SET 0,B */ REG_B |= 0x01; }
void Z80::dcCBC1() { /* SET 0,C */ REG_C |= 0x01; }
void Z80::dcCBC2() { /* SET 0,D */ REG_D |= 0x01; }
void Z80::dcCBC3() { /* SET 0,E */ REG_E |= 0x01; }
void Z80::dcCBC4() { /* SET 0,H */ REG_H |= 0x01; }
void Z80::dcCBC5() { /* SET 0,L */ REG_L |= 0x01; }
void Z80::dcCBC6() { /* SET 0,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) | 0x01; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBC7() { /* SET 0,A */ regA |= 0x01; }
void Z80::dcCBC8() { /* SET 1,B */ REG_B |= 0x02; }
void Z80::dcCBC9() { /* SET 1,C */ REG_C |= 0x02; }
void Z80::dcCBCA() { /* SET 1,D */ REG_D |= 0x02; }
void Z80::dcCBCB() { /* SET 1,E */ REG_E |= 0x02; }
void Z80::dcCBCC() { /* SET 1,H */ REG_H |= 0x02; }
void Z80::dcCBCD() { /* SET 1,L */ REG_L |= 0x02; }
void Z80::dcCBCE() { /* SET 1,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) | 0x02; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBCF() { /* SET 1,A */ regA |= 0x02; }
void Z80::dcCBD0() { /* SET 2,B */ REG_B |= 0x04; }
void Z80::dcCBD1() { /* SET 2,C */ REG_C |= 0x04; }
void Z80::dcCBD2() { /* SET 2,D */ REG_D |= 0x04; }
void Z80::dcCBD3() { /* SET 2,E */ REG_E |= 0x04; }
void Z80::dcCBD4() { /* SET 2,H */ REG_H |= 0x04; }
void Z80::dcCBD5() { /* SET 2,L */ REG_L |= 0x04; }
void Z80::dcCBD6() { /* SET 2,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) | 0x04; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBD7() { /* SET 2,A */ regA |= 0x04; }
void Z80::dcCBD8() { /* SET 3,B */ REG_B |= 0x08; }
void Z80::dcCBD9() { /* SET 3,C */ REG_C |= 0x08; }
void Z80::dcCBDA() { /* SET 3,D */ REG_D |= 0x08; }
void Z80::dcCBDB() { /* SET 3,E */ REG_E |= 0x08; }
void Z80::dcCBDC() { /* SET 3,H */ REG_H |= 0x08; }
void Z80::dcCBDD() { /* SET 3,L */ REG_L |= 0x08; }
void Z80::dcCBDE() { /* SET 3,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) | 0x08; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBDF() { /* SET 3,A */ regA |= 0x08; }
void Z80::dcCBE0() { /* SET 4,B */ REG_B |= 0x10; }
void Z80::dcCBE1() { /* SET 4,C */ REG_C |= 0x10; }
void Z80::dcCBE2() { /* SET 4,D */ REG_D |= 0x10; }
void Z80::dcCBE3() { /* SET 4,E */ REG_E |= 0x10; }
void Z80::dcCBE4() { /* SET 4,H */ REG_H |= 0x10; }
void Z80::dcCBE5() { /* SET 4,L */ REG_L |= 0x10; }
void Z80::dcCBE6() { /* SET 4,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) | 0x10; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBE7() { /* SET 4,A */ regA |= 0x10; }
void Z80::dcCBE8() { /* SET 5,B */ REG_B |= 0x20; }
void Z80::dcCBE9() { /* SET 5,C */ REG_C |= 0x20; }
void Z80::dcCBEA() { /* SET 5,D */ REG_D |= 0x20; }
void Z80::dcCBEB() { /* SET 5,E */ REG_E |= 0x20; }
void Z80::dcCBEC() { /* SET 5,H */ REG_H |= 0x20; }
void Z80::dcCBED() { /* SET 5,L */ REG_L |= 0x20; }
void Z80::dcCBEE() { /* SET 5,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) | 0x20; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBEF() { /* SET 5,A */ regA |= 0x20; }
void Z80::dcCBF0() { /* SET 6,B */ REG_B |= 0x40; }
void Z80::dcCBF1() { /* SET 6,C */ REG_C |= 0x40; }
void Z80::dcCBF2() { /* SET 6,D */ REG_D |= 0x40; }
void Z80::dcCBF3() { /* SET 6,E */ REG_E |= 0x40; }
void Z80::dcCBF4() { /* SET 6,H */ REG_H |= 0x40; }
void Z80::dcCBF5() { /* SET 6,L */ REG_L |= 0x40; }
void Z80::dcCBF6() { /* SET 6,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) | 0x40; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBF7() { /* SET 6,A */ regA |= 0x40; }
void Z80::dcCBF8() { /* SET 7,B */ REG_B |= 0x80; }
void Z80::dcCBF9() { /* SET 7,C */ REG_C |= 0x80; }
void Z80::dcCBFA() { /* SET 7,D */ REG_D |= 0x80; }
void Z80::dcCBFB() { /* SET 7,E */ REG_E |= 0x80; }
void Z80::dcCBFC() { /* SET 7,H */ REG_H |= 0x80; }
void Z80::dcCBFD() { /* SET 7,L */ REG_L |= 0x80; }
void Z80::dcCBFE() { /* SET 7,(HL) */ uint8_t work8 = Z80Ops::peek8(REG_HL) | 0x80; Z80Ops::addressOnBus(REG_HL, 1); Z80Ops::poke8(REG_HL, work8); }
void Z80::dcCBFF() { /* SET 7,A */ regA |= 0x80;}

void (*Z80::dcCB[256])() = {
    &dcCB00, &dcCB01, &dcCB02, &dcCB03,
    &dcCB04, &dcCB05, &dcCB06, &dcCB07,
    &dcCB08, &dcCB09, &dcCB0A, &dcCB0B,
    &dcCB0C, &dcCB0D, &dcCB0E, &dcCB0F,

    &dcCB10, &dcCB11, &dcCB12, &dcCB13,
    &dcCB14, &dcCB15, &dcCB16, &dcCB17,
    &dcCB18, &dcCB19, &dcCB1A, &dcCB1B,
    &dcCB1C, &dcCB1D, &dcCB1E, &dcCB1F,

    &dcCB20, &dcCB21, &dcCB22, &dcCB23,
    &dcCB24, &dcCB25, &dcCB26, &dcCB27,
    &dcCB28, &dcCB29, &dcCB2A, &dcCB2B,
    &dcCB2C, &dcCB2D, &dcCB2E, &dcCB2F,

    &dcCB30, &dcCB31, &dcCB32, &dcCB33,
    &dcCB34, &dcCB35, &dcCB36, &dcCB37,
    &dcCB38, &dcCB39, &dcCB3A, &dcCB3B,
    &dcCB3C, &dcCB3D, &dcCB3E, &dcCB3F,

    &dcCB40, &dcCB41, &dcCB42, &dcCB43,
    &dcCB44, &dcCB45, &dcCB46, &dcCB47,
    &dcCB48, &dcCB49, &dcCB4A, &dcCB4B,
    &dcCB4C, &dcCB4D, &dcCB4E, &dcCB4F,

    &dcCB50, &dcCB51, &dcCB52, &dcCB53,
    &dcCB54, &dcCB55, &dcCB56, &dcCB57,
    &dcCB58, &dcCB59, &dcCB5A, &dcCB5B,
    &dcCB5C, &dcCB5D, &dcCB5E, &dcCB5F,

    &dcCB60, &dcCB61, &dcCB62, &dcCB63,
    &dcCB64, &dcCB65, &dcCB66, &dcCB67,
    &dcCB68, &dcCB69, &dcCB6A, &dcCB6B,
    &dcCB6C, &dcCB6D, &dcCB6E, &dcCB6F,

    &dcCB70, &dcCB71, &dcCB72, &dcCB73,
    &dcCB74, &dcCB75, &dcCB76, &dcCB77,
    &dcCB78, &dcCB79, &dcCB7A, &dcCB7B,
    &dcCB7C, &dcCB7D, &dcCB7E, &dcCB7F,

    &dcCB80, &dcCB81, &dcCB82, &dcCB83,
    &dcCB84, &dcCB85, &dcCB86, &dcCB87,
    &dcCB88, &dcCB89, &dcCB8A, &dcCB8B,
    &dcCB8C, &dcCB8D, &dcCB8E, &dcCB8F,

    &dcCB90, &dcCB91, &dcCB92, &dcCB93,
    &dcCB94, &dcCB95, &dcCB96, &dcCB97,
    &dcCB98, &dcCB99, &dcCB9A, &dcCB9B,
    &dcCB9C, &dcCB9D, &dcCB9E, &dcCB9F,

    &dcCBA0, &dcCBA1, &dcCBA2, &dcCBA3,
    &dcCBA4, &dcCBA5, &dcCBA6, &dcCBA7,
    &dcCBA8, &dcCBA9, &dcCBAA, &dcCBAB,
    &dcCBAC, &dcCBAD, &dcCBAE, &dcCBAF,

    &dcCBB0, &dcCBB1, &dcCBB2, &dcCBB3,
    &dcCBB4, &dcCBB5, &dcCBB6, &dcCBB7,
    &dcCBB8, &dcCBB9, &dcCBBA, &dcCBBB,
    &dcCBBC, &dcCBBD, &dcCBBE, &dcCBBF,

    &dcCBC0, &dcCBC1, &dcCBC2, &dcCBC3,
    &dcCBC4, &dcCBC5, &dcCBC6, &dcCBC7,
    &dcCBC8, &dcCBC9, &dcCBCA, &dcCBCB,
    &dcCBCC, &dcCBCD, &dcCBCE, &dcCBCF,

    &dcCBD0, &dcCBD1, &dcCBD2, &dcCBD3,
    &dcCBD4, &dcCBD5, &dcCBD6, &dcCBD7,
    &dcCBD8, &dcCBD9, &dcCBDA, &dcCBDB,
    &dcCBDC, &dcCBDD, &dcCBDE, &dcCBDF,

    &dcCBE0, &dcCBE1, &dcCBE2, &dcCBE3,
    &dcCBE4, &dcCBE5, &dcCBE6, &dcCBE7,
    &dcCBE8, &dcCBE9, &dcCBEA, &dcCBEB,
    &dcCBEC, &dcCBED, &dcCBEE, &dcCBEF,

    &dcCBF0, &dcCBF1, &dcCBF2, &dcCBF3,
    &dcCBF4, &dcCBF5, &dcCBF6, &dcCBF7,
    &dcCBF8, &dcCBF9, &dcCBFA, &dcCBFB,
    &dcCBFC, &dcCBFD, &dcCBFE, &dcCBFF

};

//Subconjunto de instrucciones 0xDD / 0xFD
/*
 * Hay que tener en cuenta el manejo de secuencias códigos DD/FD que no
 * hacen nada. Según el apartado 3.7 del documento
 * [http://www.myquest.nl/z80undocumented/z80-documented-v0.91.pdf]
 * secuencias de códigos como FD DD 00 21 00 10 NOP NOP NOP LD HL,1000h
 * activan IY con el primer FD, IX con el segundo DD y vuelven al
 * registro HL con el código NOP. Es decir, si detrás del código DD/FD no
 * viene una instrucción que maneje el registro HL, el código DD/FD
 * "se olvida" y hay que procesar la instrucción como si nunca se
 * hubiera visto el prefijo (salvo por los 4 t-estados que ha costado).
 * Naturalmente, en una serie repetida de DDFD no hay que comprobar las
 * interrupciones entre cada prefijo.
 */


void Z80::dcDDFD09(RegisterPair& regIXY) { /* ADD IX,BC */
    Z80Ops::addressOnBus(getPairIR().word, 7);
    add16(regIXY, REG_BC);
}
void Z80::dcDDFD19(RegisterPair& regIXY) { /* ADD IX,DE */
    Z80Ops::addressOnBus(getPairIR().word, 7);
    add16(regIXY, REG_DE);
}
void Z80::dcDDFD21(RegisterPair& regIXY) { /* LD IX,nn */
    regIXY.word = Z80Ops::peek16(REG_PC);
    REG_PC += 2;
}
void Z80::dcDDFD22(RegisterPair& regIXY) { /* LD (nn),IX */
    const uint16_t addr = Z80Ops::peek16(REG_PC);
    Z80Ops::poke16(addr, regIXY);
    REG_WZ = addr + 1;
    REG_PC += 2;
}
void Z80::dcDDFD23(RegisterPair& regIXY) { /* INC IX */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    ++regIXY.word;
}
void Z80::dcDDFD24(RegisterPair& regIXY) { /* INC IXh */
    inc8(regIXY.byte8.hi);
}
void Z80::dcDDFD25(RegisterPair& regIXY) { /* DEC IXh */
    dec8(regIXY.byte8.hi);
}
void Z80::dcDDFD26(RegisterPair& regIXY) { /* LD IXh,n */
    regIXY.byte8.hi = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}
void Z80::dcDDFD29(RegisterPair& regIXY) { /* ADD IX,IX */
    Z80Ops::addressOnBus(getPairIR().word, 7);
    add16(regIXY, regIXY.word);
}
void Z80::dcDDFD2A(RegisterPair& regIXY) { /* LD IX,(nn) */
    const uint16_t addr = Z80Ops::peek16(REG_PC);
    regIXY.word = Z80Ops::peek16(addr);
    REG_WZ = addr + 1;
    REG_PC += 2;
}
void Z80::dcDDFD2B(RegisterPair& regIXY) { /* DEC IX */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    --regIXY.word;

    if (REG_PC == 0x04d4 && Config::realtape_mode != REALTAPE_FORCE_SAVE) { // Save trap

        static bool SaveFileExists;

        if (REG_HL == 0x1F80) {
            struct stat stat_buf;
            SaveFileExists = Tape::tape && // Check if exists a tap opened
                             Tape::tapeFileType == TAPE_FTYPE_TAP && // check if file is tap
                             !stat(Tape::tapeFullPathName.c_str(), &stat_buf); // this check for SD fail

            // if (!SaveFileExists) OSD::osdCenteredMsg(OSD_TAPE_SELECT_ERR[Config::lang], LEVEL_WARN);

            if (SaveFileExists && Tape::tapeIsReadOnly) {
                OSD::osdCenteredMsg(OSD_READONLY_FILE_WARN[Config::lang], LEVEL_WARN);
                SaveFileExists = false;
            }

        }

        if (SaveFileExists) {
            regA = (REG_HL == 0x1F80) ? 0x00 : 0xFF;
            ++regIXY.word;
            --REG_DE;
            Tape::Save();
            REG_PC = 0x555;
        }
    }
}
void Z80::dcDDFD2C(RegisterPair& regIXY) { /* INC IXl */
    inc8(regIXY.byte8.lo);
}
void Z80::dcDDFD2D(RegisterPair& regIXY) { /* DEC IXl */
    dec8(regIXY.byte8.lo);
}
void Z80::dcDDFD2E(RegisterPair& regIXY) { /* LD IXl,n */
    regIXY.byte8.lo = Z80Ops::peek8(REG_PC);
    ++REG_PC;
}
void Z80::dcDDFD34(RegisterPair& regIXY) { /* INC (IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    uint8_t work8 = Z80Ops::peek8(REG_WZ);
    Z80Ops::addressOnBus(REG_WZ, 1);
    inc8(work8);
    Z80Ops::poke8(REG_WZ, work8);
}
void Z80::dcDDFD35(RegisterPair& regIXY) { /* DEC (IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    uint8_t work8 = Z80Ops::peek8(REG_WZ);
    Z80Ops::addressOnBus(REG_WZ, 1);
    dec8(work8);
    Z80Ops::poke8(REG_WZ, work8);
}
void Z80::dcDDFD36(RegisterPair& regIXY) { /* LD (IX+d),n */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    ++REG_PC;
    uint8_t work8 = Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 2);
    ++REG_PC;
    Z80Ops::poke8(REG_WZ, work8);
}
void Z80::dcDDFD39(RegisterPair& regIXY) { /* ADD IX,SP */
    Z80Ops::addressOnBus(getPairIR().word, 7);
    add16(regIXY, REG_SP);
}
void Z80::dcDDFD44(RegisterPair& regIXY) { /* LD B,IXh */
    REG_B = regIXY.byte8.hi;
}
void Z80::dcDDFD45(RegisterPair& regIXY) { /* LD B,IXl */
    REG_B = regIXY.byte8.lo;
}
void Z80::dcDDFD46(RegisterPair& regIXY) { /* LD B,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    REG_B = Z80Ops::peek8(REG_WZ);
}
void Z80::dcDDFD4C(RegisterPair& regIXY) { /* LD C,IXh */
    REG_C = regIXY.byte8.hi;
}
void Z80::dcDDFD4D(RegisterPair& regIXY) { /* LD C,IXl */
    REG_C = regIXY.byte8.lo;
}
void Z80::dcDDFD4E(RegisterPair& regIXY) { /* LD C,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    REG_C = Z80Ops::peek8(REG_WZ);
}
void Z80::dcDDFD54(RegisterPair& regIXY) { /* LD D,IXh */
    REG_D = regIXY.byte8.hi;
}
void Z80::dcDDFD55(RegisterPair& regIXY) { /* LD D,IXl */
    REG_D = regIXY.byte8.lo;
}
void Z80::dcDDFD56(RegisterPair& regIXY) { /* LD D,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    REG_D = Z80Ops::peek8(REG_WZ);
}
void Z80::dcDDFD5C(RegisterPair& regIXY) { /* LD E,IXh */
    REG_E = regIXY.byte8.hi;
}
void Z80::dcDDFD5D(RegisterPair& regIXY) { /* LD E,IXl */
    REG_E = regIXY.byte8.lo;
}
void Z80::dcDDFD5E(RegisterPair& regIXY) { /* LD E,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    REG_E = Z80Ops::peek8(REG_WZ);
}
void Z80::dcDDFD60(RegisterPair& regIXY) { /* LD IXh,B */
    regIXY.byte8.hi = REG_B;
}
void Z80::dcDDFD61(RegisterPair& regIXY) { /* LD IXh,C */
    regIXY.byte8.hi = REG_C;
}
void Z80::dcDDFD62(RegisterPair& regIXY) { /* LD IXh,D */
    regIXY.byte8.hi = REG_D;
}
void Z80::dcDDFD63(RegisterPair& regIXY) { /* LD IXh,E */
    regIXY.byte8.hi = REG_E;
}
void Z80::dcDDFD64(RegisterPair& regIXY) { /* LD IXh,IXh */
}
void Z80::dcDDFD65(RegisterPair& regIXY) { /* LD IXh,IXl */
    regIXY.byte8.hi = regIXY.byte8.lo;
}
void Z80::dcDDFD66(RegisterPair& regIXY) { /* LD H,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    REG_H = Z80Ops::peek8(REG_WZ);
}
void Z80::dcDDFD67(RegisterPair& regIXY) { /* LD IXh,A */
    regIXY.byte8.hi = regA;
}
void Z80::dcDDFD68(RegisterPair& regIXY) { /* LD IXl,B */
    regIXY.byte8.lo = REG_B;
}
void Z80::dcDDFD69(RegisterPair& regIXY) { /* LD IXl,C */
    regIXY.byte8.lo = REG_C;
}
void Z80::dcDDFD6A(RegisterPair& regIXY) { /* LD IXl,D */
    regIXY.byte8.lo = REG_D;
}
void Z80::dcDDFD6B(RegisterPair& regIXY) { /* LD IXl,E */
    regIXY.byte8.lo = REG_E;
}
void Z80::dcDDFD6C(RegisterPair& regIXY) { /* LD IXl,IXh */
    regIXY.byte8.lo = regIXY.byte8.hi;
}
void Z80::dcDDFD6D(RegisterPair& regIXY) { /* LD IXl,IXl */
}
void Z80::dcDDFD6E(RegisterPair& regIXY) { /* LD L,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    REG_L = Z80Ops::peek8(REG_WZ);
}
void Z80::dcDDFD6F(RegisterPair& regIXY) { /* LD IXl,A */
    regIXY.byte8.lo = regA;
}
void Z80::dcDDFD70(RegisterPair& regIXY) { /* LD (IX+d),B */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    Z80Ops::poke8(REG_WZ, REG_B);
}
void Z80::dcDDFD71(RegisterPair& regIXY) { /* LD (IX+d),C */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    Z80Ops::poke8(REG_WZ, REG_C);
}
void Z80::dcDDFD72(RegisterPair& regIXY) { /* LD (IX+d),D */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    Z80Ops::poke8(REG_WZ, REG_D);
}
void Z80::dcDDFD73(RegisterPair& regIXY) { /* LD (IX+d),E */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    Z80Ops::poke8(REG_WZ, REG_E);
}
void Z80::dcDDFD74(RegisterPair& regIXY) { /* LD (IX+d),H */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    Z80Ops::poke8(REG_WZ, REG_H);
}
void Z80::dcDDFD75(RegisterPair& regIXY) { /* LD (IX+d),L */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    Z80Ops::poke8(REG_WZ, REG_L);
}
void Z80::dcDDFD77(RegisterPair& regIXY) { /* LD (IX+d),A */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    Z80Ops::poke8(REG_WZ, regA);
}
void Z80::dcDDFD7C(RegisterPair& regIXY) { /* LD A,IXh */
    regA = regIXY.byte8.hi;
}
void Z80::dcDDFD7D(RegisterPair& regIXY) { /* LD A,IXl */
    regA = regIXY.byte8.lo;
}
void Z80::dcDDFD7E(RegisterPair& regIXY) { /* LD A,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    regA = Z80Ops::peek8(REG_WZ);
}
void Z80::dcDDFD84(RegisterPair& regIXY) { /* ADD A,IXh */
    add(regIXY.byte8.hi);
}
void Z80::dcDDFD85(RegisterPair& regIXY) { /* ADD A,IXl */
    add(regIXY.byte8.lo);
}
void Z80::dcDDFD86(RegisterPair& regIXY) { /* ADD A,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    add(Z80Ops::peek8(REG_WZ));
}
void Z80::dcDDFD8C(RegisterPair& regIXY) { /* ADC A,IXh */
    adc(regIXY.byte8.hi);
}
void Z80::dcDDFD8D(RegisterPair& regIXY) { /* ADC A,IXl */
    adc(regIXY.byte8.lo);
}
void Z80::dcDDFD8E(RegisterPair& regIXY) { /* ADC A,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    adc(Z80Ops::peek8(REG_WZ));
}
void Z80::dcDDFD94(RegisterPair& regIXY) { /* SUB IXh */
    sub(regIXY.byte8.hi);
}
void Z80::dcDDFD95(RegisterPair& regIXY) { /* SUB IXl */
    sub(regIXY.byte8.lo);
}
void Z80::dcDDFD96(RegisterPair& regIXY) { /* SUB (IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    sub(Z80Ops::peek8(REG_WZ));
}
void Z80::dcDDFD9C(RegisterPair& regIXY) { /* SBC A,IXh */
    sbc(regIXY.byte8.hi);
}
void Z80::dcDDFD9D(RegisterPair& regIXY) { /* SBC A,IXl */
    sbc(regIXY.byte8.lo);
}
void Z80::dcDDFD9E(RegisterPair& regIXY) { /* SBC A,(IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    sbc(Z80Ops::peek8(REG_WZ));
}
void Z80::dcDDFDA4(RegisterPair& regIXY) { /* AND IXh */
    and_(regIXY.byte8.hi);
}
void Z80::dcDDFDA5(RegisterPair& regIXY) { /* AND IXl */
    and_(regIXY.byte8.lo);
}
void Z80::dcDDFDA6(RegisterPair& regIXY) { /* AND (IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    and_(Z80Ops::peek8(REG_WZ));
}
void Z80::dcDDFDAC(RegisterPair& regIXY) { /* XOR IXh */
    xor_(regIXY.byte8.hi);
}
void Z80::dcDDFDAD(RegisterPair& regIXY) { /* XOR IXl */
    xor_(regIXY.byte8.lo);
}
void Z80::dcDDFDAE(RegisterPair& regIXY) { /* XOR (IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    xor_(Z80Ops::peek8(REG_WZ));
}
void Z80::dcDDFDB4(RegisterPair& regIXY) { /* OR IXh */
    or_(regIXY.byte8.hi);
}
void Z80::dcDDFDB5(RegisterPair& regIXY) { /* OR IXl */
    or_(regIXY.byte8.lo);
}
void Z80::dcDDFDB6(RegisterPair& regIXY) { /* OR (IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    or_(Z80Ops::peek8(REG_WZ));
}
void Z80::dcDDFDBC(RegisterPair& regIXY) { /* CP IXh */
    cp(regIXY.byte8.hi);
}
void Z80::dcDDFDBD(RegisterPair& regIXY) { /* CP IXl */
    cp(regIXY.byte8.lo);
}
void Z80::dcDDFDBE(RegisterPair& regIXY) { /* CP (IX+d) */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 5);
    ++REG_PC;
    cp(Z80Ops::peek8(REG_WZ));
}
void Z80::dcDDFDCB(RegisterPair& regIXY) { /* Subconjunto de instrucciones */
    REG_WZ = regIXY.word + (int8_t) Z80Ops::peek8(REG_PC);
    ++REG_PC;
    opCode = Z80Ops::peek8(REG_PC);
    Z80Ops::addressOnBus(REG_PC, 2);
    ++REG_PC;
    decodeDDFDCB(REG_WZ);
}
void Z80::dcDDFDDD(RegisterPair& regIXY) {
    prefixOpcode = 0xDD;
}
void Z80::dcDDFDE1(RegisterPair& regIXY) { /* POP IX */
    regIXY.word = pop();
}
void Z80::dcDDFDE3(RegisterPair& regIXY) { /* EX (SP),IX */
    // Instrucción de ejecución sutil como pocas... atento al dato.
    RegisterPair work16 = regIXY;
    regIXY.word = Z80Ops::peek16(REG_SP);
    Z80Ops::addressOnBus(REG_SP + 1, 1);
    // I can't call to poke16 from here because the Z80 do the writes in inverted order
    // Same for EX (SP), HL
    Z80Ops::poke8(REG_SP + 1, work16.byte8.hi);
    Z80Ops::poke8(REG_SP, work16.byte8.lo);
    Z80Ops::addressOnBus(REG_SP, 2);
    REG_WZ = regIXY.word;
}
void Z80::dcDDFDE5(RegisterPair& regIXY) { /* PUSH IX */
    Z80Ops::addressOnBus(getPairIR().word, 1);
    push(regIXY.word);
}
void Z80::dcDDFDE9(RegisterPair& regIXY) { /* JP (IX) */
    REG_PC = regIXY.word;
    check_trdos();
}
void Z80::dcDDFDED(RegisterPair& regIXY) {
    prefixOpcode = 0xED;
}
void Z80::dcDDFDF9(RegisterPair& regIXY) { /* LD SP,IX */
    Z80Ops::addressOnBus(getPairIR().word, 2);
    REG_SP = regIXY.word;
}
void Z80::dcDDFDFD(RegisterPair& regIXY) {
    prefixOpcode = 0xFD;
}

void Z80::dcDDFDdefault(RegisterPair& regIXY) {
    // Detrás de un DD/FD o varios en secuencia venía un código
    // que no correspondía con una instrucción que involucra a
    // IX o IY. Se trata como si fuera un código normal.
    // Sin esto, además de emular mal, falla el test
    // ld <bcdexya>,<bcdexya> de ZEXALL.
    #ifdef WITH_BREAKPOINT_SUPPORT
    if (breakpointEnabled && prefixOpcode == 0) {
        opCode = Z80Ops::breakpoint(REG_PC, opCode);
    }
    #endif
    dcOpcode[opCode]();
    // decodeOpcode();

}

void (*Z80::dcDDFD[256])(RegisterPair& regIXY) = {
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFD09     , &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFD19     , &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFD21     , &dcDDFD22     , &dcDDFD23     ,
    &dcDDFD24     , &dcDDFD25     , &dcDDFD26     , &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFD29     , &dcDDFD2A     , &dcDDFD2B     ,
    &dcDDFD2C     , &dcDDFD2D     , &dcDDFD2E     , &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD34     , &dcDDFD35     , &dcDDFD36     , &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFD39     , &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD44     , &dcDDFD45     , &dcDDFD46     , &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD4C     , &dcDDFD4D     , &dcDDFD4E     , &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD54     , &dcDDFD55     , &dcDDFD56     , &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD5C     , &dcDDFD5D     , &dcDDFD5E     , &dcDDFDdefault,

    &dcDDFD60     , &dcDDFD61     , &dcDDFD62     , &dcDDFD63     ,
    &dcDDFD64     , &dcDDFD65     , &dcDDFD66     , &dcDDFD67     ,
    &dcDDFD68     , &dcDDFD69     , &dcDDFD6A     , &dcDDFD6B     ,
    &dcDDFD6C     , &dcDDFD6D     , &dcDDFD6E     , &dcDDFD6F     ,

    &dcDDFD70     , &dcDDFD71     , &dcDDFD72     , &dcDDFD73     ,
    &dcDDFD74     , &dcDDFD75     , &dcDDFDdefault, &dcDDFD77     ,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD7C     , &dcDDFD7D     , &dcDDFD7E     , &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD84     , &dcDDFD85     , &dcDDFD86     , &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD8C     , &dcDDFD8D     , &dcDDFD8E     , &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD94     , &dcDDFD95     , &dcDDFD96     , &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFD9C     , &dcDDFD9D     , &dcDDFD9E     , &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDA4     , &dcDDFDA5     , &dcDDFDA6     , &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDAC     , &dcDDFDAD     , &dcDDFDAE     , &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDB4     , &dcDDFDB5     , &dcDDFDB6     , &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDBC     , &dcDDFDBD     , &dcDDFDBE     , &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDCB     ,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDDD     , &dcDDFDdefault, &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDE1     , &dcDDFDdefault, &dcDDFDE3     ,
    &dcDDFDdefault, &dcDDFDE5     , &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDE9     , &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDED     , &dcDDFDdefault, &dcDDFDdefault,

    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDF9     , &dcDDFDdefault, &dcDDFDdefault,
    &dcDDFDdefault, &dcDDFDFD     , &dcDDFDdefault, &dcDDFDdefault

};

// Subconjunto de instrucciones 0xDDCB

void inline Z80::decodeDDFDCB(uint16_t address) {
    uint8_t work8;
    const uint8_t opGroup = opCode & 0xC0;
    const uint8_t bitMask = 1 << ((opCode >> 3) & 0x07);

    switch (opGroup) {
        // Rotaciones y desplazamientos (0x00-0x3F)
        case 0x00:
            work8 = Z80Ops::peek8(address);
            switch (opCode & 0x38) {
                case 0x00: rlc(work8); break;
                case 0x08: rrc(work8); break;
                case 0x10: rl(work8); break;
                case 0x18: rr(work8); break;
                case 0x20: sla(work8); break;
                case 0x28: sra(work8); break;
                case 0x30: sll(work8); break;
                case 0x38: srl(work8); break;
            }
            Z80Ops::addressOnBus(address, 1);
            Z80Ops::poke8(address, work8);
            copyToRegister(work8);
            break;

        // Instrucciones BIT (0x40-0x7F)
        case 0x40:
            bitTest(bitMask, Z80Ops::peek8(address));
            sz5h3pnFlags = (sz5h3pnFlags & FLAG_SZHP_MASK) | ((address >> 8) & FLAG_53_MASK);
            Z80Ops::addressOnBus(address, 1);
            break;

        // Instrucciones RES (0x80-0xBF)
        case 0x80:
            work8 = Z80Ops::peek8(address) & ~bitMask;
            Z80Ops::addressOnBus(address, 1);
            Z80Ops::poke8(address, work8);
            copyToRegister(work8);
            break;

        // Instrucciones SET (0xC0-0xFF)
        case 0xC0:
            work8 = Z80Ops::peek8(address) | bitMask;
            Z80Ops::addressOnBus(address, 1);
            Z80Ops::poke8(address, work8);
            copyToRegister(work8);
            break;
    }
}

//Subconjunto de instrucciones 0xED

void Z80::decodeED(void) {
    switch (opCode) {
        case 0x40:
        { /* IN B,(C) */
            REG_WZ = REG_BC;
            REG_B = Ports::input(REG_WZ);
            ++REG_WZ;
            sz5h3pnFlags = sz53pn_addTable[REG_B];
            flagQ = true;
            break;
        }
        case 0x41:
        { /* OUT (C),B */
            Ports::output(REG_BC, REG_B);
            REG_WZ = REG_BC + 1;
            break;
        }
        case 0x42:
        { /* SBC HL,BC */
            Z80Ops::addressOnBus(getPairIR().word, 7);
            sbc16(REG_BC);
            break;
        }
        case 0x43:
        { /* LD (nn),BC */
            REG_WZ = Z80Ops::peek16(REG_PC);
            Z80Ops::poke16(REG_WZ, regBC);
            ++REG_WZ;
            REG_PC += 2;
            break;
        }
        case 0x44:
        case 0x4C:
        case 0x54:
        case 0x5C:
        case 0x64:
        case 0x6C:
        case 0x74:
        case 0x7C:
        { /* NEG */
            uint8_t aux = regA;
            regA = 0;
            carryFlag = false;
            sbc(aux);
            break;
        }
        case 0x45:
        case 0x4D: /* RETI */
        case 0x55:
        case 0x5D:
        case 0x65:
        case 0x6D:
        case 0x75:
        case 0x7D:
        { /* RETN */
            ffIFF1 = ffIFF2;
            REG_PC = REG_WZ = pop();
            break;
        }
        case 0x46:
        case 0x4E:
        case 0x66:
        case 0x6E:
        { /* IM 0 */
            modeINT = IntMode::IM0;
            break;
        }
        case 0x47:
        { /* LD I,A */
            /* El par IR se pone en el bus de direcciones *antes*
             * de poner A en el registro I. Detalle importante. */
            Z80Ops::addressOnBus(getPairIR().word, 1);
            regI = regA;
            break;
        }
        case 0x48:
        { /* IN C,(C) */
            REG_WZ = REG_BC;
            REG_C = Ports::input(REG_WZ);
            ++REG_WZ;
            sz5h3pnFlags = sz53pn_addTable[REG_C];
            flagQ = true;
            break;
        }
        case 0x49:
        { /* OUT (C),C */
            Ports::output(REG_BC, REG_C);
            REG_WZ = REG_BC + 1;
            break;
        }
        case 0x4A:
        { /* ADC HL,BC */
            Z80Ops::addressOnBus(getPairIR().word, 7);
            adc16(REG_BC);
            break;
        }
        case 0x4B:
        { /* LD BC,(nn) */
            REG_WZ = Z80Ops::peek16(REG_PC);
            REG_BC = Z80Ops::peek16(REG_WZ);
            ++REG_WZ;
            REG_PC += 2;
            break;
        }
        case 0x4F:
        { /* LD R,A */
            /* El par IR se pone en el bus de direcciones *antes*
             * de poner A en el registro R. Detalle importante. */
            Z80Ops::addressOnBus(getPairIR().word, 1);
            setRegR(regA);
            break;
        }
        case 0x50:
        { /* IN D,(C) */
            REG_WZ = REG_BC;
            REG_D = Ports::input(REG_WZ);
            ++REG_WZ;
            sz5h3pnFlags = sz53pn_addTable[REG_D];
            flagQ = true;
            break;
        }
        case 0x51:
        { /* OUT (C),D */
            Ports::output(REG_BC, REG_D);
            REG_WZ = REG_BC + 1;
            break;
        }
        case 0x52:
        { /* SBC HL,DE */
            Z80Ops::addressOnBus(getPairIR().word, 7);
            sbc16(REG_DE);
            break;
        }
        case 0x53:
        { /* LD (nn),DE */
            REG_WZ = Z80Ops::peek16(REG_PC);
            Z80Ops::poke16(REG_WZ++, regDE);
            REG_PC += 2;
            break;
        }
        case 0x56:
        case 0x76:
        { /* IM 1 */
            modeINT = IntMode::IM1;
            break;
        }
        case 0x57:
        { /* LD A,I */
            Z80Ops::addressOnBus(getPairIR().word, 1);
            regA = regI;
            sz5h3pnFlags = sz53n_addTable[regA];
            if (ffIFF2 && !Z80Ops::isActiveINT()) {
                sz5h3pnFlags |= PARITY_MASK;
            }
            flagQ = true;
            break;
        }
        case 0x58:
        { /* IN E,(C) */
            REG_WZ = REG_BC;
            REG_E = Ports::input(REG_WZ++);
            sz5h3pnFlags = sz53pn_addTable[REG_E];
            flagQ = true;
            break;
        }
        case 0x59:
        { /* OUT (C),E */
            Ports::output(REG_BC, REG_E);
            REG_WZ = REG_BC + 1;
            break;
        }
        case 0x5A:
        { /* ADC HL,DE */
            Z80Ops::addressOnBus(getPairIR().word, 7);
            adc16(REG_DE);
            break;
        }
        case 0x5B:
        { /* LD DE,(nn) */
            REG_WZ = Z80Ops::peek16(REG_PC);
            REG_DE = Z80Ops::peek16(REG_WZ++);
            REG_PC += 2;
            break;
        }
        case 0x5E:
        case 0x7E:
        { /* IM 2 */
            modeINT = IntMode::IM2;
            break;
        }
        case 0x5F:
        { /* LD A,R */
            Z80Ops::addressOnBus(getPairIR().word, 1);
            regA = getRegR();
            sz5h3pnFlags = sz53n_addTable[regA];
            if (ffIFF2 && !Z80Ops::isActiveINT()) {
                sz5h3pnFlags |= PARITY_MASK;
            }
            flagQ = true;
            break;
        }
        case 0x60:
        { /* IN H,(C) */
            REG_WZ = REG_BC;
            REG_H = Ports::input(REG_WZ++);
            sz5h3pnFlags = sz53pn_addTable[REG_H];
            flagQ = true;
            break;
        }
        case 0x61:
        { /* OUT (C),H */
            Ports::output(REG_BC, REG_H);
            REG_WZ = REG_BC + 1;
            break;
        }
        case 0x62:
        { /* SBC HL,HL */
            Z80Ops::addressOnBus(getPairIR().word, 7);
            sbc16(REG_HL);
            break;
        }
        case 0x63:
        { /* LD (nn),HL */
            REG_WZ = Z80Ops::peek16(REG_PC);
            Z80Ops::poke16(REG_WZ++, regHL);
            REG_PC += 2;
            break;
        }
        case 0x67:
        { /* RRD */
            // A = A7 A6 A5 A4 (HL)3 (HL)2 (HL)1 (HL)0
            // (HL) = A3 A2 A1 A0 (HL)7 (HL)6 (HL)5 (HL)4
            // Los bits 3,2,1 y 0 de (HL) se copian a los bits 3,2,1 y 0 de A.
            // Los 4 bits bajos que había en A se copian a los bits 7,6,5 y 4 de (HL).
            // Los 4 bits altos que había en (HL) se copian a los 4 bits bajos de (HL)
            // Los 4 bits superiores de A no se tocan. ¡p'habernos matao!
            uint8_t aux = regA << 4;
            REG_WZ = REG_HL;
            uint16_t memHL = Z80Ops::peek8(REG_WZ);
            regA = (regA & 0xf0) | (memHL & 0x0f);
            Z80Ops::addressOnBus(REG_WZ, 4);
            Z80Ops::poke8(REG_WZ++, (memHL >> 4) | aux);
            sz5h3pnFlags = sz53pn_addTable[regA];
            flagQ = true;
            break;
        }
        case 0x68:
        { /* IN L,(C) */
            REG_WZ = REG_BC;
            REG_L = Ports::input(REG_WZ++);
            sz5h3pnFlags = sz53pn_addTable[REG_L];
            flagQ = true;
            break;
        }
        case 0x69:
        { /* OUT (C),L */
            Ports::output(REG_BC, REG_L);
            REG_WZ = REG_BC + 1;
            break;
        }
        case 0x6A:
        { /* ADC HL,HL */
            Z80Ops::addressOnBus(getPairIR().word, 7);
            adc16(REG_HL);
            break;
        }
        case 0x6B:
        { /* LD HL,(nn) */
            REG_WZ = Z80Ops::peek16(REG_PC);
            REG_HL = Z80Ops::peek16(REG_WZ++);
            REG_PC += 2;
            break;
        }
        case 0x6F:
        { /* RLD */
            // A = A7 A6 A5 A4 (HL)7 (HL)6 (HL)5 (HL)4
            // (HL) = (HL)3 (HL)2 (HL)1 (HL)0 A3 A2 A1 A0
            // Los 4 bits bajos que había en (HL) se copian a los bits altos de (HL).
            // Los 4 bits altos que había en (HL) se copian a los 4 bits bajos de A
            // Los bits 3,2,1 y 0 de A se copian a los bits 3,2,1 y 0 de (HL).
            // Los 4 bits superiores de A no se tocan. ¡p'habernos matao!
            uint8_t aux = regA & 0x0f;
            REG_WZ = REG_HL;
            uint16_t memHL = Z80Ops::peek8(REG_WZ);
            regA = (regA & 0xf0) | (memHL >> 4);
            Z80Ops::addressOnBus(REG_WZ, 4);
            Z80Ops::poke8(REG_WZ++, (memHL << 4) | aux);
            sz5h3pnFlags = sz53pn_addTable[regA];
            flagQ = true;
            break;
        }
        case 0x70:
        { /* IN (C) */
            REG_WZ = REG_BC;
            uint8_t inPort = Ports::input(REG_WZ++);
            sz5h3pnFlags = sz53pn_addTable[inPort];
            flagQ = true;
            break;
        }
        case 0x71:
        { /* OUT (C),0 */
            Ports::output(REG_BC, 0x00); // NMOS Z80 returns 0x00
            REG_WZ = REG_BC + 1;
            break;
        }
        case 0x72:
        { /* SBC HL,SP */
            Z80Ops::addressOnBus(getPairIR().word, 7);
            sbc16(REG_SP);
            break;
        }
        case 0x73:
        { /* LD (nn),SP */
            REG_WZ = Z80Ops::peek16(REG_PC);
            Z80Ops::poke16(REG_WZ++, regSP);
            REG_PC += 2;
            break;
        }
        case 0x78:
        { /* IN A,(C) */
            REG_WZ = REG_BC;
            regA = Ports::input(REG_WZ++);
            sz5h3pnFlags = sz53pn_addTable[regA];
            flagQ = true;
            break;
        }
        case 0x79:
        { /* OUT (C),A */
            Ports::output(REG_BC, regA);
            REG_WZ = REG_BC + 1;
            break;
        }
        case 0x7A:
        { /* ADC HL,SP */
            Z80Ops::addressOnBus(getPairIR().word, 7);
            adc16(REG_SP);
            break;
        }
        case 0x7B:
        { /* LD SP,(nn) */
            REG_WZ = Z80Ops::peek16(REG_PC);
            REG_SP = Z80Ops::peek16(REG_WZ++);
            REG_PC += 2;
            break;
        }
        case 0xA0:
        { /* LDI */
            ldi();
            break;
        }
        case 0xA1:
        { /* CPI */
            cpi();
            break;
        }
        case 0xA2:
        { /* INI */
            ini();
            break;
        }
        case 0xA3:
        { /* OUTI */
            outi();
            break;
        }
        case 0xA8:
        { /* LDD */
            ldd();
            break;
        }
        case 0xA9:
        { /* CPD */
            cpd();
            break;
        }
        case 0xAA:
        { /* IND */
            ind();
            break;
        }
        case 0xAB:
        { /* OUTD */
            outd();
            break;
        }
        case 0xB0:
        { /* LDIR */
            ldi();
            if (REG_BC != 0) {
                REG_PC -= 2;
                REG_WZ = REG_PC + 1;
                Z80Ops::addressOnBus(REG_DE - 1, 5);
                sz5h3pnFlags &= ~FLAG_53_MASK;
                sz5h3pnFlags |= (REG_PCh & FLAG_53_MASK);
            }
            break;
        }
        case 0xB1:
        { /* CPIR */
            cpi();
            if ((sz5h3pnFlags & PARITY_MASK) == PARITY_MASK && (sz5h3pnFlags & ZERO_MASK) == 0) {
                REG_PC -= 2;
                REG_WZ = REG_PC + 1;
                Z80Ops::addressOnBus(REG_HL - 1, 5);
                sz5h3pnFlags &= ~FLAG_53_MASK;
                sz5h3pnFlags |= (REG_PCh & FLAG_53_MASK);
            }
            break;
        }
        case 0xB2:
        { /* INIR */
            ini();
            if (REG_B != 0) {
                REG_PC -= 2;
                REG_WZ = REG_PC + 1;
                Z80Ops::addressOnBus(REG_HL - 1, 5);
                SetAbortedINxR_OTxRFlags();
            }
            break;
        }
        case 0xB3:
        { /* OTIR */
            outi();
            if (REG_B != 0) {
                REG_PC -= 2;
                REG_WZ = REG_PC + 1;
                Z80Ops::addressOnBus(REG_BC, 5);
                SetAbortedINxR_OTxRFlags();
            }
            break;
        }
        case 0xB8:
        { /* LDDR */
            ldd();
            if (REG_BC != 0) {
                REG_PC -= 2;
                REG_WZ = REG_PC + 1;
                Z80Ops::addressOnBus(REG_DE + 1, 5);
                sz5h3pnFlags &= ~FLAG_53_MASK;
                sz5h3pnFlags |= (REG_PCh & FLAG_53_MASK);
            }
            break;
        }
        case 0xB9:
        { /* CPDR */
            cpd();
            if ((sz5h3pnFlags & PARITY_MASK) == PARITY_MASK && (sz5h3pnFlags & ZERO_MASK) == 0) {
                REG_PC -= 2;
                REG_WZ = REG_PC + 1;
                Z80Ops::addressOnBus(REG_HL + 1, 5);
                sz5h3pnFlags &= ~FLAG_53_MASK;
                sz5h3pnFlags |= (REG_PCh & FLAG_53_MASK);
            }
            break;
        }
        case 0xBA:
        { /* INDR */
            ind();
            if (REG_B != 0) {
                REG_PC -= 2;
                REG_WZ = REG_PC + 1;
                Z80Ops::addressOnBus(REG_HL + 1, 5);
                SetAbortedINxR_OTxRFlags();
            }
            break;
        }
        case 0xBB:
        { /* OTDR */
            outd();
            if (REG_B != 0) {
                REG_PC -= 2;
                REG_WZ = REG_PC + 1;
                Z80Ops::addressOnBus(REG_BC, 5);
                SetAbortedINxR_OTxRFlags();
            }
            break;
        }
    }
}

IRAM_ATTR void Z80::copyToRegister(uint8_t value)
{
    uint8_t dummy8;
    uint8_t* const regs[8] = {&REG_B, &REG_C, &REG_D, &REG_E, &REG_H, &REG_L, &dummy8, &regA };
    *regs[opCode & 0x07] = value;
}

void Z80::SetAbortedINxR_OTxRFlags() {

    sz5h3pnFlags &= ~FLAG_53_MASK;
    sz5h3pnFlags |= (REG_PCh & FLAG_53_MASK);

    uint8_t cpyB = REG_B;
    if (carryFlag) {
        cpyB += sz5h3pnFlags & ADDSUB_MASK ? -1 : 1;
        sz5h3pnFlags = (cpyB ^ REG_B) & HALFCARRY_MASK ? sz5h3pnFlags | HALFCARRY_MASK : sz5h3pnFlags & ~HALFCARRY_MASK;
    }
    uint8_t pf = (sz5h3pnFlags & PARITY_MASK) ^ sz53pn_addTable[cpyB & 0x07] ^ PARITY_MASK;
    sz5h3pnFlags = pf & PARITY_MASK ? sz5h3pnFlags | PARITY_MASK : sz5h3pnFlags & ~PARITY_MASK;

}
