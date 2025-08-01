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


#include "cpuESP.h"
#include "ESPeccy.h"
#include "MemESP.h"
#include "Ports.h"
#include "Config.h"
#include "Video.h"
#include "Z80_JLS/z80.h"

// #pragma GCC optimize("O3")

uint32_t CPU::tstates = 0;

int32_t CPU::prev_tstates = 0;
uint32_t CPU::tstates_diff = 0;

uint64_t CPU::global_tstates = 0;
uint32_t CPU::statesInFrame = 0;
uint8_t CPU::latetiming = 0;
uint8_t CPU::IntStart = 0;
uint8_t CPU::IntEnd = 0;
uint32_t CPU::stFrame = 0;

uint8_t (*Z80Ops::fetchOpcode)() = &Z80Ops::fetchOpcode_std;
uint8_t (*Z80Ops::peek8)(uint16_t address) = &Z80Ops::peek8_std;
void (*Z80Ops::poke8)(uint16_t address, uint8_t value) = &Z80Ops::poke8_std;
uint16_t (*Z80Ops::peek16)(uint16_t address) = &Z80Ops::peek16_std;
void (*Z80Ops::poke16)(uint16_t address, RegisterPair word) = &Z80Ops::poke16_std;
void (*Z80Ops::addressOnBus)(uint16_t address, int32_t wstates) = &Z80Ops::addressOnBus_std;

bool Z80Ops::is48;
bool Z80Ops::is128;
bool Z80Ops::isPentagon;
bool Z80Ops::is2a3;

///////////////////////////////////////////////////////////////////////////////

void CPU::reset() {

    Z80::reset();

    CPU::latetiming = Config::AluTiming;

    if (Config::arch == "48K") {
        Ports::getFloatBusData = &Ports::getFloatBusData48;
        Z80Ops::is48 = true;
        Z80Ops::is128 = false;
        Z80Ops::isPentagon = false;
        Z80Ops::is2a3 = false;
        statesInFrame = TSTATES_PER_FRAME_48;
        IntStart = INT_START48;
        IntEnd = INT_END48 + CPU::latetiming;
        ESPeccy::target[0] = MICROS_PER_FRAME_48;
        ESPeccy::target[1] = MICROS_PER_FRAME_48;
        ESPeccy::target[2] = MICROS_PER_FRAME_48_125SPEED;
        ESPeccy::target[3] = MICROS_PER_FRAME_48_150SPEED;

    } else if (Config::arch == "TK90X" || Config::arch == "TK95") {

        Z80Ops::is48 = true;
        Z80Ops::is128 = false;
        Z80Ops::isPentagon = false;
        Z80Ops::is2a3 = false;

        switch (Config::ALUTK) {
        case 0:
            Ports::getFloatBusData = &Ports::getFloatBusData48;
            statesInFrame = TSTATES_PER_FRAME_48;
            ESPeccy::target[0] = MICROS_PER_FRAME_48;
            ESPeccy::target[1] = MICROS_PER_FRAME_48;
            ESPeccy::target[2] = MICROS_PER_FRAME_48_125SPEED;
            ESPeccy::target[3] = MICROS_PER_FRAME_48_150SPEED;
            break;
        case 1:
            Ports::getFloatBusData = &Ports::getFloatBusDataTK;
            statesInFrame = TSTATES_PER_FRAME_TK_50;
            ESPeccy::target[0] = MICROS_PER_FRAME_TK_50;
            ESPeccy::target[1] = MICROS_PER_FRAME_TK_50;
            ESPeccy::target[2] = MICROS_PER_FRAME_TK_50_125SPEED;
            ESPeccy::target[3] = MICROS_PER_FRAME_TK_50_150SPEED;
            break;
        case 2:
            Ports::getFloatBusData = &Ports::getFloatBusDataTK;
            statesInFrame = TSTATES_PER_FRAME_TK_60;
            ESPeccy::target[0] = MICROS_PER_FRAME_TK_60;
            ESPeccy::target[1] = MICROS_PER_FRAME_TK_60;
            ESPeccy::target[2] = MICROS_PER_FRAME_TK_60_125SPEED;
            ESPeccy::target[3] = MICROS_PER_FRAME_TK_60_150SPEED;
        }

        IntStart = INT_STARTTK;
        IntEnd = INT_ENDTK + CPU::latetiming;

    } else if (Config::arch == "128K") {
        Ports::getFloatBusData = &Ports::getFloatBusData128;
        Z80Ops::is48 = false;
        Z80Ops::is128 = true;
        Z80Ops::isPentagon = false;
        Z80Ops::is2a3 = false;
        statesInFrame = TSTATES_PER_FRAME_128;
        IntStart = INT_START128;
        IntEnd = INT_END128 + CPU::latetiming;
        ESPeccy::target[0] = MICROS_PER_FRAME_128;
        ESPeccy::target[1] = MICROS_PER_FRAME_128;
        ESPeccy::target[2] = MICROS_PER_FRAME_128_125SPEED;
        ESPeccy::target[3] = MICROS_PER_FRAME_128_150SPEED;
    } else if (Config::arch == "+2A" || Config::arch=="+3") {
        Ports::getFloatBusData = &Ports::getFloatBusData2A3;
        Z80Ops::is48 = false;
        Z80Ops::is128 = false;
        Z80Ops::isPentagon = false;
        Z80Ops::is2a3 = true;
        statesInFrame = TSTATES_PER_FRAME_128;
        IntStart = INT_STARTPLUS2A3;
        IntEnd = INT_ENDPLUS2A3 + CPU::latetiming;
        ESPeccy::target[0] = MICROS_PER_FRAME_128;
        ESPeccy::target[1] = MICROS_PER_FRAME_128;
        ESPeccy::target[2] = MICROS_PER_FRAME_128_125SPEED;
        ESPeccy::target[3] = MICROS_PER_FRAME_128_150SPEED;
    } else if (Config::arch == "Pentagon") {
        Ports::getFloatBusData = &Ports::getFloatBusDataPentagon;
        Z80Ops::is48 = false;
        Z80Ops::is128 = false;
        Z80Ops::isPentagon = true;
        Z80Ops::is2a3 = false;
        statesInFrame = TSTATES_PER_FRAME_PENTAGON;
        IntStart = INT_START_PENTAGON;
        IntEnd = INT_END_PENTAGON + CPU::latetiming;
        ESPeccy::target[0] = MICROS_PER_FRAME_PENTAGON;
        ESPeccy::target[1] = MICROS_PER_FRAME_PENTAGON;
        ESPeccy::target[2] = MICROS_PER_FRAME_PENTAGON_125SPEED;
        ESPeccy::target[3] = MICROS_PER_FRAME_PENTAGON_150SPEED;
    }

    if (Config::arch == "+2A" || Config::arch=="+3") {
        Z80Ops::fetchOpcode = &Z80Ops::fetchOpcode_2A3;
        Z80Ops::peek8  = &Z80Ops::peek8_2A3;
        Z80Ops::poke8 = &Z80Ops::poke8_2A3;
        Z80Ops::peek16 = &Z80Ops::peek16_2A3;
        Z80Ops::poke16 = &Z80Ops::poke16_2A3;
        Z80Ops::addressOnBus = &Z80Ops::addressOnBus_2A3;
    } else {
        Z80Ops::fetchOpcode = &Z80Ops::fetchOpcode_std;
        Z80Ops::peek8  = &Z80Ops::peek8_std;
        Z80Ops::poke8 = &Z80Ops::poke8_std;
        Z80Ops::peek16 = &Z80Ops::peek16_std;
        Z80Ops::poke16 = &Z80Ops::poke16_std;
        Z80Ops::addressOnBus = &Z80Ops::addressOnBus_std;
    }

    stFrame = statesInFrame - IntEnd;

    tstates = 0;
    global_tstates = 0;

    prev_tstates = 0;
    tstates_diff = 0;

}

///////////////////////////////////////////////////////////////////////////////

IRAM_ATTR void CPU::loop() {

    // Reset audio buffer vars
    ESPeccy::audbufcnt = 0;
    ESPeccy::audbufcntover = 0;
    ESPeccy::audioBitBuf = 0;
    ESPeccy::audioBitbufCount = 0;
    ESPeccy::audbufcntAY = 0;
    ESPeccy::audbufcntCOVOX = 0;

    // Check NMI
    if (Z80::isNMI()) {
        Z80::execute();
        Z80::doNMI();
    }

    while (tstates < IntEnd) Z80::execute();

    if (!Z80::isHalted()) {
        stFrame = statesInFrame - IntEnd;
        if (Z80Ops::is2a3)
            Z80::exec_nocheck_2A3();
        else
        Z80::exec_nocheck();
        if (stFrame == 0) FlushOnHalt();
    } else {
        FlushOnHalt();
    }

    while (tstates < statesInFrame) Z80::execute();

    VIDEO::EndFrame();

    // // FDD calcs
    // CPU::tstates_diff += CPU::tstates - CPU::prev_tstates;

    // if ((ESPeccy::fdd.control & (kRVMWD177XHLD | kRVMWD177XHLT)) != 0) {
    //     rvmWD1793Step(&ESPeccy::fdd, CPU::tstates_diff / WD177XSTEPSTATES); // FDD
    // }

    // CPU::tstates_diff = CPU::tstates_diff % WD177XSTEPSTATES;

    global_tstates += statesInFrame; // increase global Tstates
    tstates -= statesInFrame;

    // CPU::prev_tstates = tstates;

}

///////////////////////////////////////////////////////////////////////////////

IRAM_ATTR void CPU::FlushOnHalt() {

    uint32_t stEnd = statesInFrame - IntEnd;

    uint8_t page = Z80::getRegPC() >> 14;
    if (MemESP::ramContended[page]) {

        while (tstates < stEnd ) {
            VIDEO::Draw_Opcode(true);
            Z80::incRegR(1);
        }

    } else {

        if (VIDEO::snow_toggle) {

            // ULA perfect cycle & snow effect use this code
            while (tstates < stEnd ) {
                VIDEO::Draw_Opcode(false);
                Z80::incRegR(1);
            }

        } else {

            // Flush the rest of frame
            uint32_t pre_tstates = tstates;
            while (VIDEO::Draw != &VIDEO::Blank)
                VIDEO::Draw(VIDEO::tStatesPerLine, false);
            tstates = pre_tstates;

            uint32_t incr = (stEnd - pre_tstates) >> 2;
            if (pre_tstates & 0x03) incr++;
            tstates += (incr << 2);
            Z80::incRegR(incr & 0x000000FF);

        }

    }

}

///////////////////////////////////////////////////////////////////////////////
// Z80Ops
///////////////////////////////////////////////////////////////////////////////

// Fetch opcode from RAM (+2A/3 version)
IRAM_ATTR uint8_t Z80Ops::fetchOpcode_2A3() {
    const uint16_t pc = Z80::getRegPC();
    const uint8_t pg = pc >> 14;
    const uint8_t data = MemESP::ramCurrent[pg][pc & 0x3fff];
    if (MemESP::ramContended[pg]) {
        MemESP::lastContendedMemReadWrite = data;
        VIDEO::Draw_Opcode(true);
    } else {
        VIDEO::Draw_Opcode(false);
    };

    return data;
}

// Fetch opcode from RAM (NON +2A/3 version)
IRAM_ATTR uint8_t Z80Ops::fetchOpcode_std() {
    const uint16_t pc = Z80::getRegPC();
    const uint8_t pg = pc >> 14;
    VIDEO::Draw_Opcode(MemESP::ramContended[pg]);
    return MemESP::ramCurrent[pg][pc & 0x3fff];
}

// Read byte from RAM (+2A/+3 version)
IRAM_ATTR uint8_t Z80Ops::peek8_2A3(uint16_t address) {
    const uint8_t page = address >> 14;
    const uint8_t data = MemESP::ramCurrent[page][address & 0x3fff];
    if (MemESP::ramContended[page]) {
        MemESP::lastContendedMemReadWrite = data;
        VIDEO::Draw(3,true);
    } else {
        VIDEO::Draw(3,false);
    }
    return data;
}

// Read byte from RAM (non +2A/+3 version)
IRAM_ATTR uint8_t Z80Ops::peek8_std(uint16_t address) {
    const uint8_t page = address >> 14;
    VIDEO::Draw(3,MemESP::ramContended[page]);
    return MemESP::ramCurrent[page][address & 0x3fff];
}

// Write byte to RAM (+2A/+3 version)
IRAM_ATTR void Z80Ops::poke8_2A3(uint16_t address, uint8_t value) {
    const uint8_t page = address >> 14;

    if (page == MemESP::pagingmode2A3) {
        VIDEO::Draw(3, false);
        return;
    }

    if (MemESP::ramContended[page]) {
        MemESP::lastContendedMemReadWrite = value;
        VIDEO::Draw(3, true);
    } else {
        VIDEO::Draw(3, false);
    }

    MemESP::ramCurrent[page][address & 0x3fff] = value;

}

// Write byte to RAM (non +2A/+3 version)
IRAM_ATTR void Z80Ops::poke8_std(uint16_t address, uint8_t value) {
    const uint8_t page = address >> 14;

    if (page == 0) {
        VIDEO::Draw(3, false);
        return;
    }

    VIDEO::Draw(3, MemESP::ramContended[page]);
    MemESP::ramCurrent[page][address & 0x3fff] = value;

}

// Read word from RAM (+2A/+3 version)
IRAM_ATTR uint16_t Z80Ops::peek16_2A3(uint16_t address) {
    const uint8_t page = address >> 14;

    if (page == ((address + 1) >> 14)) {    // Check if address is between two different pages
        const uint8_t msb = MemESP::ramCurrent[page][(address & 0x3fff) + 1];

        if (MemESP::ramContended[page]) {
            MemESP::lastContendedMemReadWrite = msb;
            VIDEO::Draw(3, true);
            VIDEO::Draw(3, true);
        } else
            VIDEO::Draw(6, false);

        return (msb << 8) | MemESP::ramCurrent[page][address & 0x3fff];

    } else {
        // Order matters, first read lsb, then read msb, don't "optimize"
        const uint8_t lsb = Z80Ops::peek8(address);
        const uint8_t msb = Z80Ops::peek8(address + 1);
        return (msb << 8) | lsb;

    }

}

// Read word from RAM (non +2A/+3 version)
IRAM_ATTR uint16_t Z80Ops::peek16_std(uint16_t address) {
    const uint8_t page = address >> 14;

    if (page == ((address + 1) >> 14)) {    // Check if address is between two different pages

        if (MemESP::ramContended[page]) {
            VIDEO::Draw(3, true);
            VIDEO::Draw(3, true);
        } else
            VIDEO::Draw(6, false);

        return ((MemESP::ramCurrent[page][(address & 0x3fff) + 1] << 8) | MemESP::ramCurrent[page][address & 0x3fff]);

    } else {
        // Order matters, first read lsb, then read msb, don't "optimize"
        const uint8_t lsb = Z80Ops::peek8(address);
        const uint8_t msb = Z80Ops::peek8(address + 1);
        return (msb << 8) | lsb;

    }

}

// Write word to RAM (+2A/+3 version)
IRAM_ATTR void Z80Ops::poke16_2A3(uint16_t address, RegisterPair word) {
    const uint8_t page = address >> 14;
    const uint16_t page_addr = address & 0x3fff;

    if (page_addr < 0x3fff) {    // Check if address is between two different pages

        if (page == MemESP::pagingmode2A3) {
            VIDEO::Draw(6, false);
            return;
        }

        if (MemESP::ramContended[page]) {
            MemESP::lastContendedMemReadWrite = word.byte8.hi;
            VIDEO::Draw(3, true);
            VIDEO::Draw(3, true);
        } else
            VIDEO::Draw(6, false);

        MemESP::ramCurrent[page][page_addr] = word.byte8.lo;
        MemESP::ramCurrent[page][page_addr + 1] = word.byte8.hi;

    } else {

        // Order matters, first write lsb, then write msb, don't "optimize"
        Z80Ops::poke8(address, word.byte8.lo);
        Z80Ops::poke8(address + 1, word.byte8.hi);

    }

}

// Write word to RAM (non +2A/+3 version)
IRAM_ATTR void Z80Ops::poke16_std(uint16_t address, RegisterPair word) {
    const uint8_t page = address >> 14;
    const uint16_t page_addr = address & 0x3fff;

    if (page_addr < 0x3fff) {    // Check if address is between two different pages

        if (page == 0) {
            VIDEO::Draw(6, false);
            return;
        }

        if (MemESP::ramContended[page]) {
            VIDEO::Draw(3, true);
            VIDEO::Draw(3, true);
        } else
            VIDEO::Draw(6, false);

        MemESP::ramCurrent[page][page_addr] = word.byte8.lo;
        MemESP::ramCurrent[page][page_addr + 1] = word.byte8.hi;

    } else {

        // Order matters, first write lsb, then write msb, don't "optimize"
        Z80Ops::poke8(address, word.byte8.lo);
        Z80Ops::poke8(address + 1, word.byte8.hi);

    }

}

// Put an address on bus lasting 'tstates' cycles
IRAM_ATTR void Z80Ops::addressOnBus_std(uint16_t address, int32_t wstates) {
    if (MemESP::ramContended[address >> 14]) {
        for (int i = 0; i < wstates; i++)
            VIDEO::Draw(1, true);
    } else
        VIDEO::Draw(wstates, false);
}

// Put an address on bus lasting 'tstates' cycles
IRAM_ATTR void Z80Ops::addressOnBus_2A3(uint16_t address, int32_t wstates) {
        VIDEO::Draw(wstates, false);
}

// Callback to know when the INT signal is active
IRAM_ATTR bool Z80Ops::isActiveINT(void) {
    int tmp = CPU::tstates + CPU::latetiming;
    if (tmp >= CPU::statesInFrame) tmp -= CPU::statesInFrame;
    return ((tmp >= CPU::IntStart) && (tmp < CPU::IntEnd));
}

