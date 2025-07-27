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


#include "Ports.h"
#include "Config.h"
#include "ESPeccy.h"
#include "Z80_JLS/z80.h"
#include "MemESP.h"
#include "Video.h"
#include "AySound.h"
#include "Tape.h"
#include "cpuESP.h"
#include "wd1793.h"

#include "RealTape.h"

// #pragma GCC optimize("O3")

// Values calculated for BEEPER, EAR, MIC bit mask (values 0-7)
// Taken from FPGA values suggested by Rampa
//   0: ula <= 8'h00;
//   1: ula <= 8'h24;
//   2: ula <= 8'h40;
//   3: ula <= 8'h64;
//   4: ula <= 8'hB8;
//   5: ula <= 8'hC0;
//   6: ula <= 8'hF8;
//   7: ula <= 8'hFF;
// and adjusted for BEEPER_MAX_VOLUME = 97

#if 0
    const uint8_t Ports::speaker_values[8]={ 0, 19, 34, 53, 97, 101, 130, 134 };
#else
    #define SPK_VAL         0xA0 // 0xB0
    #define MIC_VAL         0x50 // 0x40
    #define EARBIT_VAL      0x0F

    const uint8_t Ports::speaker_values[8]={
                  MIC_VAL             , // 000
                  MIC_VAL + EARBIT_VAL, // 001
        0                             , // 010
                            EARBIT_VAL, // 011
        SPK_VAL + MIC_VAL             , // 100
        SPK_VAL + MIC_VAL + EARBIT_VAL, // 101
        SPK_VAL                       , // 110
        SPK_VAL           + EARBIT_VAL  // 111
    };
#endif

uint8_t Ports::port[128];
uint8_t Ports::port254 = 0;
uint8_t Ports::LastOutTo1FFD = 0;
uint8_t Ports::LastOutTo7FFD = 0;

int16_t Ports::in254_count = 0;
bool Ports::loading = false;

uint8_t (*Ports::getFloatBusData)() = &Ports::getFloatBusData48;

uint8_t Ports::getFloatBusData48() {

    uint32_t line = CPU::tstates / 224;
    if (line < 64 || line >= 256) return 0xFF;

    uint8_t halfpix = CPU::tstates % 224;
    if (halfpix & 0x80) return 0xFF;
    halfpix -= 3;
    if (halfpix & 0x04) return 0xFF;

    line -= 64;
    return (VIDEO::grmem[(halfpix & 0x01 ? VIDEO::offAtt[line] : VIDEO::offBmp[line]) + (halfpix >> 2) + ((halfpix >> 1) & 0x01)]);

}

uint8_t Ports::getFloatBusDataTK() {

    unsigned int currentTstates = CPU::tstates;

    unsigned int line = (currentTstates / 228) - (Config::ALUTK == 1 ? 64 : 38);
    if (line >= 192) return 0xFF;

    unsigned char halfpix = (currentTstates % 228) - 99;
    if ((halfpix >= 125) || (halfpix & 0x04)) return 0xFF;

    return (VIDEO::grmem[(halfpix & 0x01 ? VIDEO::offAtt[line] : VIDEO::offBmp[line]) + (halfpix >> 2) + ((halfpix >> 1) & 0x01)]);

}

uint8_t Ports::getFloatBusData128() {

    uint32_t currentTstates = CPU::tstates - 1;

    uint32_t line = currentTstates / 228;
    if (line < 63 || line >= 255) return 0xFF;

    uint8_t halfpix = currentTstates % 228;
    if (halfpix & 0x84) return 0xFF;

    line -= 63;
    return (VIDEO::grmem[(halfpix & 0x01 ? VIDEO::offAtt[line] : VIDEO::offBmp[line]) + (halfpix >> 2) + ((halfpix >> 1) & 0x01)]);

}

uint8_t Ports::getFloatBusDataPentagon() {

	uint32_t line = CPU::tstates / 224;
    if (line < 80 || line >= 272) return 0xFF;

	uint8_t halfpix = (CPU::tstates % 224) - 63;
    if (halfpix & 0x80) return 0xFF;
    halfpix -= 3;
    if (halfpix & 0x04) return 0xFF;

    line -= 80;
    return (VIDEO::grmem[(halfpix & 0x01 ? VIDEO::offAtt[line] : VIDEO::offBmp[line]) + (halfpix >> 2) + ((halfpix >> 1) & 0x01)]);

}

uint8_t Ports::getFloatBusData2A3() {

    uint32_t currentTstates = CPU::tstates - 4;

	uint32_t line = currentTstates / 228;
    if (line < 63 || line >= 255) return MemESP::lastContendedMemReadWrite | 1;

	uint8_t halfpix = currentTstates % 228;

	// Test/Ask if this is more correct because contention lasts 129 t-states in +2A/+3
    // if (halfpix > 128) return MemESP::lastContendedMemReadWrite | 1;
    // if (halfpix & 0x04) return MemESP::lastContendedMemReadWrite | 1;

	if (halfpix & 0x84) return MemESP::lastContendedMemReadWrite | 1;

    line -= 63;
    return (VIDEO::grmem[(halfpix & 0x01 ? VIDEO::offAtt[line] : VIDEO::offBmp[line]) + (halfpix >> 2) + ((halfpix >> 1) & 0x01)]) | 1;

}

static uint32_t p_states;

const uint8_t contention2[8] = {6, 6, 5, 4, 3, 2, 1, 0};
const uint8_t contention3[129] = {
    6,  6,  12, 12, 11, 10, 9,  8,  7,  6,  12, 12, 11, 10, 9,  8,  7,  6,  12,
    12, 11, 10, 9,  8,  7,  6,  12, 12, 11, 10, 9,  8,  7,  6,  12, 12, 11, 10,
    9,  8,  7,  6,  12, 12, 11, 10, 9,  8,  7,  6,  12, 12, 11, 10, 9,  8,  7,
    6,  12, 12, 11, 10, 9,  8,  7,  6,  12, 12, 11, 10, 9,  8,  7,  6,  12, 12,
    11, 10, 9,  8,  7,  6,  12, 12, 11, 10, 9,  8,  7,  6,  12, 12, 11, 10, 9,
    8,  7,  6,  12, 12, 11, 10, 9,  8,  7,  6,  12, 12, 11, 10, 9,  8,  7,  6,
    12, 12, 11, 10, 9,  8,  7,  6,  6,  6,  5,  4,  3,  2,  1};

uint8_t tkIOcon(uint16_t a) {

    uint32_t t = (CPU::tstates % 228) - 93;
	uint32_t l = (CPU::tstates / 228) - (Config::ALUTK == 1 ? 64 : 38);

    if (t >= 228) {
        t -= 228;
        ++l;
        if (l >= (Config::ALUTK == 1 ? 312 : 262)) l = 0;
    }

    if (l < 192) {
        if ((a & 0xc000) == 0x4000) {
            if (t < 129) {
                return contention3[t];
            }
        } else {
            if (a & 0x1)
                return 0;
            else if (t < 128) {
                return contention2[t & 07];
            }
        }
    }

    return 0;

}

IRAM_ATTR uint8_t Ports::input(uint16_t address) {

    uint8_t data;
    uint8_t rambank = address >> 14;
    uint8_t address8 = address & 0xff;
    p_states = CPU::tstates;

    VIDEO::Draw(1, (Z80Ops::is48 || Z80Ops::is128) && MemESP::ramContended[rambank]); // I/O Contention (Early)

    // ULA PORT
    if ((address & 0x0001) == 0) {

        ++in254_count;

        if (Config::arch[0] == 'T' && Config::ALUTK > 0) {
            VIDEO::Draw(3 + tkIOcon(address), false);
        } else {
            VIDEO::Draw(3, Z80Ops::is48 || Z80Ops::is128);   // I/O Contention (Late)
        }

        data = Config::port254default; // For TK90X spanish and rest of machines default port value is 0xBF. For TK90X portuguese is 0x3f.

        uint8_t portHigh = ~(address >> 8) & 0xff;
        for (int row = 0, mask = 0x01; row < 8; row++, mask <<= 1) {
            if ((portHigh & mask) != 0)
                data &= port[row];
        }

        if (RealTape_enabled && (Config::realtape_mode == REALTAPE_FORCE_LOAD || Tape::tapeFileType == TAPE_FTYPE_EMPTY)) {
            Tape::tapeEarBit = RealTape_getLevel();
        }
        else
        if (Tape::tapeStatus==TAPE_LOADING) {
            Tape::Read();
        }

        if (Z80Ops::is48 && Config::Issue2) {// Issue 2 behaviour only on Spectrum 48K
            if (port254 & 0x18) bitWrite(data, 6, 1);
        } else if (!Z80Ops::is2a3) {
            if (port254 & 0x10) bitWrite(data, 6, 1);
        }
        if (Tape::tapeEarBit) data ^= 0x40;

    } else {

        if (Config::arch[0] == 'T' && Config::ALUTK > 0)
            VIDEO::Draw(3 + tkIOcon(address), false);
        else
            ioContentionLate((Z80Ops::is48 || Z80Ops::is128) && MemESP::ramContended[rambank]);

        // The default port value is 0xFF.
        data = 0xff;

        // Check if TRDOS Rom is mapped.
        if (ESPeccy::trdos) {

            switch (address8 & 0xe3) {
                case 0x03:
                    data = ESPeccy::Betadisk.ReadStatusReg();
                    // printf("WD1793 Read Status Register: %d\n",(int)data);
                    return data;
                case 0x23:
                    data = ESPeccy::Betadisk.ReadTrackReg();
                    // printf("WD1793 Read Track Register: %d\n",(int)data);
                    return data;
                case 0x43:
                    data = ESPeccy::Betadisk.ReadSectorReg();
                    // printf("WD1793 Read Sector Register: %d\n",(int)data);
                    return data;
                case 0x63:
                    data = ESPeccy::Betadisk.ReadDataReg();
                    // printf("WD1793 Read Data Register: %d\n",(int)data);
                    return data;
                case 0xe3:
                    data = ESPeccy::Betadisk.ReadSystemReg();
                    // printf("WD1793 Read Control Register: %d\n",(int)data);
                    return data;
            }
        }

        if (ESPeccy::ps2mouse) {

            if((address & 0x05ff) == 0x01df) {

                MouseDelta delta;

                while (ESPeccy::PS2Controller.mouse()->deltaAvailable()) {

                    if (ESPeccy::PS2Controller.mouse()->getNextDelta(&delta)) {

                        ESPeccy::mouseX = (ESPeccy::mouseX + delta.deltaX) & 0xff;
                        ESPeccy::mouseY = (ESPeccy::mouseY + delta.deltaY) & 0xff;
                        ESPeccy::mouseButtonL = delta.buttons.left;
                        ESPeccy::mouseButtonR = delta.buttons.right;

                    } else break;

                }

                return (uint8_t) ESPeccy::mouseX;
            }

            if((address & 0x05ff) == 0x05df) {

                MouseDelta delta;

                while (ESPeccy::PS2Controller.mouse()->deltaAvailable()) {

                    if (ESPeccy::PS2Controller.mouse()->getNextDelta(&delta)) {

                        ESPeccy::mouseX = (ESPeccy::mouseX + delta.deltaX) & 0xff;
                        ESPeccy::mouseY = (ESPeccy::mouseY + delta.deltaY) & 0xff;
                        ESPeccy::mouseButtonL = delta.buttons.left;
                        ESPeccy::mouseButtonR = delta.buttons.right;

                    } else break;

                }

                return (uint8_t) ESPeccy::mouseY;
            }

            if((address & 0x05ff) == 0x00df) {

                MouseDelta delta;

                while (ESPeccy::PS2Controller.mouse()->deltaAvailable()) {

                    if (ESPeccy::PS2Controller.mouse()->getNextDelta(&delta)) {

                        ESPeccy::mouseX = (ESPeccy::mouseX + delta.deltaX) & 0xff;
                        ESPeccy::mouseY = (ESPeccy::mouseY + delta.deltaY) & 0xff;
                        ESPeccy::mouseButtonL = delta.buttons.left;
                        ESPeccy::mouseButtonR = delta.buttons.right;

                    } else break;

                }

                return 0xff & (ESPeccy::mouseButtonL ? 0xfd : 0xff) & (ESPeccy::mouseButtonR ? 0xfe : 0xff);
            }
        }

        // Kempston Joystick
        if ((Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) && ((address & 0x00E0) == 0 || address8 == 0xDF)) return port[0x1f];

        // Fuller Joystick
        if (!ESPeccy::trdos && (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) && address8 == 0x7F) return port[0x7f];


        // Sound (AY-3-8912)
        if (ESPeccy::AY_emu) {

            // ZX81
            if (Config::romSet == "ZX81+") {
                if (address8 == 0xcf || address8 == 0xdf) {
                    return AySound::getRegisterData();
                }
            }

            // Fullerbox
            if (!ESPeccy::trdos && address8 == 0x3f) {
                return AySound::getRegisterData();
            }

            /* On the 128K/+2, reading from BFFDh will return the floating bus
             * value as normal for unattached ports, but on the +2A/+3, it will
             * return the same as reading from FFFDh */

            if ((address & 0xC002) == 0xC000)
                return AySound::getRegisterData();

            if (Z80Ops::is2a3 && (address & 0xC002) == 0x8000) {
                return AySound::getRegisterData();
            }
        }

        if (Z80Ops::is2a3) {

            // If we are on a +2A/+3 with memory paging disabled, or the port address
            // is not following the pattern 0000 xxxx xxxx xx0xb, return 0xFF.
            if (MemESP::pagingLock || (address & 4093) != address) {
                data = 0xff;
            } else {
                data = getFloatBusData2A3();
            }

        } else {
            data = getFloatBusData();
        }

        if (Z80Ops::is128 && (address & 0x8002) == 0) {

            // //  Solo en el modelo 128K, pero no en los +2/+2A/+3, si se lee el puerto
            // //  0x7ffd, el valor leído es reescrito en el puerto 0x7ffd.
            // //  http://www.speccy.org/foro/viewtopic.php?f=8&t=2374
            if (!MemESP::pagingLock) {

                MemESP::pagingLock = bitRead(data, 5);

                if (MemESP::bankLatch != (data & 0x7)) {
                    MemESP::bankLatch = data & 0x7;
                    MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];
                    MemESP::ramContended[3] = MemESP::bankLatch & 0x01 ? true: false;
                }

                if (MemESP::videoLatch != bitRead(data, 3)) {
                    MemESP::videoLatch = bitRead(data, 3);
                    VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7] : MemESP::ram[5];
                }

                MemESP::romLatch = bitRead(data, 4);
                bitWrite(MemESP::romInUse, 0, MemESP::romLatch);
                MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
            }

        }

    }

    return data;

}

IRAM_ATTR void Ports::output(uint16_t address, uint8_t data) {

    int Audiobit;
    uint8_t rambank = address >> 14;
    uint8_t address8 = address & 0xff;

    p_states = CPU::tstates;

    VIDEO::Draw(1, (Z80Ops::is48 || Z80Ops::is128) && MemESP::ramContended[rambank]); // I/O Contention (Early)

    // ULA =======================================================================
    if ((address & 0x0001) == 0) {

        port254 = data;

        // Border color
        if (VIDEO::borderColor != data & 0x07) {

            VIDEO::brdChange = true;

            if (!Z80Ops::isPentagon && !Z80Ops::is2a3)
                if (Config::arch[0] == 'T' && Config::ALUTK > 0)
                    VIDEO::Draw(tkIOcon(address),false);
                else
                    VIDEO::Draw(0,true); // Seems not needed in Pentagon and +2A/+3

            VIDEO::DrawBorder();

            VIDEO::borderColor = data & 0x07;
            VIDEO::brd = VIDEO::border32[VIDEO::borderColor];

        }

        if (ESPeccy::ESP_delay) { // Disable beeper on turbo mode

            if (Config::load_monitor) {
                Audiobit = Tape::tapeEarBit ? 255 : 0; // For load tape monitor
            }
            else
            {
                // Beeper Audio
#if 0
                Audiobit = speaker_values[((data >> 2) & 0x04) | (Tape::tapeEarBit << 1) | ((data >> 3) & 0x01)];
#else
                Audiobit = speaker_values[((data >> 2) & 0x06) | (Tape::tapeEarBit & 0x1)]; // SPK - MIC - EAR MONITOR
#endif
            }

            if (Audiobit != ESPeccy::lastaudioBit) {
                ESPeccy::BeeperGetSample();
                ESPeccy::lastaudioBit = Audiobit;
            }

        }

        // ZX81
        if (ESPeccy::AY_emu && Config::romSet == "ZX81+") {
            if (address8 == 0xcf || address8 == 0xdf || address8 == 0x1f || address8 == 0x0f) {
                if (address8 & 0x80) {  // Latch register
                    AySound::selectedRegister = data & 0x0f;
                } else { // Write data
                    ESPeccy::AYGetSample();
                    AySound::setRegisterData(data);
                }
                VIDEO::Draw(3, true);   // I/O Contention (Late)
                return;
            }
        }

        // FullerBox
        if (!ESPeccy::trdos && ESPeccy::AY_emu && (address8 == 0x3f || address8 == 0x5f)) {
            if (address8 == 0x3f) {
                AySound::selectedRegister = data & 0x0f;
            } else if (address8 == 0x5f) {
                ESPeccy::AYGetSample();
                AySound::setRegisterData(data);
            }

            if (Config::arch[0] == 'T' && Config::ALUTK > 0)
                VIDEO::Draw(3 + tkIOcon(address), false);
            else
                VIDEO::Draw(3, true);   // I/O Contention (Late)

            return;
        }

        // AY ========================================================================
        if ((ESPeccy::AY_emu) && ((address & 0x8002) == 0x8000)) {

            if ((address & 0x4000) != 0)
                AySound::selectedRegister = data & 0x0f;
            else {
                ESPeccy::AYGetSample();
                AySound::setRegisterData(data);
            }

            if (Config::arch[0] == 'T' && Config::ALUTK > 0)
                VIDEO::Draw(3 + tkIOcon(address), false);
            else
                VIDEO::Draw(3, Z80Ops::is48 || Z80Ops::is128);   // I/O Contention (Late)

            return;

        }

        if (Config::arch[0] == 'T' && Config::ALUTK > 0)
            VIDEO::Draw(3 + tkIOcon(address), false);
        else
            VIDEO::Draw(3, Z80Ops::is48 || Z80Ops::is128);   // I/O Contention (Late)

    } else {

        // ZX81
        if (ESPeccy::AY_emu && Config::romSet == "ZX81+") {
            if (address8 == 0xcf || address8 == 0xdf || address8 == 0x1f || address8 == 0x0f) {
                if (address8 & 0x80) {  // Latch register
                    AySound::selectedRegister = data & 0x0f;
                } else { // Write data
                    ESPeccy::AYGetSample();
                    AySound::setRegisterData(data);
                }
                ioContentionLate(MemESP::ramContended[rambank]);
                return;
            }
        }

        // FullerBox
        if (!ESPeccy::trdos && ESPeccy::AY_emu && (address8 == 0x3f || address8 == 0x5f)) {
            if (address8 == 0x3f) {
                AySound::selectedRegister = data & 0x0f;
            } else if (address8 == 0x5f) {
                ESPeccy::AYGetSample();
                AySound::setRegisterData(data);
            }

            if (Config::arch[0] == 'T' && Config::ALUTK > 0)
                VIDEO::Draw(3 + tkIOcon(address), false);
            else
                ioContentionLate(MemESP::ramContended[rambank]);

            return;
        }

        // AY ========================================================================
        if ((ESPeccy::AY_emu) && ((address & 0x8002) == 0x8000)) {

            if ((address & 0x4000) != 0)
                AySound::selectedRegister = data & 0x0f;
            else {
                ESPeccy::AYGetSample();
                AySound::setRegisterData(data);
            }

            goto lateIOContention;

        }

        // Check if TRDOS Rom is mapped.
        if (ESPeccy::trdos) {

            // int lowByte = address & 0xFF;

            switch (address8) {
                case 0xFF:
                    // printf("WD1793 Write Control Register: %d\n",data);
                    ESPeccy::Betadisk.WriteSystemReg(data);
                    goto lateIOContention;
                    break;
                case 0x1F:
                    // printf("WD1793 Write Command Register: %d\n",data);
                    ESPeccy::Betadisk.WriteCommandReg(data);
                    goto lateIOContention;
                    break;
                case 0x3F:
                    // printf("WD1793 Write Track Register: %d\n",data);
                    ESPeccy::Betadisk.WriteTrackReg(data);
                    goto lateIOContention;
                    break;
                case 0x5F:
                    // printf("WD1793 Write Sector Register: %d\n",data);
                    ESPeccy::Betadisk.WriteSectorReg(data);
                    goto lateIOContention;
                    break;
                case 0x7F:
                    // printf("WD1793 Write Data Register: %d\n",data);
                    ESPeccy::Betadisk.WriteDataReg(data);
                    goto lateIOContention;
                    break;
            }

        }

        switch (Config::Covox) {
            case CovoxNONE:
                break;
            case CovoxMONO:
                if ((address & 0x00FF) == 0x00FB) {
                    ESPeccy::covoxData[0] = ESPeccy::covoxData[1] = ESPeccy::covoxData[2] = ESPeccy::covoxData[3] = data;
                    ESPeccy::COVOXGetSample();
                    goto lateIOContention;
                }
                break;
            case CovoxSTEREO:
                if ((address & 0x00FF) == 0x0F) {
                    ESPeccy::covoxData[0] = ESPeccy::covoxData[1] = data;
                    ESPeccy::COVOXGetSample();
                    goto lateIOContention;
                }
                if ((address & 0x00FF) == 0x4F) {
                    ESPeccy::covoxData[2] = ESPeccy::covoxData[3] = data;
                    ESPeccy::COVOXGetSample();
                    goto lateIOContention;
                }
                break;
            case CovoxSOUNDDRIVE1:
                if ((address & 0x00AF) == 0x000F) {
                    switch (address & 0x0050) {
                        case 0x00: ESPeccy::covoxData[0] = data; ESPeccy::COVOXGetSample(); goto lateIOContention; break;
                        case 0x10: ESPeccy::covoxData[1] = data; ESPeccy::COVOXGetSample(); goto lateIOContention; break;
                        case 0x40: ESPeccy::covoxData[2] = data; ESPeccy::COVOXGetSample(); goto lateIOContention; break;
                        case 0x50: ESPeccy::covoxData[3] = data; ESPeccy::COVOXGetSample(); goto lateIOContention; break;
                        default: break;
                    }
                }
                break;
            case CovoxSOUNDDRIVE2:
                if ((address & 0x00F1) == 0x00F1) {
                    switch (address & 0x000A) {
                        case 0x0: ESPeccy::covoxData[0] = data; ESPeccy::COVOXGetSample(); goto lateIOContention; break;
                        case 0x2: ESPeccy::covoxData[1] = data; ESPeccy::COVOXGetSample(); goto lateIOContention; break;
                        case 0x8: ESPeccy::covoxData[2] = data; ESPeccy::COVOXGetSample(); goto lateIOContention; break;
                        case 0xA: ESPeccy::covoxData[3] = data; ESPeccy::COVOXGetSample(); goto lateIOContention; break;
                        default: break;
                    }
                }
                break;
            default:
                break;
        }

        lateIOContention:

        if (Config::arch[0] == 'T' && Config::ALUTK > 0)
            VIDEO::Draw(3 + tkIOcon(address), false);
        else
            ioContentionLate((Z80Ops::is48 || Z80Ops::is128) && MemESP::ramContended[rambank]);

    }

    // Spectrum 128/+2/Pentagon 128: Check port 0x7FFD for memory paging.
    // The port is partially decoded: Bits 1, and 15 must be reset.
    if ((Z80Ops::is128 || Z80Ops::isPentagon) && ((address & 0x8002) == 0)) {

        if (!MemESP::pagingLock) {

            MemESP::pagingLock = bitRead(data, 5);

            if (MemESP::bankLatch != (data & 0x7)) {
                MemESP::bankLatch = data & 0x7;
                #ifdef TIME_MACHINE_ENABLED
                MemESP::tm_bank_chg[MemESP::bankLatch] = true; // Bank selected. Mark for time machine
                #endif
                MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];
                MemESP::ramContended[3] = Z80Ops::isPentagon ? false : (MemESP::bankLatch & 0x01 ? true: false);
            }

            MemESP::romLatch = bitRead(data, 4);
            bitWrite(MemESP::romInUse, 0, MemESP::romLatch);
            MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];

            if (MemESP::videoLatch != bitRead(data, 3)) {
                MemESP::videoLatch = bitRead(data, 3);
                #ifdef TIME_MACHINE_ENABLED
                MemESP::tm_bank_chg[MemESP::videoLatch ? 7 : 5] = true; // Bank selected. Mark for time machine
                #endif
                VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7] : MemESP::ram[5];
            }

        }

        LastOutTo7FFD = data;

    // Spectrum +2A/+3: Check port 0x7FFD for memory paging.
    // The port is partially decoded: Bits 1, and 15 must be reset and bit 14 set.
    } else if ((Z80Ops::is2a3) && ((address & 0xC002) == 0x4000)) {
        if (!MemESP::pagingLock) {

            // printf("+2A/+3 0x7FFD OUT\n");

            MemESP::pagingLock = bitRead(data, 5);

            MemESP::romLatch = (bitRead(LastOutTo1FFD, 2) << 1) | bitRead(data, 4);
            MemESP::romInUse = MemESP::romLatch & 0x3;

            #ifdef TIME_MACHINE
            if (MemESP::bankLatch != (data & 0x7)) {
                MemESP::tm_bank_chg[(data & 0x7)] = true; // Bank selected. Mark for time machine
            }
            #endif

            MemESP::bankLatch = data & 0x7;

            // If +2a/+3 paging mode is normal
            // If is special (ram is rom) ignore paging, but preserve banklatch, rominuse and romlatch
            if (MemESP::pagingmode2A3 == 0) {
                MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
                MemESP::ramContended[0] = false;

                MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];
                MemESP::ramContended[3] = MemESP::bankLatch > 3;

            }

            #ifdef TIME_MACHINE
            if (MemESP::videoLatch != bitRead(data, 3)) {
                MemESP::tm_bank_chg[bitRead(data, 3) ? 7 : 5] = true; // Bank selected. Mark for time machine
            }
            #endif

            MemESP::videoLatch = bitRead(data, 3);
            VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7] : MemESP::ram[5];

        }

        LastOutTo7FFD = data;

    // Spectrum +2A/+3: Check port 0x1FFD for extra memory paging commands and disk motor switch (motor switch is not implemented).
    // The port is partially decoded: Bits 1, 13, 14 and 15 must be reset and bit 12 set.
    } else if ((Z80Ops::is2a3) && ((address & 0xF002) == 0x1000)) {
        if (!MemESP::pagingLock) {
            // printf("+2A/+3 0x1FFD OUT\n");

            if (bitRead(data, 0) == 0) {

                // printf("Paging mode normal\n");

                MemESP::pagingmode2A3 = 0;

                // Bit 2 is the high bit of the ROM bank selection
                MemESP::romLatch = (bitRead(data, 2) << 1) | bitRead(LastOutTo7FFD, 4);
                MemESP::romInUse = MemESP::romLatch & 0x3;

                MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
                MemESP::ramCurrent[1] = MemESP::ram[5];
                MemESP::ramCurrent[2] = MemESP::ram[2];
                MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];

                MemESP::ramContended[0] = MemESP::ramContended[2] = false;
                MemESP::ramContended[1] = true;
                MemESP::ramContended[3] = MemESP::bankLatch > 3;

            } else {

                // printf("Paging mode allram\n");

                MemESP::pagingmode2A3 = 0xff;

                switch ((data & 6) >> 1) {
                    case 0:
                        MemESP::ramCurrent[0] = MemESP::ram[0];
                        MemESP::ramCurrent[1] = MemESP::ram[1];
                        MemESP::ramCurrent[2] = MemESP::ram[2];
                        MemESP::ramCurrent[3] = MemESP::ram[3];

                        MemESP::ramContended[0] = false;
                        MemESP::ramContended[1] = false;
                        MemESP::ramContended[2] = false;
                        MemESP::ramContended[3] = false;

                        break;
                    case 1:
                        MemESP::ramCurrent[0] = MemESP::ram[4];
                        MemESP::ramCurrent[1] = MemESP::ram[5];
                        MemESP::ramCurrent[2] = MemESP::ram[6];
                        MemESP::ramCurrent[3] = MemESP::ram[7];

                        MemESP::ramContended[0] = true;
                        MemESP::ramContended[1] = true;
                        MemESP::ramContended[2] = true;
                        MemESP::ramContended[3] = true;

                        break;
                    case 2:
                        MemESP::ramCurrent[0] = MemESP::ram[4];
                        MemESP::ramCurrent[1] = MemESP::ram[5];
                        MemESP::ramCurrent[2] = MemESP::ram[6];
                        MemESP::ramCurrent[3] = MemESP::ram[3];

                        MemESP::ramContended[0] = true;
                        MemESP::ramContended[1] = true;
                        MemESP::ramContended[2] = true;
                        MemESP::ramContended[3] = false;

                        break;
                    case 3:
                        MemESP::ramCurrent[0] = MemESP::ram[4];
                        MemESP::ramCurrent[1] = MemESP::ram[7];
                        MemESP::ramCurrent[2] = MemESP::ram[6];
                        MemESP::ramCurrent[3] = MemESP::ram[3];

                        MemESP::ramContended[0] = true;
                        MemESP::ramContended[1] = true;
                        MemESP::ramContended[2] = true;
                        MemESP::ramContended[3] = false;

                        break;
                }
            }
        }

        LastOutTo1FFD = data;

    }

}

IRAM_ATTR void Ports::ioContentionLate(bool contend) {

    if (contend) {
        VIDEO::Draw(1, true);
        VIDEO::Draw(1, true);
        VIDEO::Draw(1, true);
    } else {
        VIDEO::Draw(3, false);
    }

}
