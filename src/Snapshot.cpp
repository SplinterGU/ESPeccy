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


#include "Snapshot.h"
#include "FileUtils.h"
#include "Config.h"
#include "cpuESP.h"
#include "Video.h"
#include "MemESP.h"
#include "ESPeccy.h"
#include "Ports.h"
#include "messages.h"
#include "OSD.h"
#include "Tape.h"
#include "AySound.h"
#include "pwm_audio.h"
#include "Z80_JLS/z80.h"
#include "ZXKeyb.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include <stdio.h>
#include <inttypes.h>
#include <string>

using namespace std;

// Change running snapshot
bool LoadSnapshot(string filename, string force_arch, string force_romset, uint8_t force_ALU) {

    bool res = false;

    uint8_t OSDprev = VIDEO::OSD;

    if (FileUtils::hasExtension(filename, "sna")) {
        res = FileSNA::load(filename, force_arch, force_romset, force_ALU);

    } else if (FileUtils::hasExtension(filename, "z80")) {
        res = FileZ80::load(filename);

    } else if (FileUtils::hasExtension(filename, "sp")) {
        res = FileSP::load(filename);

    } else if (FileUtils::hasExtension(filename, "p")) {
        res = FileP::load(filename);

    }

    if (res && OSDprev) {
        VIDEO::OSD = OSDprev;
        if (Config::aspect_16_9)
            VIDEO::Draw_OSD169 = Z80Ops::is2a3 ? VIDEO::MainScreen_OSD_2A3 : VIDEO::MainScreen_OSD;
        else
            VIDEO::Draw_OSD43  = Z80Ops::isPentagon ? VIDEO::BottomBorder_OSD_Pentagon : VIDEO::BottomBorder_OSD;
        ESPeccy::TapeNameScroller = 0;
    }

    return res;

}

// Change running snapshot
bool SaveSnapshot(string filename, bool force_saverom) {

    bool res = false;

    if (FileUtils::hasExtension(filename, "sna")) {

        res = FileSNA::save(filename, force_saverom);

    } else if (FileUtils::hasExtension(filename, "z80")) {

        res = FileZ80::save(filename, force_saverom);

    } else if (FileUtils::hasExtension(filename, "sp")) {

        res = FileSP::save(filename, force_saverom);

    }

    return res;

}

string getSnapshotCheatPath(const string& path) {
    // Buscar la posición del último separador '/' o '\' (para compatibilidad).
    size_t lastSlash = path.find_last_of("/\\");

    // Extraer el nombre del archivo sin ruta.
    std::string filename = (lastSlash == string::npos)
                            ? path
                            : path.substr(lastSlash + 1);

    // Buscar la posición del punto de la extensión.
    size_t dotPos = filename.find_last_of('.');

    // Eliminar la extensión, si existe.
    if (dotPos != string::npos) {
        filename = filename.substr(0, dotPos);
    }

    // Construir la nueva ruta: <directorio>/POKES/<archivo>.pok
    string newPath = (lastSlash == string::npos)
                        ? "POKES/" + filename + ".pok"
                        : path.substr(0, lastSlash + 1) + "POKES/" + filename + ".pok";

    return newPath;
}

// ///////////////////////////////////////////////////////////////////////////////

bool FileSNA::load(string sna_fn, string force_arch, string force_romset, uint8_t force_ALU) {

    FILE *file;
    int sna_size;
    string snapshotArch;

    file = fopen(sna_fn.c_str(), "rb");
    if (file==NULL)
    {
        printf("FileSNA: Error opening %s\n",sna_fn.c_str());
        return false;
    }

    printf("Force arch: %s, Force romset: %s, Force ALU: %d\n", force_arch.c_str(),force_romset.c_str(),force_ALU);

    fseek(file,0,SEEK_END);
    sna_size = ftell(file);
    rewind (file);

    // Check snapshot arch
    if (sna_size == SNA_48K_SIZE || sna_size == SNA_48K_WITH_ROM_SIZE) { // SNA_48K_WITH_ROM_SIZE for non-standard SNA with rom included

        if (force_arch == "" && !Z80Ops::is48) {
            force_arch = "48K";
        }

        // // If using some 48K arch it keeps unmodified. If not, we choose 48k because is SNA format default
        // if (ConfigZ80Ops::is48)
        //     snapshotArch = Config::arch;
        // else
        //     snapshotArch = "48K";

    } else if ((sna_size == SNA_128K_SIZE1) || (sna_size == SNA_128K_SIZE2)) {

        if (force_arch == "" && (Z80Ops::is48 || Z80Ops::is2a3)) {
            force_arch = "Pentagon";
        }

        // // If using some 128K arch it keeps unmodified. If not, we choose Pentagon because is SNA format default
        // if (!Z80Ops::is48)
        //     snapshotArch = Config::arch;
        // else
        //     snapshotArch = "Pentagon";

    } else if (sna_size == SNA_2A3_SIZE1) {

        if (force_arch == "" && !Z80Ops::is2a3) {
            force_arch = "+2A";
        }

    } else {
        printf("FileSNA::load: bad SNA %s: size = %d\n", sna_fn.c_str(), sna_size);
        return false;
    }

    // Change arch if needed
    if (force_arch != "" && force_arch != Config::arch) {

        bool vreset = Config::videomode;

        // If switching between TK models there's no need to reset in vidmodes > 0
        if (force_arch[0] == 'T' && Config::arch[0] == 'T') vreset = false;

        Config::requestMachine(force_arch, force_romset);

        // Condition this to 50hz mode
        if(vreset) {

            Config::SNA_Path = FileUtils::SNA_Path;
            Config::SNA_begin_row = FileUtils::fileTypes[DISK_SNAFILE].begin_row;
            Config::SNA_focus = FileUtils::fileTypes[DISK_SNAFILE].focus;
            Config::SNA_fdMode = FileUtils::fileTypes[DISK_SNAFILE].fdMode;
            Config::SNA_fileSearch = FileUtils::fileTypes[DISK_SNAFILE].fileSearch;

            Config::TAP_Path = FileUtils::TAP_Path;
            Config::TAP_begin_row = FileUtils::fileTypes[DISK_TAPFILE].begin_row;
            Config::TAP_focus = FileUtils::fileTypes[DISK_TAPFILE].focus;
            Config::TAP_fdMode = FileUtils::fileTypes[DISK_TAPFILE].fdMode;
            Config::TAP_fileSearch = FileUtils::fileTypes[DISK_TAPFILE].fileSearch;

            Config::DSK_Path = FileUtils::DSK_Path;
            Config::DSK_begin_row = FileUtils::fileTypes[DISK_DSKFILE].begin_row;
            Config::DSK_focus = FileUtils::fileTypes[DISK_DSKFILE].focus;
            Config::DSK_fdMode = FileUtils::fileTypes[DISK_DSKFILE].fdMode;
            Config::DSK_fileSearch = FileUtils::fileTypes[DISK_DSKFILE].fileSearch;

            Config::ram_file = sna_fn;
            Config::save();
            OSD::esp_hard_reset();
        }

    } else if (force_romset != "" && force_romset != Config::romSet) {

        Config::requestMachine(Config::arch, force_romset);

    }

    // Change ALU to snapshot one if present
    if (force_ALU != 0xff && force_ALU != Config::ALUTK) {

        Config::ALUTK = force_ALU;

        // Condition this to 50hz mode
        if(Config::videomode) {

            Config::SNA_Path = FileUtils::SNA_Path;
            Config::SNA_begin_row = FileUtils::fileTypes[DISK_SNAFILE].begin_row;
            Config::SNA_focus = FileUtils::fileTypes[DISK_SNAFILE].focus;
            Config::SNA_fdMode = FileUtils::fileTypes[DISK_SNAFILE].fdMode;
            Config::SNA_fileSearch = FileUtils::fileTypes[DISK_SNAFILE].fileSearch;

            Config::TAP_Path = FileUtils::TAP_Path;
            Config::TAP_begin_row = FileUtils::fileTypes[DISK_TAPFILE].begin_row;
            Config::TAP_focus = FileUtils::fileTypes[DISK_TAPFILE].focus;
            Config::TAP_fdMode = FileUtils::fileTypes[DISK_TAPFILE].fdMode;
            Config::TAP_fileSearch = FileUtils::fileTypes[DISK_TAPFILE].fileSearch;

            Config::DSK_Path = FileUtils::DSK_Path;
            Config::DSK_begin_row = FileUtils::fileTypes[DISK_DSKFILE].begin_row;
            Config::DSK_focus = FileUtils::fileTypes[DISK_DSKFILE].focus;
            Config::DSK_fdMode = FileUtils::fileTypes[DISK_DSKFILE].fdMode;
            Config::DSK_fileSearch = FileUtils::fileTypes[DISK_DSKFILE].fileSearch;

            Config::ram_file = sna_fn;
            Config::save();
            OSD::esp_hard_reset();

        }

    }

    ESPeccy::reset();

    // printf("FileSNA::load: Opening %s: size = %d\n", sna_fn.c_str(), sna_size);

    // Read in the registers
    Z80::setRegI(readByteFile(file));

    Z80::setRegHLx(readWordFileLE(file));
    Z80::setRegDEx(readWordFileLE(file));
    Z80::setRegBCx(readWordFileLE(file));
    Z80::setRegAFx(readWordFileLE(file));

    Z80::setRegHL(readWordFileLE(file));
    Z80::setRegDE(readWordFileLE(file));
    Z80::setRegBC(readWordFileLE(file));

    Z80::setRegIY(readWordFileLE(file));
    Z80::setRegIX(readWordFileLE(file));

    uint8_t inter = readByteFile(file);
    Z80::setIFF2(inter & 0x04 ? true : false);
    Z80::setIFF1(Z80::isIFF2());
    Z80::setRegR(readByteFile(file));

    Z80::setRegAF(readWordFileLE(file));
    Z80::setRegSP(readWordFileLE(file));

    Z80::setIM((Z80::IntMode)(readByteFile(file)));

    VIDEO::borderColor = readByteFile(file) & 0x07;
    VIDEO::brd = VIDEO::border32[VIDEO::borderColor];

    // Load ROM if present
    if (Z80Ops::is48 && sna_size == SNA_48K_WITH_ROM_SIZE) {
        MemESP::ramCurrent[0] = MemESP::rom[0] = MemESP::ram[7];
        readBlockFile(file, MemESP::rom[0], 0x4000);
    }

    // read 48K memory
    readBlockFile(file, MemESP::ram[5], 0x4000);
    readBlockFile(file, MemESP::ram[2], 0x4000);
    readBlockFile(file, MemESP::ram[0], 0x4000);

    if (Z80Ops::is48) {
        // in 48K mode, pop PC from stack
        uint16_t SP = Z80::getRegSP();
        Z80::setRegPC(MemESP::readword(SP));
        Z80::setRegSP(SP + 2);

    } else {
        // in 128K mode, recover stored PC
        uint16_t sna_PC = readWordFileLE(file);
        Z80::setRegPC(sna_PC);

        // tmp_port contains page switching status, including current page number (latch)
        uint8_t tmp_port = readByteFile(file);
        uint8_t tmp_latch = tmp_port & 0x07;

        // copy what was read into page 0 to correct page
        memcpy(MemESP::ram[tmp_latch], MemESP::ram[0], 0x4000);

        uint8_t tr_dos = readByteFile(file);     // Check if TR-DOS is paged

        // read remaining pages
        for (int page = 0; page < 8; page++) {
            if (page != tmp_latch && page != 2 && page != 5) {
                readBlockFile(file, MemESP::ram[page], 0x4000);
            }
        }

        // decode tmp_port
        MemESP::videoLatch = bitRead(tmp_port, 3);
        MemESP::romLatch = bitRead(tmp_port, 4);
        MemESP::pagingLock = bitRead(tmp_port, 5);
        MemESP::bankLatch = tmp_latch;

        Ports::LastOutTo7FFD = tmp_port;

        if (tr_dos) {
            MemESP::romInUse = 4;
            ESPeccy::trdos = true;
        } else {
            MemESP::romInUse = MemESP::romLatch;
            ESPeccy::trdos = false;
        }

        MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];
        MemESP::ramContended[3] = Z80Ops::isPentagon ? false : (MemESP::bankLatch & 0x01 ? true: false);

        VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7] : MemESP::ram[5];

        if (sna_size == SNA_2A3_SIZE1) {

            uint8_t tmp_port = readByteFile(file);

            if (bitRead(tmp_port, 0) == 0) {

                MemESP::pagingmode2A3 = 0;

                // Bit 2 is the high bit of the ROM bank selection
                MemESP::romLatch = bitRead(MemESP::romInUse, 0) + (bitRead(tmp_port, 2) << 1);
                MemESP::romInUse = MemESP::romLatch & 0x3;

                MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
                // MemESP::ramCurrent[1] = MemESP::ram[5];
                // MemESP::ramCurrent[2] = MemESP::ram[2];
                // MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];

                MemESP::ramContended[3] = MemESP::bankLatch >= 4 ? true: false;

            } else {

                MemESP::pagingmode2A3 = 0xff;

                switch ((tmp_port & 6) >> 1) {
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

            Ports::LastOutTo1FFD = tmp_port;

            AySound::selectedRegister = readByteFile(file);

            for(int i=0; i < 16; i++) AySound::regs[i] = readByteFile(file);

            for(int i=0; i < 16; i++) AySound::updateReg[i](); // Update all registers

        }

    }

    fclose(file);

    return true;

}

// ///////////////////////////////////////////////////////////////////////////////

bool FileSNA::isStateAvailable(string filename) {

    FILE *f = fopen(filename.c_str(), "rb");
    if (f == NULL)
        return false;
    else
        fclose(f);

    return true;

}

// ///////////////////////////////////////////////////////////////////////////////

static bool writeMemPage(uint8_t page, FILE *file, bool blockMode)
{
    page = page & 0x07;
    uint8_t* buffer = MemESP::ram[page];

    // printf("writing page %d in [%s] mode\n", page, blockMode ? "block" : "byte");

    if (blockMode) {
        // Writing blocks should be faster, but fails sometimes when flash is getting full.
        for (int offset = 0; offset < MEM_PG_SZ; offset+=0x4000) {
            // printf("writing block from page %d at offset %d\n", page, offset);
            if (1 != fwrite(&buffer[offset],0x4000,1,file)) {
                printf("error writing block from page %d at offset %d\n", page, offset);
                return false;
            }
        }
    }
    else {
        for (int offset = 0; offset < MEM_PG_SZ; offset++) {
            uint8_t b = buffer[offset];
            if (1 != fwrite(&b,1,1,file)) {
                printf("error writing byte from page %d at offset %d\n", page, offset);
                return false;
            }
        }
    }
    return true;
}

// ///////////////////////////////////////////////////////////////////////////////

bool FileSNA::save(string sna_file, bool force_saverom) {

    // Try to save using pages
    if (FileSNA::save(sna_file, true, force_saverom)) return true;

    OSD::osdCenteredMsg(OSD_PSNA_SAVE_WARN, LEVEL_WARN);

    // Try to save byte-by-byte
    return FileSNA::save(sna_file, false, force_saverom);

}

// ///////////////////////////////////////////////////////////////////////////////

bool FileSNA::save(string sna_file, bool blockMode, bool force_saverom) {

    FILE *file;

    file = fopen(sna_file.c_str(), "wb");
    if (file==NULL)
    {
        printf("FileSNA: Error opening %s for writing",sna_file.c_str());
        return false;
    }

    OSD::progressDialog(OSD_PSNA_SAVING,OSD_PLEASE_WAIT[Config::lang],0,0,true);

    // write registers: begin with I
    writeByteFile(Z80::getRegI(), file);

    writeWordFileLE(Z80::getRegHLx(), file);
    writeWordFileLE(Z80::getRegDEx(), file);
    writeWordFileLE(Z80::getRegBCx(), file);
    writeWordFileLE(Z80::getRegAFx(), file);

    writeWordFileLE(Z80::getRegHL(), file);
    writeWordFileLE(Z80::getRegDE(), file);
    writeWordFileLE(Z80::getRegBC(), file);

    writeWordFileLE(Z80::getRegIY(), file);
    writeWordFileLE(Z80::getRegIX(), file);

    uint8_t inter = Z80::isIFF2() ? 0x04 : 0;
    writeByteFile(inter, file);
    writeByteFile(Z80::getRegR(), file);

    writeWordFileLE(Z80::getRegAF(), file);

    uint16_t SP = Z80::getRegSP();

    // if (Config::arch == "48K" || Config::arch == "TK90X" || Config::arch == "TK95") {
    if (Z80Ops::is48) {
        // decrement stack pointer it for pushing PC to stack, only on 48K
        SP -= 2;
        MemESP::writeword(SP, Z80::getRegPC());
    }
    writeWordFileLE(SP, file);

    writeByteFile(Z80::getIM(), file);

    uint8_t bordercol = VIDEO::borderColor;
    writeByteFile(bordercol, file);

    if ( Z80Ops::is48 ) {
        if ( MemESP::rom[0] == MemESP::ram[7] || force_saverom ) {
            writeBlockFile(file, MemESP::rom[0], 0x4000);
        }
    }

    // write RAM pages in 48K address space (0x4000 - 0xFFFF)
    uint8_t pages[3] = {5, 2, 0};

    if (Z80Ops::is128 || Z80Ops::is2a3 || Z80Ops::isPentagon)
        pages[2] = MemESP::bankLatch;

    for (uint8_t ipage = 0; ipage < 3; ipage++) {
        uint8_t page = pages[ipage];
        if (!writeMemPage(page, file, blockMode)) {
            fclose(file);
            return false;
        }
    }

    if (Z80Ops::is128 || Z80Ops::is2a3 || Z80Ops::isPentagon) {
    // if (Config::arch != "48K") {

        // write pc
        writeWordFileLE( Z80::getRegPC(), file);
        // printf("PC: %u\n",(unsigned int)Z80::getRegPC());

        // write memESP bank control port
        uint8_t tmp_port = MemESP::bankLatch;
        bitWrite(tmp_port, 3, MemESP::videoLatch);
        bitWrite(tmp_port, 4, MemESP::romLatch);
        bitWrite(tmp_port, 5, MemESP::pagingLock);
        writeByteFile(tmp_port, file);
        // printf("7FFD: %u\n",(unsigned int)tmp_port);

        if (ESPeccy::trdos)
            writeByteFile(1, file);     // TR-DOS paged
        else
            writeByteFile(0, file);     // TR-DOS not paged

        // write remaining ram pages
        for (int page = 0; page < 8; page++) {
            if (page != MemESP::bankLatch && page != 2 && page != 5) {
                if (!writeMemPage(page, file, blockMode)) {
                    fclose(file);
                    OSD::progressDialog("","",0,2,true);
                    return false;

                }
            }
        }
    }

    if (Z80Ops::is2a3) {
        // write 1ffd port
        writeByteFile(Ports::LastOutTo1FFD, file);

        writeByteFile(AySound::selectedRegister, file);
        for(int i=0; i < 16; i++)
            writeByteFile(AySound::regs[i],file);

    }

    fclose(file);

    OSD::progressDialog("","",0,2,true);

    return true;

}

static uint16_t mkword(uint8_t lobyte, uint8_t hibyte) {
    return lobyte | (hibyte << 8);
}

bool FileZ80::load(string z80_fn) {

    bool keepArch = false;

    FILE *file;

    file = fopen(z80_fn.c_str(), "rb");
    if (file == NULL) {
        printf("FileZ80: Error opening %s\n",z80_fn.c_str());
        return false;
    }

    // Check Z80 version and arch
    uint8_t z80version;
    uint8_t mch;
    string z80_arch = "";
    uint16_t ahb_len;

    fseek(file,6,SEEK_SET);

    if (mkword(readByteFile(file),readByteFile(file)) != 0) { // Version 1

        z80version = 1;
        mch = 0;
        z80_arch = "48K";

    } else { // Version 2 o 3

        fseek(file,30,SEEK_SET);
        ahb_len = mkword(readByteFile(file),readByteFile(file));

        // additional header block length
        if (ahb_len == 23)
            z80version = 2;
        else if (ahb_len == 54 || ahb_len == 55)
            z80version = 3;
        else {
            OSD::osdCenteredMsg("Z80 load: unknown version", LEVEL_ERROR);
            printf("Z80.load: unknown version, ahblen = %u\n", (unsigned int) ahb_len);
            fclose(file);
            return false;
        }

        fseek(file,34,SEEK_SET);
        mch = readByteFile(file); // Machine

        if (z80version == 2) {
            if (mch == 0) z80_arch = "48K";
            if (mch == 1) z80_arch = "48K"; // + if1
            // if (mch == 2) z80_arch = "SAMRAM";
            if (mch == 3) z80_arch = "128K";
            if (mch == 4) z80_arch = "128K"; // + if1
        }
        else if (z80version == 3) {
            if (mch == 0) z80_arch = "48K";
            if (mch == 1) z80_arch = "48K"; // + if1
            // if (mch == 2) z80_arch = "SAMRAM";
            if (mch == 3) z80_arch = "48K"; // + mgt
            if (mch == 4) z80_arch = "128K";
            if (mch == 5) z80_arch = "128K"; // + if1
            if (mch == 6) z80_arch = "128K"; // + mgt
            if (mch == 7) z80_arch = "+2A"; // Spectrum +3
            if (mch == 9) z80_arch = "Pentagon";
            if (mch == 12) z80_arch = "128K"; // Spectrum +2
            if (mch == 13) z80_arch = "+2A"; // Spectrum +2A
        }

    }

    // printf("Z80 version %u, AHB Len: %u, machine code: %u\n",(unsigned char)z80version,(unsigned int)ahb_len, (unsigned char)mch);

    if (z80_arch == "") {
        OSD::osdCenteredMsg("Z80 load: unknown machine", LEVEL_ERROR);
        printf("Z80.load: unknown machine, machine code = %u\n", (unsigned char)mch);
        fclose(file);
        return false;
    }


    if (z80_arch == "48K") {
        if (Config::arch == "48K" || Config::arch == "TK90X" || Config::arch == "TK95") keepArch = true;
        else keepArch = false;
    } else {
        // 128k or Pentagon
        if (Config::arch == "128K" || Config::arch == "+2A" || Config::arch == "Pentagon") keepArch = true;
        else keepArch = false;
    }

    // printf("fileTypes -> Path: %s, begin_row: %d, focus: %d\n",FileUtils::SNA_Path.c_str(),FileUtils::fileTypes[DISK_SNAFILE].begin_row,FileUtils::fileTypes[DISK_SNAFILE].focus);
    // printf("Config    -> Path: %s, begin_row: %d, focus: %d\n",Config::Path.c_str(),(int)Config::begin_row,(int)Config::focus);

    // Manage arch change
    if (Config::arch != z80_arch) {

        if (!keepArch) {

            string z80_romset = "";

            // printf("z80_arch: %s mch: %d pref_romset48: %s pref_romset128: %s z80_romset: %s\n",z80_arch.c_str(),mch,Config::pref_romSet_48.c_str(),Config::pref_romSet_128.c_str(),z80_romset.c_str());

            if (z80_arch == "48K") {
                if (Config::pref_romSet_48 == "48K" || Config::pref_romSet_48 == "48Kes")
                    z80_romset = Config::pref_romSet_48;
            } else
            if (z80_arch == "128K") {
                if (mch == 12) { // +2
                    if (Config::pref_romSet_128 == "+2es" || Config::pref_romSet_128 == "+2fr" || Config::pref_romSet_128 == "128Kcs")
                        z80_romset = Config::pref_romSet_128;
                    else
                        z80_romset = "+2";
                } else {
                    if (Config::pref_romSet_128 == "128K" || Config::pref_romSet_128 == "128Kes" || Config::pref_romSet_128 == "128Kcs")
                        z80_romset = Config::pref_romSet_128;
                }
            }

            // printf("z80_arch: %s mch: %d pref_romset48: %s pref_romset128: %s z80_romset: %s\n",z80_arch.c_str(),mch,Config::pref_romSet_48.c_str(),Config::pref_romSet_128.c_str(),z80_romset.c_str());

            Config::requestMachine(z80_arch, z80_romset);

            // Condition this to 50hz mode
            if(Config::videomode) {

                Config::SNA_Path = FileUtils::SNA_Path;
                Config::SNA_begin_row = FileUtils::fileTypes[DISK_SNAFILE].begin_row;
                Config::SNA_focus = FileUtils::fileTypes[DISK_SNAFILE].focus;
                Config::SNA_fdMode = FileUtils::fileTypes[DISK_SNAFILE].fdMode;
                Config::SNA_fileSearch = FileUtils::fileTypes[DISK_SNAFILE].fileSearch;

                Config::TAP_Path = FileUtils::TAP_Path;
                Config::TAP_begin_row = FileUtils::fileTypes[DISK_TAPFILE].begin_row;
                Config::TAP_focus = FileUtils::fileTypes[DISK_TAPFILE].focus;
                Config::TAP_fdMode = FileUtils::fileTypes[DISK_TAPFILE].fdMode;
                Config::TAP_fileSearch = FileUtils::fileTypes[DISK_TAPFILE].fileSearch;

                Config::DSK_Path = FileUtils::DSK_Path;
                Config::DSK_begin_row = FileUtils::fileTypes[DISK_DSKFILE].begin_row;
                Config::DSK_focus = FileUtils::fileTypes[DISK_DSKFILE].focus;
                Config::DSK_fdMode = FileUtils::fileTypes[DISK_DSKFILE].fdMode;
                Config::DSK_fileSearch = FileUtils::fileTypes[DISK_DSKFILE].fileSearch;

                Config::ram_file = z80_fn;
                Config::save();
                OSD::esp_hard_reset();
            }

        }

    } else {

        if (z80_arch == "128K") {

            string z80_romset = "";

            // printf("z80_arch: %s mch: %d pref_romset48: %s pref_romset128: %s z80_romset: %s\n",z80_arch.c_str(),mch,Config::pref_romSet_48.c_str(),Config::pref_romSet_128.c_str(),z80_romset.c_str());

            if (mch == 12) { // +2

                if (Config::romSet != "+2" && Config::romSet != "+2es" && Config::romSet != "+2fr" && Config::romSet != "128Kcs") {

                    if (Config::pref_romSet_128 == "+2es" || Config::pref_romSet_128 == "+2fr" || Config::pref_romSet_128 == "128Kcs")
                        z80_romset = Config::pref_romSet_128;
                    else
                        z80_romset = "+2";

                    Config::requestMachine(z80_arch, z80_romset);

                }

            } else {

                if (Config::romSet != "128K" && Config::romSet != "128Kes" && Config::romSet != "128Kcs") {

                    if (Config::pref_romSet_128 == "128Kes" || Config::pref_romSet_128 == "128Kcs")
                        z80_romset = Config::pref_romSet_128;
                    else
                        z80_romset = "128K";

                    Config::requestMachine(z80_arch, z80_romset);
                }

            }

            // printf("z80_arch: %s mch: %d pref_romset48: %s pref_romset128: %s z80_romset: %s\n",z80_arch.c_str(),mch,Config::pref_romSet_48.c_str(),Config::pref_romSet_128.c_str(),z80_romset.c_str());

        }

    }

    ESPeccy::reset();

    keepArch = false;

    // Get file size
    fseek(file,0,SEEK_END);
    uint32_t file_size = ftell(file);
    rewind (file);

    uint32_t dataOffset = 0;

    // stack space for header, should be enough for
    // version 1 (30 bytes)
    // version 2 (55 bytes) (30 + 2 + 23)
    // version 3 (87 bytes) (30 + 2 + 55) or (86 bytes) (30 + 2 + 54)
    uint8_t header[87];

    // read first 30 bytes
    for (uint8_t i = 0; i < 30; i++) {
        header[i] = readByteFile(file);
        dataOffset ++;
    }

    // additional vars
    uint8_t b12, b29;

    // begin loading registers
    Z80::setRegA  (       header[0]);
    Z80::setFlags (       header[1]);

    // Z80::setRegBC (mkword(header[2], header[3]));
    Z80::setRegC(header[2]);
    Z80::setRegB(header[3]);

    // Z80::setRegHL (mkword(header[4], header[5]));
    Z80::setRegL(header[4]);
    Z80::setRegH(header[5]);

    Z80::setRegPC (mkword(header[6], header[7]));
    Z80::setRegSP (mkword(header[8], header[9]));

    Z80::setRegI  (       header[10]);

    // Z80::setRegR  (       header[11]);
    uint8_t regR = header[11] & 0x7f;
    if ((header[12] & 0x01) != 0) {
        regR |= 0x80;
    }
    Z80::setRegR(regR);

    b12 =                 header[12];

    VIDEO::borderColor = (b12 >> 1) & 0x07;
    VIDEO::brd = VIDEO::border32[VIDEO::borderColor];

    // Z80::setRegDE (mkword(header[13], header[14]));
    Z80::setRegE(header[13]);
    Z80::setRegD(header[14]);

    // Z80::setRegBCx(mkword(header[15], header[16]));
    Z80::setRegCx(header[15]);
    Z80::setRegBx(header[16]);

    // Z80::setRegDEx(mkword(header[17], header[18]));
    Z80::setRegEx(header[17]);
    Z80::setRegDx(header[18]);

    // Z80::setRegHLx(mkword(header[19], header[20]));
    Z80::setRegLx(header[19]);
    Z80::setRegHx(header[20]);

    // Z80::setRegAFx(mkword(header[22], header[21])); // watch out for order!!!
    Z80::setRegAx(header[21]);
    Z80::setRegFx(header[22]);

    Z80::setRegIY (mkword(header[23], header[24]));
    Z80::setRegIX (mkword(header[25], header[26]));

    Z80::setIFF1  (       header[27] ? true : false);
    Z80::setIFF2  (       header[28] ? true : false);
    b29 =                 header[29];
    Z80::setIM((Z80::IntMode)(b29 & 0x03));

    // spectrum.setIssue2((z80Header1[29] & 0x04) != 0); // TO DO: Implement this

    uint16_t RegPC = Z80::getRegPC();

    bool dataCompressed = (b12 & 0x20) ? true : false;

    if (z80version == 1) {

        // version 1, the simplest, 48K only.
        uint32_t memRawLength = file_size - dataOffset;

        if (dataCompressed) {
            // assuming stupid 00 ED ED 00 terminator present, should check for it instead of assuming
            uint16_t dataLen = (uint16_t)(memRawLength - 4);

            // load compressed data into memory
            loadCompressedMemData(file, dataLen, 0x4000, 0xC000);
        } else {
            uint16_t dataLen = (memRawLength < 0xC000) ? memRawLength : 0xC000;

            // load uncompressed data into memory
            for (int i = 0; i < dataLen; i++)
                MemESP::writebyte(0x4000 + i, readByteFile(file));
        }

        // latches for 48K
        MemESP::romLatch = 0;
        MemESP::romInUse = 0;
        MemESP::bankLatch = 0;
        MemESP::pagingLock = 1;
        MemESP::videoLatch = 0;

    } else {

        // read 2 more bytes
        for (uint8_t i = 30; i < 32; i++) {
            header[i] = readByteFile(file);
            dataOffset ++;
        }

        // additional header block length
        uint16_t ahblen = mkword(header[30], header[31]);

        // read additional header block
        for (uint8_t i = 32; i < 32 + ahblen; i++) {
            header[i] = readByteFile(file);
            dataOffset ++;
        }

        // program counter
        RegPC = mkword(header[32], header[33]);
        Z80::setRegPC(RegPC);

        if (z80_arch == "48K") {

            MemESP::romLatch = 0;
            MemESP::romInUse = 0;
            MemESP::bankLatch = 0;
            MemESP::pagingLock = 1;
            MemESP::videoLatch = 0;

            uint16_t pageStart[12] = {0, 0, 0, 0, 0x8000, 0xC000, 0, 0, 0x4000, 0, 0};

            uint32_t dataLen = file_size;
            while (dataOffset < dataLen) {
                uint8_t hdr0 = readByteFile(file); dataOffset ++;
                uint8_t hdr1 = readByteFile(file); dataOffset ++;
                uint8_t hdr2 = readByteFile(file); dataOffset ++;
                uint16_t compDataLen = mkword(hdr0, hdr1);

                uint16_t memoff = pageStart[hdr2];

                // z80 with rom
                if ( !hdr2 ) {
                    MemESP::ramCurrent[0] = MemESP::rom[0] = MemESP::ram[7];

                    if (compDataLen == 0xffff) {
                        // load uncompressed data into memory
                        // printf("Loading uncompressed data\n");
                        compDataLen = 0x4000;

                        for (int i = 0; i < compDataLen; i++)
                            MemESP::ram[7][i] = readByteFile(file);

                    } else {
                        // Block is compressed
                        loadCompressedMemPage(file, compDataLen, MemESP::ram[7], 0x4000);
                    }

                } else {
                    if (compDataLen == 0xffff) {
                        // Uncompressed data
                        compDataLen = 0x4000;

                        for (int i = 0; i < compDataLen; i++)
                            MemESP::writebyte(memoff + i, readByteFile(file));

                    } else {

                        loadCompressedMemData(file, compDataLen, memoff, 0x4000);

                    }
                }

                dataOffset += compDataLen;

            }

        } else if ((z80_arch == "128K") || (z80_arch == "+2A") || (z80_arch == "Pentagon")) {

            // paging register
            uint8_t b35 = header[35];
            // printf("Paging register: %u\n",b35);
            MemESP::videoLatch = bitRead(b35, 3);
            MemESP::romLatch = bitRead(b35, 4);
            MemESP::pagingLock = bitRead(b35, 5);
            MemESP::bankLatch = b35 & 0x07;
            MemESP::romInUse = MemESP::romLatch;

            Ports::LastOutTo7FFD = b35;

            MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];
            MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];

            if (z80_arch == "+2A")
                MemESP::ramContended[3] = MemESP::bankLatch >= 4 ? true: false;
            else
                MemESP::ramContended[3] = Z80Ops::isPentagon ? false : (MemESP::bankLatch & 0x01 ? true: false);

            // +2A / +3 header
            if (ahblen == 55) {

                uint8_t b86 = header[86];

                if (bitRead(b86, 0) == 0) {

                    MemESP::pagingmode2A3 = 0;

                    // Bit 2 is the high bit of the ROM bank selection
                    MemESP::romLatch = bitRead(MemESP::romInUse, 0) + (bitRead(b86, 2) << 1);
                    MemESP::romInUse = MemESP::romLatch & 0x3;
                    MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse];

                } else {

                    MemESP::pagingmode2A3 = 0xff;

                    switch ((b86 & 6) >> 1) {
                        case 0:
                            MemESP::ramCurrent[0] = MemESP::ram[0];
                            MemESP::ramCurrent[1] = MemESP::ram[1];
                            MemESP::ramCurrent[2] = MemESP::ram[2];
                            MemESP::ramCurrent[3] = MemESP::ram[3];
                            break;
                        case 1:
                            MemESP::ramCurrent[0] = MemESP::ram[4];
                            MemESP::ramCurrent[1] = MemESP::ram[5];
                            MemESP::ramCurrent[2] = MemESP::ram[6];
                            MemESP::ramCurrent[3] = MemESP::ram[7];
                            break;
                        case 2:
                            MemESP::ramCurrent[0] = MemESP::ram[4];
                            MemESP::ramCurrent[1] = MemESP::ram[5];
                            MemESP::ramCurrent[2] = MemESP::ram[6];
                            MemESP::ramCurrent[3] = MemESP::ram[3];
                            break;
                        case 3:
                            MemESP::ramCurrent[0] = MemESP::ram[4];
                            MemESP::ramCurrent[1] = MemESP::ram[7];
                            MemESP::ramCurrent[2] = MemESP::ram[6];
                            MemESP::ramCurrent[3] = MemESP::ram[3];
                            break;
                    }
                }

            }

            VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7] : MemESP::ram[5];

            uint8_t* pages[12] = {
                MemESP::rom[0], MemESP::rom[2], MemESP::rom[1],
                MemESP::ram[0], MemESP::ram[1], MemESP::ram[2], MemESP::ram[3],
                MemESP::ram[4], MemESP::ram[5], MemESP::ram[6], MemESP::ram[7],
                MemESP::rom[3] };

            // const char* pagenames[12] = { "rom0", "IDP", "rom1",
            //     "ram0", "ram1", "ram2", "ram3", "ram4", "ram5", "ram6", "ram7", "MFR" };

            uint32_t dataLen = file_size;
            while (dataOffset < dataLen) {

                uint8_t hdr0 = readByteFile(file); dataOffset ++;
                uint8_t hdr1 = readByteFile(file); dataOffset ++;
                uint8_t hdr2 = readByteFile(file); dataOffset ++;
                uint16_t compDataLen = mkword(hdr0, hdr1);

                if (compDataLen == 0xffff) {

                    // load uncompressed data into memory
                    // printf("Loading uncompressed data\n");

                    compDataLen = 0x4000;

                    if ((hdr2 > 2) && (hdr2 < 11))
                        for (int i = 0; i < compDataLen; i++)
                            pages[hdr2][i] = readByteFile(file);

                } else {

                    // Block is compressed

                    if ((hdr2 > 2) && (hdr2 < 11))
                        loadCompressedMemPage(file, compDataLen, pages[hdr2], 0x4000);

                }

                dataOffset += compDataLen;

            }
        }
    }

    fclose(file);

    return true;

}

void FileZ80::loadCompressedMemData(FILE *f, uint16_t dataLen, uint16_t memoff, uint16_t memlen) {

    uint16_t dataOff = 0;
    uint8_t ed_cnt = 0;
    uint8_t repcnt = 0;
    uint8_t repval = 0;
    uint16_t memidx = 0;

    while(dataOff < dataLen && memidx < memlen) {
        uint8_t databyte = readByteFile(f);
        if (ed_cnt == 0) {
            if (databyte != 0xED)
                MemESP::writebyte(memoff + memidx++, databyte);
            else
                ed_cnt++;
        }
        else if (ed_cnt == 1) {
            if (databyte != 0xED) {
                MemESP::writebyte(memoff + memidx++, 0xED);
                MemESP::writebyte(memoff + memidx++, databyte);
                ed_cnt = 0;
            }
            else
                ed_cnt++;
        }
        else if (ed_cnt == 2) {
            repcnt = databyte;
            ed_cnt++;
        }
        else if (ed_cnt == 3) {
            repval = databyte;
            for (uint16_t i = 0; i < repcnt; i++)
                MemESP::writebyte(memoff + memidx++, repval);
            ed_cnt = 0;
        }
    }
}

void FileZ80::loadCompressedMemPage(FILE *f, uint16_t dataLen, uint8_t* memPage, uint16_t memlen)
{
    uint16_t dataOff = 0;
    uint8_t ed_cnt = 0;
    uint8_t repcnt = 0;
    uint8_t repval = 0;
    uint16_t memidx = 0;

    while(dataOff < dataLen && memidx < memlen) {
        uint8_t databyte = readByteFile(f);
        if (ed_cnt == 0) {
            if (databyte != 0xED)
                memPage[memidx++] = databyte;
            else
                ed_cnt++;
        }
        else if (ed_cnt == 1) {
            if (databyte != 0xED) {
                memPage[memidx++] = 0xED;
                memPage[memidx++] = databyte;
                ed_cnt = 0;
            }
            else
                ed_cnt++;
        }
        else if (ed_cnt == 2) {
            repcnt = databyte;
            ed_cnt++;
        }
        else if (ed_cnt == 3) {
            repval = databyte;
            for (uint16_t i = 0; i < repcnt; i++)
                memPage[memidx++] = repval;
            ed_cnt = 0;
        }
    }
}

void DoKeyboardLoad_48K() {

    ESPeccy::reset(); // Reset machine

    for (int i=0; i<125; i++) CPU::loop(); // Run 125 frames (about 1 1/4 seconds)

    bitWrite(Ports::port[6], 3, false); // Set J pressed
    CPU::loop();
    bitWrite(Ports::port[6], 3, true); // Set J released
    for (int i=0; i<2; i++) CPU::loop();

    bitWrite(Ports::port[7], 1, false); // Set SS pressed
    bitWrite(Ports::port[5], 0, false); // Set P pressed
    CPU::loop();
    bitWrite(Ports::port[5], 0, true); // Set P released
    for (int i=0; i<4; i++) CPU::loop();

    bitWrite(Ports::port[5], 0, false); // Set P pressed
    CPU::loop();
    bitWrite(Ports::port[5], 0, true); // Set P released
    bitWrite(Ports::port[7], 1, true); // Set SS released
    for (int i=0; i<4; i++) CPU::loop();

    bitWrite(Ports::port[6], 0, false); // Set ENTER pressed
    CPU::loop();
    bitWrite(Ports::port[6], 0, true); // Set ENTER released

}

void FileZ80::loader48() {
    DoKeyboardLoad_48K();
}

void DoKeyboardLoad_128K_Pentagon() {

    ESPeccy::reset(); // Reset machine

    for (int i=0; i<60; i++) CPU::loop(); // Run 100 frames (about 2 seconds)

    bitWrite(Ports::port[6], 0, false); // Set ENTER pressed for Speccy
    CPU::loop(); // Run 1 frame more for reading ENTER key
    bitWrite(Ports::port[6], 0, true); // Set ENTER released for Speccy

}

void DoKeyboardLoad_128K_es() {

    ESPeccy::reset(); // Reset machine

    for (int i=0; i<50; i++) CPU::loop(); // Run 50 frames (about 1/2 seconds)

    bitWrite(Ports::port[6], 1, false); // Set L pressed
    CPU::loop();
    bitWrite(Ports::port[6], 1, true); // Set L released
    for (int i=0; i<2; i++) CPU::loop();

    bitWrite(Ports::port[5], 1, false); // Set O pressed
    CPU::loop();
    bitWrite(Ports::port[5], 1, true); // Set O released
    for (int i=0; i<2; i++) CPU::loop();

    bitWrite(Ports::port[1], 0, false); // Set A pressed
    CPU::loop();
    bitWrite(Ports::port[1], 0, true); // Set A released
    for (int i=0; i<2; i++) CPU::loop();

    bitWrite(Ports::port[1], 2, false); // Set D pressed
    CPU::loop();
    bitWrite(Ports::port[1], 2, true); // Set D released
    for (int i=0; i<2; i++) CPU::loop();

    bitWrite(Ports::port[7], 1, false); // Set SS pressed
    bitWrite(Ports::port[5], 0, false); // Set P pressed
    CPU::loop();
    bitWrite(Ports::port[5], 0, true); // Set P released
    for (int i=0; i<4; i++) CPU::loop();

    bitWrite(Ports::port[5], 0, false); // Set P pressed
    CPU::loop();
    bitWrite(Ports::port[5], 0, true); // Set P released
    bitWrite(Ports::port[7], 1, true); // Set SS released
    for (int i=0; i<4; i++) CPU::loop();

    bitWrite(Ports::port[6], 0, false); // Set ENTER pressed
    CPU::loop();
    bitWrite(Ports::port[6], 0, true); // Set ENTER released

}

void DoKeyboardLoad_ZX81() {

    ESPeccy::reset(); // Reset machine

    for (int i=0; i<15; i++) CPU::loop(); // Run 15 frames for ZX81+ ROM to load

    for (int i=0; i<4; i++) {
        bitWrite(Ports::port[0], 0, false); // Set CAPS SHIFT pressed
        bitWrite(Ports::port[4], 4, false); // Set 6 pressed
        CPU::loop();
        bitWrite(Ports::port[0], 0, true); // Set CAPS SHIFT released
        bitWrite(Ports::port[4], 4, true); // Set 6 released
        CPU::loop();
    }

    bitWrite(Ports::port[6], 0, false); // Set ENTER pressed
    CPU::loop();
    bitWrite(Ports::port[6], 0, true); // Set ENTER released
    CPU::loop();

    for (int i=0; i < 142; i++) CPU::loop(); // Run 142 frames for ZX81+ to start emulation

    bitWrite(Ports::port[7], 2, false); // Set M pressed (PAUSE command)
    for (int i=0; i<2; i++) CPU::loop();
    bitWrite(Ports::port[7], 2, true); // Set M released
    for (int i=0; i<4; i++) CPU::loop();

    bitWrite(Ports::port[3], 4, false); // Set 5 pressed
    for (int i=0; i<2; i++) CPU::loop();
    bitWrite(Ports::port[3], 4, true); // Set 5 released
    for (int i=0; i<4; i++) CPU::loop();

    for (int i=0; i<3; i++) {
        bitWrite(Ports::port[4], 0, false); // Set 0 pressed for Speccy
        for (int i=0; i<2; i++) CPU::loop();
        bitWrite(Ports::port[4], 0, true); // Set 0 released
        for (int i=0; i<4; i++) CPU::loop();
    }

    bitWrite(Ports::port[6], 0, false); // Set ENTER pressed
    for (int i=0; i<2; i++) CPU::loop();
    bitWrite(Ports::port[6], 0, true); // Set ENTER released
    for (int i=0; i<4; i++) CPU::loop();

    for (int i=0; i<22; i++) CPU::loop(); // Process PAUSE 5000 command

}


void FileZ80::loader128() {

    // unsigned char *z80_array;
    // uint32_t dataLen;

    if (Config::arch == "128K") {

        if (Config::romSet == "128K") {
            // z80_array = (unsigned char *) load128;
            // dataLen = sizeof(load128);
            DoKeyboardLoad_128K_Pentagon();
        } else if (Config::romSet == "128Kes") {
            // z80_array = (unsigned char *) load128spa;
            // dataLen = sizeof(load128spa);
            DoKeyboardLoad_128K_es();
        } else if (Config::romSet == "+2") {
            // z80_array = (unsigned char *) loadplus2;
            // dataLen = sizeof(loadplus2);
            DoKeyboardLoad_128K_Pentagon();
        } else if (Config::romSet == "+2es") {
            // z80_array = (unsigned char *) loadplus2;
            // dataLen = sizeof(loadplus2);
            DoKeyboardLoad_128K_Pentagon();
        } else if (Config::romSet == "+2fr") {
            // z80_array = (unsigned char *) loadplus2;
            // dataLen = sizeof(loadplus2);
            DoKeyboardLoad_128K_Pentagon();
        } else if (Config::romSet == "ZX81+") {
            // z80_array = (unsigned char *) loadzx81;
            // dataLen = sizeof(loadzx81);
            DoKeyboardLoad_ZX81();
        }

    } else if (Config::arch == "+2A") {

        DoKeyboardLoad_128K_Pentagon();

    } else if (Config::arch == "Pentagon") {

        // z80_array = (unsigned char *) loadpentagon;
        // dataLen = sizeof(loadpentagon);

        DoKeyboardLoad_128K_Pentagon();

    }

}

// ///////////////////////////////////////////////////////////////////////////////

bool FileP::load(string p_fn) {

    FILE *file;
    int p_size;

    file = fopen(p_fn.c_str(), "rb");
    if (file==NULL) {
        printf("FileP: Error opening %s\n",p_fn.c_str());
        return false;
    }

    fseek(file,0,SEEK_END);
    p_size = ftell(file);
    rewind (file);

    if (p_size > (0x4000 - 9)) {
        printf("FileP: Invalid .P file %s\n",p_fn.c_str());
        return false;
    }

    // Manage arch change
    if (Config::arch != "128K" || Config::romSet != "ZX81+") {
        Config::requestMachine("128K", "ZX81+");
        // Condition this to 50hz mode
        if(Config::videomode) {

            Config::SNA_Path = FileUtils::SNA_Path;
            Config::SNA_begin_row = FileUtils::fileTypes[DISK_SNAFILE].begin_row;
            Config::SNA_focus = FileUtils::fileTypes[DISK_SNAFILE].focus;
            Config::SNA_fdMode = FileUtils::fileTypes[DISK_SNAFILE].fdMode;
            Config::SNA_fileSearch = FileUtils::fileTypes[DISK_SNAFILE].fileSearch;

            Config::TAP_Path = FileUtils::TAP_Path;
            Config::TAP_begin_row = FileUtils::fileTypes[DISK_TAPFILE].begin_row;
            Config::TAP_focus = FileUtils::fileTypes[DISK_TAPFILE].focus;
            Config::TAP_fdMode = FileUtils::fileTypes[DISK_TAPFILE].fdMode;
            Config::TAP_fileSearch = FileUtils::fileTypes[DISK_TAPFILE].fileSearch;

            Config::DSK_Path = FileUtils::DSK_Path;
            Config::DSK_begin_row = FileUtils::fileTypes[DISK_DSKFILE].begin_row;
            Config::DSK_focus = FileUtils::fileTypes[DISK_DSKFILE].focus;
            Config::DSK_fdMode = FileUtils::fileTypes[DISK_DSKFILE].fdMode;
            Config::DSK_fileSearch = FileUtils::fileTypes[DISK_DSKFILE].fileSearch;

            Config::ram_file = p_fn;
            Config::save();
            OSD::esp_hard_reset();
        }
    }

    FileZ80::loader128();

    // Read data and copy to address 16393
    fread(&MemESP::ramCurrent[1][0x9], p_size, 1, file);

    //uint16_t address = 16393;
    //uint8_t page = address >> 14;
    //fread(&MemESP::ramCurrent[page][address & 0x3fff], p_size, 1, file);

    fclose(file);

    return true;

}

size_t FileZ80::saveCompressedMemData(FILE *f, uint16_t memoff, uint16_t memlen, bool onlygetsize) {
    size_t size = 0;
    uint16_t memidx = 0;

    while(memidx < memlen) {
        uint8_t byte = MemESP::readbyte(memoff + memidx);
        uint16_t repcnt = 1;

        // Contar repeticiones del mismo byte para aplicar RLE
        while ((memidx + repcnt < memlen) && (repcnt < 255) && (MemESP::readbyte(memoff + memidx + repcnt) == byte)) {
            repcnt++;
        }

        if (repcnt > 4 || (byte == 0xED && repcnt >= 2)) {
            // Aplicar compresión RLE para más de 4 repeticiones o si el byte es 0xED
            if (!onlygetsize) {
                writeByteFile(0xED, f);
                writeByteFile(0xED, f);
                writeByteFile(repcnt, f);
                writeByteFile(byte, f);
            }
            size+=4;
        } else {
            // No aplicar compresión si no hay suficientes repeticiones
            for (uint16_t i = 0; i < repcnt; i++) {
                if (!onlygetsize) writeByteFile(byte, f);
                size++;
                // Si el byte es 0xED, el siguiente byte no debe estar en un bloque de repetición
                // En el caso de 0xED, repcnt = 1, asi que este codigo ocupa el caso de 0xED individuales seguidos de un bloque de repeticion (o un 0xED al final del bloque)
                if (byte == 0xED && memidx + i + 1 < memlen) {
                    if (!onlygetsize) writeByteFile(MemESP::readbyte(memoff + memidx + i + 1), f);
                    size++;
                    memidx++;
                }
            }
        }

        memidx += repcnt;
    }

    return size;
}

size_t FileZ80::saveCompressedMemPage(FILE *f, uint8_t* memPage, uint16_t memlen, bool onlygetsize) {
    size_t size = 0;
    uint16_t memidx = 0;

    while(memidx < memlen) {
        uint8_t byte = memPage[memidx];
        uint16_t repcnt = 1;

        // Contar repeticiones del mismo byte para aplicar RLE
        while ((memidx + repcnt < memlen) && (repcnt < 255) && (memPage[memidx + repcnt] == byte)) {
            repcnt++;
        }

        if (repcnt > 4 || (byte == 0xED && repcnt >= 2)) {
            // Aplicar compresión RLE para más de 4 repeticiones o si el byte es 0xED
            if (!onlygetsize) {
                writeByteFile(0xED, f);
                writeByteFile(0xED, f);
                writeByteFile(repcnt, f);
                writeByteFile(byte, f);
            }
            size+=4;
        } else {
            // No aplicar compresión si no hay suficientes repeticiones
            for (uint16_t i = 0; i < repcnt; i++) {
                if (!onlygetsize) writeByteFile(byte, f);
                size++;
                // Si el byte es 0xED, el siguiente byte no debe estar en un bloque de repetición
                // En el caso de 0xED, repcnt = 1, asi que este codigo ocupa el caso de 0xED individuales seguidos de un bloque de repeticion (o un 0xED al final del bloque)
                if (byte == 0xED && memidx + i + 1 < memlen) {
                    if (!onlygetsize) writeByteFile(memPage[memidx + i + 1], f);
                    size++;
                    memidx++;
                }
            }
        }

        memidx += repcnt;
    }

    return size;
}



bool FileZ80::save(string z80_fn, bool force_saverom) {

    FILE *file;

    file = fopen(z80_fn.c_str(), "wb");
    if (file==NULL)
    {
        printf("FileZ80: Error opening %s for writing",z80_fn.c_str());
        return false;
    }

    OSD::progressDialog(OSD_PSNA_SAVING,OSD_PLEASE_WAIT[Config::lang],0,0,true);

    // write registers
/*
    Offset  Length  Description
    ---------------------------
    0       1       A register
    1       1       F register
    2       2       BC register pair (LSB, i.e. C, first)
    4       2       HL register pair
    6       2       Program counter
    8       2       Stack pointer
    10      1       Interrupt register
    11      1       Refresh register (Bit 7 is not significant!)
    12      1       Bit 0  : Bit 7 of the R-register
                    Bit 1-3: Border colour
                    Bit 4  : 1=Basic SamRom switched in
                    Bit 5  : 1=Block of data is compressed
                    Bit 6-7: No meaning
    13      2       DE register pair
    15      2       BC' register pair
    17      2       DE' register pair
    19      2       HL' register pair
    21      1       A' register
    22      1       F' register
    23      2       IY register (Again LSB first)
    25      2       IX register
    27      1       Interrupt flipflop, 0=DI, otherwise EI
    28      1       IFF2 (not particularly important...)
    29      1       Bit 0-1: Interrupt mode (0, 1 or 2)
                    Bit 2  : 1=Issue 2 emulation
                    Bit 3  : 1=Double interrupt frequency
                    Bit 4-5: 1=High video synchronisation
                             3=Low video synchronisation
                             0,2=Normal
                    Bit 6-7: 0=Cursor/Protek/AGF joystick
                             1=Kempston joystick
                             2=Sinclair 2 Left joystick (or user
                               defined, for version 3 .z80 files)
                             3=Sinclair 2 Right joystick
*/

    writeByteFile(Z80::getRegA(), file);
    writeByteFile(Z80::getFlags(), file);

    writeByteFile(Z80::getRegC(), file);
    writeByteFile(Z80::getRegB(), file);

    writeByteFile(Z80::getRegL(), file);
    writeByteFile(Z80::getRegH(), file);

    writeWordFileLE(0, file);
    writeWordFileLE(Z80::getRegSP(), file);

    writeByteFile(Z80::getRegI(), file);

    writeByteFile(Z80::getRegR(), file);

    // Z80::setRegR  (       header[11]);
    uint8_t b12 = ( Z80::getRegR() & 0x80 ) ? 0x01 : 0;
    b12 |= ( VIDEO::borderColor & 0x07 ) << 1;
    writeByteFile(b12, file);

    writeByteFile(Z80::getRegE(), file);
    writeByteFile(Z80::getRegD(), file);

    writeByteFile(Z80::getRegCx(), file);
    writeByteFile(Z80::getRegBx(), file);

    writeByteFile(Z80::getRegEx(), file);
    writeByteFile(Z80::getRegDx(), file);

    writeByteFile(Z80::getRegLx(), file);
    writeByteFile(Z80::getRegHx(), file);

    writeByteFile(Z80::getRegAx(), file);
    writeByteFile(Z80::getRegFx(), file);

    writeWordFileLE(Z80::getRegIY(), file);
    writeWordFileLE(Z80::getRegIX(), file);

    writeByteFile(Z80::isIFF1() ? 1 : 0, file);
    writeByteFile(Z80::isIFF2() ? 1 : 0, file);

    uint8_t b29 = ( (uint8_t) Z80::getIM() ) & 0x03;
    b29 |= ((Z80Ops::is48) && (Config::Issue2)) ? 0x04 : 0;

    writeByteFile(b29, file);

    // additional header
/*
    Offset  Length  Description
    ---------------------------
  * 30      2       Length of additional header block (see below)
  * 32      2       Program counter
  * 34      1       Hardware mode (see below)
  * 35      1       If in SamRam mode, bitwise state of 74ls259.
                    For example, bit 6=1 after an OUT 31,13 (=2*6+1)
                    If in 128 mode, contains last OUT to 0x7ffd
        If in Timex mode, contains last OUT to 0xf4
  * 36      1       Contains 0xff if Interface I rom paged
        If in Timex mode, contains last OUT to 0xff
  * 37      1       Bit 0: 1 if R register emulation on
                    Bit 1: 1 if LDIR emulation on
        Bit 2: AY sound in use, even on 48K machines
        Bit 6: (if bit 2 set) Fuller Audio Box emulation
        Bit 7: Modify hardware (see below)
  * 38      1       Last OUT to port 0xfffd (soundchip register number)
  * 39      16      Contents of the sound chip registers
    55      2       Low T state counter
    57      1       Hi T state counter
    58      1       Flag byte used by Spectator (QL spec. emulator)
                    Ignored by Z80 when loading, zero when saving
    59      1       0xff if MGT Rom paged
    60      1       0xff if Multiface Rom paged. Should always be 0.
    61      1       0xff if 0-8191 is ROM, 0 if RAM
    62      1       0xff if 8192-16383 is ROM, 0 if RAM
    63      10      5 x keyboard mappings for user defined joystick
    73      10      5 x ASCII word: keys corresponding to mappings above
    83      1       MGT type: 0=Disciple+Epson,1=Disciple+HP,16=Plus D
    84      1       Disciple inhibit button status: 0=out, 0ff=in
    85      1       Disciple inhibit flag: 0=rom pageable, 0ff=not
 ** 86      1       Last OUT to port 0x1ffd
*/
    writeWordFileLE(55, file); // Version 3                                         // off: 30

    writeWordFileLE(Z80::getRegPC(), file);                                         // off: 32

    uint8_t mch = 0;
    if ( Z80Ops::is48 ) {
        mch = 0 ;
    } else
    if ( Z80Ops::is128 ) {
        if (Config::romSet == "+2" || Config::romSet == "+2es" || Config::romSet == "+2fr") {
            mch = 12 ; // Spectrum +2
        } else {
            mch = 4 ;
        }
    } else
    if ( Z80Ops::isPentagon ) {
        mch = 9;
    }
    writeByteFile(mch, file);                                                       // off: 34

    // write memESP bank control port
    uint8_t tmp_port = 0;
    if (Z80Ops::is128 || Z80Ops::isPentagon) {
        tmp_port = MemESP::bankLatch & 0x07;
        bitWrite(tmp_port, 3, MemESP::videoLatch);
        bitWrite(tmp_port, 4, MemESP::romLatch);
        bitWrite(tmp_port, 5, MemESP::pagingLock);
    }
    writeByteFile(tmp_port, file);                                                  // off: 35

    // TODO
    writeByteFile(0, file);                                                         // off: 36
    writeByteFile(0, file);                                                         // off: 37
    writeByteFile(0, file);                                                         // off: 38
    writeByteFile(0, file);                                                         // off: 39-54
        writeByteFile(0, file);                                                     // off: 40
        writeByteFile(0, file);                                                     // off: 41
        writeByteFile(0, file);                                                     // off: 42
        writeByteFile(0, file);                                                     // off: 43
        writeByteFile(0, file);                                                     // off: 44
        writeByteFile(0, file);                                                     // off: 45
        writeByteFile(0, file);                                                     // off: 46
        writeByteFile(0, file);                                                     // off: 47
        writeByteFile(0, file);                                                     // off: 48
        writeByteFile(0, file);                                                     // off: 49
        writeByteFile(0, file);                                                     // off: 50
        writeByteFile(0, file);                                                     // off: 51
        writeByteFile(0, file);                                                     // off: 52
        writeByteFile(0, file);                                                     // off: 53
        writeByteFile(0, file);                                                     // off: 54
    // TODO
    writeByteFile(0, file);                                                         // off: 55-86
        writeByteFile(0, file);                                                     // off: 56
        writeByteFile(0, file);                                                     // off: 57
        writeByteFile(0, file);                                                     // off: 58
        writeByteFile(0, file);                                                     // off: 59
        writeByteFile(0, file);                                                     // off: 60
        writeByteFile(0, file);                                                     // off: 61
        writeByteFile(0, file);                                                     // off: 62
        writeByteFile(0, file);                                                     // off: 63
        writeByteFile(0, file);                                                     // off: 64
        writeByteFile(0, file);                                                     // off: 65
        writeByteFile(0, file);                                                     // off: 66
        writeByteFile(0, file);                                                     // off: 67
        writeByteFile(0, file);                                                     // off: 68
        writeByteFile(0, file);                                                     // off: 69
        writeByteFile(0, file);                                                     // off: 70
        writeByteFile(0, file);                                                     // off: 71
        writeByteFile(0, file);                                                     // off: 72
        writeByteFile(0, file);                                                     // off: 73
        writeByteFile(0, file);                                                     // off: 74
        writeByteFile(0, file);                                                     // off: 75
        writeByteFile(0, file);                                                     // off: 76
        writeByteFile(0, file);                                                     // off: 77
        writeByteFile(0, file);                                                     // off: 78
        writeByteFile(0, file);                                                     // off: 79
        writeByteFile(0, file);                                                     // off: 80
        writeByteFile(0, file);                                                     // off: 81
        writeByteFile(0, file);                                                     // off: 82
        writeByteFile(0, file);                                                     // off: 83
        writeByteFile(0, file);                                                     // off: 84
        writeByteFile(0, file);                                                     // off: 85
        writeByteFile(0, file);                                                     // off: 86

    if (Z80Ops::is48) {
        uint16_t pageStart[12] = {0, 0, 0, 0, 0x8000, 0xC000, 0, 0, 0x4000, 0, 0};

        // z80 with rom
        if ( MemESP::rom[0] == MemESP::ram[7] || force_saverom ) {
            size_t dataLen = saveCompressedMemPage(NULL, MemESP::rom[0], 0x4000, true);
            if (dataLen >= 0x4000) {
                writeWordFileLE(0xffff, file); // no compress
                writeByteFile(0, file); // page
                for (int off = 0; off < 0x4000; ++off) {
                    writeByteFile(MemESP::rom[0][off], file);
                }
            } else {
                writeWordFileLE(dataLen, file); // compressed size
                writeByteFile(0, file); // page
                saveCompressedMemPage(file, MemESP::rom[0], 0x4000, false);
            }
        }

        for (int i = 0; i < sizeof(pageStart)/sizeof(pageStart[0]); ++i) {
            if ( pageStart[i] ) {
                size_t dataLen = saveCompressedMemData(NULL, pageStart[i], 0x4000, true);
                if (dataLen >= 0x4000) {
                    writeWordFileLE(0xffff, file); // no compress
                    writeByteFile(i, file); // page
                    for (int off = 0; off < 0x4000; ++off) {
                        writeByteFile(MemESP::readbyte(pageStart[i]+off), file);
                    }
                } else {
                    writeWordFileLE(dataLen, file); // compressed size
                    writeByteFile(i, file); // page
                    saveCompressedMemData(file, pageStart[i], 0x4000, false);
                }
            }
        }
    } else {
        uint8_t* pages[12] = {
            0, 0, 0,
            MemESP::ram[0], MemESP::ram[1], MemESP::ram[2], MemESP::ram[3],
            MemESP::ram[4], MemESP::ram[5], MemESP::ram[6], MemESP::ram[7],
            0 };

        for (int i = 0; i < sizeof(pages)/sizeof(pages[0]); ++i) {
            if ( pages[i] ) {
                size_t dataLen = saveCompressedMemPage(NULL, pages[i], 0x4000, true);
                if (dataLen >= 0x4000) {
                    writeWordFileLE(0xffff, file); // no compress
                    writeByteFile(i, file); // page
                    for (int off = 0; off < 0x4000; ++off) {
                        writeByteFile(pages[i][off], file);
                    }
                } else {
                    writeWordFileLE(dataLen, file); // compressed size
                    writeByteFile(i, file); // page
                    saveCompressedMemPage(file, pages[i], 0x4000, false);
                }
            }
        }
    }

    fclose(file);

    OSD::progressDialog("","",0,2, false);

    return true;

}


// ///////////////////////////////////////////////////////////////////////////////

/*
      Ficheros *.SP:

Offset    Longitud    Descripci¢n
------   ----------  -------------------
  0       2 bytes    "SP" (53h, 50h) Signatura.
  2       1 palabra  Longitud del programa en bytes (el emulador actualmente
                     s¢lo genera programas de 49152 bytes)
  4       1 palabra  Posici¢n inicial del programa (el emulador actualmente
                     s¢lo genera programas que comiencen en la pos. 16384)
  6       1 palabra  Registro BC del Z80
  8       1 palabra  Registro DE del Z80
 10       1 palabra  Registro HL del Z80
 12       1 palabra  Registro AF del Z80
 14       1 palabra  Registro IX del Z80
 16       1 palabra  Registro IY del Z80
 18       1 palabra  Registro BC' del Z80
 20       1 palabra  Registro DE' del Z80
 22       1 palabra  Registro HL' del Z80
 24       1 palabra  Registro AF' del Z80
 26       1 byte     Registro R (de refresco) del Z80
 27       1 byte     Registro I (de interrupciones) del Z80
 28       1 palabra  Registro SP del Z80
 30       1 palabra  Registro PC del Z80
 32       1 palabra  Reservada para uso futuro, siempre 0
 34       1 byte     Color del borde al comenzar
 35       1 byte     Reservado para uso futuro, siempre 0
 36       1 palabra  Palabra de estado codificada por bits. Formato:

                     Bit     Descripci¢n
                     ---     -----------
                     15-8    Reservados para uso futuro
                     7-6     Reservados para uso interno, siempre 0
                     5       Estado del Flash: 0 - tinta INK, papel PAPER
                                               1 - tinta PAPER, papel INK
                     4       Interrupci¢n pendiente de ejecutarse
                     3       Reservado para uso futuro
                     2       Biestable IFF2 (uso interno)
                     1       Modo de interrupci¢n: 0=IM1; 1=IM2
                     0       Biestable IFF1 (estado de interrupci¢n):
                                 0 - Interrupciones desactivadas (DI)
                                 1 - Interrupciones activadas (EI)
*/

bool FileSP::load(string sp_fn) {

    FILE *file;

    file = fopen(sp_fn.c_str(), "rb");
    if (!file) {
        printf("FileSP: Error opening %s\n",sp_fn.c_str());
        return false;
    }

    uint8_t sign[2] = { 0, 0 };
    sign[0] = readByteFile(file);
    sign[1] = readByteFile(file);
    if ( sign[0] != 'S' && sign[1] != 'P' ) {
        printf("FileSP: invalid format %s\n",sp_fn.c_str());
        fclose(file);
        return false;
    }

    // data size
    uint16_t dataSize = readWordFileLE(file);
    // start address
    uint16_t startAddress = readWordFileLE(file);
    if ( ( dataSize != 0 && dataSize != 49152 ) || ( startAddress != 0 && startAddress != 16384 ) ) {
        printf("FileSP: invalid format %s\n",sp_fn.c_str());
        fclose(file);
        return false;
    }

    // Change arch if needed
    if (!Z80Ops::is48) {

        bool vreset = Config::videomode;

        Config::requestMachine("48K", "");

        // Condition this to 50hz mode
        if(vreset) {
            Config::SNA_Path = FileUtils::SNA_Path;
            Config::SNA_begin_row = FileUtils::fileTypes[DISK_SNAFILE].begin_row;
            Config::SNA_focus = FileUtils::fileTypes[DISK_SNAFILE].focus;
            Config::SNA_fdMode = FileUtils::fileTypes[DISK_SNAFILE].fdMode;
            Config::SNA_fileSearch = FileUtils::fileTypes[DISK_SNAFILE].fileSearch;

            Config::TAP_Path = FileUtils::TAP_Path;
            Config::TAP_begin_row = FileUtils::fileTypes[DISK_TAPFILE].begin_row;
            Config::TAP_focus = FileUtils::fileTypes[DISK_TAPFILE].focus;
            Config::TAP_fdMode = FileUtils::fileTypes[DISK_TAPFILE].fdMode;
            Config::TAP_fileSearch = FileUtils::fileTypes[DISK_TAPFILE].fileSearch;

            Config::DSK_Path = FileUtils::DSK_Path;
            Config::DSK_begin_row = FileUtils::fileTypes[DISK_DSKFILE].begin_row;
            Config::DSK_focus = FileUtils::fileTypes[DISK_DSKFILE].focus;
            Config::DSK_fdMode = FileUtils::fileTypes[DISK_DSKFILE].fdMode;
            Config::DSK_fileSearch = FileUtils::fileTypes[DISK_DSKFILE].fileSearch;

            Config::ram_file = sp_fn;
            Config::save();
            OSD::esp_hard_reset();
        }

    }

    ESPeccy::reset();

    // Read in the registers
    Z80::setRegBC(readWordFileLE(file));
    Z80::setRegDE(readWordFileLE(file));
    Z80::setRegHL(readWordFileLE(file));

    Z80::setFlags(readByteFile(file));
    Z80::setRegA(readByteFile(file));

    Z80::setRegIX(readWordFileLE(file));
    Z80::setRegIY(readWordFileLE(file));

    Z80::setRegBCx(readWordFileLE(file));
    Z80::setRegDEx(readWordFileLE(file));
    Z80::setRegHLx(readWordFileLE(file));

    Z80::setRegFx(readByteFile(file));
    Z80::setRegAx(readByteFile(file));

    Z80::setRegR(readByteFile(file));

    Z80::setRegI(readByteFile(file));

    Z80::setRegSP(readWordFileLE(file));
    Z80::setRegPC(readWordFileLE(file));

    readWordFileLE(file);

    VIDEO::borderColor = readByteFile(file);

    readByteFile(file);

    // flags
    uint16_t flags = readWordFileLE(file);
    Z80::setIFF2(flags & 0x04 ? true : false);
    Z80::setIM(flags & 0x02 ? Z80::IM2 : Z80::IM1);
    Z80::setIFF1(flags & 0x01 ? true : false);

    VIDEO::brd = VIDEO::border32[VIDEO::borderColor];

    MemESP::romLatch = 0;
    MemESP::romInUse = 0;
    MemESP::bankLatch = 0;
    MemESP::pagingLock = 1;
    MemESP::videoLatch = 0;

    // read ROM page if present
    if (!startAddress && !dataSize) {
        readBlockFile(file, MemESP::ram[7], 0x4000);
        MemESP::ramCurrent[0] = MemESP::rom[0] = MemESP::ram[7];
    }

    // read 48K memory
    readBlockFile(file, MemESP::ram[5], 0x4000);
    readBlockFile(file, MemESP::ram[2], 0x4000);
    readBlockFile(file, MemESP::ram[0], 0x4000);

    fclose(file);

    return true;

}

bool FileSP::save(string sp_fn, bool force_saverom) {

    // this format is only for 48k
    if (Z80Ops::is128 || Z80Ops::isPentagon)
    // if (Config::arch != "48K" && Config::arch != "TK90X" && Config::arch != "TK95")
        return false;

    FILE *file;

    file = fopen(sp_fn.c_str(), "wb");
    if (file==NULL)
    {
        printf("FileSP: Error opening %s for writing",sp_fn.c_str());
        return false;
    }

    OSD::progressDialog(OSD_PSNA_SAVING,OSD_PLEASE_WAIT[Config::lang],0,0,true);

    // write signature
    uint8_t sign[2] = { 'S', 'P' };
    writeByteFile(sign[0], file);
    writeByteFile(sign[1], file);

    if ( MemESP::rom[0] == MemESP::ram[7] || force_saverom ) {
        // data size
        writeWordFileLE(0, file);

        // start address
        writeWordFileLE(0, file);
    } else {
        // data size
        writeWordFileLE(49152, file);

        // start address
        writeWordFileLE(16384, file);
    }

    // write registers

    writeWordFileLE(Z80::getRegBC(), file);
    writeWordFileLE(Z80::getRegDE(), file);
    writeWordFileLE(Z80::getRegHL(), file);

    writeByteFile(Z80::getFlags(), file);
    writeByteFile(Z80::getRegA(), file);

    writeWordFileLE(Z80::getRegIX(), file);
    writeWordFileLE(Z80::getRegIY(), file);

    writeWordFileLE(Z80::getRegBCx(), file);
    writeWordFileLE(Z80::getRegDEx(), file);
    writeWordFileLE(Z80::getRegHLx(), file);

    writeByteFile(Z80::getRegFx(), file);
    writeByteFile(Z80::getRegAx(), file);

    writeByteFile(Z80::getRegR(), file);

    writeByteFile(Z80::getRegI(), file);

    writeWordFileLE(Z80::getRegSP(), file);
    writeWordFileLE(Z80::getRegPC(), file);

    writeWordFileLE(0, file);

    writeByteFile(VIDEO::borderColor & 0x07, file);

    writeByteFile(0, file);

    // write memESP bank control port
    uint16_t tmp_port = 0;
    bitWrite(tmp_port, 2, Z80::isIFF2());
    if ( Z80::getIM() == Z80::IM2 ) bitWrite(tmp_port, 1, 1); // IM0 ???
    bitWrite(tmp_port, 0, Z80::isIFF1());
    writeWordFileLE(tmp_port, file);

    if ( MemESP::rom[0] == MemESP::ram[7] || force_saverom ) {
        writeBlockFile(file, MemESP::rom[0], 0x4000);
    }

    writeBlockFile(file, MemESP::ram[5], 0x4000);
    writeBlockFile(file, MemESP::ram[2], 0x4000);
    writeBlockFile(file, MemESP::ram[0], 0x4000);

    fclose(file);

    OSD::progressDialog("","",0,2, false);

    return true;

}
