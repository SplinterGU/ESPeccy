/*
ESPeccy - Sinclair ZX Spectrum emulator for the Espressif ESP32 SoC

Copyright (c) 2024 Juan José Ponteprino [SplinterGU]
https://github.com/SplinterGU/ESPeccy

This file is part of ESPeccy.

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


#include "ROMLoad.h"
#include "FileUtils.h"
#include "Config.h"
#include "cpuESP.h"
#include "Video.h"
#include "MemESP.h"
#include "ESPeccy.h"
#include "messages.h"
#include "OSD.h"
#include "Tape.h"
#include "Z80_JLS/z80.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include <stdio.h>
#include <inttypes.h>
#include <string>

using namespace std;

bool ROMLoad::load(string rom_fn, bool reset) {

    FILE *file;

    file = fopen(rom_fn.c_str(), "rb");
    if (!file) {
        printf("ROM Load: Error opening %s\n", rom_fn.c_str());
        return false;
    }

    fseek(file,0,SEEK_END);
    int rom_size = ftell(file);
    rewind (file);

    if (rom_size != 16384) {
        printf("ROM Load: invalid rom file %s\n",rom_fn.c_str());
        fclose(file);
        return false;
    }

    // Change arch if needed (only 48k)
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

            Config::ram_file = NO_RAM_FILE;
            Config::last_ram_file = NO_RAM_FILE;

            Config::rom_file = rom_fn;

            Config::save();
            OSD::esp_hard_reset();
        }

    }

    if (reset) ESPeccy::reset();

    // read ROM page if present
    readBlockFile(file, MemESP::ram[7], 0x4000);
    MemESP::ramCurrent[0] = MemESP::rom[0] = MemESP::ram[7];

    fclose(file);

    return true;

}

