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


#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include <stdio.h>
#include <inttypes.h>
#include <string>

using namespace std;

bool LoadSnapshot(string filename, string force_arch, string force_romset, uint8_t force_ALU);
bool SaveSnapshot(string filename, bool force_saverom = false);

string getSnapshotCheatPath(const string& path);

class FileSNA
{
public:
    static bool load(string sna_fn, string force_arch, string force_romset, uint8_t force_ALU);
    static bool save(string sna_fn, bool force_saverom = false);
    static bool save(string sna_fn, bool blockMode, bool force_saverom);
    static bool isStateAvailable(string filename);
};

class FileZ80
{
public:
    static bool load(string z80_fn);
    static void loader48();
    static void loader128();
//    static bool keepArch;
    static bool save(string z80_fn, bool force_saverom = false);

private:
    static void loadCompressedMemData(FILE *f, uint16_t dataLen, uint16_t memStart, uint16_t memlen);
    static void loadCompressedMemPage(FILE *f, uint16_t dataLen, uint8_t* memPage, uint16_t memlen);
    static size_t saveCompressedMemData(FILE *f, uint16_t memoff, uint16_t memlen, bool onlygetsize);
    static size_t saveCompressedMemPage(FILE *f, uint8_t* memPage, uint16_t memlen, bool onlygetsize);
};

class FileP
{
public:
    static bool load(string p_fn);
};

class FileSP
{
public:
    static bool load(string sp_fn);
    static bool save(string sp_fn, bool force_saverom = false);
};

#endif
