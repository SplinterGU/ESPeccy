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


#ifndef FileUtils_h
#define FileUtils_h

#include <stdio.h>
#include <inttypes.h>
#include <string>
#include <vector>
#include <unordered_map>

#include "sdmmc_cmd.h"

using namespace std;

#include "MemESP.h"

// Defines
#define ASCII_NL 10

enum {
    DISK_ALLFILE = 0,
    DISK_SNAFILE,
    DISK_TAPFILE,
    DISK_DSKFILE,
    DISK_ROMFILE,
    DISK_ESPFILE,
    DISK_CHTFILE,
    DISK_SCRFILE,
    DISK_UPGFILE,
    DISK_KBDFILE
};

#define DISK_DIR    0x80

struct DISK_FTYPE {
    string fileExts;
    string indexFilename;
    int begin_row;
    int focus;
    uint8_t fdMode;
    string fileSearch;
};

class FileUtils
{
public:
    static string getLCaseExt(const string& filename);

    static int getFileType(const std::string& filename) {
        static const std::unordered_map<std::string, int> extensionMap = {
            {"sna", DISK_SNAFILE}, {"z80", DISK_SNAFILE},
            {"sp",  DISK_SNAFILE}, {"p",   DISK_SNAFILE},
            {"tap", DISK_TAPFILE}, {"tzx", DISK_TAPFILE},
            {"trd", DISK_DSKFILE}, {"scl", DISK_DSKFILE},
            {"bin", DISK_ROMFILE}, {"rom", DISK_ROMFILE},
            {"esp", DISK_ESPFILE},
            {"pok", DISK_CHTFILE},
            {"scr", DISK_SCRFILE},
            {"upg", DISK_UPGFILE},
            {"kbd", DISK_KBDFILE}
        };

        std::string ext = getLCaseExt(filename);
        auto it = extensionMap.find(ext);
        return (it != extensionMap.end()) ? it->second : -1;
    }

    static string getFileNameWithoutExt(const string& file);

    static size_t fileSize(const char * mFile);

    static void initFileSystem();
    static bool mountSDCard(int PIN_MISO, int PIN_MOSI, int PIN_CLK, int PIN_CS);
    static void unmountSDCard();

    static bool isMountedSDCard();
    static bool isSDReady();

    static int getDirStats(const string& filedir, const vector<string>& filexts, unsigned long* hash, unsigned int* elements, unsigned int* ndirs);
    static void DirToFile(string Dir, uint8_t ftype /*string fileExts*/, unsigned long hash, unsigned int item_count);
    static bool hasExtension(string filename, string extension);

    static void deleteFilesWithExtension(const char *folder_path, const char *extension);

    static string getResolvedPath(const string& path);
    static string createTmpDir();

    static string MountPoint;
    static bool SDReady;

    static string ALL_Path; // Current ALL path on the SD
    static string SNA_Path; // Current SNA path on the SD
    static string TAP_Path; // Current TAP path on the SD
    static string DSK_Path; // Current DSK path on the SD
    static string ROM_Path; // Current ROM path on the SD
    static string ESP_Path; // Current ROM path on the SD
    static string CHT_Path; // Current POK path on the SD
    static string SCR_Path; // Current SCR path on the SD
    static string UPG_Path; // Current UPG path on the SD
    static string KBD_Path; // Current UPG path on the SD

    static DISK_FTYPE fileTypes[];

private:
    friend class Config;
    static sdmmc_card_t *card;
};

#define MOUNT_POINT_SD "/sd"
#define DISK_SCR_DIR "/.c"
#define DISK_PSNA_DIR "/.p"
#define DISK_PSNA_FILE "persist"

#define NO_RAM_FILE "none"
#define NO_ROM_FILE "none"

#define SNA_48K_SIZE 49179
#define SNA_48K_WITH_ROM_SIZE 65563
#define SNA_128K_SIZE1 131103
#define SNA_128K_SIZE2 147487
#define SNA_2A3_SIZE1 131121

// Experimental values for PSRAM
#define DIR_CACHE_SIZE 256
#define DIR_CACHE_SIZE_OVERSCAN 256

// Values for no PSRAM
#define DIR_CACHE_SIZE_NO_PSRAM 32
#define DIR_CACHE_SIZE_OVERSCAN_NO_PSRAM 16

#define FILENAMELEN 128

#define SDCARD_HOST_MAXFREQ 19000

// inline utility functions for uniform access to file/memory
// and making it easy to to implement SNA/Z80 functions

static inline std::string basename(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    return (pos == std::string::npos) ? path : path.substr(pos + 1);
}


static inline uint8_t readByteFile(FILE *f) {
    uint8_t result;

    if (fread(&result, 1, 1, f) != 1)
        return -1;

    return result;
}

static inline uint16_t readWordFileLE(FILE *f) {
    uint8_t lo = readByteFile(f);
    uint8_t hi = readByteFile(f);
    return lo | (hi << 8);
}

static inline uint16_t readWordFileBE(FILE *f) {
    uint8_t hi = readByteFile(f);
    uint8_t lo = readByteFile(f);
    return lo | (hi << 8);
}

static inline size_t readBlockFile(FILE *f, uint8_t* dstBuffer, size_t size) {
    return fread(dstBuffer, size/*0x4000*/, 1, f) * size;
}

static inline void writeByteFile(uint8_t value, FILE *f) {
    fwrite(&value,1,1,f);
}

static inline void writeWordFileLE(uint16_t value, FILE *f) {
    uint8_t lo =  value       & 0xFF;
    uint8_t hi = (value >> 8) & 0xFF;
    fwrite(&lo,1,1,f);
    fwrite(&hi,1,1,f);
}

static inline size_t writeBlockFile(FILE *f, uint8_t* srcBuffer, size_t size) {
    return fwrite(srcBuffer, size/*0x4000*/, 1, f) * size;
}

// static inline void writeWordFileBE(uint16_t value, File f) {
//     uint8_t hi = (value >> 8) & 0xFF;
//     uint8_t lo =  value       & 0xFF;
//     f.write(hi);
//     f.write(lo);
// }

// static inline size_t writeBlockFile(uint8_t* srcBuffer, File f, size_t size) {
//     return f.write(srcBuffer, size);
// }

// static inline uint8_t readByteMem(uint8_t*& ptr) {
//     uint8_t value = *ptr++;
//     return value;
// }

// static inline uint16_t readWordMemLE(uint8_t*& ptr) {
//     uint8_t lo = *ptr++;
//     uint8_t hi = *ptr++;
//     return lo | (hi << 8);
// }

// static inline uint16_t readWordMemBE(uint8_t*& ptr) {
//     uint8_t hi = *ptr++;
//     uint8_t lo = *ptr++;
//     return lo | (hi << 8);
// }

// static inline size_t readBlockMem(uint8_t*& srcBuffer, uint8_t* dstBuffer, size_t size) {
//     memcpy(dstBuffer, srcBuffer, size);
//     srcBuffer += size;
//     return size;
// }

// static inline void writeByteMem(uint8_t value, uint8_t*& ptr) {
//     *ptr++ = value;
// }

// static inline void writeWordMemLE(uint16_t value, uint8_t*& ptr) {
//     uint8_t lo =  value       & 0xFF;
//     uint8_t hi = (value >> 8) & 0xFF;
//     *ptr++ = lo;
//     *ptr++ = hi;
// }

// static inline void writeWordMemBE(uint16_t value, uint8_t*& ptr) {
//     uint8_t hi = (value >> 8) & 0xFF;
//     uint8_t lo =  value       & 0xFF;
//     *ptr++ = hi;
//     *ptr++ = lo;
// }

// static inline size_t writeBlockMem(uint8_t* srcBuffer, uint8_t*& dstBuffer, size_t size) {
//     memcpy(dstBuffer, srcBuffer, size);
//     dstBuffer += size;
//     return size;
// }

#endif // FileUtils_h
