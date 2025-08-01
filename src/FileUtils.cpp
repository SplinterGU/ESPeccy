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


#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <algorithm>
#include "FileUtils.h"
#include "Config.h"
#include "cpuESP.h"
#include "MemESP.h"
#include "ESPeccy.h"
#include "hardpins.h"
#include "messages.h"
#include "OSD.h"
#include "roms.h"
#include "Video.h"
#include "ZXKeyb.h"

#include "esp_vfs.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "CharMem32.h"

using namespace std;

string FileUtils::MountPoint = MOUNT_POINT_SD; // Start with SD
bool FileUtils::SDReady = false;
sdmmc_card_t *FileUtils::card;

string FileUtils::ALL_Path = "/"; // Current path on the SD
string FileUtils::SNA_Path = "/"; // Current path on the SD
string FileUtils::TAP_Path = "/"; // Current path on the SD
string FileUtils::DSK_Path = "/"; // Current path on the SD
string FileUtils::ROM_Path = "/"; // Current path on the SD
string FileUtils::ESP_Path = "/.p/"; // Current path on the SD
string FileUtils::CHT_Path = "/"; // Current path on the SD
string FileUtils::SCR_Path = "/"; // Current path on the SD
string FileUtils::UPG_Path = "/"; // Current path on the SD
string FileUtils::KBD_Path = "/"; // Current path on the SD

DISK_FTYPE FileUtils::fileTypes[] = {
    {"sna,z80,sp,p,tap,tzx,trd,scl,bin,rom,esp,pok,scr", ".all.idx", 2,2,0,""},
    {"sna,z80,sp,p",".s",2,2,0,""},
    {"tap,tzx,",".t",2,2,0,""},
    {"trd,scl",".d",2,2,0,""},
    {"bin,rom",".r",2,2,0,""},
    {"esp",".e",2,2,0,""},
    {"pok",".c.idx",2,2,0,""},
    {"scr",".scr.idx",2,2,0,""},
    {"upg",".upg.idx",2,2,0,""},
    {"kbd",".kbd",2,2,0,""}
};

string toLower(const std::string& str) {
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(), ::tolower);
    return lowerStr;
}

// get extension in lowercase
string FileUtils::getLCaseExt(const string& filename) {
    size_t dotPos = filename.rfind('.'); // find the last dot position
    if (dotPos == string::npos) {
        return ""; // dot position don't found
    }

    // get the substring after dot
    string extension = filename.substr(dotPos + 1);

    // convert extension to lowercase
//    for (char& c : extension) {
//        c = ::tolower(static_cast<unsigned char>(c));
//    }

//    return extension;

    return toLower( extension );

}

string FileUtils::getFileNameWithoutExt(const string& file) {
    size_t lastSlash = file.find_last_of("/\\");
    size_t lastDot = file.find_last_of('.');
    if (lastDot == std::string::npos || (lastSlash != std::string::npos && lastDot < lastSlash)) {
        return file.substr(lastSlash + 1); // No have extension
    }
    return file.substr(lastSlash + 1, lastDot - lastSlash - 1);
}

size_t FileUtils::fileSize(const char * mFile) {
    struct stat stat_buf;
    if ( !mFile ) return -1;
    int status = stat(mFile, &stat_buf);
    if ( status == -1 || ! ( stat_buf.st_mode & S_IFREG ) ) return -1;
    return stat_buf.st_size;
}

void FileUtils::initFileSystem() {

    // Try to mount SD card on LILYGO TTGO VGA32 Board or ESPectrum Board
    if (!SDReady) SDReady = mountSDCard(PIN_NUM_MISO_LILYGO_ESPECTRUM,PIN_NUM_MOSI_LILYGO_ESPECTRUM,PIN_NUM_CLK_LILYGO_ESPECTRUM,PIN_NUM_CS_LILYGO_ESPECTRUM);

    // Try to mount SD card on Olimex ESP32-SBC-FABGL Board
    if ((!ZXKeyb::Exists) && (!SDReady)) SDReady = mountSDCard(PIN_NUM_MISO_SBCFABGL,PIN_NUM_MOSI_SBCFABGL,PIN_NUM_CLK_SBCFABGL,PIN_NUM_CS_SBCFABGL);

}

bool FileUtils::mountSDCard(int PIN_MISO, int PIN_MOSI, int PIN_CLK, int PIN_CS) {

    // Init SD Card
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 0
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2048, // 4000,
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH1);
    if (ret != ESP_OK) {
        printf("SD Card init: Failed to initialize bus.\n");
        vTaskDelay(20 / portTICK_PERIOD_MS);
        return false;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);

    sdspi_device_config_t slot_config =  {
    .host_id   = SDSPI_DEFAULT_HOST,
    .gpio_cs   = GPIO_NUM_13,
    .gpio_cd   = SDSPI_SLOT_NO_CD,
    .gpio_wp   = SDSPI_SLOT_NO_WP,
    .gpio_int  = GPIO_NUM_NC, \
    };
    slot_config.gpio_cs = (gpio_num_t) PIN_CS;
    slot_config.host_id = SPI2_HOST;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT_SD, &host, &slot_config, &mount_config, &FileUtils::card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            printf("Failed to mount filesystem.\n");
        } else {
            printf("Failed to initialize the card.\n");
        }
        spi_bus_free(SPI2_HOST);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        return false;
    }

    // This seems to fix problems when video framebuffer is too big (400x300 i.e.)
    host.max_freq_khz = SDCARD_HOST_MAXFREQ;
    host.set_card_clk(host.slot, SDCARD_HOST_MAXFREQ);

    printf("SD Host max freq: %d\n",host.max_freq_khz);

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    vTaskDelay(20 / portTICK_PERIOD_MS);

    return true;

}

void FileUtils::unmountSDCard() {
    // Unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT_SD, card);
    // //deinitialize the bus after all devices are removed
    spi_bus_free(SPI2_HOST);

    SDReady = false;
}

bool FileUtils::isMountedSDCard() {
    // Dirty SDCard mount detection
    DIR* dir = opendir("/sd");
    if ( !dir ) return false;
    struct dirent* de = readdir(dir);
    if ( !de && ( errno == EIO || errno == EBADF ) ) {
        closedir(dir);
        return false;
    }
    closedir(dir);
    return true;
}

bool FileUtils::isSDReady() {

    if ( FileUtils::SDReady && !FileUtils::isMountedSDCard() ) {
        FileUtils::unmountSDCard();
    }

    if ( !FileUtils::SDReady ) {
        FileUtils::initFileSystem();
        if ( !FileUtils::SDReady ) {
            OSD::osdCenteredMsg(ERR_FS_EXT_FAIL[Config::lang], LEVEL_ERROR);
            return false;
        }
    }

    return true;

}

int FileUtils::getDirStats(const string& filedir, const vector<string>& filexts, unsigned long* hash, unsigned int* elements, unsigned int* ndirs) {
    *hash = 0; // Name checksum variable
    unsigned long high;
    DIR* dir;
    struct dirent* de;

    *elements = 0;
    *ndirs = 0;

    string fdir = (filedir.back() == '/') ? filedir.substr(0, filedir.length() - 1) : filedir;
    if ((dir = opendir(fdir.c_str())) != nullptr) {
        while ((de = readdir(dir)) != nullptr) {
            string fname = de->d_name;
            if (de->d_type == DT_REG || de->d_type == DT_DIR) {
                if (fname.compare(0, 1, ".") != 0) {
                    if ((de->d_type == DT_DIR) || (std::find(filexts.begin(), filexts.end(), FileUtils::getLCaseExt(fname)) != filexts.end())) {
                        // Calculate name checksum
                        for (int i = 0; i < fname.length(); i++) {
                            *hash = (*hash << 4) + fname[i];
                            if (high = *hash & 0xF0000000) *hash ^= high >> 24;
                            *hash &= ~high;
                        }
                        if (de->d_type == DT_REG)
                            (*elements)++; // Count elements in dir
                        else if (de->d_type == DT_DIR)
                            (*ndirs)++;
                    }
                }
            }
        }
        closedir(dir);
        return 0;
    }

    return -1;
}

string FileUtils::getResolvedPath(const string& path) {
    char *resolved_path = NULL;
    if ((resolved_path = realpath(path.c_str(), resolved_path)) == NULL) {
        printf("Error resolving path\n");
        return "";
    }
    string ret = string(resolved_path);
    free(resolved_path);
    return ret;
}

string FileUtils::createTmpDir() {
    string tempDir = MountPoint + "/.tmp";

    // Verificar si el directorio ya existía
    struct stat info;

    // Crear el directorio si no existe
    if (!(stat(tempDir.c_str(), &info) == 0 && (info.st_mode & S_IFDIR))) {
        if (mkdir(tempDir.c_str(), 0755) != 0) {
            printf( "TMP directory creation failed\n" );
            return "";
        }
    }

    return tempDir;
}

#if 1
#define MAX_FILEXT_COUNT 16

void FileUtils::DirToFile(const string fpath, uint8_t ftype, unsigned long hash, unsigned int item_count) {
    FILE* fin = nullptr;
    FILE* fout = nullptr;

    char line[FILENAMELEN + 1];

    // Use line for avoid a new buffer
    strncpy(line, fpath.c_str(), sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    size_t flen = strlen(line);
    if (flen > 0 && line[flen - 1] == '/') line[flen - 1] = 0;

    char indexFile[256];
    snprintf(indexFile, sizeof(indexFile), "%s/%s", line, fileTypes[ftype].indexFilename.c_str());
    remove(indexFile);

    if (!item_count) {
        fout = fopen(indexFile, "wb");
        if (!fout) return;
        if (strcmp(line, MountPoint.c_str()) != 0) fprintf(fout, "  ..%*s\n", FILENAMELEN - 5, "");
        fprintf(fout, "%020lu", hash);
        fclose(fout);
        printf("total iterations 0\n");
        return;
    }

    char fname1[FILENAMELEN + 1] = "";
    char fname2[FILENAMELEN + 1] = "";
    char fnameLastSaved[FILENAMELEN + 1] = "";

    char filexts[MAX_FILEXT_COUNT][4];
    int extCount = 0;

    // Parse extensions a mano
    const char* extsrc = fileTypes[ftype].fileExts.c_str();
    const char* start = extsrc;
    while (*start && extCount < MAX_FILEXT_COUNT) {
        const char* end = strchr(start, ',');
        if (!end) end = start + strlen(start);
        size_t len = end - start;
        if (len >= sizeof(filexts[0])) len = sizeof(filexts[0]) - 1;
        strncpy(filexts[extCount], start, len);
        filexts[extCount][len] = 0;
        extCount++;
        start = *end ? end + 1 : end;
    }

    DIR* dir = opendir(line);
    if (!dir) {
        printf("Error opening %s\n", line);
        return;
    }

    OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_1[Config::lang], 0, 0);

    int items_processed = 0;
    struct dirent* de;

    OSD::elements = 0;
    OSD::ndirs = 0;

    bool readFile1 = false, readFile2 = true;
    bool eof1 = true, eof2 = false;
    bool holdFile2 = false;

    int n = 1;

    // line = fdir
    if (strcmp(line, MountPoint.c_str()) != 0) {
        strcpy(fname1, "  ..");
        eof1 = false;
    }

    char* tempDir = strdup(FileUtils::createTmpDir().c_str());
    if (!tempDir || !*tempDir) {
        printf("error createTmpDir\n");
        closedir(dir);
        OSD::progressDialog("", "", 0, 2);
        free(tempDir);
        return;
    }

    int bufferSize;
    if (Config::videomode < 2) {
        if (Config::psramsize > 0) {
            bufferSize = item_count > DIR_CACHE_SIZE ? DIR_CACHE_SIZE : item_count;  // Size of buffer to read and sort
        } else {
            bufferSize = item_count > DIR_CACHE_SIZE_NO_PSRAM ? DIR_CACHE_SIZE_NO_PSRAM : item_count;  // Size of buffer to read and sort
        }
    } else {
        if (Config::psramsize > 0) {
            bufferSize = item_count > DIR_CACHE_SIZE_OVERSCAN ? DIR_CACHE_SIZE_OVERSCAN : item_count;  // Size of buffer to read and sort
        } else {
            bufferSize = item_count > DIR_CACHE_SIZE_OVERSCAN_NO_PSRAM ? DIR_CACHE_SIZE_OVERSCAN_NO_PSRAM : item_count;  // Size of buffer to read and sort
        }
    }

    #define ITEM_SIZE ((FILENAMELEN + 1 + sizeof(uint32_t) - 1) & ~3)
    uint8_t *_buffer = (uint8_t *) heap_caps_malloc(bufferSize * ITEM_SIZE, MALLOC_CAP_32BIT);
    if (!_buffer) {
        printf("error buffer allocation\n");
        closedir(dir);
        OSD::progressDialog("", "", 0, 2);
        free(tempDir);
        return;
    }

    CharMem32 buffer(_buffer);

    #define BUFFER(i,o) (buffer[(i) * ITEM_SIZE + (o)])
    #define BUFFERSTRCASECMP(str1,str2,size)   buffer.strncasecmp((str1)*ITEM_SIZE,(str2)*ITEM_SIZE,size)

    int bufCount = 0;

    int iterations = 0;

    char tmpName[256];

    while (!eof2 || (fin && !feof(fin))) {
        fnameLastSaved[0] = '\0';
        holdFile2 = false;
        iterations++;
        snprintf(tmpName, sizeof(tmpName), "%s/%s.tmp.%d", tempDir, fileTypes[ftype].indexFilename.c_str(), n & 1);
        if (fout) fclose(fout);
        fout = fopen(tmpName, "wb");
        if (!fout) {
            printf("error create tmpName [%s]\n", tmpName);
            if (fin) fclose(fin);
            closedir(dir);
            OSD::progressDialog("", "", 0, 2);
            free(tempDir);
            heap_caps_free(_buffer);
            return;
        }
        setvbuf(fout, NULL, _IOFBF, 1024);

        while (1) {
            if (readFile1) {
                if (!fin || feof(fin)) eof1 = true;
                if (!eof1) {
                    size_t res = fread(line, 1, FILENAMELEN, fin);
                    line[FILENAMELEN] = 0;
                    if (!res || feof(fin)) eof1 = true;
                    else strncpy(fname1, line, FILENAMELEN);
                }
                readFile1 = false;
            }

            if (readFile2) {
                if (bufCount == 0) {
                    while (bufCount < bufferSize && (de = readdir(dir))) {
                        if (de->d_name[0] == '.') continue;
                        const char* ext = strrchr(de->d_name, '.');
                        bool match = false;
                        if (de->d_type == DT_DIR) {
                            BUFFER(bufCount, 0) = ' '; buffer.memmove(bufCount*ITEM_SIZE+1, de->d_name, FILENAMELEN - 1);
                            OSD::ndirs++;
                            match = true;
                        } else if (ext) {
                            for (int i = 0; i < extCount; ++i) {
                                if (strcasecmp(ext + 1, filexts[i]) == 0) {
                                    for(int n = 0; n < FILENAMELEN + 1; ++n) { BUFFER(bufCount,n) = de->d_name[n]; if (!de->d_name[n]) break; }
                                    //strncpy(BUFFER(bufCount), de->d_name, FILENAMELEN);
                                    OSD::elements++;
                                    match = true;
                                    break;
                                }
                            }
                        }
                        if (match) bufCount++;
                    }

                    // simple bubble sort
                    for (int i = 0; i < bufCount - 1; ++i) {
                        for (int j = i + 1; j < bufCount; ++j) {
                            if (BUFFERSTRCASECMP(i,j,FILENAMELEN) > 0) {
                                buffer.memmove(line, i*ITEM_SIZE, FILENAMELEN);
                                buffer.memmove(i*ITEM_SIZE, j*ITEM_SIZE, FILENAMELEN);
                                buffer.memmove(j*ITEM_SIZE, line, FILENAMELEN);
                            }
                        }
                    }
                }

                if (bufCount > 0) {
                    buffer.memmove(fname2, 0, FILENAMELEN);
                    for (int i = 1; i < bufCount; ++i) {
                        buffer.memmove((i-1)*ITEM_SIZE, i*ITEM_SIZE, FILENAMELEN);
                    }
                    bufCount--;
                    items_processed++;
                    OSD::progressDialog("", "", (float)100 / ((float)item_count / (float)items_processed), 1);
                } else if (!de) eof2 = true;

                readFile2 = false;
                holdFile2 = false;
            }

            line[0] = '\0'; // used for fnameToSave
            line[FILENAMELEN] = '\0';
            if (eof1) {
                if (eof2 || holdFile2 || strcasecmp(fnameLastSaved, fname2) > 0) break;
                strncpy(line, fname2, FILENAMELEN);
                readFile2 = true;
            } else if (eof2 || strcasecmp(fname1, fname2) < 0) {
                strncpy(line, fname1, FILENAMELEN);
                readFile1 = true;
            } else if (strcasecmp(fname1, fname2) > 0 && strcasecmp(fnameLastSaved, fname2) > 0) {
                holdFile2 = true;
                strncpy(line, fname1, FILENAMELEN);
                readFile1 = true;
            } else {
                if (strcasecmp(fnameLastSaved, fname2) > 0) break;
                strncpy(line, fname2, FILENAMELEN);
                readFile2 = true;
            }

            if (*line) {
                strncpy(fnameLastSaved, line, FILENAMELEN);
                fnameLastSaved[FILENAMELEN] = '\0';
                bool clean_buffer = false;
                for(int i = 0; i < FILENAMELEN - 1; i++) {
                    if (!clean_buffer && !line[i]) clean_buffer = true;
                    line[i] = clean_buffer ? ' ' : line[i];
                }
                line[FILENAMELEN-1] = '\n';
                fwrite(line, 1, FILENAMELEN, fout);
            }
        }

        if (fin) fclose(fin);
        fclose(fout);
        fin = nullptr;
        fout = nullptr;

        if (eof1 && eof2) break;

        fin = fopen(tmpName, "rb");
        if (!fin) {
            closedir(dir);
            OSD::progressDialog("", "", 0, 2);
            free(tempDir);
            heap_caps_free(_buffer);
            return;
        }
        setvbuf(fin, NULL, _IOFBF, 512);
        eof1 = false;
        readFile1 = true;
        n++;
    }

    heap_caps_free(_buffer);

    if (fin) fclose(fin);
    if (fout) fclose(fout);
    closedir(dir);

    // printf("rename [%s] -> [%s]\n", tmpName, indexFile);
    rename(tmpName, indexFile);

    fout = fopen(indexFile, "a");
    if (!fout) {
        OSD::progressDialog("", "", 0, 2);
        free(tempDir);
        return;
    }
    fprintf(fout, "%020lu", hash);
    fclose(fout);

    if (n) {
        snprintf(indexFile, sizeof(indexFile), "%s/%s.tmp.%d", tempDir, fileTypes[ftype].indexFilename.c_str(), (n - 1) & 1);
        remove(indexFile);
        OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_3[Config::lang], 0, 1);
        OSD::progressDialog("", "", 100.0f, 1);
    }

    OSD::progressDialog("", "", 0, 2);
    printf("total iterations %d\n", iterations);

    free(tempDir);
}
#else
void FileUtils::DirToFile(string fpath, uint8_t ftype, unsigned long hash, unsigned int item_count) {
    FILE* fin = nullptr;
    FILE* fout = nullptr;
    char line[FILENAMELEN + 1];
    string fname1 = "";
    string fname2 = "";
    string fnameLastSaved = "";

    // Populate filexts with valid filename extensions
    std::vector<std::string> filexts;
    size_t pos = 0;
    string ss = fileTypes[ftype].fileExts;
    while ((pos = ss.find(",")) != string::npos) {
        // printf("%s , ",ss.substr(0,pos).c_str());
        filexts.push_back(ss.substr(0, pos));
        ss.erase(0, pos + 1);
    }

    filexts.push_back(ss.substr(0));

    // Remove previous dir file
    remove((fpath + fileTypes[ftype].indexFilename).c_str());

    string fdir = fpath.substr(0, fpath.length() - 1);
    DIR* dir = opendir(fdir.c_str());
    if (dir == NULL) {
        printf("Error opening %s\n", fpath.c_str());
        return;
    }

    OSD::progressDialog(OSD_FILE_INDEXING[Config::lang],OSD_FILE_INDEXING_1[Config::lang],0,0);

    int items_processed = 0;
    struct dirent* de;

    OSD::elements = 0;
    OSD::ndirs = 0;

    bool readFile1 = false, readFile2 = true;
    bool eof1 = true, eof2 = false;
    bool holdFile2 = false;

    int n = 1;

    if (fpath != ( MountPoint + "/" ) ) {
        fname1 = "  ..";
        eof1 = false;
    }

    string tempDir = FileUtils::createTmpDir();
    if ( tempDir == "" ) {
        closedir(dir);
        // Close progress dialog
        OSD::progressDialog("","",0,2);
        return;
    }

    int bufferSize;
    if (Config::videomode < 2) {
        if (Config::psramsize > 0) {
            bufferSize = item_count > DIR_CACHE_SIZE ? DIR_CACHE_SIZE : item_count;  // Size of buffer to read and sort
        } else {
            bufferSize = item_count > DIR_CACHE_SIZE_NO_PSRAM ? DIR_CACHE_SIZE_NO_PSRAM : item_count;  // Size of buffer to read and sort
        }
    } else {
        if (Config::psramsize > 0) {
            bufferSize = item_count > DIR_CACHE_SIZE_OVERSCAN ? DIR_CACHE_SIZE_OVERSCAN : item_count;  // Size of buffer to read and sort
        } else {
            bufferSize = item_count > DIR_CACHE_SIZE_OVERSCAN_NO_PSRAM ? DIR_CACHE_SIZE_OVERSCAN_NO_PSRAM : item_count;  // Size of buffer to read and sort
        }
    }
    std::vector<std::string> buffer;

    int iterations = 0;

    while ( !eof2 || ( fin && !feof(fin)) ) {
        fnameLastSaved = "";

        holdFile2 = false;

        iterations++;

        fout = fopen((tempDir + "/" + fileTypes[ftype].indexFilename + ".tmp." + std::to_string(n&1)).c_str(), "wb");
        if ( !fout ) {
            if ( fin ) fclose( fin );
            closedir( dir );
            // Close progress dialog
            OSD::progressDialog("","",0,2);
            return;
        }

        if (setvbuf(fout, NULL, _IOFBF, 1024) != 0) {
            printf("setvbuf failed\n");
        }

        while (1) {

            if ( readFile1 ) {
                if ( !fin || feof( fin ) ) eof1 = true;
                if ( !eof1 ) {
                    size_t res = fread( line, sizeof(char), FILENAMELEN, fin);
                    if ( !res || feof( fin ) ) {
                        eof1 = true;
                    } else {
                        line[FILENAMELEN-1] = '\0';
                        fname1.assign(line);
                    }
                }
                readFile1 = false;
            }

            if ( readFile2 ) {

                if (buffer.empty()) { // Fill buffer with directory entries

                    if ( bufferSize ) {

                        while ( buffer.size() < bufferSize && (de = readdir(dir)) != nullptr ) {
                            if (de->d_name[0] != '.') {
                                string fname = de->d_name;
                                if (de->d_type == DT_DIR) {
                                    buffer.push_back( " " + fname );
                                    OSD::ndirs++;
                                } else if (de->d_type == DT_REG && std::find(filexts.begin(), filexts.end(), getLCaseExt(fname)) != filexts.end()) {
                                    buffer.push_back( fname );
                                    OSD::elements++;
                                }
                            }
                        }

                        // Sort buffer loaded with processed directory entries
                        sort(buffer.begin(), buffer.end(), [](const string& a, const string& b) {
                            return ::toLower(a) < toLower(b);
                        });

                    } else {

                        eof2 = true;
                        readFile2 = false;

                    }

                }

                if (!buffer.empty()) {

                    fname2 = buffer.front();

                    buffer.erase(buffer.begin()); // Remove first element from buffer

                    items_processed++;

                    OSD::progressDialog("","",(float) 100 / ((float) item_count / (float) items_processed),1);

                } else {
                    if ( !de ) eof2 = true;
                }

                readFile2 = false;
                holdFile2 = false;
            }

            string fnameToSave = "";

            if ( eof1 ) {
                if ( eof2 || holdFile2 || strcasecmp(fnameLastSaved.c_str(), fname2.c_str()) > 0 ) {
                    break;
                }
                fnameToSave = fname2;
                readFile2 = true;
            } else
            // eof2 || fname1 < fname2
            // si fname2 > fname1 entonces grabar fname1, ya que fname2 esta ordenado y no puede venir uno menor en este grupo
            if ( eof2 || strcasecmp(fname1.c_str(), fname2.c_str()) < 0 ) {
                fnameToSave = fname1;
                readFile1 = true;
            } else
            if ( strcasecmp(fname1.c_str(), fname2.c_str()) > 0 && strcasecmp(fnameLastSaved.c_str(), fname2.c_str()) > 0 ) {
                // fname1 > fname2 && last > fname2
                holdFile2 = true;
                fnameToSave = fname1;
                readFile1 = true;
            } else {
                if ( strcasecmp(fnameLastSaved.c_str(), fname2.c_str()) > 0 ) {
                    break;
                }
                fnameToSave = fname2;
                readFile2 = true;
            }

            if ( fnameToSave != "" ) {
                string sw;
                if ( fnameToSave.length() > FILENAMELEN - 1 )   sw = fnameToSave.substr(0,FILENAMELEN - 1) + "\n";
                else                                            sw = fnameToSave + string(FILENAMELEN - 1 - fnameToSave.size(), ' ') + "\n";
                fwrite(sw.c_str(), sizeof(char), sw.length(), fout);
                fnameLastSaved = fnameToSave;
            }
        }

        if ( fin ) {
            fclose(fin);
            fin = nullptr;
        }

        fclose(fout);

        if ( eof1 && eof2 ) break;

        fin = fopen((tempDir + "/" + fileTypes[ftype].indexFilename + ".tmp." + std::to_string(n&1)).c_str(), "rb");
        if ( !fin ) {
            buffer.clear(); // Clear vector
            std::vector<std::string>().swap(buffer); // free memory

            filexts.clear(); // Clear vector
            std::vector<std::string>().swap(filexts); // free memory

            closedir( dir );
            // Close progress dialog
            OSD::progressDialog("","",0,2);
            return;
        }

        if (setvbuf(fin, NULL, _IOFBF, 512) != 0) {
            printf("setvbuf failed\n");
        }

        eof1 = false;
        readFile1 = true;

        n++;
    }

    buffer.clear(); // Clear vector
    std::vector<std::string>().swap(buffer); // free memory

    filexts.clear(); // Clear vector
    std::vector<std::string>().swap(filexts); // free memory

    if ( fin ) fclose(fin);
    closedir(dir);

    rename((tempDir + "/" + fileTypes[ftype].indexFilename + ".tmp." + std::to_string(n&1)).c_str(), (fpath + fileTypes[ftype].indexFilename).c_str());

    // Add directory hash to last line of file
    fout = fopen((fpath + fileTypes[ftype].indexFilename).c_str(), "a");
    if ( !fout ) {
        // Close progress dialog
        OSD::progressDialog("","",0,2);
        return;
    }

    fprintf(fout, "%020lu", hash);
    fclose(fout);

    if ( n ) {

        OSD::progressDialog(OSD_FILE_INDEXING[Config::lang],OSD_FILE_INDEXING_3[Config::lang],0,1);
        remove((tempDir + "/" + fileTypes[ftype].indexFilename + ".tmp." + std::to_string((n-1)&1)).c_str());

        OSD::progressDialog("","",(float) 100, 1);
    }

    // Close progress dialog
    OSD::progressDialog("","",0,2);

    printf("total iterations %d\n", iterations );

}
#endif

bool FileUtils::hasExtension(string filename, string extension) {
    return ( getLCaseExt(filename) == toLower(extension) );
}

void FileUtils::deleteFilesWithExtension(const char *folder_path, const char *extension) {

    DIR *dir;
    struct dirent *entry;
    dir = opendir(folder_path);

    if (dir == NULL) {
        // perror("Unable to open directory");
        return;
    }

    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0) {
            if (strstr(entry->d_name, extension) != NULL) {
                char file_path[512];
                snprintf(file_path, sizeof(file_path), "%s/%s", folder_path, entry->d_name);
                if (remove(file_path) == 0) {
                    printf("Deleted file: %s\n", entry->d_name);
                } else {
                    printf("Failed to delete file: %s\n", entry->d_name);
                }
            }
        }
    }

    closedir(dir);

}
