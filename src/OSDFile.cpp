/*

ESPeccy, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

This project is a fork of ESPectrum.
ESPectrum is developed by Víctor Iborra [Eremus] and David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on previous work:
- ZX-ESPectrum-Wiimote (2020, 2022) by David Crespo [dcrespo3d]
  https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote
- ZX-ESPectrum by Ramón Martinez and Jorge Fuertes
  https://github.com/rampa069/ZX-ESPectrum
- Original project by Pete Todd
  https://github.com/retrogubbins/paseVGA

Copyright (c) 2024 Juan José Ponteprino [SplinterGU]
https://github.com/SplinterGU/ESPeccy

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

#include <string>
#include <algorithm>
#include <sys/stat.h>
#include "errno.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"

using namespace std;

#include "OSDMain.h"
#include "FileUtils.h"
#include "Config.h"
#include "ESPectrum.h"
#include "cpuESP.h"
#include "Video.h"
#include "messages.h"
#include <math.h>
#include "ZXKeyb.h"
#include "pwm_audio.h"
#include "Z80_JLS/z80.h"
#include "Tape.h"

FILE *dirfile;
unsigned int OSD::elements;
unsigned int OSD::fdSearchElements;
unsigned int OSD::ndirs;

RowScrollContext OSD::fdRowScrollCTX;

uint8_t OSD::fdCursorFlash;
bool OSD::fdSearchRefresh;

// Función para convertir una cadena de dígitos en un número
// se agrega esta funcion porque atoul crashea si no hay digitos en el buffer
unsigned long getLong(char *buffer) {
    unsigned long result = 0;
    char * p = buffer;

    while (p && isdigit(*p)) {
        result = result * 10 + (*p - '0');
        ++p;
    }

    return result;
}

void OSD::fd_StatusbarDraw(const string& statusbar, bool fdMode) {
    // Print status bar
//    menuAt(mf_rows, 0);
    menuAt(h/OSD_FONT_H-1,0);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));

    if (fdMode) {
        VIDEO::vga.print((" " + std::string(cols - 2, ' ') + " ").c_str());
    } else {
        string text = " " + RotateLine(statusbar, &statusBarScrollCTX, cols - 12 - 2, 125, 25) + " ";
        VIDEO::vga.print(text.c_str());
    }
}

// Run a new file menu
string OSD::fileDialog(string &fdir, string title, uint8_t ftype, uint8_t mfcols, uint8_t mfrows) {

    bool scr_loaded = false;

    int screen_number = 0;
    off_t screen_offset = 0;

    string lastFile = "";
    int last_screen_number = 0;
    off_t last_screen_offset = 0;

    // struct stat stat_buf;
    long dirfilesize;
    bool reIndex;

    // Columns and Rows
    cols = mfcols;
    mf_rows = mfrows + (Config::aspect_16_9 ? 0 : 1);

    // CRT Overscan compensation
    if (Config::videomode == 2) {
//        x = 18;
        x = 0;
        if (menu_level == 0) {
            if (Config::arch[0] == 'T' && Config::ALUTK == 2) {
                y = Config::aspect_16_9 ? OSD_FONT_H * 2 : OSD_FONT_H;
            } else {
                y = OSD_FONT_H * 3;
            }
        }
    } else {
        x = 0;
        if (menu_level == 0) y = 0;
    }

    // Position
    if (menu_level == 0) {
        x += (Config::aspect_16_9 ? OSD_FONT_W * 4 : OSD_FONT_W * 3);
        y += (Config::aspect_16_9 ? OSD_FONT_H : OSD_FONT_H);
    } else {
        x += (Config::aspect_16_9 ? OSD_FONT_W * 4 : OSD_FONT_W * 3) + (48 /*60*/ * menu_level);
        y += (Config::aspect_16_9 ? OSD_FONT_H : OSD_FONT_H) + (4 /*8*/ * (menu_level - 1));
    }

    bool is_scr = menu_level == 0 && ftype == DISK_SCRFILE;
    bool thumb_enabled = menu_level == 0 && (((ftype == DISK_TAPFILE || ftype == DISK_SNAFILE || ftype == DISK_ROMFILE) && Config::thumbsEnabled) || is_scr);

    // Adjust dialog size if needed
    w = (cols * OSD_FONT_W) + 2;
    printf("X: %d w: %d Cols: %d scrW: %d\n",x,w,cols,scrW);
    int limW = OSD::scrW - (Config::aspect_16_9 ? OSD_FONT_W * 4: OSD_FONT_W * 2);
    while (x + w >= limW) {
        cols--;
        w = (cols * OSD_FONT_W) + 2;
        printf("X: %d w: %d Cols: %d scrW: %d\n",x,w,cols,scrW);
    };

    h = ((mf_rows + 1) * OSD_FONT_H) + 2 + (thumb_enabled ? 192 / 2 : 0);
    printf("Y: %d h: %d mf_rows: %d scrH: %d\n",y,h,mf_rows,scrH);
    while (y + h >= OSD::scrH - OSD_FONT_H) {
        mf_rows--;
        h = ((mf_rows + 1) * OSD_FONT_H) + 2 + (thumb_enabled ? 192 / 2 : 0);
        printf("Y: %d h: %d mf_rows: %d scrH: %d\n",y,h,mf_rows,scrH);
    };

    // Adjust begin_row & focus in case of values doesn't fit in current dialog size
    // printf("Focus: %d, Begin_row: %d, mf_rows: %d\n",(int) FileUtils::fileTypes[ftype].focus,(int) FileUtils::fileTypes[ftype].begin_row,(int) mf_rows);
    if (FileUtils::fileTypes[ftype].focus > mf_rows - 1) {
        FileUtils::fileTypes[ftype].begin_row += FileUtils::fileTypes[ftype].focus - (mf_rows - 1);
        FileUtils::fileTypes[ftype].focus = mf_rows - 1;
    } else
    if (FileUtils::fileTypes[ftype].focus + (FileUtils::fileTypes[ftype].begin_row - 2) < mf_rows) {
        FileUtils::fileTypes[ftype].focus += FileUtils::fileTypes[ftype].begin_row - 2;
        FileUtils::fileTypes[ftype].begin_row = 2;
    }

    // menu = title + "\n" + fdir + "\n";
    menu = title + "\n" + ( fdir.length() == 1 ? fdir : fdir.substr(0,fdir.length()-1)) + "\n";
    WindowDraw(); // Draw menu outline
    fd_PrintRow(1, IS_INFO);    // Path

    ResetRowScrollContext(statusBarScrollCTX);

reset:

    // Draw blank rows
    uint8_t row = 2;
    for (; row < mf_rows; row++) {
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        menuAt(row, 0);
        VIDEO::vga.print(std::string(cols, ' ').c_str());
    }

    if (thumb_enabled) {
        for (int r = row; r < h/OSD_FONT_H-1; r++) {
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 0));
            menuAt(r, 0);
            VIDEO::vga.print(std::string(cols, ' ').c_str());
        }
    }

    // Draw shortcut help
    string StatusBar = "";
    if (ftype == DISK_TAPFILE || ftype == DISK_SNAFILE) { // Dirty hack
        if (ZXKeyb::Exists || Config::zxunops2) {
//            StatusBar += Config::lang == 0 ? "\x05+\x06+N: New | " :
//                         Config::lang == 1 ? "\x05+\x06+N: Nuevo | " :
//                                             "\x05+\x06+N: Novo | ";
            StatusBar += Config::lang == 0 ? "\x05+\x06+N: New | " :
                         Config::lang == 1 ? "\x05+\x06+N: Nuevo | " :
                                             "\x05+\x06+N: Novo | ";
        } else {
            StatusBar += Config::lang == 0 ? "F2: New | " :
                         Config::lang == 1 ? "F2: Nuevo | " :
                                             "F2: Novo | " ;
        }

        if (ftype == DISK_SNAFILE) {
            if (ZXKeyb::Exists || Config::zxunops2) {
                StatusBar += Config::lang == 0 ? "\x05+\x06+R: New w/ROM | " :
                             Config::lang == 1 ? "\x05+\x06+R: Nuevo c/ROM | " :
                                                 "\x05+\x06+R: Novo c/ROM | ";
            } else {
                StatusBar += Config::lang == 0 ? "S+F2: New w/ROM | " :
                             Config::lang == 1 ? "S+F2: Nuevo c/ROM | " :
                                                 "S+F2: Novo c/ROM | " ;
            }
        }

    }

    if (ZXKeyb::Exists || Config::zxunops2) {
        StatusBar += Config::lang == 0 ? "\x05+\x06+F: Search | \x05+\x06+D: Delete" :
                     Config::lang == 1 ? "\x05+\x06+F: Buscar | \x05+\x06+D: Borrar" :
                                         "\x05+\x06+F: Procurar | \x05+\x06+D: Excluir" ;
    } else {
        StatusBar += Config::lang == 0 ? "F3: Search | F8: Delete" :
                     Config::lang == 1 ? "F3: Buscar | F8: Borrar" :
                                         "F3: Procurar | F8: Excluir" ;
    }

    if ( thumb_enabled && !is_scr ) {
        if ( ftype == DISK_TAPFILE ) {
            if (ZXKeyb::Exists || Config::zxunops2) {
                StatusBar += Config::lang == 0 ? " | \x05+\x06+\x1B\x1A: Change SCR | \x05+\x06+Z/X/C/V: SCR offset" :
                             Config::lang == 1 ? " | \x05+\x06+\x1B\x1A: Cambiar SCR | \x05+\x06+Z/X/C/V: Centrar SCR" :
                                                 " | \x05+\x06+\x1B\x1A: Cambiar SCR | \x05+\x06+Z/X/C/V: Centralizar SCR";
            } else {
                StatusBar += Config::lang == 0 ? " | S+\x1B\x1A: Change SCR | CTRL+\x1B\x19\x18\x1A: SCR offset" :
                             Config::lang == 1 ? " | S+\x1B\x1A: Cambiar SCR | CTRL+\x1B\x19\x18\x1A: Centrar SCR" :
                                                 " | S+\x1B\x1A: Cambiar SCR | CTRL+\x1B\x19\x18\x1A: Centralizar SCR";
            }
        }

        if (ZXKeyb::Exists || Config::zxunops2) {
            StatusBar += Config::lang == 0 ? " | \x05+\x06+G: Save SCR" :
                         Config::lang == 1 ? " | \x05+\x06+G: Guarda SCR" :
                                             " | \x05+\x06+G: Salvar SCR";
        } else {
            StatusBar += Config::lang == 0 ? " | PtrScr: Save SCR" :
                         Config::lang == 1 ? " | PtrScr: Guarda SCR" :
                                             " | PtrScr: Salvar SCR";
        }
    }

    while(1) {
        fdCursorFlash = 0;

        reIndex = false;
        string filedir = FileUtils::MountPoint + fdir;

        std::vector<std::string> filexts;
        size_t pos = 0;
        string ss = FileUtils::fileTypes[ftype].fileExts;
        while ((pos = ss.find(",")) != std::string::npos) {
            filexts.push_back(ss.substr(0, pos));
            ss.erase(0, pos + 1);
        }
        filexts.push_back(ss.substr(0));

        unsigned long hash = 0; // Name checksum variables

        // Get Dir Stats
        int result = FileUtils::getDirStats(filedir, filexts, &hash, &elements, &ndirs);

        filexts.clear(); // Clear vector
        std::vector<std::string>().swap(filexts); // free memory

        if ( result == -1 ) {
            printf("Error opening %s\n",filedir.c_str());

            FileUtils::unmountSDCard();

            OSD::restoreBackbufferData();

            click();
            return "";

        }

        // Open dir file for read
        printf("Checking existence of index file %s\n",(filedir + FileUtils::fileTypes[ftype].indexFilename).c_str());
        dirfile = fopen((filedir + FileUtils::fileTypes[ftype].indexFilename).c_str(), "r");
        if (dirfile == NULL) {
            printf("No dir file found: reindexing\n");
            reIndex = true;
        } else {
            fseek(dirfile,0,SEEK_END);
            dirfilesize = ftell(dirfile);

            fseek(dirfile, dirfilesize - 20, SEEK_SET);

            char fhash[21];
            memset( fhash, '\0', sizeof(fhash));
            fgets(fhash, sizeof(fhash), dirfile);

            // If calc hash and file hash are different refresh dir index
            if ( getLong(fhash) != hash ||
                dirfilesize - 20 != FILENAMELEN * ( ndirs+elements + ( filedir != ( FileUtils::MountPoint + "/" ) ? 1 : 0 ) ) ) {
                reIndex = true;
            }
        }

        // There was no index or hashes are different: reIndex
        if (reIndex) {

            if ( dirfile ) {
                fclose(dirfile);
                dirfile = nullptr;
            }

            FileUtils::DirToFile(filedir, ftype, hash, ndirs + elements ); // Prepare filelist

            // stat((filedir + FileUtils::fileTypes[ftype].indexFilename).c_str(), &stat_buf);

            dirfile = fopen((filedir + FileUtils::fileTypes[ftype].indexFilename).c_str(), "r");
            if (dirfile == NULL) {
                printf("Error opening index file\n");

                FileUtils::unmountSDCard();

                OSD::restoreBackbufferData();
                click();

                return "";
            }

            fseek(dirfile,0,SEEK_END);
            dirfilesize = ftell(dirfile);

            // Reset position
            FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;

        }

        if (FileUtils::fileTypes[ftype].fdMode) {
            // Clean Status Bar
            menuAt(h/OSD_FONT_H-1,0);
            //menuAt(row, 0);
            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
            VIDEO::vga.print(std::string(cols, ' ').c_str());

            // Recalc items number
            long prevpos = ftell(dirfile);

            unsigned int foundcount = 0;
            fdSearchElements = 0;
            rewind(dirfile);
            char buf[FILENAMELEN+1];
            string search = FileUtils::fileTypes[ftype].fileSearch;
            std::transform(search.begin(), search.end(), search.begin(), ::toupper);
            while(1) {
                fgets(buf, sizeof(buf), dirfile);
                if (feof(dirfile)) break;
                if (buf[0] == ASCII_SPC) {
                    foundcount++;
                    // printf("%s",buf);
                }else {
                    char *p = buf; while(*p) *p++ = toupper(*p);
                    char *pch = strstr(buf, search.c_str());
                    if (pch != NULL) {
                        foundcount++;
                        fdSearchElements++;
                        // printf("%s",buf);
                    }
                }
            }

            if (foundcount) {
                // Redraw rows
                real_rows = foundcount + 2; // Add 2 for title and status bar
                virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
                last_begin_row = last_focus = 0;
            } else {
                fseek(dirfile,prevpos,SEEK_SET);
            }

            fdSearchRefresh = false;

        } else {

            real_rows = (dirfilesize / FILENAMELEN) + 2; // Add 2 for title and status bar
            virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);

            last_begin_row = last_focus = 0;

            fdSearchElements = elements;

        }

        if ((real_rows > mf_rows) && ((FileUtils::fileTypes[ftype].begin_row + mf_rows - 2) > real_rows)) {
            FileUtils::fileTypes[ftype].focus += (FileUtils::fileTypes[ftype].begin_row + mf_rows - 2) - real_rows;
            FileUtils::fileTypes[ftype].begin_row = real_rows - (mf_rows - 2);
        }

        fd_Redraw(title, fdir, ftype); // Draw content

        // First clean statusbar
        menuAt(h/OSD_FONT_H-1,0);
        //menuAt(row, 0);
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
        VIDEO::vga.print(std::string(cols, ' ').c_str());

        fd_StatusbarDraw(StatusBar, FileUtils::fileTypes[ftype].fdMode);

        bool mode_E = false;

        fabgl::VirtualKeyItem Menukey;
        const string fat32forbidden="\\/:*?\"<>|\x7F"; // Characters not valid for FAT32 filenames

        int idle = 0;

        while (1) {

            if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEFILEBROWSER);

            ESPectrum::readKbdJoy();

            // Process external keyboard
            if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {

                idle = 0;

                ResetRowScrollContext(fdRowScrollCTX);

                // Print elements
                VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));

                unsigned int elem = FileUtils::fileTypes[ftype].fdMode ? fdSearchElements : elements;
                if (elem) {
                    // menuAt(mf_rows, cols - (real_rows > virtual_rows ? 13 : 12));
                    //menuAt(mf_rows, cols - 12);
                    menuAt(h/OSD_FONT_H-1, cols - 12);
                    char elements_txt[13];
                    int nitem = (FileUtils::fileTypes[ftype].begin_row + FileUtils::fileTypes[ftype].focus ) - (4 + ndirs) + (fdir.length() == 1);
                    snprintf(elements_txt, sizeof(elements_txt), "%d/%d ", nitem > 0 ? nitem : 0 , elem);
                    VIDEO::vga.print(std::string(12 - strlen(elements_txt), ' ').c_str());
                    VIDEO::vga.print(elements_txt);
                } else {
                    //menuAt(mf_rows, cols - 12);
                    menuAt(h/OSD_FONT_H-1, cols - 12);
                    VIDEO::vga.print(std::string(12,' ').c_str());
                }

                if (ESPectrum::readKbd(&Menukey, KBDREAD_MODEFILEBROWSER)) {

                    if (!Menukey.down) continue;

                    int fsearch = VirtualKey2ASCII(Menukey, &mode_E);

                    // Search first ocurrence of letter if we're not on that letter yet
                    if ( fsearch /*((Menukey.vk >= fabgl::VK_a) && (Menukey.vk <= fabgl::VK_Z)) || Menukey.vk == fabgl::VK_SPACE || ((Menukey.vk >= fabgl::VK_0) && (Menukey.vk <= fabgl::VK_9))*/) {

                        // fat32 filter
                        if (fat32forbidden.find(fsearch) != std::string::npos) continue;

                        if (FileUtils::fileTypes[ftype].fdMode) {

                            if (fsearch && FileUtils::fileTypes[ftype].fileSearch.length() < FILENAMELEN /*MAXSEARCHLEN*/) {
                                FileUtils::fileTypes[ftype].fileSearch += char(fsearch);
                                fdSearchRefresh = true;
                                mode_E = false;
                                click();
                            }

                        } else {
                            uint8_t letra = rowGet(menu,FileUtils::fileTypes[ftype].focus).at(0);
                            // printf("%d %d\n",(int)letra,fsearch);
                            if (toupper(letra) != toupper(fsearch)) {
                                // Seek first ocurrence of letter/number
                                long prevpos = ftell(dirfile);
                                char buf[FILENAMELEN+1];
                                int cnt = 0;
                                fseek(dirfile,0,SEEK_SET);
                                while(!feof(dirfile)) {
                                    fgets(buf, sizeof(buf), dirfile);
                                    // printf("%c %d\n",buf[0],int(buf[0]));
                                    if (toupper(buf[0]) == toupper(char(fsearch))) break;
                                    cnt++;
                                }
                                // printf("Cnt: %d Letra: %d\n",cnt,int(letra));
                                if (!feof(dirfile)) {
                                    last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                                    last_focus = FileUtils::fileTypes[ftype].focus;
                                    if (real_rows > virtual_rows) {
                                        int m = cnt + virtual_rows - real_rows;
                                        if (m > 0) {
                                            FileUtils::fileTypes[ftype].focus = m + 2;
                                            FileUtils::fileTypes[ftype].begin_row = cnt - m + 2;
                                        } else {
                                            FileUtils::fileTypes[ftype].focus = 2;
                                            FileUtils::fileTypes[ftype].begin_row = cnt + 2;
                                        }
                                    } else {
                                        FileUtils::fileTypes[ftype].focus = cnt + 2;
                                        FileUtils::fileTypes[ftype].begin_row = 2;
                                    }
                                    // printf("Real rows: %d; Virtual rows: %d\n",real_rows,virtual_rows);
                                    // printf("Focus: %d, Begin_row: %d\n",(int) FileUtils::fileTypes[ftype].focus,(int) FileUtils::fileTypes[ftype].begin_row);
                                    fd_Redraw(title,fdir,ftype);
                                    click();
                                } else
                                    fseek(dirfile,prevpos,SEEK_SET);

                            }
                        }

                    } else if (Menukey.vk == fabgl::VK_F2 && ( ftype == DISK_TAPFILE || ftype == DISK_SNAFILE ) ) {  // Dirty hack

                        bool save_withrom = Menukey.SHIFT;

                        string new_tap = OSD::input( 1, h/OSD_FONT_H-1, Config::lang == 0 ? "Name: " :
                                                                        Config::lang == 1 ? "Nomb: " :
                                                                                            "Nome: "
                                                                      , FILENAMELEN - 1
                                                                      , cols - 3 - 6
                                                                      , zxColor(7,1), zxColor(5,0)
                                                                      , ""
                                                                      , fat32forbidden );

                        if ( new_tap != "" ) {

                            fclose(dirfile);
                            dirfile = NULL;

                            FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;

                            string ext;
                            if ( ftype == DISK_TAPFILE ) ext = ".tap";
                            else if ( FileUtils::hasExtension(new_tap,"z80") || FileUtils::hasExtension(new_tap,"sna") || FileUtils::hasExtension(new_tap, "sp") ) {
                                ext = "";
                            } else
                                ext = ".sna";

                            return ( save_withrom ? "*" : "N" ) + new_tap + ext;

                        } else {
                            fd_StatusbarDraw(StatusBar, FileUtils::fileTypes[ftype].fdMode);
                        }

                    } else if (Menukey.vk == fabgl::VK_F3) {

                        FileUtils::fileTypes[ftype].fdMode ^= 1;

                        // status bard position & color
                        //menuAt(mf_rows, 0);
                        menuAt(h/OSD_FONT_H-1, 0);
                        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));

                        if (FileUtils::fileTypes[ftype].fdMode) {
                            // Clean status bar
                            VIDEO::vga.print(std::string(cols, ' ').c_str());
                            fdCursorFlash = 63;
                            fdSearchRefresh = FileUtils::fileTypes[ftype].fileSearch != "";

                        } else {

                            // Restore status bar
                            fd_StatusbarDraw(StatusBar, FileUtils::fileTypes[ftype].fdMode);

                            if (FileUtils::fileTypes[ftype].fileSearch != "") {
                                real_rows = (dirfilesize / FILENAMELEN) + 2; // Add 2 for title and status bar
                                virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
                                last_begin_row = last_focus = 0;
                                FileUtils::fileTypes[ftype].focus = 2;
                                FileUtils::fileTypes[ftype].begin_row = 2;
                                fd_Redraw(title, fdir, ftype);
                            }

                        }

                        click();

                    } else if (Menukey.CTRL && Menukey.vk == fabgl::VK_LEFT) {
                        if (thumb_enabled && !is_scr && ftype == DISK_TAPFILE && screen_offset > -256) screen_offset--;
                    } else if (Menukey.CTRL && Menukey.vk == fabgl::VK_RIGHT) {
                        if (thumb_enabled && !is_scr && ftype == DISK_TAPFILE && screen_offset < 256) screen_offset++;
                    } else if (Menukey.CTRL && Menukey.vk == fabgl::VK_UP) {
                        if (thumb_enabled && !is_scr && ftype == DISK_TAPFILE && screen_offset > -256 + 32) screen_offset-=32;
                    } else if (Menukey.CTRL && !is_scr && Menukey.vk == fabgl::VK_DOWN) {
                        if (thumb_enabled && !is_scr && ftype == DISK_TAPFILE && screen_offset < 256 - 32) screen_offset+=32;
                    } else if (Menukey.SHIFT && !is_scr && Menukey.vk == fabgl::VK_LEFT) {
                        if (thumb_enabled && screen_number > 0) screen_number--;
                    } else if (Menukey.SHIFT && !is_scr && Menukey.vk == fabgl::VK_RIGHT) {
                        if (thumb_enabled) screen_number++;
                    } else if (Menukey.SHIFT && Menukey.vk == fabgl::VK_PRINTSCREEN) {
                        CaptureToBmp();
                    } else if (Menukey.vk == fabgl::VK_PRINTSCREEN) {
                        if (thumb_enabled && scr_loaded && !is_scr) {
                            string _fname = lastFile;
                            rtrim(_fname);
                            OSD::saveSCR((FileUtils::MountPoint+fdir+_fname).c_str(), (uint32_t *) VIDEO::SaveRect);
                            screen_offset = 0;
                            screen_number = 0;
                            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 0));
                            menuAt(row+((h/OSD_FONT_H)-row)/2, cols/4);
                            VIDEO::vga.print(" ");
                            menuAt(row+((h/OSD_FONT_H)-row)/2, cols*3/4);
                            VIDEO::vga.print(" ");
                        }
                    } else if (Menukey.vk == fabgl::VK_F8) {
                        click();

                        filedir = rowGet(menu,FileUtils::fileTypes[ftype].focus);

                        if (filedir[0] != ASCII_SPC) {
                            rtrim(filedir);
                            if ( !access(( FileUtils::MountPoint + fdir + filedir ).c_str(), W_OK) ) {
                                string title = MENU_DELETE_CURRENT_FILE[Config::lang];
                                string msg = OSD_DLG_SURE[Config::lang];
                                uint8_t res = msgDialog(title,msg);
                                menu_saverect = true;

                                if (res == DLG_YES) {
                                    if ( FileUtils::getResolvedPath( FileUtils::MountPoint + fdir + filedir ) == FileUtils::getResolvedPath( Tape::tapeSaveName ) ) Tape::Eject();
                                    remove(( FileUtils::MountPoint + fdir + filedir ).c_str());
                                    fd_Redraw(title, fdir, ftype);
                                    menu_saverect = true;
                                    goto reset;
                                }
                            } else {
                                OSD::osdCenteredMsg(OSD_READONLY_FILE_WARN[Config::lang], LEVEL_WARN);
                            }
                            click();
                        }
                    } else if (Menukey.vk == fabgl::VK_UP || Menukey.vk == fabgl::VK_JOY1UP || Menukey.vk == fabgl::VK_JOY2UP) {
                        if (FileUtils::fileTypes[ftype].focus == 2 && FileUtils::fileTypes[ftype].begin_row > 2) {
                            last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                            FileUtils::fileTypes[ftype].begin_row--;
                            fd_Redraw(title, fdir, ftype);
                        } else if (FileUtils::fileTypes[ftype].focus > 2) {
                            last_focus = FileUtils::fileTypes[ftype].focus;
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus--, IS_NORMAL);
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED);
                        }
                        click();
                    } else if (Menukey.vk == fabgl::VK_DOWN || Menukey.vk == fabgl::VK_JOY1DOWN || Menukey.vk == fabgl::VK_JOY2DOWN) {
                        if (FileUtils::fileTypes[ftype].focus == virtual_rows - 1 && FileUtils::fileTypes[ftype].begin_row + virtual_rows - 2 < real_rows) {
                            last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                            FileUtils::fileTypes[ftype].begin_row++;
                            fd_Redraw(title, fdir, ftype);
                        } else if (FileUtils::fileTypes[ftype].focus < virtual_rows - 1) {
                            last_focus = FileUtils::fileTypes[ftype].focus;
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus++, IS_NORMAL);
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED);
                        }
                        click();
                    } else if (Menukey.vk == fabgl::VK_PAGEUP || (Menukey.vk == fabgl::VK_LEFT && Config::osd_LRNav == 0) || Menukey.vk == fabgl::VK_JOY1LEFT || Menukey.vk == fabgl::VK_JOY2LEFT) {
                        if (FileUtils::fileTypes[ftype].begin_row > virtual_rows) {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row -= virtual_rows - 2;
                        } else {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row = 2;
                        }
                        fd_Redraw(title, fdir, ftype);
                        click();
                    } else if (Menukey.vk == fabgl::VK_PAGEDOWN || (Menukey.vk == fabgl::VK_RIGHT && Config::osd_LRNav == 0) || Menukey.vk == fabgl::VK_JOY1RIGHT || Menukey.vk == fabgl::VK_JOY2RIGHT) {
                        if (real_rows - FileUtils::fileTypes[ftype].begin_row  - virtual_rows > virtual_rows) {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row += virtual_rows - 2;
                        } else {
                            FileUtils::fileTypes[ftype].focus = virtual_rows - 1;
                            FileUtils::fileTypes[ftype].begin_row = real_rows - virtual_rows + 2;
                        }
                        fd_Redraw(title, fdir, ftype);
                        click();
                    } else if (Menukey.vk == fabgl::VK_HOME) {
                        last_focus = FileUtils::fileTypes[ftype].focus;
                        last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                        FileUtils::fileTypes[ftype].focus = 2;
                        FileUtils::fileTypes[ftype].begin_row = 2;
                        fd_Redraw(title, fdir, ftype);
                        click();
                    } else if (Menukey.vk == fabgl::VK_END) {
                        last_focus = FileUtils::fileTypes[ftype].focus;
                        last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                        FileUtils::fileTypes[ftype].focus = virtual_rows - 1;
                        FileUtils::fileTypes[ftype].begin_row = real_rows - virtual_rows + 2;
                        // printf("Focus: %d, Lastfocus: %d\n",FileUtils::fileTypes[ftype].focus,(int) last_focus);
                        fd_Redraw(title, fdir, ftype);
                        click();
                    } else if (Menukey.vk == fabgl::VK_BACKSPACE || (Menukey.vk == fabgl::VK_LEFT && Config::osd_LRNav == 1)) {
                        if (FileUtils::fileTypes[ftype].fdMode && Menukey.vk == fabgl::VK_BACKSPACE) {
                            if (FileUtils::fileTypes[ftype].fileSearch.length()) {
                                FileUtils::fileTypes[ftype].fileSearch.pop_back();
                                fdSearchRefresh = true;
                                click();
                            }
                        } else {
                            if (fdir != "/") { // if non root directory goto previous directory
                                fclose(dirfile);
                                dirfile = NULL;

                                fdir.pop_back();
                                fdir = fdir.substr(0,fdir.find_last_of("/") + 1);

                                FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;

                                click();

                                break;

                            } else if ( menu_level > 0 ) { // exit directly if non-menu fileDialog
                                OSD::restoreBackbufferData();

                                fclose(dirfile);
                                dirfile = NULL;

                                click();
                                return "";
                            }

                        }

                    } else if (Menukey.vk == fabgl::VK_RETURN
                            || Menukey.vk == fabgl::VK_JOY1B
                            || Menukey.vk == fabgl::VK_JOY2B
                            || Menukey.vk == fabgl::VK_JOY1C
                            || Menukey.vk == fabgl::VK_JOY2C
                            || (Menukey.vk == fabgl::VK_RIGHT && Config::osd_LRNav == 1 && trim_copy(rowGet(menu,FileUtils::fileTypes[ftype].focus)) != "..")) {

                        fclose(dirfile);
                        dirfile = NULL;

                        filedir = rowGet(menu,FileUtils::fileTypes[ftype].focus);

                        // printf("%s\n",filedir.c_str());
                        if (filedir[0] == ASCII_SPC) {
                            if (filedir[1] == ASCII_SPC) {
                                fdir.pop_back();
                                fdir = fdir.substr(0,fdir.find_last_of("/") + 1);
                            } else {
                                filedir.erase(0,1);
                                trim(filedir);
                                fdir = fdir + filedir + "/";
                            }
                            FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                            // printf("Fdir: %s\n",fdir.c_str());
                            break;
                        } else {

                            OSD::restoreBackbufferData();

                            rtrim(filedir);
                            click();

                            // if ((Menukey.CTRL && Menukey.vk == fabgl::VK_RETURN) || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2C) return "S" + filedir;

                            return (is_scr && !scr_loaded) ? "" : "R" + filedir;

                        }

                    } else if (Menukey.vk == fabgl::VK_ESCAPE || Menukey.vk == fabgl::VK_JOY1A || Menukey.vk == fabgl::VK_JOY2A) {

                        OSD::restoreBackbufferData();

                        fclose(dirfile);
                        dirfile = NULL;
                        click();
                        return "";
                    }

                }

            } else {
                fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED);
                if (thumb_enabled && (idle > 20 || Config::instantPreview)) {
                    string _fname = rowGet(menu,FileUtils::fileTypes[ftype].focus);

                    if (lastFile != _fname) {
                        screen_number = 0;
                        screen_offset = 0;
                        scr_loaded = false;
                    }

                    if (lastFile != _fname || screen_number != last_screen_number || screen_offset != last_screen_offset) {
                        int retPreview;
                        lastFile = _fname;
                        last_screen_number = screen_number;
                        last_screen_offset = screen_offset;
                        if (_fname[0] != ' ') {
                            rtrim(_fname);
                            retPreview = OSD::renderScreen(x+(w/2)-128/2, y+1+mf_rows*OSD_FONT_H, (FileUtils::MountPoint+fdir+_fname).c_str(), screen_number, &screen_offset);
                        } else {
                            retPreview = RENDER_PREVIEW_ERROR;
                        }

                        if (retPreview == RENDER_PREVIEW_ERROR) {
                            for (int r = row; r < h/OSD_FONT_H-1; r++) {
                                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 0));
                                menuAt(r, 0);
                                VIDEO::vga.print(std::string(cols, ' ').c_str());
                            }
                            string no_preview_txt = Config::lang == 0 ? "NO PREVIEW AVAILABLE" :
                                                    Config::lang == 1 ? "SIN VISTA PREVIA" :
                                                                        "TELA N\x8EO DISPON\x8BVEL";
                            menuAt(row+((h/OSD_FONT_H)-row)/2, cols/2 - no_preview_txt.length()/2);
                            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 0));
                            VIDEO::vga.print(no_preview_txt.c_str());
                            scr_loaded = false;
                        } else if (retPreview == RENDER_PREVIEW_OK) {
                            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 0));
                            menuAt(row+((h/OSD_FONT_H)-row)/2, cols/2-(128/OSD_FONT_W)/2-3);
                            VIDEO::vga.print((screen_number > 0)?"<":" ");
                            menuAt(row+((h/OSD_FONT_H)-row)/2, cols/2+(128/OSD_FONT_W)/2+2);
                            VIDEO::vga.print(" ");
                            scr_loaded = true;
                        } else if (retPreview == RENDER_PREVIEW_OK_MORE) {
                            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 0));
                            menuAt(row+((h/OSD_FONT_H)-row)/2, cols/2-(128/OSD_FONT_W)/2-3);
                            VIDEO::vga.print((screen_number > 0)?"<":" ");
                            menuAt(row+((h/OSD_FONT_H)-row)/2, cols/2+(128/OSD_FONT_W)/2+2);
                            VIDEO::vga.print(">");
                            scr_loaded = true;
                        } else if (retPreview == RENDER_PREVIEW_REQUEST_NO_FOUND) {
                            if (screen_number > 0) screen_number--;
                            else scr_loaded = false;
                        }
                    }
                }
                idle++;
            }

            if (FileUtils::fileTypes[ftype].fdMode) {

                if ((++fdCursorFlash & 0xf) == 0) {
                    //menuAt(mf_rows, 1);
                    menuAt(h/OSD_FONT_H-1, 1);
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                    VIDEO::vga.print(Config::lang == 0 ? "Find: " :
                                     Config::lang == 1 ? "B\xA3sq: " :
                                                         "Proc:"
                                    );
                    VIDEO::vga.print( FileUtils::fileTypes[ftype].fileSearch.size() > MAXSEARCHLEN ? FileUtils::fileTypes[ftype].fileSearch.substr( FileUtils::fileTypes[ftype].fileSearch.size() - MAXSEARCHLEN).c_str() : FileUtils::fileTypes[ftype].fileSearch.c_str());

                    if (fdCursorFlash > 63) {
                        VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(7, 1));
                        if (fdCursorFlash == 128) fdCursorFlash = 0;
                    }
                    VIDEO::vga.print(mode_E?"E":"L");
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                    VIDEO::vga.print(std::string(FileUtils::fileTypes[ftype].fileSearch.size() > MAXSEARCHLEN ? MAXSEARCHLEN : MAXSEARCHLEN - FileUtils::fileTypes[ftype].fileSearch.size(), ' ').c_str());
                }

                if (fdSearchRefresh) {

                    // Recalc items number
                    long prevpos = ftell(dirfile);

                    unsigned int foundcount = 0;
                    fdSearchElements = 0;
                    rewind(dirfile);
                    char buf[FILENAMELEN+1];
                    string search = FileUtils::fileTypes[ftype].fileSearch;
                    std::transform(search.begin(), search.end(), search.begin(), ::toupper);
                    while(1) {
                        fgets(buf, sizeof(buf), dirfile);
                        if (feof(dirfile)) break;
                        if (buf[0] == ASCII_SPC) {
                            foundcount++;
                        }else {
                            char *p = buf; while(*p) *p++ = toupper(*p);
                            char *pch = strstr(buf, search.c_str());
                            if (pch != NULL) {
                                foundcount++;
                                fdSearchElements++;
                            }
                        }
                    }

                    if (foundcount) {
                        // Redraw rows
                        real_rows = foundcount + 2; // Add 2 for title and status bar
                        virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
                        last_begin_row = last_focus = 0;
                        FileUtils::fileTypes[ftype].focus = 2;
                        FileUtils::fileTypes[ftype].begin_row = 2;
                        fd_Redraw(title, fdir, ftype);
                    } else {
                        fseek(dirfile,prevpos,SEEK_SET);
                    }

                    fdSearchRefresh = false;
                }

            } else {
                fd_StatusbarDraw(StatusBar, FileUtils::fileTypes[ftype].fdMode);
            }

            vTaskDelay(5 / portTICK_PERIOD_MS);

        }

    }

}

// Redraw inside rows
void OSD::fd_Redraw(string title, string fdir, uint8_t ftype) {

    if ((FileUtils::fileTypes[ftype].focus != last_focus) || (FileUtils::fileTypes[ftype].begin_row != last_begin_row)) {

        // printf("fd_Redraw\n");

        // Read bunch of rows
        menu = title + "\n" + ( fdir.length() == 1 ? fdir : fdir.substr(0,fdir.length()-1)) + "\n";
        char buf[FILENAMELEN+1];
        if (FileUtils::fileTypes[ftype].fdMode == 0 || FileUtils::fileTypes[ftype].fileSearch == "") {
            fseek(dirfile, (FileUtils::fileTypes[ftype].begin_row - 2) * FILENAMELEN, SEEK_SET);
            for (int i = 2; i < virtual_rows; i++) {
                fgets(buf, sizeof(buf), dirfile);
                if (feof(dirfile)) break;
                menu += buf;
            }
        } else {
            rewind(dirfile);
            int i = 2;
            int count = 2;
            string search = FileUtils::fileTypes[ftype].fileSearch;
            std::transform(search.begin(), search.end(), search.begin(), ::toupper);
            char upperbuf[FILENAMELEN+1];
            while (1) {
                fgets(buf, sizeof(buf), dirfile);
                if (feof(dirfile)) break;
                if (buf[0] == ASCII_SPC) {
                    if (i >= FileUtils::fileTypes[ftype].begin_row) {
                        menu += buf;
                        if (++count == virtual_rows) break;
                    }
                    i++;
                } else {
                    for(int i=0; buf[i]; i++) upperbuf[i] = toupper(buf[i]);
                    char *pch = strstr(upperbuf, search.c_str());
                    if (pch != NULL) {
                        if (i >= FileUtils::fileTypes[ftype].begin_row) {
                            menu += buf;
                            if (++count == virtual_rows) break;
                        }
                        i++;
                    }
                }
            }
        }

        fd_PrintRow(1, IS_INFO); // Print status bar

        uint8_t row = 2;
        for (; row < virtual_rows; row++) {
            if (row == FileUtils::fileTypes[ftype].focus) {
                fd_PrintRow(row, IS_FOCUSED);
            } else {
                fd_PrintRow(row, IS_NORMAL);
            }
        }

        if (real_rows > virtual_rows) {
            menuScrollBar(FileUtils::fileTypes[ftype].begin_row - 1);
        } else {
            for (; row < mf_rows; row++) {
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                menuAt(row, 0);
                VIDEO::vga.print(std::string(cols, ' ').c_str());
            }
        }

        last_focus = FileUtils::fileTypes[ftype].focus;
        last_begin_row = FileUtils::fileTypes[ftype].begin_row;
    }

}

// Print a virtual row
void OSD::fd_PrintRow(uint8_t virtual_row_num, uint8_t line_type) {

    uint8_t margin;

    string line = rowGet(menu, virtual_row_num);

    bool isDir = (line[0] == ASCII_SPC);

    trim(line);

    switch (line_type) {
    case IS_TITLE:
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
        margin = 2;
        break;
    case IS_INFO:
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    case IS_FOCUSED:
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    default:
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        margin = (real_rows > virtual_rows ? 3 : 2);
    }

    menuAt(virtual_row_num, 0);

    VIDEO::vga.print(" ");

    const string extra_line = isDir ? " <DIR>" : "";
    size_t extra_margin = extra_line.length();

    if (line.length() <= cols - margin - extra_margin) {
        line = line + std::string(cols - margin - extra_margin - line.length(), ' ') + extra_line;
    } else {
        if ( !isDir && line_type == IS_INFO ) {
            line = ".." + line.substr(line.length() - (cols - margin) + 2) + extra_line;
        } else {
            int space_fill = cols - margin - extra_margin - line.length();
            if ( space_fill < 0 ) {
                int max_first_column_size = cols - margin - extra_margin;
                if (line_type == IS_FOCUSED || line_type == IS_SELECTED_FOCUSED) {
                    line = RotateLine(line, &fdRowScrollCTX, max_first_column_size, 125, 25) + extra_line;
                } else {
                    line = line.substr(0, max_first_column_size) + extra_line;
                }
            } else {
                line += string(space_fill, ' ') + extra_line; // second column, fixed part, after tab
            }
        }
    }

    VIDEO::vga.print(line.c_str());

    VIDEO::vga.print(" ");

}
