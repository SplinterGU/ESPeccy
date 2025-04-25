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


#include <string>
#include <algorithm>
#include <sys/stat.h>
#include "errno.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"

using namespace std;

#include "FileUtils.h"
#include "Config.h"
#include "ESPeccy.h"
#include "cpuESP.h"
#include "Video.h"
#include "messages.h"
#include "OSD.h"
#include <math.h>
#include "ZXKeyb.h"
#include "pwm_audio.h"
#include "Z80_JLS/z80.h"
#include "Tape.h"

#define MENU_MAX_ROWS 13

// Scroll
#define UP true
#define DOWN false

extern Font SystemFont;

uint8_t OSD::menuBGColor;
uint8_t OSD::menuFGColor;

RowScrollContext OSD::rowScrollCTX;
RowScrollContext OSD::statusBarScrollCTX;

void OSD::menuSaveState(MenuState& state) {
    state.begin_row = begin_row;
    state.focus = focus;
    state.last_focus = last_focus;
    state.last_begin_row = last_begin_row;
}

void OSD::menuRestoreState(const MenuState& state) {
    begin_row = state.begin_row;
    focus = state.focus;
    last_focus = state.last_focus;
    last_begin_row = state.last_begin_row;
}

void OSD::ResetRowScrollContext(RowScrollContext &context) {
    context.rowTimeStartScroll = 0;
    context.rowTimeScroll = 0;
    context.rowScrollPos = 0;
    context.rowScrollStatus = false;
}
/**
 * @brief Rotates a string based on the scroll context and rotation type.
 *
 * @param line Original string to rotate.
 * @param context Scroll context containing current scroll position and status.
 * @param maxLength Maximum length for the output string.
 * @param startThreshold Threshold for starting the scroll.
 * @param scrollInterval Interval between scroll steps.
 * @return std::string Rotated string based on the specified context.
 */
std::string OSD::RotateLine(const std::string &line, RowScrollContext *context, int maxLength, int startThreshold, int scrollInterval) {
    int len = line.length();

    // Si la longitud es menor al maxLength, rellena con espacios al final
    if (len < maxLength) {
        return line + std::string(maxLength - len, ' ');
    }

    // Incrementa el contador de inicio de scroll hasta el umbral
    if (context->rowTimeStartScroll < startThreshold) {
        context->rowTimeStartScroll++;
    }

    // Inicia la rotación solo si se alcanzó el umbral de inicio
    if (context->rowTimeStartScroll == startThreshold) {
        context->rowTimeScroll++;

        // Aplica el scroll al alcanzar el intervalo de scroll
        if (context->rowTimeScroll == scrollInterval) {
            if (Config::osd_AltRot == 1) {
                // Rota hacia adelante o hacia atrás según el estado del scroll
                if (!context->rowScrollStatus) {
                    context->rowScrollPos++;
                    if (context->rowScrollPos >= len - maxLength) {
                        context->rowScrollStatus = true; // Cambia de dirección
                        context->rowTimeStartScroll = 0;
                    }
                } else {
                    context->rowScrollPos--;
                    if (context->rowScrollPos <= 0) {
                        context->rowScrollStatus = false; // Cambia de dirección
                        context->rowTimeStartScroll = 0;
                    }
                }
            } else {
                context->rowScrollPos = (context->rowScrollPos + 1) % (len + maxLength);
            }
            // Reinicia el contador de scroll
            context->rowTimeScroll = 0;
        }
    }

    std::string rotatedLine;

    // Calcula la posición de rotación y genera la cadena rotada
    int offset = context->rowScrollPos % len;

    // Lógica de rotación según Config::osd_AltRot
    if (Config::osd_AltRot == 1) {
        if (offset < 0) offset += len;
        rotatedLine = line.substr(offset) + line.substr(0, offset);
    } else {
        // Calcula la posición de rotación y genera la cadena rotada
        if ( context->rowScrollPos >= len ) {
            rotatedLine = std::string(maxLength - (offset % maxLength), SCROLL_SEP_CHAR) + line.substr(0, offset);
        } else {
            rotatedLine = line.substr(offset) + std::string(maxLength, SCROLL_SEP_CHAR) + line.substr(0, offset);
        }
    }

    // Devuelve la cadena rotada limitada a maxLength
    return rotatedLine.substr(0, maxLength);
}


int OSD::prepare_checkbox_menu(string &menu, string curopt) {

    int mpos = -1;
    int rpos;
    int itempos = 0;
    int m_curopt = 0;
    while(1) {
        mpos = menu.find("[",mpos + 1);
        if (mpos == string::npos) break;
        rpos = menu.find("]",mpos + 1) - mpos - 1;
        itempos++;
        string rmenu = menu.substr(mpos + 1, rpos );
        trim(rmenu);
        if (rmenu == curopt) {
            menu.replace(mpos + 1, rpos,"*");
            m_curopt = itempos;
        } else
            menu.replace(mpos + 1, rpos," ");
    }

    return m_curopt;

}

// Get real row number for a virtual one
unsigned short OSD::menuRealRowFor(uint8_t virtual_row_num) { return begin_row + virtual_row_num - 1; }

// Menu relative AT
void OSD::menuAt(short int row, short int col) {
    if (col < 0)
        col = cols - 2 - col;
    if (row < 0)
        row = virtual_rows - 2 - row;
    VIDEO::setCursor(x + 1 + (col * OSD_FONT_W), y + 1 + (row * OSD_FONT_H));
}

void OSD::menuPrintRow(uint8_t virtual_row_num, uint8_t line_type) {
    PrintRow(virtual_row_num, line_type, true);
}

void OSD::statusbarDraw(const string& statusbar) {
    VIDEO::setCursor(x + 1, y + 1 + (virtual_rows * OSD_FONT_H));
    VIDEO::setTextColor(zxColor(WHITE, BRIGHT_ON), zxColor(CYAN, BRIGHT_OFF));
    string text = " " + RotateLine(statusbar, &statusBarScrollCTX, cols - 2, 125, 25) + " ";
    VIDEO::print(text.c_str());
}

// Draw the complete menu
void OSD::WindowDraw() {

    // Set font
    VIDEO::setFont(SystemFont);

    if (menu_level == 0) SaveRectpos = 0;

    OSD::saveBackbufferData();

    // Menu border
    VIDEO::rect(x, y, w, h, zxColor(BLACK, BRIGHT_OFF));

    // Title Background
    for (uint8_t i = 0; i < OSD_FONT_H; ++i) {
        VIDEO::line(x, y + i + 1, x + w - 1, y + i + 1, zxColor(BLACK, BRIGHT_OFF));
    }

    // Title
    PrintRow(0, IS_TITLE);

    // Rainbow
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; ++i) {
            VIDEO::line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], BRIGHT_ON));
        }
        rb_paint_x += 5;
    }

}

static void renameSlot(int slot, string new_name) {
    char buffer[SLOTNAME_LEN+1] = {0};  // Buffer to store each line, extra char for null-terminator
    const string catalogPath = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + "catalog";

    FILE *catalogFile = fopen(catalogPath.c_str(), "rb+");
    if ( !catalogFile ) {
        catalogFile = fopen(catalogPath.c_str(), "wb+");
    }
    if ( !catalogFile ) {
        OSD::osdCenteredMsg(ERR_FS_EXT_FAIL[Config::lang], LEVEL_WARN, 1000);
    } else {
        if ( catalogFile ) {
            fseek(catalogFile, 0, SEEK_END);
            long catalogSize = ftell( catalogFile );

            long pos = ( slot - 1 ) * SLOTNAME_LEN;

            // write missing items if it happen
            while ( pos > catalogSize ) {
                uint8_t clean_buffer[ SLOTNAME_LEN + 1 ] = { 0 };
                if ( ( pos % SLOTNAME_LEN ) != 0 ) {
                    fwrite( clean_buffer, pos % SLOTNAME_LEN, 1, catalogFile );
                    catalogSize += pos % SLOTNAME_LEN;

                } else {
                    fwrite( clean_buffer, SLOTNAME_LEN, 1, catalogFile );
                    catalogSize += SLOTNAME_LEN;
                }
            }

            if (fseek(catalogFile, pos, SEEK_SET) == 0) {
                strcpy( buffer, new_name.c_str());
                fwrite( buffer, sizeof(uint8_t), SLOTNAME_LEN, catalogFile);

                if ( new_name == "" ) {
                    const string fname = FileUtils::MountPoint + DISK_PSNA_DIR + "/persist" + to_string( slot );
                    struct stat stat_buf;
                    int status = stat( (fname + ".sna" ).c_str(), &stat_buf);
                    if ( status == -1 || ! ( stat_buf.st_mode & S_IFREG ) ) {
                        new_name =  ( Config::lang == 0 ? "<Free Slot " :
                                      Config::lang == 1 ? "<Ranura Libre " :
                                                          "<Slot Livre ") + to_string(slot) + ">";
                    } else {
                        new_name = "Snapshot " + to_string(slot);
                    }
                }
                OSD::menu = OSD::rowReplace(OSD::menu, slot, new_name);
            }
        }
        fclose(catalogFile);
    }

}

int OSD::menuProcessSnapshot(fabgl::VirtualKeyItem Menukey) {
    int idx = menuRealRowFor( focus );

    if (Menukey.vk == fabgl::VK_F2 && begin_row - 1 + focus < real_rows) {
        click();
        uint8_t flags = 0;

        string new_name = input(1, focus, "", SLOTNAME_LEN, min(SLOTNAME_LEN, cols - 4), zxColor(BLACK, BRIGHT_OFF), zxColor(WHITE, BRIGHT_OFF), rowGet(menu, idx), "", &flags);
        if ( !( flags & INPUT_CANCELED ) ) { // if not canceled
            renameSlot(idx, new_name);
        }

        last_focus = focus - 1; // force redraw
        menuRedraw();

        return 0;

    } else if (Menukey.vk == fabgl::VK_F8) {
        click();

        const string fname = FileUtils::MountPoint + DISK_PSNA_DIR + "/persist" + to_string( idx );

        struct stat stat_buf;
        int status = stat( (fname + ".sna" ).c_str(), &stat_buf);
        if ( status == -1 || ! ( stat_buf.st_mode & S_IFREG ) ) {
            osdCenteredMsg(OSD_PSNA_NOT_AVAIL, LEVEL_INFO, 1000);
        } else {
            string title = MENU_DELETE_SNA[Config::lang];
            string msg = OSD_DLG_SURE[Config::lang];
            uint8_t res = msgDialog(title,msg);

            if (res == DLG_YES) {
                menu_saverect = true;

                remove( ( fname + ".sna" ).c_str() );
                remove( ( fname + ".esp" ).c_str() );

                renameSlot(idx, "");

                const string new_name =  ( Config::lang == 0 ? "<Free Slot " :
                                     Config::lang == 1 ? "<Ranura Libre " :
                                                         "<Slot Livre ") + to_string(idx) + ">";
                menu = rowReplace(menu, idx, new_name);
                last_focus = focus - 1; // force redraw
                menuRedraw();
            }
        }
        lastPreviewFile = "";
        return 0;
    } else if (Menukey.vk == fabgl::VK_RETURN || ((Menukey.vk == fabgl::VK_RIGHT) && (Config::osd_LRNav == 1)) || Menukey.vk == fabgl::VK_JOY1B || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2B || Menukey.vk == fabgl::VK_JOY2C) {
        int idx = menuRealRowFor( focus );
        const string fname = FileUtils::MountPoint + DISK_PSNA_DIR + "/persist" + to_string( idx );

        struct stat stat_buf;
        int status = stat( (fname + ".sna" ).c_str(), &stat_buf);
        if ( status == -1 || ! ( stat_buf.st_mode & S_IFREG ) ) {
            click();
            osdCenteredMsg(OSD_PSNA_NOT_AVAIL, LEVEL_INFO, 1000);
            return 0;

        } else {
            // Persist file exist continue normal process from main menuRun
            return 1;
        }
    }

    return 1;
}

int OSD::menuProcessSnapshotSave(fabgl::VirtualKeyItem Menukey) {
    if (Menukey.vk == fabgl::VK_RETURN || ((Menukey.vk == fabgl::VK_RIGHT) && (Config::osd_LRNav == 1)) || Menukey.vk == fabgl::VK_JOY1B || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2B || Menukey.vk == fabgl::VK_JOY2C) {
        // use continue normal process from main menuRun
        return 1;
    }

    return  menuProcessSnapshot(Menukey);

}

// Run a new menu
short OSD::menuRun(const string& new_menu, const string& statusbar, int (*proc_cb)(fabgl::VirtualKeyItem Menukey) ) {

    menuBGColor = zxColor(WHITE, BRIGHT_ON);
    menuFGColor = zxColor(BLACK, BRIGHT_ON);

    fabgl::VirtualKeyItem Menukey;

    menu = new_menu;

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
        y += OSD_FONT_H;
        prev_y[0] = 0;
    } else {
        x += (Config::aspect_16_9 ? OSD_FONT_W * 4 : OSD_FONT_W * 3) + (54 /*60*/ * menu_level);
        if (menu_saverect && !prev_y[menu_level]) {
            y += (4 + (OSD_FONT_H * menu_prevopt));
            prev_y[menu_level] = y;
        } else {
            y = prev_y[menu_level];
        }
    }

    for ( int i = menu_level + 1; i < 5; ++i ) prev_y[i] = 0;

    // Rows
    real_rows = rowCount(menu);
    virtual_rows = (real_rows > MENU_MAX_ROWS ? MENU_MAX_ROWS : real_rows);
    // begin_row = last_begin_row = last_focus = focus = 1;

    // Columns
    cols = 0;
    uint8_t col_count = 0, col_count_partial = 0;
    uint8_t cols_title = 0;
    bool with_tab = false;
    for (unsigned short i = 0; i < menu.length(); ++i) {
        if ((menu.at(i) == ASCII_TAB)) {
            with_tab = true;
            col_count_partial = col_count + 2; // space for el tab
            col_count = 0;
        } else
        if ((menu.at(i) == ASCII_NL)) {
            if (col_count + col_count_partial > cols) cols = col_count + col_count_partial;
            if (!cols_title) cols_title = cols;
            col_count_partial = 0;
            col_count = 0;
        } else
            col_count++;
    }

    if (cols_title + 6 > cols) cols = cols_title + 6; // color bars in title

    // printf("Cols: %d\n",cols);

    int max_cols = (menu_level == 0 ? 41 : 31);

    cols += (real_rows > virtual_rows ? 1 : 0); // For scrollbar

    cols += with_tab ? 1 : 2;

//    if ( statusbar != "" && cols < statusbar.length() + 1 ) cols = statusbar.length() + 1;

    if (cols > max_cols) cols = max_cols;

// printf("Cols final: %d\n",cols);

    // Size
    w = (cols * OSD_FONT_W) + 2;
    h = ((virtual_rows + (statusbar!=""?1:0) ) * OSD_FONT_H) + 2;

    int rmax = scrW == 320 ? 52 : 55;
    if ( x + cols * OSD_FONT_W > rmax * OSD_FONT_W ) x = ( rmax - cols ) * OSD_FONT_W;

    if ( y + h > VIDEO::yres() - OSD_FONT_H * 2 ) y = VIDEO::yres() - h - OSD_FONT_H * 2;

    WindowDraw(); // Draw menu outline

    if (!use_current_menu_state) {
        begin_row = 1;
        focus = menu_curopt;
        last_begin_row = last_focus = 0;
    }

    menuRedraw(); // Draw menu content

    ResetRowScrollContext(statusBarScrollCTX);

    if (statusbar != "") statusbarDraw(statusbar);

    while (1) {

        if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEFILEBROWSER);

        ESPeccy::readKbdJoy();

        // Process external keyboard
        if (ESPeccy::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPeccy::readKbd(&Menukey, KBDREAD_MODEFILEBROWSER)) {
                if (!Menukey.down) continue;
                if (proc_cb) {
                    int retcb = proc_cb(Menukey);
                    // 0 = stop this key process (stop propagation)
                    // 1 = continue process
                    // else return with callback return result
                    if (!retcb) continue;
                    if (retcb != 1) {
                        if (menu_level!=0) OSD::restoreBackbufferData(true);
                        use_current_menu_state = false;
                        return retcb;
                    }
                }
                if (Menukey.vk == fabgl::VK_UP || Menukey.vk == fabgl::VK_JOY1UP || Menukey.vk == fabgl::VK_JOY2UP) {
                    if (focus == 1 and begin_row > 1) {
                        menuScroll(DOWN);
                    } else {
                        last_focus = focus;
                        focus--;
                        if (focus < 1) {
                            focus = virtual_rows - 1;
                            last_begin_row = begin_row;
                            begin_row = real_rows - virtual_rows + 1;
                            menuRedraw();
                            menuPrintRow(focus, IS_FOCUSED);
                        }
                        else {
                            menuPrintRow(focus, IS_FOCUSED);
                            menuPrintRow(last_focus, IS_NORMAL);
                        }
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_DOWN || Menukey.vk == fabgl::VK_JOY1DOWN || Menukey.vk == fabgl::VK_JOY2DOWN) {
                    if (focus == virtual_rows - 1 && virtual_rows + begin_row - 1 < real_rows) {
                        menuScroll(UP);
                    } else {
                        last_focus = focus;
                        focus++;
                        if (focus > virtual_rows - 1) {
                            focus = 1;
                            last_begin_row = begin_row;
                            begin_row = 1;
                            menuRedraw();
                            menuPrintRow(focus, IS_FOCUSED);
                        }
                        else {
                            menuPrintRow(focus, IS_FOCUSED);
                            menuPrintRow(last_focus, IS_NORMAL);
                        }
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEUP || ((Menukey.vk == fabgl::VK_LEFT) && (Config::osd_LRNav == 0)) || Menukey.vk == fabgl::VK_JOY1LEFT || Menukey.vk == fabgl::VK_JOY2LEFT) {
                    if (begin_row > virtual_rows) {
                        focus = 1;
                        begin_row -= virtual_rows - 1;
                    } else {
                        focus = 1;
                        begin_row = 1;
                    }
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEDOWN || ((Menukey.vk == fabgl::VK_RIGHT) && (Config::osd_LRNav == 0)) || Menukey.vk == fabgl::VK_JOY1RIGHT || Menukey.vk == fabgl::VK_JOY2RIGHT) {
                    if (real_rows - begin_row - virtual_rows > virtual_rows) {
                        focus = 1;
                        begin_row += virtual_rows - 1;
                    } else {
                        focus = virtual_rows - 1;
                        begin_row = real_rows - virtual_rows + 1;
                    }
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_HOME) {
                    focus = 1;
                    begin_row = 1;
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_END) {
                    focus = virtual_rows - 1;
                    begin_row = real_rows - virtual_rows + 1;
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_RETURN || ((Menukey.vk == fabgl::VK_RIGHT) && (Config::osd_LRNav == 1)) || Menukey.vk == fabgl::VK_JOY1B || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2B || Menukey.vk == fabgl::VK_JOY2C) {
                    click();
                    menu_prevopt = menuRealRowFor(focus);
                    use_current_menu_state = false;
                    return menu_prevopt;
                } else if (Menukey.vk == fabgl::VK_ESCAPE || ((Menukey.vk == fabgl::VK_LEFT) && (Config::osd_LRNav == 1)) || Menukey.vk == fabgl::VK_F1 || Menukey.vk == fabgl::VK_JOY1A || Menukey.vk == fabgl::VK_JOY2A) {
                    if (menu_level!=0) OSD::restoreBackbufferData(true);
                    click();
                    use_current_menu_state = false;
                    return 0;
                }
            }
        }

        if (statusbar != "") statusbarDraw(statusbar);

        vTaskDelay(5 / portTICK_PERIOD_MS);

    }

}

// Run a new menu
unsigned short OSD::simpleMenuRun(const string& new_menu, uint16_t posx, uint16_t posy, uint8_t max_rows, uint8_t max_cols) {

    menuBGColor = zxColor(WHITE, BRIGHT_ON);
    menuFGColor = zxColor(BLACK, BRIGHT_ON);

    fabgl::VirtualKeyItem Menukey;

    menu = new_menu;

    x = posx;
    y = posy;

    // Rows
    real_rows = rowCount(menu);
    virtual_rows = real_rows > max_rows ? max_rows : real_rows;

    // Columns
    cols = max_cols;

    // Size
    w = (cols * OSD_FONT_W) + 2;
    h = (virtual_rows * OSD_FONT_H) + 2;

    // Set font
    VIDEO::setFont(SystemFont);

    if (menu_saverect && menu_level == 0) SaveRectpos = 0;

    OSD::saveBackbufferData();

    // Menu border
    VIDEO::rect(x, y, w, h, zxColor(BLACK, BRIGHT_OFF));

    // Title
    PrintRow(0, IS_TITLE);

    begin_row = 1;
    focus = menu_curopt;
    last_begin_row = last_focus = 0;

    menuRedraw(); // Draw menu content

    while (1) {

        if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEFILEBROWSER);

        ESPeccy::readKbdJoy();

        // Process external keyboard
        if (ESPeccy::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPeccy::readKbd(&Menukey, KBDREAD_MODEFILEBROWSER)) {
                if (!Menukey.down) continue;
                if (Menukey.vk == fabgl::VK_UP || Menukey.vk == fabgl::VK_JOY1UP || Menukey.vk == fabgl::VK_JOY2UP) {
                    if (focus == 1 and begin_row > 1) {
                        menuScroll(DOWN);
                    } else {
                        last_focus = focus;
                        focus--;
                        if (focus < 1) {
                            focus = virtual_rows - 1;
                            last_begin_row = begin_row;
                            begin_row = real_rows - virtual_rows + 1;
                            menuRedraw();
                            menuPrintRow(focus, IS_FOCUSED);
                        }
                        else {
                            menuPrintRow(focus, IS_FOCUSED);
                            menuPrintRow(last_focus, IS_NORMAL);
                        }
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_DOWN || Menukey.vk == fabgl::VK_JOY1DOWN || Menukey.vk == fabgl::VK_JOY2DOWN) {
                    if (focus == virtual_rows - 1 && virtual_rows + begin_row - 1 < real_rows) {
                        menuScroll(UP);
                    } else {
                        last_focus = focus;
                        focus++;
                        if (focus > virtual_rows - 1) {
                            focus = 1;
                            last_begin_row = begin_row;
                            begin_row = 1;
                            menuRedraw();
                            menuPrintRow(focus, IS_FOCUSED);
                        }
                        else {
                            menuPrintRow(focus, IS_FOCUSED);
                            menuPrintRow(last_focus, IS_NORMAL);
                        }
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEUP || Menukey.vk == fabgl::VK_LEFT || Menukey.vk == fabgl::VK_JOY1LEFT || Menukey.vk == fabgl::VK_JOY2LEFT) {
                    if (begin_row > virtual_rows) {
                        focus = 1;
                        begin_row -= virtual_rows - 1;
                    } else {
                        focus = 1;
                        begin_row = 1;
                    }
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEDOWN || Menukey.vk == fabgl::VK_RIGHT || Menukey.vk == fabgl::VK_JOY1RIGHT || Menukey.vk == fabgl::VK_JOY2RIGHT) {
                    if (real_rows - begin_row - virtual_rows > virtual_rows) {
                        focus = 1;
                        begin_row += virtual_rows - 1;
                    } else {
                        focus = virtual_rows - 1;
                        begin_row = real_rows - virtual_rows + 1;
                    }
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_HOME) {
                    focus = 1;
                    begin_row = 1;
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_END) {
                    focus = virtual_rows - 1;
                    begin_row = real_rows - virtual_rows + 1;
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_RETURN /*|| Menukey.vk == fabgl::VK_SPACE*/ || Menukey.vk == fabgl::VK_JOY1B || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2B || Menukey.vk == fabgl::VK_JOY2C) {
                    OSD::restoreBackbufferData(true);
                    click();
                    menu_prevopt = menuRealRowFor(focus);
                    std::string().swap(menu); // Reset Menu for save free usage
                    return menu_prevopt;
                } else if (Menukey.vk == fabgl::VK_ESCAPE || Menukey.vk == fabgl::VK_F1 || Menukey.vk == fabgl::VK_JOY1A || Menukey.vk == fabgl::VK_JOY2A) {
                    OSD::restoreBackbufferData(true);
                    click();
                    std::string().swap(menu); // Reset Menu for save free usage
                    return 0;
                }
            }
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);

    }

}

// Run a new menu slot with preview (dirty)
short OSD::menuSlotsWithPreview(const string& new_menu, const string& statusbar, int (*proc_cb)(fabgl::VirtualKeyItem Menukey) ) {

    if (menu_level != 0) return 0; // only menu_level 0

    menuBGColor = zxColor(BLUE, BRIGHT_ON);
    menuFGColor = zxColor(CYAN, BRIGHT_ON);

    fabgl::VirtualKeyItem Menukey;

    menu = new_menu;

    // CRT Overscan compensation
    if (Config::videomode == 2) {
        x = 0;
        if (Config::arch[0] == 'T' && Config::ALUTK == 2) {
            y = Config::aspect_16_9 ? OSD_FONT_H * 2 : OSD_FONT_H;
        } else {
            y = OSD_FONT_H * 3;
        }
    } else {
        x = 0;
        y = 0;
    }

    // Position
    x += (Config::aspect_16_9 ? OSD_FONT_W * 4 : OSD_FONT_W * 3);
    y += OSD_FONT_H;

    // Rows
    real_rows = rowCount(menu);
    virtual_rows = (real_rows > MENU_MAX_ROWS ? MENU_MAX_ROWS : real_rows);

// printf("Cols final: %d\n",cols);

    cols = scrW / OSD_FONT_W - 6 - ( 128 / OSD_FONT_W + 1 );

    // Size
    w = (cols * OSD_FONT_W) + 2 + 128;
    h = ((virtual_rows + (statusbar!=""?1:0) ) * OSD_FONT_H) + 2;

    int rmax = scrW == 320 ? 52 : 55;
    if ( x + cols * OSD_FONT_W > rmax * OSD_FONT_W ) x = ( rmax - cols ) * OSD_FONT_W;

    WindowDraw(); // Draw menu outline

    // Scrollbar Background (don't worry about update or scroll)
    for (uint8_t i = h - OSD_FONT_H - 1; i < h - 2; ++i) {
        VIDEO::line(x + 1, y + i + 1, x + w - 2, y + i + 1, zxColor(CYAN, BRIGHT_OFF));
    }

    if (!use_current_menu_state) {
        begin_row = 1;
        focus = menu_curopt;
        last_begin_row = last_focus = 0;
    }

    menuRedraw(); // Draw menu content

    ResetRowScrollContext(statusBarScrollCTX);

    if (statusbar != "") statusbarDraw(statusbar);

    int idle = 0;
    lastPreviewFile = "";

    while (1) {

        if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEFILEBROWSER);

        ESPeccy::readKbdJoy();

        // Process external keyboard
        if (ESPeccy::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPeccy::readKbd(&Menukey, KBDREAD_MODEFILEBROWSER)) {
                if (!Menukey.down) continue;
                if (proc_cb) {
                    int retcb = proc_cb(Menukey);
                    // 0 = stop this key process (stop propagation)
                    // 1 = continue process
                    // else return with callback return result
                    if (!retcb) continue;
                    if (retcb != 1) {
                        if (menu_level!=0) OSD::restoreBackbufferData(true);
                        use_current_menu_state = false;
                        std::string().swap(menu); // Reset Menu for save free usage
                        return retcb;
                    }
                }
                if (Menukey.vk == fabgl::VK_UP || Menukey.vk == fabgl::VK_JOY1UP || Menukey.vk == fabgl::VK_JOY2UP) {
                    if (focus == 1 and begin_row > 1) {
                        menuScroll(DOWN);
                    } else {
                        last_focus = focus;
                        focus--;
                        if (focus < 1) {
                            focus = virtual_rows - 1;
                            last_begin_row = begin_row;
                            begin_row = real_rows - virtual_rows + 1;
                            menuRedraw();
                            menuPrintRow(focus, IS_FOCUSED);
                        }
                        else {
                            menuPrintRow(focus, IS_FOCUSED);
                            menuPrintRow(last_focus, IS_NORMAL);
                        }
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_DOWN || Menukey.vk == fabgl::VK_JOY1DOWN || Menukey.vk == fabgl::VK_JOY2DOWN) {
                    if (focus == virtual_rows - 1 && virtual_rows + begin_row - 1 < real_rows) {
                        menuScroll(UP);
                    } else {
                        last_focus = focus;
                        focus++;
                        if (focus > virtual_rows - 1) {
                            focus = 1;
                            last_begin_row = begin_row;
                            begin_row = 1;
                            menuRedraw();
                            menuPrintRow(focus, IS_FOCUSED);
                        }
                        else {
                            menuPrintRow(focus, IS_FOCUSED);
                            menuPrintRow(last_focus, IS_NORMAL);
                        }
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEUP || ((Menukey.vk == fabgl::VK_LEFT) && (Config::osd_LRNav == 0)) || Menukey.vk == fabgl::VK_JOY1LEFT || Menukey.vk == fabgl::VK_JOY2LEFT) {
                    if (begin_row > virtual_rows) {
                        focus = 1;
                        begin_row -= virtual_rows - 1;
                    } else {
                        focus = 1;
                        begin_row = 1;
                    }
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEDOWN || ((Menukey.vk == fabgl::VK_RIGHT) && (Config::osd_LRNav == 0)) || Menukey.vk == fabgl::VK_JOY1RIGHT || Menukey.vk == fabgl::VK_JOY2RIGHT) {
                    if (real_rows - begin_row - virtual_rows > virtual_rows) {
                        focus = 1;
                        begin_row += virtual_rows - 1;
                    } else {
                        focus = virtual_rows - 1;
                        begin_row = real_rows - virtual_rows + 1;
                    }
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_HOME) {
                    focus = 1;
                    begin_row = 1;
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_END) {
                    focus = virtual_rows - 1;
                    begin_row = real_rows - virtual_rows + 1;
                    menuRedraw();
                    click();
                } else if (Menukey.vk == fabgl::VK_RETURN || ((Menukey.vk == fabgl::VK_RIGHT) && (Config::osd_LRNav == 1)) || Menukey.vk == fabgl::VK_JOY1B || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2B || Menukey.vk == fabgl::VK_JOY2C) {
                    click();
                    menu_prevopt = menuRealRowFor(focus);
                    use_current_menu_state = false;
                    std::string().swap(menu); // Reset Menu for save free usage
                    return menu_prevopt;
                } else if (Menukey.vk == fabgl::VK_ESCAPE || ((Menukey.vk == fabgl::VK_LEFT) && (Config::osd_LRNav == 1)) || Menukey.vk == fabgl::VK_F1 || Menukey.vk == fabgl::VK_JOY1A || Menukey.vk == fabgl::VK_JOY2A) {
                    if (menu_level!=0) OSD::restoreBackbufferData(true);
                    click();
                    use_current_menu_state = false;
                    std::string().swap(menu); // Reset Menu for save free usage
                    return 0;
                }
            }
        } else {
            // menuPrintRow(focus, IS_FOCUSED);
            if (idle > 20 || Config::instantPreview) {
                uint8_t slotnumber = menuRealRowFor(focus);

                char persistfname[sizeof(DISK_PSNA_FILE) + 7];

                sprintf(persistfname, DISK_PSNA_FILE "%u.sna",slotnumber);

                string _fname = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfname;

                if (lastPreviewFile != _fname ) {
                    int retPreview;
                    lastPreviewFile = _fname;
                    if (_fname[0] != ' ') {
                        rtrim(_fname);
                        retPreview = OSD::renderScreen(x + w - 128 - 1, y + 1 + OSD_FONT_H, _fname.c_str(), 0);

                        // fix issue render overlap (antialiasing issue)
                        VIDEO::line(x + w - 1 - 128, y + h - 1 - OSD_FONT_H, x + w - 2, y + h - 1 - OSD_FONT_H, zxColor(CYAN, BRIGHT_OFF));

                    } else {
                        retPreview = RENDER_PREVIEW_ERROR;
                    }

                    if (retPreview == RENDER_PREVIEW_ERROR) {
                        // Clean Preview Area
                        for (uint8_t i = 0; i < 192 / 2; ++i) {
                            VIDEO::line(x + w - 128 - 1,
                                        y + i + OSD_FONT_H + 1,
                                        x + w - 2,
                                        y + i + OSD_FONT_H + 1,
                                        zxColor(BLUE, BRIGHT_OFF));
                        }

                        VIDEO::line(x + w - 1 - 128, y + OSD_FONT_H + 1              ,
                                    x + w - 2      , y + OSD_FONT_H + 1 + 192 / 2 - 1,
                                    zxColor(RED, BRIGHT_OFF));
                        VIDEO::line(x + w - 1 - 128, y + OSD_FONT_H + 1 + 192 / 2 - 1,
                                    x + w - 2      , y + OSD_FONT_H + 1              ,
                                    zxColor(RED, BRIGHT_OFF));

                    }
                }
            }
            idle++;

        }

        if (statusbar != "") statusbarDraw(statusbar);

        vTaskDelay(5 / portTICK_PERIOD_MS);

    }

}

// Scroll
void OSD::menuScroll(bool dir) {
    if ((dir == DOWN) && (begin_row > 1)) {
        last_begin_row = begin_row;
        begin_row--;
    } else if (dir == UP and (begin_row + virtual_rows - 1) < real_rows) {
        last_begin_row = begin_row;
        begin_row++;
    } else {
        return;
    }
    menuRedraw();
}

// Redraw inside rows
void OSD::menuRedraw() {
    if ((focus != last_focus) || (begin_row != last_begin_row)) {

        for (uint8_t row = 1; row < virtual_rows; row++) {
            if (row == focus) {
                menuPrintRow(row, IS_FOCUSED);
            } else {
                menuPrintRow(row, IS_NORMAL);
            }
        }
        menuScrollBar(begin_row);
        last_focus = focus;
        last_begin_row = begin_row;

    }

}

// Draw menu scroll bar
void OSD::menuScrollBar(unsigned short br) {

    if (real_rows > virtual_rows) {
        // Top handle
        menuAt(1, -1);
        if (br > 1) {
            VIDEO::setTextColor(zxColor(WHITE, BRIGHT_OFF), zxColor(BLACK, BRIGHT_OFF));
            VIDEO::print("+");
        } else {
            VIDEO::setTextColor(zxColor(WHITE, BRIGHT_OFF), zxColor(BLACK, BRIGHT_OFF));
            VIDEO::print("-");
        }

        // Complete bar
        unsigned short holder_x = x + (OSD_FONT_W * (cols - 1)) + 1;
        unsigned short holder_y = y + (OSD_FONT_H * 2);
        unsigned short holder_h = OSD_FONT_H * (virtual_rows - 3);
        unsigned short holder_w = OSD_FONT_W;
        VIDEO::fillRect(holder_x, holder_y, holder_w, holder_h + 1, zxColor(WHITE, BRIGHT_OFF));
        holder_y++;

        // Scroll bar
        float shown_pct = (float)virtual_rows / (float)real_rows * 100.0;
        float begin_pct = (float)(br - 1) / (float)real_rows * 100.0;
        unsigned long bar_h = (float)holder_h / 100.0 * (float)shown_pct;
        unsigned long bar_y = (float)holder_h / 100.0 * (float)begin_pct;

        if (bar_h == 0) bar_h = 1;

        while ((bar_y + bar_h) >= holder_h) {
            bar_y--;
        }

        VIDEO::fillRect(holder_x + 1, holder_y + bar_y, holder_w - 1, bar_h, zxColor(BLACK, BRIGHT_OFF));

        // Bottom handle
        menuAt(-1, -1);
        if ((br + virtual_rows - 1) < real_rows) {
            VIDEO::setTextColor(zxColor(WHITE, BRIGHT_OFF), zxColor(BLACK, BRIGHT_OFF));
            VIDEO::print("+");
        } else {
            VIDEO::setTextColor(zxColor(WHITE, BRIGHT_OFF), zxColor(BLACK, BRIGHT_OFF));
            VIDEO::print("-");
        }
    }
}

// Print a virtual row
void OSD::PrintRow(uint8_t virtual_row_num, uint8_t line_type, bool is_menu) {

    uint8_t margin;

    string line = rowGet(menu, is_menu ? menuRealRowFor(virtual_row_num) : virtual_row_num);

    switch (line_type) {
    case IS_TITLE:
        VIDEO::setTextColor(zxColor(WHITE, BRIGHT_ON), zxColor(BLACK, BRIGHT_OFF));
        margin = 2;
        break;
    case IS_FOCUSED:
        VIDEO::setTextColor(zxColor(BLACK, BRIGHT_ON), zxColor(CYAN, BRIGHT_ON));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    case IS_SELECTED:
        VIDEO::setTextColor(zxColor(BLACK, BRIGHT_ON), zxColor(YELLOW, BRIGHT_ON));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    case IS_SELECTED_FOCUSED:
        VIDEO::setTextColor(zxColor(BLACK, BRIGHT_ON), zxColor(GREEN, BRIGHT_ON));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    default:
        //VIDEO::setTextColor(zxColor(CYAN, BRIGHT_ON), zxColor(BLUE, BRIGHT_ON));
        VIDEO::setTextColor(menuFGColor, menuBGColor);
        margin = (real_rows > virtual_rows ? 3 : 2);
    }

    int tab_pos = line.find(ASCII_TAB);
    if (tab_pos != line.npos) {
        int line_len_without_tab = line.length() - 1;
        int space_fill = cols - margin - line_len_without_tab;
        if ( space_fill < 0 ) {
            int max_first_column_size = cols - margin - ( line_len_without_tab - tab_pos ) /* second_column_len */;
            if (line_type == IS_FOCUSED || line_type == IS_SELECTED_FOCUSED) {
                line = RotateLine(line.substr(0, tab_pos), &rowScrollCTX, max_first_column_size, 125, 25) + line.substr(tab_pos + 1);
            } else {
                line = line.substr(0, max_first_column_size) + line.substr(tab_pos + 1);
            }
        } else {
            line = line.substr(0, tab_pos) // first column
                 + string(space_fill, ' ') // space fill
                 + line.substr(tab_pos + 1); // second column, fixed part, after tab
        }
    }

    menuAt(virtual_row_num, 0);

    VIDEO::print(" ");

    if ((!is_menu || virtual_row_num == 0) && line.substr(0,7) == "ESPeccy") {
        VIDEO::setTextColor(zxColor(RED, BRIGHT_OFF), zxColor(BLACK, BRIGHT_OFF));
        VIDEO::print("ESP");
        VIDEO::setTextColor(zxColor(WHITE, BRIGHT_ON), zxColor(BLACK, BRIGHT_OFF));
        VIDEO::print(("eccy " + Config::arch).c_str());
        for (uint8_t i = line.length(); i < (cols - margin); ++i)
            VIDEO::print(" ");
    } else {
        if (line.length() < cols - margin) {
            VIDEO::print((line + string(cols - margin - line.length(), ' ')).c_str());
        } else {
            VIDEO::print(line.substr(0, cols - margin).c_str());
        }
    }

    VIDEO::print(" ");

}


void OSD::tapemenuStatusbarRedraw() {
    if ( Tape::tapeFileType == TAPE_FTYPE_TAP ) {
        string options;
        if ( !Tape::tapeIsReadOnly ) {
            if (ZXKeyb::Exists || Config::zxunops2) {
                options = Config::lang == 0 ? "\x05+ENT: Select | \x05+\x06+N: Rename | \x05+\x06+M: Move | \x05+\x06+D: Delete " :
                          Config::lang == 1 ? "\x05+ENT: Seleccionar | \x05+\x06+N: Renombrar | \x05+\x06+M: Mover | \x05+\x06+D: Borrar " :
                                              "\x05+ENT: Selecionar | \x05+\x06+N: Renomear | \x05+\x06+M: Mover | \x05+\x06+D: Excluir ";
            } else {
                options = Config::lang == 0 ? "SPC: Select | F2: Rename | F6: Move | F8: Delete " :
                          Config::lang == 1 ? "ESP: Seleccionar | F2: Renombrar | F6: Mover | F8: Borrar " :
                                              "ESP: Selecionar | F2: Renomear | F6: Mover | F8: Excluir ";
            }
        } else {
            options = Config::lang == 0 ? "[Read-Only TAP]" :
                      Config::lang == 1 ? "[TAP de solo lectura]" :
                                          "[TAP somente leitura]";
        }

        statusbarDraw((const string) options);
    }
}


// Redraw inside rows
void OSD::tapemenuRedraw(string title, bool force) {

    if ( force || focus != last_focus || begin_row != last_begin_row ) {
        // Read bunch of rows
        menu = title + "\n";
        if ( Tape::tapeNumBlocks ) {
            for (int i = begin_row - 1; i < virtual_rows + begin_row - 2; ++i) {
                if (i >= Tape::tapeNumBlocks) break;
                if (Tape::tapeFileType == TAPE_FTYPE_TAP)
                    menu += Tape::tapeBlockReadData(i);
                else
                    menu += Tape::tzxBlockReadData(i);
            }
            if ( Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && begin_row - 1 + virtual_rows >= real_rows ) menu += "\n";
        } else {
            menu += Config::lang == 0 ? "<Empty>\n" :
                    Config::lang == 1 ? "<Vacio>\n" :
                                        "<Vazio>\n";
        }

        for (uint8_t row = 1; row < virtual_rows; row++) {
            if (row == focus) {
                PrintRow(row, ( Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Tape::isSelectedBlock(begin_row - 2 + row) ) ? IS_SELECTED_FOCUSED : IS_FOCUSED);
            } else {
                PrintRow(row, ( Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Tape::isSelectedBlock(begin_row - 2 + row) ) ? IS_SELECTED : IS_NORMAL);
            }
        }

        tapemenuStatusbarRedraw();

        menuScrollBar(begin_row);

        last_focus = focus;
        last_begin_row = begin_row;

    }
}

// Tape Browser Menu
int OSD::menuTape(string title) {

    if ( !Tape::tape ) return -1;

    menuBGColor = zxColor(BLUE, BRIGHT_ON);
    menuFGColor = zxColor(CYAN, BRIGHT_ON);

    fabgl::VirtualKeyItem Menukey;

    uint32_t tapeBckPos = ftell(Tape::tape);

    // Tape::TapeListing.erase(Tape::TapeListing.begin(),Tape::TapeListing.begin() + 2);

    Tape::selectedBlocks.clear();

    real_rows = Tape::tapeNumBlocks + 1;
    virtual_rows = (real_rows > 8 ? 8 : real_rows) + ( ( Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly ) ? (Tape::tapeNumBlocks ? 1 : 0) : 0 ); // previous max real_rows 14
    // begin_row = last_begin_row = last_focus = focus = 1;

    // ATENCION: NO ALCANZA LA MEMORIA. PARA LOS DIALOGOS DE CONFIRMACION.
    // Se necesita recargar una vez que se borra un bloque, porque el tamaño de la ventana puede cambiar
    if ( menu_level > 0 && virtual_rows > 8 ) virtual_rows = 8;

    if ( !Tape::tapeNumBlocks ) virtual_rows++;

    if (Tape::tapeCurBlock > 17) {
        begin_row = Tape::tapeCurBlock - 16;
        focus = 18;
    } else{
        begin_row = 1;
        focus = Tape::tapeCurBlock + 1;
    }

    menu_curopt = focus;

    // CRT Overscan compensation
    if (Config::videomode == 2) {
//        x = 18;
        x = 0;
        if (Config::arch[0] == 'T' && Config::ALUTK == 2) {
            y = Config::aspect_16_9 ? OSD_FONT_H * 2 : OSD_FONT_H;
        } else {
            y = OSD_FONT_H * 3;
        }
    } else {
        x = 0;
        y = 0;
    }

    x += (Config::aspect_16_9 ? OSD_FONT_W * 4 : OSD_FONT_W * 3);
    y += OSD_FONT_H;

    // Columns
    cols = 50; // 47 for block info + 2 pre and post space + 1 for scrollbar

    // Size
    w = (cols * OSD_FONT_W) + 2;
    h = ((virtual_rows + ( Tape::tapeFileType == TAPE_FTYPE_TAP ? 1 : 0 )) * OSD_FONT_H) + 2;

    menu = title + "\n";

    WindowDraw();

    last_begin_row = last_focus = 0;

    tapemenuRedraw(title);

    ResetRowScrollContext(statusBarScrollCTX);

    while (1) {

        if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEFILEBROWSER);

        ESPeccy::readKbdJoy();

        // Process external keyboard
        if (ESPeccy::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPeccy::readKbd(&Menukey, KBDREAD_MODEFILEBROWSER)) {

                if (!Menukey.down) continue;

                if (Menukey.vk == fabgl::VK_UP || Menukey.vk == fabgl::VK_JOY1UP || Menukey.vk == fabgl::VK_JOY2UP) {
                    if (focus == 1 && begin_row > 1) {
                        last_begin_row = begin_row;
                        begin_row--;
                        tapemenuRedraw(title);
                    } else if (focus > 1) {
                        last_focus = focus;
                        focus--;
                        PrintRow(focus, ( Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Tape::isSelectedBlock(begin_row - 2 + focus) ) ? IS_SELECTED_FOCUSED : IS_FOCUSED);
                        PrintRow(focus + 1, ( Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Tape::isSelectedBlock(begin_row - 2 + focus + 1) ) ? IS_SELECTED : IS_NORMAL);
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_DOWN || Menukey.vk == fabgl::VK_JOY1DOWN || Menukey.vk == fabgl::VK_JOY2DOWN) {
                    if (focus == virtual_rows - 1) {
                        if ((begin_row + virtual_rows - 1 - ( Tape::tapeFileType == TAPE_FTYPE_TAP ? (!Tape::tapeIsReadOnly ? 1 : 0) : 0 ) ) < real_rows) {
                            last_begin_row = begin_row;
                            begin_row++;
                            tapemenuRedraw(title);
                        }
                    } else if (focus < virtual_rows - 1) {
                        last_focus = focus;
                        focus++;
                        PrintRow(focus, ( Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Tape::isSelectedBlock(begin_row - 2 + focus) ) ? IS_SELECTED_FOCUSED : IS_FOCUSED);
                        PrintRow(focus - 1, ( Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Tape::isSelectedBlock(begin_row - 2 + focus - 1) ) ? IS_SELECTED : IS_NORMAL);
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEUP || Menukey.vk == fabgl::VK_LEFT || Menukey.vk == fabgl::VK_JOY1LEFT || Menukey.vk == fabgl::VK_JOY2LEFT) {
                    last_focus = focus;
                    last_begin_row = begin_row;
                    if (begin_row > virtual_rows) {
                        begin_row -= virtual_rows - 1;
                    } else {
                        begin_row = 1;
                    }
                    focus = 1;
                    tapemenuRedraw(title);
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEDOWN || Menukey.vk == fabgl::VK_RIGHT || Menukey.vk == fabgl::VK_JOY1RIGHT || Menukey.vk == fabgl::VK_JOY2RIGHT) {
                    last_focus = focus;
                    last_begin_row = begin_row;
                    if (real_rows - begin_row - virtual_rows > virtual_rows) {
                        focus = 1;
                        begin_row += virtual_rows - 1;
                    } else {
                        focus = virtual_rows - 1;
                        begin_row = real_rows - virtual_rows + 1 + ( Tape::tapeFileType == TAPE_FTYPE_TAP ? (!Tape::tapeIsReadOnly ? 1 : 0) : 0 );
                    }
                    tapemenuRedraw(title);
                    click();
                } else if (Menukey.vk == fabgl::VK_HOME) {
                    last_focus = focus;
                    last_begin_row = begin_row;
                    focus = 1;
                    begin_row = 1;
                    tapemenuRedraw(title);
                    click();
                } else if (Menukey.vk == fabgl::VK_END) {
                    last_focus = focus;
                    last_begin_row = begin_row;
                    focus = virtual_rows - 1;
                    begin_row = real_rows - virtual_rows + 1 + ( Tape::tapeFileType == TAPE_FTYPE_TAP ? (!Tape::tapeIsReadOnly ? 1 : 0) : 0 );
                    tapemenuRedraw(title);
                    click();
                } else if (Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && (Menukey.vk == fabgl::VK_SPACE || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2C)) {
                    if ( begin_row - 1 + focus < real_rows ) Tape::selectBlockToggle(begin_row - 2 + focus);

                    if (focus == virtual_rows - 1 ) {
                        if ((begin_row + virtual_rows - 2) < real_rows) {
                            last_begin_row = begin_row;
                            begin_row++;
                            tapemenuRedraw(title);
                            click();
                        } else {
                            PrintRow(focus, Tape::isSelectedBlock(begin_row - 2 + focus) ? IS_SELECTED_FOCUSED : IS_FOCUSED);
                        }
                    } else if (focus < virtual_rows - 1) {
                        last_focus = focus;
                        focus++;
                        PrintRow(focus, Tape::isSelectedBlock(begin_row - 2 + focus) ? IS_SELECTED_FOCUSED : IS_FOCUSED);
                        PrintRow(focus - 1, Tape::isSelectedBlock(begin_row - 2 + focus - 1) ? IS_SELECTED : IS_NORMAL);
                        click();
                    }
                } else if (Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Menukey.vk == fabgl::VK_F2 && begin_row - 1 + focus < real_rows) {

                    long current_pos = ftell( Tape::tape );

                    int blocknum = begin_row - 2 + focus;
                    TapeBlock::BlockType blocktype = Tape::getBlockType(blocknum);
                    switch( Tape::getBlockType(begin_row - 2 + focus) ) {
                        case TapeBlock::Program_header:
                        case TapeBlock::Number_array_header:
                        case TapeBlock::Character_array_header:
                        case TapeBlock::Code_header: {
                            uint8_t flags;
                            string new_name = input(21, focus, "", 10, 10, zxColor(BLACK, BRIGHT_OFF), zxColor(WHITE, BRIGHT_OFF), Tape::getBlockName(blocknum), "", &flags);
                            if ( new_name != "" && !(flags & INPUT_CANCELED) ) {
                                Tape::renameBlock( begin_row - 2 + focus, new_name );
                            }
                            tapemenuRedraw(title, true);
                            break;
                        }
                        default:
                            osdCenteredMsg(OSD_BLOCK_TYPE_ERR[Config::lang], LEVEL_WARN, 1000);
                            break;
                    }

                    fseek( Tape::tape, current_pos, SEEK_SET );

                } else if (Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Menukey.vk == fabgl::VK_F6) {
                    click();
                    if ( Tape::selectedBlocks.empty() ) {
                        osdCenteredMsg(OSD_BLOCK_SELECT_ERR[Config::lang], LEVEL_WARN, 1000);
                    } else {
                        Tape::moveSelectedBlocks(begin_row - 2 + focus);
                        tapemenuRedraw(title, true);
                    }
                } else if (Tape::tapeFileType == TAPE_FTYPE_TAP && !Tape::tapeIsReadOnly && Menukey.vk == fabgl::VK_F8) {
                    click();

                    if ( Tape::selectedBlocks.empty() && begin_row - 1 + focus == real_rows ) {
                        osdCenteredMsg(OSD_BLOCK_SELECT_ERR[Config::lang], LEVEL_WARN, 1000);
                    } else {
                        string title = Tape::selectedBlocks.empty() ? MENU_DELETE_CURRENT_TAP_BLOCK[Config::lang] : MENU_DELETE_TAP_BLOCKS[Config::lang];
                        string msg = OSD_DLG_SURE[Config::lang];
                        uint8_t res = msgDialog(title,msg);

                        if (res == DLG_YES) {
                            if ( Tape::selectedBlocks.empty() ) Tape::selectBlockToggle(begin_row - 2 + focus);
                            Tape::removeSelectedBlocks();
                            menu_saverect = true;
                            std::string().swap(menu); // Reset Menu for save free usage
                            return -2;
                        }
                    }

                } else if ( Menukey.vk == fabgl::VK_RETURN /*|| Menukey.vk == fabgl::VK_SPACE*/ || Menukey.vk == fabgl::VK_JOY1B || Menukey.vk == fabgl::VK_JOY2B) {
                    click();
                    if (Tape::tapeFileType == TAPE_FTYPE_TAP) {
                        if ( begin_row - 1 + focus < real_rows ) {
                            Tape::CalcTapBlockPos(begin_row + focus - 2);
                            // printf("Ret value: %d\n", begin_row + focus - 2);
                            std::string().swap(menu); // Reset Menu for save free usage
                            return (begin_row + focus - 2);
                        }
                    } else {
                        Tape::CalcTZXBlockPos(begin_row + focus - 2);
                        // printf("Ret value: %d\n", begin_row + focus - 2);
                        std::string().swap(menu); // Reset Menu for save free usage
                        return (begin_row + focus - 2);
                    }
                } else if (Menukey.vk == fabgl::VK_ESCAPE || Menukey.vk == fabgl::VK_JOY1A || Menukey.vk == fabgl::VK_JOY2A) {

                    // if (Tape::tapeStatus==TAPE_LOADING) {
                        fseek(Tape::tape, tapeBckPos, SEEK_SET);
                    // }

                    if (menu_level!=0) OSD::restoreBackbufferData(true);
                    click();
                    std::string().swap(menu); // Reset Menu for save free usage
                    return -1;
                }
            }
        }

        tapemenuStatusbarRedraw();

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}


/* Cheats */

size_t OSD::rowCountCheat(void *data) {
    return CheatMngr::getCheatCount();
}

size_t OSD::colsCountCheat(void *data) {
    // 30 Cheat name
    // 1 tab
    // 1 space
    // 3 [?] or '   ' (3 spaces)
    // 1 space
    // 3 [*]
    // ----
    // 40 totals
    return 40;
}

void OSD::menuRedrawCheat(const string& title, bool force) {
    if ( force || focus != last_focus || begin_row != last_begin_row ) {
        // Read bunch of rows
        menu = title + "\n";
        size_t numRows = CheatMngr::getCheatCount();
        if ( numRows ) {
            for (int i = begin_row - 1; i < virtual_rows + begin_row - 2 && i < numRows; ++i) {
                Cheat cheat = CheatMngr::getCheat(i);
                if (cheat.pokeCount) { // this allways must be true
                    menu += CheatMngr::getCheatName(cheat)/*.substr(0,30)*/ + "\t" + ((cheat.inputCount) ? " +" : "  " ) + " [" + (cheat.enabled ? "*" : " ") + "]\n";
                }
            }
        }

        for (uint8_t row = 1; row < virtual_rows; row++) {
            PrintRow(row, row == focus ? IS_FOCUSED : IS_NORMAL);
        }

        menuScrollBar(begin_row);

        last_focus = focus;
        last_begin_row = begin_row;

    }
}

int OSD::menuProcessCheat(fabgl::VirtualKeyItem Menukey) {
    int idx = menuRealRowFor( focus );

    if ( menu_level == 0 && ((Menukey.vk == fabgl::VK_LEFT && Config::osd_LRNav == 1) || Menukey.vk == fabgl::VK_JOY1B || Menukey.vk == fabgl::VK_JOY2B )) {
        return 0;

    } else
    if (Menukey.vk == fabgl::VK_F2) {
        click();
        return SHRT_MIN;

    } else
    if ((Menukey.vk == fabgl::VK_SPACE || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2C ) && begin_row - 1 + focus < real_rows) {
        click();

        CheatMngr::toggleCheat(idx - 1);

        // Move cursor to next row
        if (focus == virtual_rows - 1 && virtual_rows + begin_row - 1 < real_rows) {
            last_begin_row = begin_row;
            begin_row++;

        } else {
            last_focus = focus;
            focus++;
            if (focus > virtual_rows - 1) {
                focus = 1;
                last_begin_row = begin_row;
                begin_row = 1;
            }
        }

        return SHRT_MAX;

    } else
    if ((Menukey.vk == fabgl::VK_RETURN || (Menukey.vk == fabgl::VK_RIGHT && Config::osd_LRNav == 1) || Menukey.vk == fabgl::VK_JOY1A || Menukey.vk == fabgl::VK_JOY2A ) && begin_row - 1 + focus < real_rows) {
        click();

        Cheat t = CheatMngr::getCheat(idx - 1);

        if ( t.inputCount > 0 ) {
            currentCheat = t;
            return -idx;
        }

        return 0;

    }

    return 1;
}

size_t OSD::rowCountPoke(void *data) {
    return currentCheat.inputCount;
}

size_t OSD::colsCountPoke(void *data) {
    // 6 poke number
    // 1 :
    // 1 tab
    // 1 space
    // 4 input
    // ----
    // 13 totals
    return 13;
}

void OSD::menuRedrawPoke(const string& title, bool force) {
    if ( force || focus != last_focus || begin_row != last_begin_row ) {
        // Read bunch of rows
        menu = title + "\n";
        size_t numRows = currentCheat.inputCount;
        if ( numRows ) {
            for (int i = begin_row - 1; i < virtual_rows + begin_row - 2 && i < numRows; ++i) {
                Poke poke = CheatMngr::getInputPoke(currentCheat, i);
                string value = to_string(poke.value);
                menu += to_string( i + 1 ) + ":\t " + ((value.size() < 3) ? value.insert(0, 3 - value.size(), ' ') : value ) + "\n";
            }
        }

        for (uint8_t row = 1; row < virtual_rows; row++) {
            PrintRow(row, row == focus ? IS_FOCUSED : IS_NORMAL);
        }

        menuScrollBar(begin_row);

        last_focus = focus;
        last_begin_row = begin_row;

    }
}

int OSD::menuProcessPokeInput(fabgl::VirtualKeyItem Menukey) {
    int idx = menuRealRowFor( focus );

    if ((Menukey.vk == fabgl::VK_RETURN ||
        (Menukey.vk == fabgl::VK_RIGHT && Config::osd_LRNav == 1) ||
         Menukey.vk == fabgl::VK_JOY1B ||
         Menukey.vk == fabgl::VK_JOY1C ||
         Menukey.vk == fabgl::VK_JOY2B ||
         Menukey.vk == fabgl::VK_JOY2C )
                                         && idx - 1 < currentCheat.inputCount) {
        click();
        uint8_t flags = 0;

        Poke poke = CheatMngr::getInputPoke(currentCheat, idx - 1);
//        string value = to_string(poke.value);
        string new_value = input(cols - 5, focus, "", 3, 3, zxColor(BLACK, BRIGHT_OFF), zxColor(WHITE, BRIGHT_OFF), "", "0123456789", &flags, FILTER_ALLOWED );
        if ( !( flags & INPUT_CANCELED ) && new_value != "" ) { // if not canceled
            int nv = stoi(new_value);
            if ( nv < 256 ) poke = CheatMngr::setPokeValue(currentCheat, idx - 1, (uint8_t) nv);
            string value = to_string(poke.value);
            string new_row = to_string( idx ) + ":\t " + ((value.size() < 3) ? value.insert(0, 3 - value.size(), ' ') : value ) + " ";
            OSD::menu = OSD::rowReplace(menu, idx, new_row);
        }
        return 0;

    }

    return 1;
}


// Run a new menu
short OSD::menuGenericRun(const string& title, const string& statusbar, void *user_data, size_t (*rowCount)(void *), size_t (*colsCount)(void *), void (*menuRedraw)(const string&, bool), int (*proc_cb)(fabgl::VirtualKeyItem Menukey) ) {

    fabgl::VirtualKeyItem Menukey;

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
        y += OSD_FONT_H;
        prev_y[0] = 0;
    } else {
        x += (Config::aspect_16_9 ? OSD_FONT_W * 4 : OSD_FONT_W * 3) + (54 /*60*/ * menu_level);
        if (menu_saverect && !prev_y[menu_level]) {
            y += (4 + (OSD_FONT_H * menu_prevopt));
            prev_y[menu_level] = y;
        } else {
            y = prev_y[menu_level];
        }
    }

    for ( int i = menu_level + 1; i < 5; ++i ) prev_y[i] = 0;

    // Rows
    real_rows = rowCount(user_data) + 1;
    virtual_rows = (real_rows > MENU_MAX_ROWS ? MENU_MAX_ROWS : real_rows);

    // Columns
    cols = colsCount(user_data);

    if (title.size() + 7 > cols) cols = title.size() + 7; // colors bars in title

    int max_cols = (menu_level == 0 ? 41 : 31);

    cols += (real_rows > virtual_rows ? 1 : 0); // For scrollbar

//    if ( statusbar != "" && cols < statusbar.length() + 1 ) cols = statusbar.length() + 1;

    if (cols > max_cols) cols = max_cols;

    // Size
    w = (cols * OSD_FONT_W) + 2;
    h = ((virtual_rows + ( statusbar != "" ? 1 : 0 ) ) * OSD_FONT_H) + 2;

    int rmax = scrW == 320 ? 52 : 55;
    if ( x + cols * OSD_FONT_W > rmax * OSD_FONT_W ) x = ( rmax - cols ) * OSD_FONT_W;

    menu = title + "\n"; // for row 0 (remember fix this)

    WindowDraw(); // Draw menu outline

    if (!use_current_menu_state) {
        begin_row = 1;
        focus = menu_curopt;
        last_begin_row = last_focus = 0;
    }

    menuRedraw(title, true); // Draw menu content

    ResetRowScrollContext(statusBarScrollCTX);

    if (statusbar != "") statusbarDraw(statusbar);

    while (1) {

        if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEFILEBROWSER);

        ESPeccy::readKbdJoy();

        // Process external keyboard
        if (ESPeccy::PS2Controller.keyboard()->virtualKeyAvailable()) {

            ResetRowScrollContext(rowScrollCTX);

            if (ESPeccy::readKbd(&Menukey, KBDREAD_MODEFILEBROWSER)) {
                if (!Menukey.down) continue;
                if (proc_cb) {
                    int retcb = proc_cb(Menukey);
                    // 0 = stop this key process (stop propagation)
                    // 1 = continue process
                    // else return with callback return result
                    if (!retcb) {
                        menuRedraw(title, true);
                        continue;
                    }
                    if (retcb != 1) {
                        if (menu_level!=0) OSD::restoreBackbufferData(true);
                        use_current_menu_state = false;
                        std::string().swap(menu); // Reset Menu for save free usage
                        return retcb;
                    }
                }
                if (Menukey.vk == fabgl::VK_UP || Menukey.vk == fabgl::VK_JOY1UP || Menukey.vk == fabgl::VK_JOY2UP) {
                    if (focus == 1 && begin_row > 1) {
                        last_begin_row = begin_row;
                        begin_row--;
                        menuRedraw(title, false);
                    } else {
                        last_focus = focus;
                        focus--;
                        if (focus < 1) {
                            focus = virtual_rows - 1;
                            last_begin_row = begin_row;
                            begin_row = real_rows - virtual_rows + 1;
                            menuRedraw(title, false);
                        } else {
                            PrintRow(last_focus, IS_NORMAL);
                        }
                        PrintRow(focus, IS_FOCUSED);
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_DOWN || Menukey.vk == fabgl::VK_JOY1DOWN || Menukey.vk == fabgl::VK_JOY2DOWN) {
                    if (focus == virtual_rows - 1 && virtual_rows + begin_row - 1 < real_rows) {
                        last_begin_row = begin_row;
                        begin_row++;
                        menuRedraw(title, false);
                    } else {
                        last_focus = focus;
                        focus++;
                        if (focus > virtual_rows - 1) {
                            focus = 1;
                            last_begin_row = begin_row;
                            begin_row = 1;
                            menuRedraw(title, false);
                        } else {
                            PrintRow(last_focus, IS_NORMAL);
                        }
                        PrintRow(focus, IS_FOCUSED);
                    }
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEUP || ((Menukey.vk == fabgl::VK_LEFT) && (Config::osd_LRNav == 0)) || Menukey.vk == fabgl::VK_JOY1LEFT || Menukey.vk == fabgl::VK_JOY2LEFT) {
                    if (begin_row > virtual_rows) {
                        last_focus = focus;
                        last_begin_row = begin_row;
                        begin_row -= virtual_rows - 1;
                    } else {
                        begin_row = 1;
                    }
                    focus = 1;
                    menuRedraw(title, false);
                    click();
                } else if (Menukey.vk == fabgl::VK_PAGEDOWN || ((Menukey.vk == fabgl::VK_RIGHT) && (Config::osd_LRNav == 0)) || Menukey.vk == fabgl::VK_JOY1RIGHT || Menukey.vk == fabgl::VK_JOY2RIGHT) {
                    last_focus = focus;
                    last_begin_row = begin_row;
                    if (real_rows - begin_row - virtual_rows > virtual_rows) {
                        focus = 1;
                        begin_row += virtual_rows - 1;
                    } else {
                        focus = virtual_rows - 1;
                        begin_row = real_rows - virtual_rows + 1;
                    }
                    menuRedraw(title, false);
                    click();
                } else if (Menukey.vk == fabgl::VK_HOME) {
                    last_focus = focus;
                    last_begin_row = begin_row;
                    focus = 1;
                    begin_row = 1;
                    menuRedraw(title, false);
                    click();
                } else if (Menukey.vk == fabgl::VK_END) {
                    last_focus = focus;
                    last_begin_row = begin_row;
                    focus = virtual_rows - 1;
                    begin_row = real_rows - virtual_rows + 1;
                    menuRedraw(title, false);
                    click();
                } else if (Menukey.vk == fabgl::VK_RETURN || ((Menukey.vk == fabgl::VK_RIGHT) && (Config::osd_LRNav == 1)) || Menukey.vk == fabgl::VK_JOY1B || Menukey.vk == fabgl::VK_JOY1C || Menukey.vk == fabgl::VK_JOY2B || Menukey.vk == fabgl::VK_JOY2C) {
                    click();
                    use_current_menu_state = false;
                    std::string().swap(menu); // Reset Menu for save free usage
                    return begin_row + focus - 1;
                } else if (Menukey.vk == fabgl::VK_ESCAPE || ((Menukey.vk == fabgl::VK_LEFT) && (Config::osd_LRNav == 1)) || Menukey.vk == fabgl::VK_F1 || Menukey.vk == fabgl::VK_JOY1A || Menukey.vk == fabgl::VK_JOY2A) {
                    if (menu_level!=0) OSD::restoreBackbufferData(true);
                    click();
                    use_current_menu_state = false;
                    std::string().swap(menu); // Reset Menu for save free usage
                    return 0;
                }

            }

        } else {
            PrintRow(focus, IS_FOCUSED);

        }

        if (statusbar != "") statusbarDraw(statusbar);

        vTaskDelay(5 / portTICK_PERIOD_MS);

    }

}
