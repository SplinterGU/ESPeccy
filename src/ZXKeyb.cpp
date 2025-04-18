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


#include "ZXKeyb.h"
#include "ESPeccy.h"
#include "RealTape.h"
#include "Config.h"

uint8_t ZXKeyb::ZXcols[8] = { 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf };
uint8_t ZXKeyb::Exists = 0;

void ZXKeyb::setup()
{
    // setup shift register pins as outputs
    gpio_set_direction((gpio_num_t)SR_CLK, (gpio_mode_t)GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)SR_LOAD, (gpio_mode_t)GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)SR_DATA, (gpio_mode_t)GPIO_MODE_OUTPUT);

    // setup columns as inputs
    gpio_set_direction((gpio_num_t)KM_COL_0, (gpio_mode_t)GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)KM_COL_1, (gpio_mode_t)GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)KM_COL_2, (gpio_mode_t)GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)KM_COL_3, (gpio_mode_t)GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)KM_COL_4, (gpio_mode_t)GPIO_MODE_INPUT);

    // Check if membrane keyboard is present
    putRows(0xFF);
    if (gpio_get_level((gpio_num_t)KM_COL_1) && gpio_get_level((gpio_num_t)KM_COL_2) && gpio_get_level((gpio_num_t)KM_COL_4)) Exists = 1;

}

void ZXKeyb::process() {

    // Put row pattern to 8-tap membrane connector (using shift register)
    // and read column pattern from 5-tap membrane connector.
    // row order depends on actual row association with address lines, see
    // https://www.1000bit.it/support/manuali/sinclair/zxspectrum/sm/matrix.gif
    putRows(0b11011111); ZXcols[0] = getCols();
    putRows(0b11111011); ZXcols[1] = getCols();
    putRows(0b11111101); ZXcols[2] = getCols();
    putRows(0b11111110); ZXcols[3] = getCols();
    putRows(0b11110111); ZXcols[4] = getCols();
    putRows(0b11101111); ZXcols[5] = getCols();
    putRows(0b10111111); ZXcols[6] = getCols();
    putRows(0b01111111); ZXcols[7] = getCols();

}

// This function puts a row pattern into the membrane keyboard, for selecting a given row.
// Selection logic is active low, a 0 bit will select the row.
// A shift register is used for this task, so we'll need 3 output pins instead of 8.
void ZXKeyb::putRows(uint8_t row_pattern) {

    // NOTICE: many delays have been commented out.
    // If keyboard readings are erratic, maybe they should be recovered.

    gpio_set_level((gpio_num_t)SR_LOAD, 0); // disable load pin, keep previous output
    // delayMicroseconds(1);

    for (uint8_t i = 0; i < 8; i++) {   // traverse bits in byte

        gpio_set_level((gpio_num_t)SR_CLK, 0);  // clock falling edge
        // delayMicroseconds(1);

        gpio_set_level((gpio_num_t)SR_DATA, row_pattern & 0x80);    // put row bit to shift register serial input

        delayMicroseconds(1);   // just to be safe, wait just before rising edge

        gpio_set_level((gpio_num_t)SR_CLK, 1);  // rising edge occurs here
        // delayMicroseconds(1);

        row_pattern <<= 1;  // shift row bit pattern

    }

    gpio_set_level((gpio_num_t)SR_LOAD, 1); // enable load pin, update output

    // this sleep is MANDATORY, do NOT remove it
    // or else (first column bits read will be wrong)
    delayMicroseconds(1);

}

// This function reads all 5 columns from the corresponding GPIO pins and concatenates them
// into the lowest 5 bits of a byte.
uint8_t ZXKeyb::getCols() {

    uint8_t cols = gpio_get_level((gpio_num_t)KM_COL_4) << 1;
    cols |= gpio_get_level((gpio_num_t)KM_COL_3); cols <<= 1;
    cols |= gpio_get_level((gpio_num_t)KM_COL_2); cols <<= 1;
    cols |= gpio_get_level((gpio_num_t)KM_COL_1); cols <<= 1;
    if (RealTape_enabled && Config::realtape_gpio_num == KM_COL_0)
        cols |= 1;
    else
        cols |= gpio_get_level((gpio_num_t)KM_COL_0);
    cols |= 0xa0;   // Keep bits 5,7 up

    return cols;

}

void ZXKeyb::ZXKbdRead(uint8_t mode) {

    #define REPDEL 140 // As in real ZX Spectrum (700 ms.) if this function is called every 5 ms.
    #define REPPER 20 // As in real ZX Spectrum (100 ms.) if this function is called every 5 ms.

    static int zxDel = REPDEL;
    static int lastzxK = fabgl::VK_NONE;
    static bool lastSSstatus = !ZXKBD_SS;
    static bool lastCSstatus = !ZXKBD_CS;

    bool shift_down = false, ctrl_down = false;

    process();

    fabgl::VirtualKey injectKey = fabgl::VK_NONE;

    switch(mode) {
        case KBDREAD_MODENORMAL:
            // Detect and process physical kbd menu key combinations
            // CS+SS+<1..0> -> F1..F10 Keys, CS+SS+Q -> F11, CS+SS+W -> F12, CS+SS+S -> Capture screen
            if (ZXKBD_CS && ZXKBD_SS) {
                     if (ZXKBD_1)       {                                           injectKey = fabgl::VK_F1;               }   // Main Menu
                else if (ZXKBD_2)       {                                           injectKey = fabgl::VK_F2;               }   // Load State
                else if (ZXKBD_3)       {   shift_down = true;                      injectKey = fabgl::VK_F2;               }   // Save State
                else if (ZXKBD_5)       {                                           injectKey = fabgl::VK_F5;               }   // File Browser
                else if (ZXKBD_6)       {   shift_down = true;                      injectKey = fabgl::VK_F5;               }   // Tape Browser
                else if (ZXKBD_7)       {                                           injectKey = fabgl::VK_F6;               }   // Play/Stop
                else if (ZXKBD_8)       {                                           injectKey = fabgl::VK_F8;               }   // CPU Stats
                else if (ZXKBD_9)       {                                           injectKey = fabgl::VK_F9;               }   // Vol-
                else if (ZXKBD_0)       {                                           injectKey = fabgl::VK_F10;              }   // Vol+
                else if (ZXKBD_Q)       {                                           injectKey = fabgl::VK_F11;              }   // Hard Reset
                else if (ZXKBD_W)       {                                           injectKey = fabgl::VK_F12;              }   // ESP32 Reset
                else if (ZXKBD_P)       {                                           injectKey = fabgl::VK_PAUSE;            }   // P -> Pause
                else if (ZXKBD_I)       {   shift_down = true;                      injectKey = fabgl::VK_F1;               }   // I -> INFO
                else if (ZXKBD_E)       {   shift_down = true;                      injectKey = fabgl::VK_F6;               }   // E -> Eject tape
                else if (ZXKBD_R)       {                       ctrl_down = true;   injectKey = fabgl::VK_F11;              }   // R -> Reset to TR-DOS
                else if (ZXKBD_T)       {                       ctrl_down = true;   injectKey = fabgl::VK_F2;               }   // T -> Turbo
                else if (ZXKBD_B)       {   shift_down = true;                      injectKey = fabgl::VK_PRINTSCREEN;      }   // B -> BMP capture
                else if (ZXKBD_O)       {                       ctrl_down = true;   injectKey = fabgl::VK_F9;               }   // O -> Poke
                else if (ZXKBD_U)       {   shift_down = true;                      injectKey = fabgl::VK_F9;               }   // U -> Cheats
                else if (ZXKBD_N)       {                       ctrl_down = true;   injectKey = fabgl::VK_F10;              }   // N -> NMI
                else if (ZXKBD_K)       {                       ctrl_down = true;   injectKey = fabgl::VK_F1;               }   // K -> Help / Kbd layout
                else if (ZXKBD_G)       {                                           injectKey = fabgl::VK_PRINTSCREEN;      }   // G -> Capture SCR
                else if (ZXKBD_Z)       {                       ctrl_down = true;   injectKey = fabgl::VK_F5;               }   // Z -> CenterH
                else if (ZXKBD_X)       {                       ctrl_down = true;   injectKey = fabgl::VK_F6;               }   // X -> CenterH
                else if (ZXKBD_C)       {                       ctrl_down = true;   injectKey = fabgl::VK_F7;               }   // C -> CenterV
                else if (ZXKBD_V)       {                       ctrl_down = true;   injectKey = fabgl::VK_F8;               }   // V -> CenterV
            }
            break;

        case KBDREAD_MODEFILEBROWSER:
            if (ZXKBD_CS && !ZXKBD_SS) { // CS + !SS
                     if (ZXKBD_ENTER)   {                                           injectKey = fabgl::VK_JOY1C;            }   // CS + ENTER -> SPACE / SELECT
                else if (ZXKBD_SPACE)   {                                           injectKey = fabgl::VK_ESCAPE;           }   // BREAK -> ESCAPE
                else if (ZXKBD_0)       {                                           injectKey = fabgl::VK_BACKSPACE;        }   // CS + 0 -> BACKSPACE
                else if (ZXKBD_7)       {                                           injectKey = fabgl::VK_UP;               }   // 7 -> VK_UP
                else if (ZXKBD_6)       {                                           injectKey = fabgl::VK_DOWN;             }   // 6 -> VK_DOWN
                else if (ZXKBD_5)       {                                           injectKey = fabgl::VK_LEFT;             }   // 5 -> VK_LEFT
                else if (ZXKBD_8)       {                                           injectKey = fabgl::VK_RIGHT;            }   // 8 -> VK_RIGHT
            }
            else
            if (ZXKBD_CS && ZXKBD_SS) {
                     if (ZXKBD_7)       {                                           injectKey = fabgl::VK_PAGEUP;           }   // 7 -> VK_PAGEUP
                else if (ZXKBD_6)       {                                           injectKey = fabgl::VK_PAGEDOWN;         }   // 6 -> VK_PAGEDOWN
                else if (ZXKBD_5)       {   shift_down = true;                      injectKey = fabgl::VK_LEFT;             }   // 5 -> SS + VK_LEFT
                else if (ZXKBD_8)       {   shift_down = true;                      injectKey = fabgl::VK_RIGHT;            }   // 8 -> SS + VK_RIGHT
                else if (ZXKBD_G)       {                                           injectKey = fabgl::VK_PRINTSCREEN;      }   // G -> USE THIS SCR
                else if (ZXKBD_N)       {                                           injectKey = fabgl::VK_F2;               }   // N -> NUEVO / RENOMBRAR
                else if (ZXKBD_R)       {   shift_down = true;                      injectKey = fabgl::VK_F2;               }   // R -> NUEVO Con ROM
                else if (ZXKBD_M)       {                                           injectKey = fabgl::VK_F6;               }   // M -> MOVE / MOVER
                else if (ZXKBD_D)       {                                           injectKey = fabgl::VK_F8;               }   // D -> DELETE / BORRAR
                else if (ZXKBD_F)       {                                           injectKey = fabgl::VK_F3;               }   // F -> FIND / BUSQUEDA
                else if (ZXKBD_Z)       {                       ctrl_down = true;   injectKey = fabgl::VK_LEFT;             }   // Z -> CTRL + LEFT
                else if (ZXKBD_X)       {                       ctrl_down = true;   injectKey = fabgl::VK_RIGHT;            }   // X -> CTRL + RIGHT
                else if (ZXKBD_C)       {                       ctrl_down = true;   injectKey = fabgl::VK_UP;               }   // C -> CTRL + UP
                else if (ZXKBD_V)       {                       ctrl_down = true;   injectKey = fabgl::VK_DOWN;             }   // V -> CTRL + DOWN
            }
            break;

        case KBDREAD_MODEINPUT:
        {
            if (ZXKBD_CS && !ZXKBD_SS) { // CS
                     if (ZXKBD_7)       {                                           injectKey = fabgl::VK_END;              }   // 7 -> END
                else if (ZXKBD_6)       {                                           injectKey = fabgl::VK_HOME;             }   // 6 -> HOME
                else if (ZXKBD_5)       {                                           injectKey = fabgl::VK_LEFT;             }   // 5 -> LEFT
                else if (ZXKBD_8)       {                                           injectKey = fabgl::VK_RIGHT;            }   // 8 -> RIGHT
                else if (ZXKBD_0)       {                                           injectKey = fabgl::VK_BACKSPACE;        }   // CS + 0 -> BACKSPACE
                else if (ZXKBD_SPACE)   {                                           injectKey = fabgl::VK_ESCAPE;           }   // CS + SPACE && !SS -> ESCAPE
            }
            break;
        }

        case KBDREAD_MODEINPUTMULTI:
        {
            if (ZXKBD_CS && !ZXKBD_SS) { // CS
                     if (ZXKBD_7)       {                                           injectKey = fabgl::VK_UP;               }   // 7 -> VK_UP
                else if (ZXKBD_6)       {                                           injectKey = fabgl::VK_DOWN;             }   // 6 -> VK_DOWN
                else if (ZXKBD_5)       {                                           injectKey = fabgl::VK_LEFT;             }   // 5 -> LEFT
                else if (ZXKBD_8)       {                                           injectKey = fabgl::VK_RIGHT;            }   // 8 -> RIGHT
                else if (ZXKBD_0)       {                                           injectKey = fabgl::VK_BACKSPACE;        }   // CS + 0 -> BACKSPACE
                else if (ZXKBD_SPACE)   {                                           injectKey = fabgl::VK_ESCAPE;           }   // CS + SPACE && !SS -> ESCAPE
            }
            break;
        }

        case KBDREAD_MODEKBDLAYOUT:
                 if (ZXKBD_SPACE)       {                                           injectKey = fabgl::VK_ESCAPE;           }   // SPACE -> ESCAPE
            else if (ZXKBD_ENTER)       {                                           injectKey = fabgl::VK_ESCAPE;           }   // RETURN -> ESCAPE
            else if (ZXKBD_CS && ZXKBD_SS) { // CS + SS
                if (ZXKBD_K)            {                                           injectKey = fabgl::VK_ESCAPE;           }
            }
            else if (ZXKBD_CS && !ZXKBD_SS) { // CS
                     if (ZXKBD_5)       {                                           injectKey = fabgl::VK_LEFT;             }   // 5 -> LEFT
                else if (ZXKBD_8)       {                                           injectKey = fabgl::VK_RIGHT;            }   // 8 -> RIGHT
            }
            break;

        case KBDREAD_MODEDIALOG:
            if (ZXKBD_CS && !ZXKBD_SS) { // CS
                     if (ZXKBD_5)       {                                           injectKey = fabgl::VK_LEFT;             }   // 5 -> LEFT
                else if (ZXKBD_8)       {                                           injectKey = fabgl::VK_RIGHT;            }   // 8 -> RIGHT
                else if (ZXKBD_SPACE)   {                                           injectKey = fabgl::VK_ESCAPE;           }   // CS + SPACE && !SS -> ESCAPE
            }
            else if (ZXKBD_CS && ZXKBD_SS) { // CS + SS
                if (ZXKBD_I)            {                                           injectKey = fabgl::VK_F1;               }   // For exit from HWInfo
            }
            break;

        case KBDREAD_MODEBIOS:
            if (ZXKBD_CS && !ZXKBD_SS) { // CS
                     if (ZXKBD_7)       {                                           injectKey = fabgl::VK_UP;               }   // 7 -> VK_UP
                else if (ZXKBD_6)       {                                           injectKey = fabgl::VK_DOWN;             }   // 6 -> VK_DOWN
                else if (ZXKBD_5)       {                                           injectKey = fabgl::VK_LEFT;             }   // 5 -> LEFT
                else if (ZXKBD_8)       {                                           injectKey = fabgl::VK_RIGHT;            }   // 8 -> RIGHT
                else if (ZXKBD_0)       {                                           injectKey = fabgl::VK_BACKSPACE;        }   // CS + 0 -> BACKSPACE
                else if (ZXKBD_SPACE)   {                                           injectKey = fabgl::VK_ESCAPE;           }   // CS + SPACE && !SS -> ESCAPE
            }
            else
            if (!ZXKBD_CS && ZXKBD_SS) { // SS
                if (ZXKBD_S || ZXKBD_0) {                                           injectKey = fabgl::VK_F10;              }
            }
            break;

    }


    if (injectKey == fabgl::VK_NONE) {
        ctrl_down = ZXKBD_SS;
        shift_down = ZXKBD_CS;

        bool curSSstatus = ZXKBD_SS; // SS
        if (lastSSstatus != curSSstatus) {
            ESPeccy::PS2Controller.keyboard()->injectVirtualKey(fabgl::VK_LCTRL, curSSstatus, false);
            lastSSstatus = curSSstatus;
        }

        bool curCSstatus = ZXKBD_CS; // CS
        if (lastCSstatus != curCSstatus) {
            ESPeccy::PS2Controller.keyboard()->injectVirtualKey(fabgl::VK_LSHIFT, curCSstatus, false);
            lastCSstatus = curCSstatus;
        }

             if (ZXKBD_Z) injectKey = ZXKBD_CS ? fabgl::VK_Z : fabgl::VK_z;
        else if (ZXKBD_X) injectKey = ZXKBD_CS ? fabgl::VK_X : fabgl::VK_x;
        else if (ZXKBD_C) injectKey = ZXKBD_CS ? fabgl::VK_C : fabgl::VK_c;
        else if (ZXKBD_V) injectKey = ZXKBD_CS ? fabgl::VK_V : fabgl::VK_v;

        else if (ZXKBD_A) injectKey = ZXKBD_CS ? fabgl::VK_A : fabgl::VK_a;
        else if (ZXKBD_S) injectKey = ZXKBD_CS ? fabgl::VK_S : fabgl::VK_s;
        else if (ZXKBD_D) injectKey = ZXKBD_CS ? fabgl::VK_D : fabgl::VK_d;
        else if (ZXKBD_F) injectKey = ZXKBD_CS ? fabgl::VK_F : fabgl::VK_f;
        else if (ZXKBD_G) injectKey = ZXKBD_CS ? fabgl::VK_G : fabgl::VK_g;

        else if (ZXKBD_Q) injectKey = ZXKBD_CS ? fabgl::VK_Q : fabgl::VK_q;
        else if (ZXKBD_W) injectKey = ZXKBD_CS ? fabgl::VK_W : fabgl::VK_w;
        else if (ZXKBD_E) injectKey = ZXKBD_CS ? fabgl::VK_E : fabgl::VK_e;
        else if (ZXKBD_R) injectKey = ZXKBD_CS ? fabgl::VK_R : fabgl::VK_r;
        else if (ZXKBD_T) injectKey = ZXKBD_CS ? fabgl::VK_T : fabgl::VK_t;

        else if (ZXKBD_P) injectKey = ZXKBD_CS ? fabgl::VK_P : fabgl::VK_p;
        else if (ZXKBD_O) injectKey = ZXKBD_CS ? fabgl::VK_O : fabgl::VK_o;
        else if (ZXKBD_I) injectKey = ZXKBD_CS ? fabgl::VK_I : fabgl::VK_i;
        else if (ZXKBD_U) injectKey = ZXKBD_CS ? fabgl::VK_U : fabgl::VK_u;
        else if (ZXKBD_Y) injectKey = ZXKBD_CS ? fabgl::VK_Y : fabgl::VK_y;

        else if (ZXKBD_ENTER) injectKey = fabgl::VK_RETURN;
        else if (ZXKBD_L) injectKey = ZXKBD_CS ? fabgl::VK_L : fabgl::VK_l;
        else if (ZXKBD_K) injectKey = ZXKBD_CS ? fabgl::VK_K : fabgl::VK_k;
        else if (ZXKBD_J) injectKey = ZXKBD_CS ? fabgl::VK_J : fabgl::VK_j;
        else if (ZXKBD_H) injectKey = ZXKBD_CS ? fabgl::VK_H : fabgl::VK_h;

        else if (ZXKBD_SPACE) injectKey = fabgl::VK_SPACE;
        else if (ZXKBD_M) injectKey = ZXKBD_CS ? fabgl::VK_M : fabgl::VK_m;
        else if (ZXKBD_N) injectKey = ZXKBD_CS ? fabgl::VK_N : fabgl::VK_n;
        else if (ZXKBD_B) injectKey = ZXKBD_CS ? fabgl::VK_B : fabgl::VK_b;

        else if (ZXKBD_1) injectKey = fabgl::VK_1;
        else if (ZXKBD_2) injectKey = fabgl::VK_2;
        else if (ZXKBD_3) injectKey = fabgl::VK_3;
        else if (ZXKBD_4) injectKey = fabgl::VK_4;
        else if (ZXKBD_5) injectKey = fabgl::VK_5;

        else if (ZXKBD_0) injectKey = fabgl::VK_0;
        else if (ZXKBD_9) injectKey = fabgl::VK_9;
        else if (ZXKBD_8) injectKey = fabgl::VK_8;
        else if (ZXKBD_7) injectKey = fabgl::VK_7;
        else if (ZXKBD_6) injectKey = fabgl::VK_6;

    }

    if (injectKey != fabgl::VK_NONE) {
        if (zxDel == 0) {
            if (shift_down||ctrl_down) {
                VirtualKeyItem vki;
                vki.vk = injectKey;
                vki.down = true;
                vki.SHIFT = shift_down;
                vki.CTRL = ctrl_down;
                ESPeccy::PS2Controller.keyboard()->injectVirtualKey(vki, false);
                vki.down = false;
                ESPeccy::PS2Controller.keyboard()->injectVirtualKey(vki, false);
            } else {
                ESPeccy::PS2Controller.keyboard()->injectVirtualKey(injectKey, true, false);
                ESPeccy::PS2Controller.keyboard()->injectVirtualKey(injectKey, false, false);
            }
            zxDel = lastzxK == injectKey ? REPPER : REPDEL;
            lastzxK = injectKey;
        }
    } else {
        zxDel = 0;
        lastzxK = fabgl::VK_NONE;
    }

    if (zxDel > 0) zxDel--;

}
