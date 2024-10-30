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

#include <stdio.h>
#include <string>
#include <functional>

#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"

#include "ESPectrum.h"
#include "Snapshot.h"
#include "Config.h"
#include "FileUtils.h"
#include "OSDMain.h"
#include "Ports.h"
#include "MemESP.h"
#include "cpuESP.h"
#include "Video.h"
#include "messages.h"
#include "AySound.h"
#include "Tape.h"
#include "Z80_JLS/z80.h"
#include "pwm_audio.h"
#include "fabgl.h"
#include "wd1793.h"

#include "ZXKeyb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_efuse.h"
#include "soc/efuse_reg.h"

#include "BuildDate.h"

using namespace std;

//=======================================================================================
// KEYBOARD
//=======================================================================================
fabgl::PS2Controller ESPectrum::PS2Controller;
bool ESPectrum::ps2kbd2 = false;

//=======================================================================================
// AUDIO
//=======================================================================================
uint8_t ESPectrum::audioBuffer[ESP_AUDIO_SAMPLES_PENTAGON] = { 0 };
uint32_t* ESPectrum::overSamplebuf;
signed char ESPectrum::aud_volume = ESP_VOLUME_DEFAULT;
uint32_t ESPectrum::audbufcnt = 0;
uint32_t ESPectrum::audbufcntover = 0;
uint32_t ESPectrum::audbufcntAY = 0;
uint32_t ESPectrum::faudbufcntAY = 0;
int ESPectrum::lastaudioBit = 0;
//int ESPectrum::faudioBit = 0;
int ESPectrum::samplesPerFrame;
bool ESPectrum::AY_emu = false;
int ESPectrum::Audio_freq[4];
unsigned char ESPectrum::audioSampleDivider;
unsigned char ESPectrum::audioAYDivider;
unsigned char ESPectrum::audioOverSampleDivider;
static int audioBitBuf = 0;
static unsigned char audioBitbufCount = 0;
QueueHandle_t audioTaskQueue;
TaskHandle_t ESPectrum::audioTaskHandle;
uint8_t param;

int runBios = 0;

//=======================================================================================
// TAPE OSD
//=======================================================================================

int ESPectrum::TapeNameScroller = 0;

//=======================================================================================
// BETADISK
//=======================================================================================

bool ESPectrum::trdos = false;
WD1793 ESPectrum::Betadisk;

//=======================================================================================
// ARDUINO FUNCTIONS
//=======================================================================================

#define NOP() asm volatile ("nop")

IRAM_ATTR unsigned long millis()
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

IRAM_ATTR void delayMicroseconds(int64_t us)
{
    int64_t m = esp_timer_get_time();
    if(us){
        int64_t e = (m + us);
        if(m > e){ //overflow
            while(esp_timer_get_time() > e){
                NOP();
            }
        }
        while(esp_timer_get_time() < e){
            NOP();
        }
    }
}

//=======================================================================================
// TIMING / SYNC
//=======================================================================================

double ESPectrum::totalseconds = 0;
double ESPectrum::totalsecondsnodelay = 0;
int64_t ESPectrum::target[4];
int ESPectrum::sync_cnt = 0;
volatile bool ESPectrum::vsync = false;
int64_t ESPectrum::ts_start;
int64_t ESPectrum::elapsed;
int64_t ESPectrum::idle;
uint8_t ESPectrum::ESP_delay = 1; // EMULATION SPEED: 0-> MAX. SPEED (NO SOUND), 1-> 100% SPEED, 2-> 125% SPEED, 3-> 150% SPEED
int ESPectrum::ESPoffset = 0;

//=======================================================================================
// LOGGING / TESTING
//=======================================================================================

int ESPectrum::ESPtestvar = 0;
int ESPectrum::ESPtestvar1 = 0;
int ESPectrum::ESPtestvar2 = 0;

#define START_MSG_DURATION 20

void ShowStartMsg() {

    fabgl::VirtualKeyItem Nextkey;

    VIDEO::vga.clear(zxColor(7,0));

    OSD::drawOSD(false);

    VIDEO::vga.fillRect(OSD::osdInsideX(), OSD::osdInsideY(), OSD_COLS * OSD_FONT_W, 50, zxColor(0, 0));

    // Decode Logo in EBF8 format
    int logo_w = (ESPectrum_logo[5] << 8) + ESPectrum_logo[4]; // Get Width
    int logo_h = (ESPectrum_logo[7] << 8) + ESPectrum_logo[6]; // Get Height
    int pos_x = OSD::osdInsideX() + ( OSD_COLS * OSD_FONT_W - logo_w ) / 2;
    int pos_y = OSD::osdInsideY() + ( 50 - logo_h ) / 2;

    OSD::drawCompressedBMP(pos_x, pos_y, ESPectrum_logo);

    OSD::osdAt(7, 1);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));

    char nextChar;
    const char *text = StartMsg[Config::lang];

    int osdCol = 0, osdRow = 0;

    for ( int i = 0; i < strlen(text); ++i ) {
        nextChar = text[i];
        if (nextChar != 13) {
            if (nextChar == 10) {
                char fore = text[++i];
                char back = text[++i];
                int foreint = (fore >= 'A') ? (fore - 'A' + 10) : (fore - '0');
                int backint = (back >= 'A') ? (back - 'A' + 10) : (back - '0');
                VIDEO::vga.setTextColor(zxColor(foreint & 0x7, foreint >> 3), zxColor(backint & 0x7, backint >> 3));
                continue;
            }
            VIDEO::vga.print((const char)nextChar);
        } else {
            VIDEO::vga.print("\n");
        }
    }

    char msg[38];
    for (int i=START_MSG_DURATION; i >= 0; i--) {
        OSD::osdAt(19, 1);
        // sprintf(msg,Config::lang ? "Este mensaje se cerrar" "\xA0" " en %02d segundos" : "This message will close in %02d seconds",i);
        sprintf(msg,STARTMSG_CLOSE[Config::lang],i);
        VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
        VIDEO::vga.print(msg);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    VIDEO::vga.clear(zxColor(7,0));

    // Disable StartMsg
    Config::StartMsg = false;
    // Save all keys after new flash or update
    Config::save();

}

void ESPectrum::showMemInfo(const char* caption) {

    string textout;

    // // Get chip information
    // esp_chip_info_t chip_info;
    // esp_chip_info(&chip_info);

    // printf(" ------------------------------------------------------------\n");
    // printf(" Hardware info - %s \n", caption);
    // printf(" ------------------------------------------------------------\n");
    // // Chip models for ESP32
    // textout = " Chip model    : ";
    // uint32_t chip_ver = esp_efuse_get_pkg_ver();
    // uint32_t pkg_ver = chip_ver & 0x7;
    // switch (pkg_ver) {
    //     case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ6 :
    //         if (chip_info.revision == 3)
    //             textout += "ESP32-D0WDQ6-V3";
    //         else
    //             textout += "ESP32-D0WDQ6";
    //         break;
    //     case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ5 :
    //         if (chip_info.revision == 3)
    //             textout += "ESP32-D0WD-V3";
    //         else
    //             textout += "ESP32-D0WD";
    //         break;
    //     case EFUSE_RD_CHIP_VER_PKG_ESP32D2WDQ5 :
    //         textout += "ESP32-D2WD";
    //         break;
    //     case EFUSE_RD_CHIP_VER_PKG_ESP32PICOD2 :
    //         textout += "ESP32-PICO-D2";
    //         break;
    //     case EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4 :
    //         textout += "ESP32-PICO-D4";
    //         break;
    //     case EFUSE_RD_CHIP_VER_PKG_ESP32PICOV302 :
    //         textout += "ESP32-PICO-V3-02";
    //         break;
    //     case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDR2V3 :
    //          textout += "ESP32-D0WDR2-V3";
    //         break;
    //     default:
    //         textout += "Unknown";
    // }
    // textout += "\n";
    // printf(textout.c_str());

    // textout = " Chip cores    : " + to_string(chip_info.cores) + "\n";
    // printf(textout.c_str());

    // textout = " Chip revision : " + to_string(chip_info.revision) + "\n";
    // printf(textout.c_str());

    // textout = " Flash size    : " + to_string(spi_flash_get_chip_size() / (1024 * 1024)) + (chip_info.features & CHIP_FEATURE_EMB_FLASH ? "MB embedded" : "MB external") + "\n";
    // printf(textout.c_str());

    multi_heap_info_t info;

    // heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
    // uint32_t psramsize = (info.total_free_bytes + info.total_allocated_bytes) >> 10;
    // textout = " PSRAM size    : " + ( psramsize == 0 ? "N/A or disabled" : to_string(psramsize) + " MB") + "\n";
    // printf(textout.c_str());

    // textout = " IDF Version   : " + (string)(esp_get_idf_version()) + "\n";
    // printf(textout.c_str());

    // printf("\n Memory info\n");
    // printf(" ------------------------------------------------------------\n");

    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
    textout = " Total free bytes         : " + to_string(info.total_free_bytes) + "\n";
    printf(textout.c_str());

    textout = " Minimum free ever        : " + to_string(info.minimum_free_bytes) + "\n";
    printf(textout.c_str());

    // textout = " Largest free block       : " + to_string(info.largest_free_block) + "\n";
    // printf(textout.c_str());

    // textout = " Free (MALLOC_CAP_32BIT)  : " + to_string(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT)) + "\n";
    // printf(textout.c_str());

    // UBaseType_t wm;
    // wm = uxTaskGetStackHighWaterMark(NULL);
    // textout = " Main  Task Stack HWM     : " + to_string(wm) + "\n";
    // printf(textout.c_str());

    // wm = uxTaskGetStackHighWaterMark(ESPectrum::audioTaskHandle);
    // textout = " Audio Task Stack HWM     : " + to_string(wm) + "\n";
    // printf(textout.c_str());

    // wm = uxTaskGetStackHighWaterMark(VIDEO::videoTaskHandle);
    // textout = " Video Task Stack HWM     : " + (Config::videomode ? to_string(wm) : "N/A") + "\n";
    // printf(textout.c_str());
    // printf("\n ------------------------------------------------------------\n\n");

}

//=======================================================================================
// BOOT KEYBOARD
//=======================================================================================
void ESPectrum::bootKeyboard() {

    auto Kbd = PS2Controller.keyboard();
    fabgl::VirtualKeyItem NextKey;
    int i = 0;

    // printf("Boot kbd!\n");

    for (; i < 200; i++) {

        if (ZXKeyb::Exists) {

            // Process physical keyboard
            ZXKeyb::process();

            // Detect and process physical kbd menu key combinations
            if (!bitRead(ZXKeyb::ZXcols[3], 1)) { // 2
                runBios = 1;

            } else
            if (!bitRead(ZXKeyb::ZXcols[3], 2)) { // 3
                runBios = 3;
            }

        }

        while (Kbd->virtualKeyAvailable()) {

            bool r = Kbd->getNextVirtualKey(&NextKey);

            if (r && NextKey.down) {

                // Check keyboard status
                switch (NextKey.vk) {
                    case fabgl::VK_F2:
                        runBios = 1;
                        break;
                    case fabgl::VK_F3:
                        runBios = 3;
                        break;
                }

            }

        }

        if (runBios) break;

        delayMicroseconds(1000);

    }

    if (runBios) {
        Config::videomode = runBios - 1;
        Config::aspect_16_9 = false;
        Config::scanlines = false;
    }

}

void ESPectrum::showBIOS() {

    Config::load(); // Restore original config values

    auto Kbd = PS2Controller.keyboard();
    fabgl::VirtualKeyItem NextKey;

    VIDEO::vga.clear(zxColor(7, 0));

    int base_row = OSD_FONT_H * 4;
    int base_col = OSD_FONT_W * 4;
    int total_rows = OSD::scrH / OSD_FONT_H - 8;
    int total_cols = OSD::scrW / OSD_FONT_W - 8;

    #define PRINT_FILLED_ROW(text)  VIDEO::vga.print(text); VIDEO::vga.print(string(total_cols - strlen(text), ' ').c_str())
    #define PRINT_FILLED_ROW_ALIGN_RIGHT(text)  VIDEO::vga.print(string(total_cols - strlen(text), ' ').c_str()); VIDEO::vga.print(text)
    #define SET_CURSOR(col,row) VIDEO::vga.setCursor(base_col + (col) * OSD_FONT_W, base_row + (row) * OSD_FONT_H)

    // Opciones del menú
    const char* menuOptions[] = {"Main", "Advanced", "Config", "Exit"};
    const int menuCount = sizeof(menuOptions)/sizeof(menuOptions[0]);

    const char* menuAdvanced[] = {"Resolution", "Frequency", "Scanlines"};
    const int menuAdvancedCount = sizeof(menuAdvanced) / sizeof(menuAdvanced[0]);

    const char* menuConfig[] = {"Backup Settings", "Restore Settings", "Reset Settings"};
    const int menuConfigCount = sizeof(menuConfig) / sizeof(menuConfig[0]);

    const char* menuExit[] = {"Save Changes & Exit", "Discard Changes & Exit"};
    const int menuExitCount = sizeof(menuExit) / sizeof(menuExit[0]);

    const char* menuOptionsResolution[] = {"320x240 (4:3)", "360x200 (16:9)"};
    const int menuOptionsResolutionCount = sizeof(menuOptionsResolution) / sizeof(menuOptionsResolution[0]);

    const char* menuOptionsFrequency[] = {"60Hz (VGA)", "50Hz (VGA)", "15kHz (CRT)"};
    const int menuOptionsFrequencyCount = sizeof(menuOptionsFrequency) / sizeof(menuOptionsFrequency[0]);

    const char* menuOptionsScanlines[] = {"No", "Yes"};
    const int menuOptionsScanlinesCount = sizeof(menuOptionsScanlines) / sizeof(menuOptionsScanlines[0]);

    int selectedOption = 0;
    int selectedAdvancedOption = 0;
    int selectedConfigOption = 0;
    int selectedExitOption = 0;

    // Renderizar el menú inicial
    auto renderMenu = [&](int highlight) {
        SET_CURSOR(0, 0);
        int len = 0;
        for (int i = 0; i < menuCount; ++i) {
            VIDEO::vga.setTextColor(i == highlight ? zxColor(1, 1) : zxColor(7, 1), i == highlight ? zxColor(7, 1) : zxColor(1, 0));
            VIDEO::vga.print(" ");
            VIDEO::vga.print(menuOptions[i]);
            VIDEO::vga.print(" ");
            len += strlen(menuOptions[i]) + 2;
        }
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
        VIDEO::vga.print(string(total_cols - len, ' ').c_str());
    };

    auto renderOptions = [&](const char *options[], const char *values[], const int optionsCount, int highlight) {
        SET_CURSOR(1, 2); // Ajustar la posición para el submenú
        for (int i = 0; i < optionsCount; ++i) {
            // Color del texto, resaltado para el elemento seleccionado
            VIDEO::vga.setTextColor(i == highlight ? zxColor(7, 1) : zxColor(1, 0),
                                    i == highlight ? zxColor(0, 0) : zxColor(7, 0));

            // Imprimir el nombre de la opción
            VIDEO::vga.print(" ");
            VIDEO::vga.print(options[i]);

            // Calcular espacios en blanco para alinear valores
            int padding = total_cols - 19 /* Help Column */ - 2 - strlen(options[i]) -
                          (values && values[i] ? strlen(values[i]) : 0) - 2; // espacio antes del valor si existe

            // Añadir los espacios para alineación
            VIDEO::vga.print(string(padding, ' ').c_str());

            // Si hay valores, imprimir el valor alineado a la derecha
            if (values && values[i]) {
                VIDEO::vga.print(values[i]);
            }

            VIDEO::vga.print(" \n");
        }
    };

    auto screen_clear = [&](bool fullwidth = false) {
        int color = zxColor(7, 0);
        for (int y = base_row + OSD_FONT_H * 2; y < base_row + ( total_rows - 2 ) * OSD_FONT_H; y++)
            for (int x = base_col + OSD_FONT_W; x < base_col + ( total_cols - ( fullwidth ? 0 : 20 )) * OSD_FONT_W; x++)
                VIDEO::vga.dotFast(x, y, color);

        const int top = base_row + OSD_FONT_H + OSD_FONT_H / 2;
        const int buttom = base_row + ( total_rows - 1 ) * OSD_FONT_H - OSD_FONT_H / 2;
        const int left = base_col + OSD_FONT_W / 2;
        const int right = base_col + ( total_cols - 1 ) * OSD_FONT_W + OSD_FONT_W / 2;

        VIDEO::vga.line(  left,    top, right,    top, zxColor(1, 0));
        VIDEO::vga.line(  left, buttom, right, buttom, zxColor(1, 0));
        VIDEO::vga.line(  left,    top,  left, buttom, zxColor(1, 0));
        VIDEO::vga.line( right,    top, right, buttom, zxColor(1, 0));

        VIDEO::vga.line( right - 19 * OSD_FONT_W,    top, right - 19 * OSD_FONT_W, buttom, zxColor(1, 0));

        SET_CURSOR(total_cols - 19, total_rows - 7);
        VIDEO::vga.setTextColor(zxColor(1, 0), zxColor(7, 0));
        VIDEO::vga.print("\x1A \x1B Select Screen\n");
        VIDEO::vga.print("\x18 \x19 Select Item\n");
        VIDEO::vga.print("Enter: Select/Chg.\n");
        if ( ZXKeyb::Exists ) {
            VIDEO::vga.print("SS+S: Save & Exit\n");
            VIDEO::vga.print("BREAK: Exit\n");
        } else {
            VIDEO::vga.print("F10: Save & Exit\n");
            VIDEO::vga.print("ESC: Exit\n");
        }

    };

    #define BIOS_DLG_ALERT   0
    #define BIOS_DLG_CONFIRM 1

    auto msg_dialog = [&](const char *title, const char *message, int type = BIOS_DLG_ALERT) {
        // Calcular el ancho del título
        int title_length = strlen(title);

        // Inicializar el ancho del diálogo con el ancho del título
        int dialog_width = title_length;

        // Calcular la altura y el ancho del mensaje
        int message_height = 0; // Contador de líneas
        const char *p = message, *pi = message;
        while(*p) {
            if (*p == '\n') {
                if (p - pi > dialog_width) dialog_width = p - pi;
                pi = p + 1;
                message_height++;
            }
            ++p;
        }

        if (pi < p && p - pi > dialog_width) dialog_width = p - pi;

        dialog_width += 4; // 2 caracteres de margen a cada lado

        // Ajustar el alto total del diálogo
        int dialog_height = message_height + 5; // Incluye el título, márgenes y botones

        int left = (total_cols - dialog_width) / 2;
        int right = left + dialog_width;
        int top = (total_rows - dialog_height) / 2;
        int bottom = top + 2 + dialog_height;

        // Limpiar el área del diálogo
        for (int y = base_row + top * OSD_FONT_H; y < base_row + bottom * OSD_FONT_H; y++) {
            for (int x = base_col + left * OSD_FONT_W; x < base_col + right * OSD_FONT_W; x++) {
                VIDEO::vga.dotFast(x, y, zxColor(7, 0));
            }
        }

        // Dibujar el borde del diálogo
        VIDEO::vga.line(base_col +  left * OSD_FONT_W - OSD_FONT_W / 2, base_row +    top * OSD_FONT_H + OSD_FONT_H / 2, base_col + right * OSD_FONT_W + OSD_FONT_W / 2, base_row +    top * OSD_FONT_H + OSD_FONT_H / 2, zxColor(1, 0));
        VIDEO::vga.line(base_col +  left * OSD_FONT_W - OSD_FONT_W / 2, base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2, base_col + right * OSD_FONT_W + OSD_FONT_W / 2, base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2, zxColor(1, 0));
        VIDEO::vga.line(base_col +  left * OSD_FONT_W - OSD_FONT_W / 2, base_row +    top * OSD_FONT_H + OSD_FONT_H / 2, base_col +  left * OSD_FONT_W - OSD_FONT_W / 2, base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2, zxColor(1, 0));
        VIDEO::vga.line(base_col + right * OSD_FONT_W + OSD_FONT_W / 2, base_row +    top * OSD_FONT_H + OSD_FONT_H / 2, base_col + right * OSD_FONT_W + OSD_FONT_W / 2, base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2, zxColor(1, 0));

        VIDEO::vga.fillRect(base_col + left * OSD_FONT_W, base_row + ( top + 1 ) * OSD_FONT_H, dialog_width * OSD_FONT_W, dialog_height * OSD_FONT_H, zxColor(1,0));

        // Mostrar el título en la primera línea dentro del cuadro
        SET_CURSOR(left + dialog_width / 2 - strlen(title) / 2, top);
        VIDEO::vga.setTextColor(zxColor(1, 0), zxColor(7, 0));
        VIDEO::vga.print(title);

        if ( message_height > 0 ) {
            // Mostrar el mensaje en la tercera línea
            SET_CURSOR(left + 1, top + 2);
            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
            VIDEO::vga.print(message);
        } else {
            // Mostrar el mensaje en la tercera línea
            SET_CURSOR(left + 1, top + 2);

            int current_line = 0;
            pi = p = message;
            while(*p) {
                if (*p == '\n') {
                    SET_CURSOR(left + dialog_width / 2 - (p - pi) / 2, top + 2 + current_line); // +2 por la línea del título y el margen superior
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
                    for(; pi < p - 1; ++pi ) VIDEO::vga.print(*pi);
                    ++current_line;
                }
                ++p;
            }
            if (pi < p) {
                SET_CURSOR(left + dialog_width / 2 - (p - pi) / 2, top + 2 + current_line); // +2 por la línea del título y el margen superior
                VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
                for(; pi < p - 1; ++pi ) VIDEO::vga.print(*pi);
            }
        }

        // Mostrar los botones "OK" y "Cancel" en la quinta línea
        int selectedButton = 0; // 0 = OK, 1 = CancelDLG_ALERT
        auto renderButtons = [&](int type = BIOS_DLG_ALERT) {
            SET_CURSOR(left + dialog_width / 2 - ( ( type == BIOS_DLG_CONFIRM ) ? 11 : 5 ), top + 4 + message_height );
            VIDEO::vga.setTextColor(selectedButton == 0 ? zxColor(1, 1) : zxColor(7, 0), selectedButton == 0 ? zxColor(7, 1) : zxColor(1, 0));
            VIDEO::vga.print("[   OK   ]");

            if ( type == BIOS_DLG_CONFIRM ) {
                VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
                VIDEO::vga.print("  ");

                VIDEO::vga.setTextColor(selectedButton == 1 ? zxColor(1, 1) : zxColor(7, 0), selectedButton == 1 ? zxColor(7, 1) : zxColor(1, 0));
                VIDEO::vga.print("[ CANCEL ]");
            }
        };
        renderButtons(type);

        // Esperar la selección del usuario
        while (true) {
            ZXKeyb::ZXKbdRead();
            while (Kbd->virtualKeyAvailable()) {
                fabgl::VirtualKeyItem NextKey;
                if (Kbd->getNextVirtualKey(&NextKey) && NextKey.down) {
                    switch (NextKey.vk) {
                        case fabgl::VK_LEFT:
                        case fabgl::VK_RIGHT:
                            if (type == BIOS_DLG_CONFIRM) {
                                selectedButton = 1 - selectedButton; // Cambiar entre 0 y 1
                                renderButtons(BIOS_DLG_CONFIRM);
                            }
                            break;
                        case fabgl::VK_RETURN:
                        case fabgl::VK_SPACE:
                            return selectedButton == 0; // Retorna true si seleccionó OK, false si seleccionó Cancel
                        case fabgl::VK_ESCAPE:
                            return false;
                    }
                }
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    };

    // Mostrar información de hardware
    auto showHardwareInfo = [&]() {
        screen_clear();

        // Mostrar información de chip
        SET_CURSOR(1, 2);
        VIDEO::vga.setTextColor(zxColor(1, 0), zxColor(7, 0));

        // Get chip information
        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);

        // Chip models for ESP32
        string textout = " Chip model    : ";
        uint32_t chip_ver = esp_efuse_get_pkg_ver();
        uint32_t pkg_ver = chip_ver & 0x7;
        switch (pkg_ver) {
            case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ6 :
                textout += (chip_info.revision == 3) ? "ESP32-D0WDQ6-V3" : "ESP32-D0WDQ6";
                break;
            case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ5 :
                textout += (chip_info.revision == 3) ? "ESP32-D0WD-V3" : "ESP32-D0WD";
                break;
            case EFUSE_RD_CHIP_VER_PKG_ESP32D2WDQ5 :
                textout += "ESP32-D2WD";
                break;
            case EFUSE_RD_CHIP_VER_PKG_ESP32PICOD2 :
                textout += "ESP32-PICO-D2";
                break;
            case EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4 :
                textout += "ESP32-PICO-D4";
                break;
            case EFUSE_RD_CHIP_VER_PKG_ESP32PICOV302 :
                textout += "ESP32-PICO-V3-02";
                break;
            case EFUSE_RD_CHIP_VER_PKG_ESP32D0WDR2V3 :
                textout += "ESP32-D0WDR2-V3";
                break;
            default:
                textout += "Unknown";
        }
        textout += "\n";
        VIDEO::vga.print(textout.c_str());

        // Continuar mostrando información del hardware

        textout = " Chip cores    : " + to_string(chip_info.cores) + "\n"; VIDEO::vga.print(textout.c_str());
        textout = " Chip revision : " + to_string(chip_info.revision) + "\n"; VIDEO::vga.print(textout.c_str());
        textout = " Flash size    : " + to_string(spi_flash_get_chip_size() / (1024 * 1024)) + (chip_info.features & CHIP_FEATURE_EMB_FLASH ? "MB embedded" : "MB external") + "\n"; VIDEO::vga.print(textout.c_str());
        multi_heap_info_t info; heap_caps_get_info(&info, MALLOC_CAP_SPIRAM); uint32_t psramsize = (info.total_free_bytes + info.total_allocated_bytes) >> 10;
        textout = " PSRAM size    : " + ( psramsize == 0 ? "N/A or disabled" : to_string(psramsize) + " MB") + "\n"; VIDEO::vga.print(textout.c_str());
        textout = " IDF Version   : " + (string)(esp_get_idf_version()) + "\n"; VIDEO::vga.print(textout.c_str());

    };

    // Iniciar el menú
    renderMenu(selectedOption);
    showHardwareInfo();

    SET_CURSOR(0, total_rows - 1);
    string footer = "ESPeccy BIOS - commit: " + string(getShortBuildDate()) + " ";
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
    PRINT_FILLED_ROW_ALIGN_RIGHT(footer.c_str());

    // Lógica de navegación del menú
    bool exitMenu = false;

    bool exit_to_main = false;

    auto mainMenuNav = [&](const std::function<void()>& escCancel, const std::function<void()>& f10Cancel) {
        switch (NextKey.vk) {
            case fabgl::VK_RIGHT:
                selectedOption = (selectedOption + 1) % menuCount;
                renderMenu(selectedOption);
                exit_to_main = true;
                break;

            case fabgl::VK_LEFT:
                selectedOption = (selectedOption - 1 + menuCount) % menuCount;
                renderMenu(selectedOption);
                exit_to_main = true;
                break;

            case fabgl::VK_ESCAPE:
                if ( msg_dialog("Exit BIOS Setup", "Are you sure you want to exit?\nUnsaved changes will be lost.", BIOS_DLG_CONFIRM) ) {
                    OSD::esp_hard_reset();
                } else {
                    screen_clear(true);
                    escCancel();
                }
                break;

            case fabgl::VK_s:
            case fabgl::VK_S:
                if (!ZXKeyb::Exists) break;

            case fabgl::VK_F10:
                if ( msg_dialog("Confirm Save & Exit", "Are you sure you want to save\nchanges and exit?", BIOS_DLG_CONFIRM) ) {
                    Config::save();
                    OSD::esp_hard_reset();
                } else {
                    screen_clear(true);
                    f10Cancel();
                }
                break;
        }
    };


    auto optionsNav = [&](int &selectedOption, int menuCount, const std::function<void()> &renderMenu) {
        switch (NextKey.vk) {
            case fabgl::VK_DOWN:
                selectedOption = (selectedOption + 1) % menuCount;
                screen_clear();
                renderMenu();
                break;

            case fabgl::VK_UP:
                selectedOption = (selectedOption - 1 + menuCount) % menuCount;
                screen_clear();
                renderMenu();
                break;
        }
    };

    while (!exitMenu) {
        int oldSelectedOptions = selectedOption;

        ZXKeyb::ZXKbdRead();
        while (Kbd->virtualKeyAvailable()) {
            bool r = Kbd->getNextVirtualKey(&NextKey);
            if (r && NextKey.down) mainMenuNav([](){}, [](){});
        }

        if (selectedOption != oldSelectedOptions || exit_to_main ) {
            exit_to_main = false;
            // Acción según la opción seleccionada
            switch (selectedOption) {
                case 0: // Acción para MAIN
                    screen_clear();
                    showHardwareInfo();
                    break;
                case 1: // Acción para ADVANCED
                {
                    selectedAdvancedOption = 0;

                    auto renderAdvancedOptions = [&]() {
                        const char *valuesAvanced[3] = { menuOptionsResolution[Config::aspect_16_9], menuOptionsFrequency[Config::videomode], menuOptionsScanlines[Config::scanlines] };
                        renderOptions(menuAdvanced, valuesAvanced, menuAdvancedCount, selectedAdvancedOption);
                    };

                    // Renderizar menú avanzado
                    screen_clear();
                    renderAdvancedOptions();

                    // Lógica para el menú avanzado
                    bool exitAdvancedMenu = false;
                    while (!exitAdvancedMenu) {
                        ZXKeyb::ZXKbdRead();
                        while (Kbd->virtualKeyAvailable()) {
                            bool r = Kbd->getNextVirtualKey(&NextKey);
                            if (r && NextKey.down) {

                                mainMenuNav([&renderAdvancedOptions](){renderAdvancedOptions();}, [&renderAdvancedOptions](){renderAdvancedOptions();});
                                optionsNav(selectedAdvancedOption, menuAdvancedCount, [&renderAdvancedOptions](){renderAdvancedOptions();});

                                switch (NextKey.vk) {
                                    case fabgl::VK_RETURN:
                                    case fabgl::VK_SPACE:
                                        switch (selectedAdvancedOption) {
                                            case 0: // Acción para RESOLUTION
                                                Config::aspect_16_9 = (Config::aspect_16_9 + 1) % 2;
                                                break;
                                            case 1: // Acción para FREQUENCY
                                                Config::videomode = (Config::videomode + 1) % 3;
                                                break;
                                            case 2: // Acción para SCANLINES
                                                Config::scanlines = (Config::scanlines + 1) % 2;
                                                break;
                                        }

                                        screen_clear();
                                        renderAdvancedOptions();
                                        break;
                                }
                            }
                        }

                        if (exit_to_main) break;

                        vTaskDelay(100 / portTICK_PERIOD_MS);

                    }

                    if (exit_to_main) break;

                    screen_clear();
                    renderAdvancedOptions();
                    break;
                }
                case 2: // Acción para CONFIG
                {
                    selectedConfigOption = 0;
                    // Renderizar menú de visualización
                    screen_clear();
                    renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);

                    auto remountSD = [&]() {
                        if ( FileUtils::SDReady && !FileUtils::isMountedSDCard() ) FileUtils::unmountSDCard();
                        if ( !FileUtils::SDReady ) FileUtils::initFileSystem();
                    };

                    bool exitConfigMenu = false;
                    while (!exitConfigMenu) {
                        ZXKeyb::ZXKbdRead();
                        while (Kbd->virtualKeyAvailable()) {
                            bool r = Kbd->getNextVirtualKey(&NextKey);
                            if (r && NextKey.down) {

                                mainMenuNav([&renderOptions, &menuConfig, &menuConfigCount, &selectedConfigOption](){renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);},
                                            [&renderOptions, &menuConfig, &menuConfigCount, &selectedConfigOption](){renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);});
                                optionsNav(selectedConfigOption, menuConfigCount, [&renderOptions, &menuConfig, &menuConfigCount, &selectedConfigOption](){renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);});

                                switch (NextKey.vk) {
                                    case fabgl::VK_RETURN:
                                    case fabgl::VK_SPACE:
                                        switch (selectedConfigOption) {
                                            case 0: // Acción para BACKUP
                                            {
                                                if (msg_dialog("Confirm Backup", "Insert a valid SD card.\nPress OK to save BIOS settings,\nor Cancel to abort.", BIOS_DLG_CONFIRM)) {
                                                    screen_clear(true);
                                                    renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);
                                                    remountSD();
                                                    bool status = !FileUtils::SDReady || Config::saveToSD();
                                                    if (status) msg_dialog("Backup Error", "Failed to write backup.\nPlease check SD card and try again.");
                                                    else msg_dialog("Backup Completed", "BIOS settings successfully saved to SD card.");
                                                }
                                                screen_clear(true);
                                                renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);
                                                break;
                                            }
                                            case 1: // Acción para RESTORE
                                            {
                                                if (msg_dialog("Confirm Restore", "Insert the SD card with the backup.\nPress OK to restore settings,\nor Cancel to abort.", BIOS_DLG_CONFIRM)) {
                                                    screen_clear(true);
                                                    renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);
                                                    remountSD();
                                                    bool status = !FileUtils::SDReady || Config::loadFromSD();
                                                    if (status) msg_dialog("Restore Error", "Failed to restore settings.\nPlease verify SD card and try again.");
                                                    else msg_dialog("Restore Completed", "BIOS settings successfully restored from SD card.");
                                                }
                                                screen_clear(true);
                                                renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);
                                                break;
                                            }
                                            case 2: // Acción para RESET
                                                if (msg_dialog("Reset Configuration & Reboot", "Do you really want to reset all settings?", BIOS_DLG_CONFIRM)) {
                                                    nvs_flash_erase();
                                                    OSD::esp_hard_reset();
                                                } else {
                                                    screen_clear(true);
                                                    renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);
                                                }
                                                break;
                                        }
                                        break;
                                }
                            }
                        }

                        if (exit_to_main) break;

                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }

                    if (exit_to_main) break;

                    screen_clear();
                    renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);
                    break;
                }
                case 3:
                {
                    selectedExitOption = 0;
                    // Renderizar menú de visualización
                    screen_clear();
                    renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);

                    bool exitConfigMenu = false;
                    while (!exitConfigMenu) {
                        ZXKeyb::ZXKbdRead();
                        while (Kbd->virtualKeyAvailable()) {
                            bool r = Kbd->getNextVirtualKey(&NextKey);
                            if (r && NextKey.down) {

                                mainMenuNav([&renderOptions, &menuExit, &menuExitCount, &selectedExitOption](){renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);},
                                            [&renderOptions, &menuExit, &menuExitCount, &selectedExitOption](){renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);});
                                optionsNav(selectedExitOption, menuExitCount, [&renderOptions, &menuExit, &menuExitCount, &selectedExitOption](){renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);});

                                switch (NextKey.vk) {
                                    case fabgl::VK_RETURN:
                                    case fabgl::VK_SPACE:
                                        switch (selectedExitOption) {
                                            case 0: // Acción para Save Changes & Exit
                                                if ( msg_dialog("Confim Save & Exit", "Are you sure you want to save\nchanges and exit?", BIOS_DLG_CONFIRM) ) {
                                                    Config::save();
                                                    OSD::esp_hard_reset();
                                                } else {
                                                    screen_clear(true);
                                                    renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);
                                                }
                                                break;
                                            case 1: // Acción para Discard Changes & Exit
                                                if ( msg_dialog("Exit BIOS Setup", "Are you sure you want to exit?\nUnsaved changes will be lost.", BIOS_DLG_CONFIRM) ) {
                                                    OSD::esp_hard_reset();
                                                } else {
                                                    screen_clear(true);
                                                    renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);
                                                }
                                                break;
                                        }
                                        break;
                                }
                            }
                        }

                        if (exit_to_main) break;

                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }

                    if (exit_to_main) break;

                    screen_clear();
                    renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);
                    break;
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    VIDEO::vga.clear(zxColor(7, 0));
}

//=======================================================================================
// SETUP
//=======================================================================================

void ESPectrum::setup()
{

    if (Config::slog_on) {
        printf("------------------------------------\n");
        printf("| ESPectrum: booting               |\n");
        printf("------------------------------------\n");
        showMemInfo();
    }

    //=======================================================================================
    // PHYSICAL KEYBOARD (SINCLAIR 8 + 5 MEMBRANE KEYBOARD)
    //=======================================================================================

    ZXKeyb::setup();

    //=======================================================================================
    // LOAD CONFIG
    //=======================================================================================

    Config::load();

    if (Config::StartMsg) Config::save(); // Firmware updated or reflashed: save all config data

    // printf("---------------------------------\n");
    // printf("Ram file: %s\n",Config::ram_file.c_str());
    // printf("Arch: %s\n",Config::arch.c_str());
    // printf("pref Arch: %s\n",Config::pref_arch.c_str());
    // printf("romSet: %s\n",Config::romSet.c_str());
    // printf("romSet48: %s\n",Config::romSet48.c_str());
    // printf("romSet128: %s\n",Config::romSet128.c_str());
    // printf("romSetTK90X: %s\n",Config::romSetTK90X.c_str());
    // printf("romSetTK95: %s\n",Config::romSetTK95.c_str());
    // printf("pref_romSet_48: %s\n",Config::pref_romSet_48.c_str());
    // printf("pref_romSet_128: %s\n",Config::pref_romSet_128.c_str());
    // printf("pref_romSet_TK90X: %s\n",Config::pref_romSet_TK90X.c_str());
    // printf("pref_romSet_TK95: %s\n",Config::pref_romSet_TK95.c_str());

    // Set arch if there's no snapshot to load
    if (Config::ram_file == NO_RAM_FILE) {

        if (Config::pref_arch.substr(Config::pref_arch.length()-1) == "R") {

            Config::pref_arch.pop_back();
            Config::save("pref_arch");

        } else {

            if (Config::pref_arch != "Last") Config::arch = Config::pref_arch;

            if (Config::arch == "48K") {
                if (Config::pref_romSet_48 != "Last")
                    Config::romSet = Config::pref_romSet_48;
                else
                    Config::romSet = Config::romSet48;
            } else if (Config::arch == "128K") {
                if (Config::pref_romSet_128 != "Last")
                    Config::romSet = Config::pref_romSet_128;
                else
                    Config::romSet = Config::romSet128;
            } else if (Config::arch == "TK90X") {
                if (Config::pref_romSet_TK90X != "Last")
                    Config::romSet = Config::pref_romSet_TK90X;
                else
                    Config::romSet = Config::romSetTK90X;
            } else if (Config::arch == "TK95") {
                if (Config::pref_romSet_TK95 != "Last")
                    Config::romSet = Config::pref_romSet_TK95;
                else
                    Config::romSet = Config::romSetTK95;
            } else Config::romSet = "Pentagon";

            printf("Arch: %s, Romset: %s\n",Config::arch.c_str(), Config::romSet.c_str());

        }

    }

    // printf("---------------------------------\n");
    // printf("Ram file: %s\n",Config::ram_file.c_str());
    // printf("Arch: %s\n",Config::arch.c_str());
    // printf("pref Arch: %s\n",Config::pref_arch.c_str());
    // printf("romSet: %s\n",Config::romSet.c_str());
    // printf("romSet48: %s\n",Config::romSet48.c_str());
    // printf("romSet128: %s\n",Config::romSet128.c_str());
    // printf("pref_romSet_48: %s\n",Config::pref_romSet_48.c_str());
    // printf("pref_romSet_128: %s\n",Config::pref_romSet_128.c_str());

    //=======================================================================================
    // INIT PS/2 KEYBOARD
    //=======================================================================================

    ESPectrum::ps2kbd2 = (Config::ps2_dev2 != 0);

    if (ZXKeyb::Exists) {
        PS2Controller.begin(ps2kbd2 ? PS2Preset::KeyboardPort0 : PS2Preset::zxKeyb, KbdMode::CreateVirtualKeysQueue);
    } else {
        PS2Controller.begin(ps2kbd2 ? PS2Preset::KeyboardPort0_KeybJoystickPort1 : PS2Preset::KeyboardPort0, KbdMode::CreateVirtualKeysQueue);
    }

    ps2kbd2 &= !ZXKeyb::Exists;

    // Set Scroll Lock Led as current CursorAsJoy value
    PS2Controller.keyboard()->setLEDs(false, false, Config::CursorAsJoy);
    if(ps2kbd2)
        PS2Controller.keybjoystick()->setLEDs(false, false, Config::CursorAsJoy);

    // Set TAB and GRAVEACCENT behaviour
    if (Config::TABasfire1) {
        ESPectrum::VK_ESPECTRUM_FIRE1 = fabgl::VK_TAB;
        ESPectrum::VK_ESPECTRUM_FIRE2 = fabgl::VK_GRAVEACCENT;
        ESPectrum::VK_ESPECTRUM_TAB = fabgl::VK_NONE;
        ESPectrum::VK_ESPECTRUM_GRAVEACCENT = fabgl::VK_NONE;
    } else {
        ESPectrum::VK_ESPECTRUM_FIRE1 = fabgl::VK_NONE;
        ESPectrum::VK_ESPECTRUM_FIRE2 = fabgl::VK_NONE;
        ESPectrum::VK_ESPECTRUM_TAB = fabgl::VK_TAB;
        ESPectrum::VK_ESPECTRUM_GRAVEACCENT = fabgl::VK_GRAVEACCENT;
    }

    if (Config::slog_on) {
        showMemInfo("Keyboard started");
    }

    // Get chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Config::esp32rev = chip_info.revision;

    if (Config::slog_on) {

        printf("\n");
        printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
                CONFIG_IDF_TARGET,
                chip_info.cores,
                (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
                (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
        printf("silicon revision %d, ", chip_info.revision);
        printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
                (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
        printf("IDF Version: %s\n",esp_get_idf_version());
        printf("\n");

        if (Config::slog_on) printf("Executing on core: %u\n", xPortGetCoreID());

        showMemInfo();

    }

    //=======================================================================================
    // BOOTKEYS: Read keyboard for 200 ms. checking boot keys
    //=======================================================================================

    // printf("Waiting boot keys\n");
    bootKeyboard();
    // printf("End Waiting boot keys\n");

    //=======================================================================================
    // MEMORY SETUP
    //=======================================================================================

    MemESP::Init();

    // Load romset
    Config::requestMachine(Config::arch, Config::romSet);

    MemESP::Reset();

    if (Config::slog_on) showMemInfo("RAM Initialized");

    //=======================================================================================
    // VIDEO
    //=======================================================================================

    VIDEO::Init();
    VIDEO::Reset();

    if (Config::slog_on) showMemInfo("VGA started");

    if (Config::StartMsg) ShowStartMsg(); // Show welcome message

    //=======================================================================================
    // INIT FILESYSTEM
    //=======================================================================================

    FileUtils::initFileSystem();

    if (Config::slog_on) showMemInfo("File system started");

    //=======================================================================================
    // BIOS
    //=======================================================================================

    if (runBios) showBIOS();

    //=======================================================================================
    // AUDIO
    //=======================================================================================

    overSamplebuf = (uint32_t *) heap_caps_malloc(ESP_AUDIO_SAMPLES_PENTAGON << 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);
    if (overSamplebuf == NULL) printf("Can't allocate oversamplebuffer\n");

    // Create Audio task
    audioTaskQueue = xQueueCreate(1, sizeof(uint8_t));
    // Latest parameter = Core. In ESPIF, main task runs on core 0 by default. In Arduino, loop() runs on core 1.
    xTaskCreatePinnedToCore(&ESPectrum::audioTask, "audioTask", 2048 /* 1024 /* 1536 */, NULL, configMAX_PRIORITIES - 1, &audioTaskHandle, 1);

    // Set samples per frame and AY_emu flag depending on arch
    if (Config::arch == "48K") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_48;
        audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_48;
        audioAYDivider = ESP_AUDIO_AY_DIV_48;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_48;
        AY_emu = Config::AY48;
        Audio_freq[0] = ESP_AUDIO_FREQ_48;
        Audio_freq[1] = ESP_AUDIO_FREQ_48;
        Audio_freq[2] = ESP_AUDIO_FREQ_48_125SPEED;
        Audio_freq[3] = ESP_AUDIO_FREQ_48_150SPEED;
    } else if (Config::arch == "TK90X" || Config::arch == "TK95") {

        switch (Config::ALUTK) {
        case 0:
            samplesPerFrame=ESP_AUDIO_SAMPLES_48;
            audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_48;
            audioAYDivider = ESP_AUDIO_AY_DIV_48;
            audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_48;
            Audio_freq[0] = ESP_AUDIO_FREQ_48;
            Audio_freq[1] = ESP_AUDIO_FREQ_48;
            Audio_freq[2] = ESP_AUDIO_FREQ_48_125SPEED;
            Audio_freq[3] = ESP_AUDIO_FREQ_48_150SPEED;
            break;
        case 1:
            samplesPerFrame=ESP_AUDIO_SAMPLES_TK_50;
            audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_TK_50;
            audioAYDivider = ESP_AUDIO_AY_DIV_TK_50;
            audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_TK_50;
            Audio_freq[0] = ESP_AUDIO_FREQ_TK_50;
            Audio_freq[1] = ESP_AUDIO_FREQ_TK_50;
            Audio_freq[2] = ESP_AUDIO_FREQ_TK_50_125SPEED;
            Audio_freq[3] = ESP_AUDIO_FREQ_TK_50_150SPEED;
            break;
        case 2:
            samplesPerFrame=ESP_AUDIO_SAMPLES_TK_60;
            audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_TK_60;
            audioAYDivider = ESP_AUDIO_AY_DIV_TK_60;
            audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_TK_60;
            Audio_freq[0] = ESP_AUDIO_FREQ_TK_60;
            Audio_freq[1] = ESP_AUDIO_FREQ_TK_60;
            Audio_freq[2] = ESP_AUDIO_FREQ_TK_60_125SPEED;
            Audio_freq[3] = ESP_AUDIO_FREQ_TK_60_150SPEED;
        }

        AY_emu = Config::AY48;

    } else if (Config::arch == "128K") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_128;
        audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_128;
        audioAYDivider = ESP_AUDIO_AY_DIV_128;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_128;
        AY_emu = true;
        Audio_freq[0] = ESP_AUDIO_FREQ_128;
        Audio_freq[1] = ESP_AUDIO_FREQ_128;
        Audio_freq[2] = ESP_AUDIO_FREQ_128_125SPEED;
        Audio_freq[3] = ESP_AUDIO_FREQ_128_150SPEED;
    } else if (Config::arch == "Pentagon") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_PENTAGON;
        audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_PENTAGON;
        audioAYDivider = ESP_AUDIO_AY_DIV_PENTAGON;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_PENTAGON;
        AY_emu = true;
        Audio_freq[0] = ESP_AUDIO_FREQ_PENTAGON;
        Audio_freq[1] = ESP_AUDIO_FREQ_PENTAGON;
        Audio_freq[2] = ESP_AUDIO_FREQ_PENTAGON_125SPEED;
        Audio_freq[3] = ESP_AUDIO_FREQ_PENTAGON_150SPEED;
    }

    if (Config::tape_player) {
        AY_emu = false; // Disable AY emulation if tape player mode is set
        ESPectrum::aud_volume = ESP_VOLUME_MAX;
    } else
        ESPectrum::aud_volume = Config::volume;

    ESPoffset = 0;

    // AY Sound
    AySound::init();
    AySound::set_sound_format(Audio_freq[ESP_delay],1,8);
    AySound::set_stereo(AYEMU_MONO,NULL);
    AySound::reset();

    // Init tape
    Tape::Init();
    Tape::tapeFileName = "none";
    Tape::tapeStatus = TAPE_STOPPED;
    Tape::SaveStatus = SAVE_STOPPED;
    Tape::romLoading = false;

    if (Z80Ops::is128) { // Apply pulse length compensation for 128K
        Tape::tapeCompensation = FACTOR128K;
    } else if ((Config::arch=="TK90X" || Config::arch == "TK95") && Config::ALUTK > 0) { // Apply pulse length compensation for Microdigital ALU
        Tape::tapeCompensation = FACTORALUTK;
    } else
        Tape::tapeCompensation = 1;

    // Init CPU
    Z80::create();

    // Set Ports starting values
    for (int i = 0; i < 128; i++) Ports::port[i] = 0xBF;
    if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) Ports::port[0x1f] = 0; // Kempston
    if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) Ports::port[0x7f] = 0xff; // Fuller

    // Read joystick default definition
    for (int n = 0; n < 24; n++)
        ESPectrum::JoyVKTranslation[n] = (fabgl::VirtualKey) Config::joydef[n];

    // Init disk controller
    Betadisk.Init();

    // Reset cpu
    CPU::reset();

    // Clear Cheat data
    CheatMngr::closeCheatFile();

    // Load snapshot if present in Config::ram_file
    if (Config::ram_file != NO_RAM_FILE) {

        FileUtils::SNA_Path = Config::SNA_Path;
        FileUtils::fileTypes[DISK_SNAFILE].begin_row = Config::SNA_begin_row;
        FileUtils::fileTypes[DISK_SNAFILE].focus = Config::SNA_focus;
        FileUtils::fileTypes[DISK_SNAFILE].fdMode = Config::SNA_fdMode;
        FileUtils::fileTypes[DISK_SNAFILE].fileSearch = Config::SNA_fileSearch;

        FileUtils::TAP_Path = Config::TAP_Path;
        FileUtils::fileTypes[DISK_TAPFILE].begin_row = Config::TAP_begin_row;
        FileUtils::fileTypes[DISK_TAPFILE].focus = Config::TAP_focus;
        FileUtils::fileTypes[DISK_TAPFILE].fdMode = Config::TAP_fdMode;
        FileUtils::fileTypes[DISK_TAPFILE].fileSearch = Config::TAP_fileSearch;

        FileUtils::DSK_Path = Config::DSK_Path;
        FileUtils::fileTypes[DISK_DSKFILE].begin_row = Config::DSK_begin_row;
        FileUtils::fileTypes[DISK_DSKFILE].focus = Config::DSK_focus;
        FileUtils::fileTypes[DISK_DSKFILE].fdMode = Config::DSK_fdMode;
        FileUtils::fileTypes[DISK_DSKFILE].fileSearch = Config::DSK_fileSearch;

        LoadSnapshot(Config::ram_file,"","",0xff);
        OSD::LoadCheatFile(Config::last_ram_file);
        Config::ram_file = Config::last_ram_file;

        Config::last_ram_file = Config::ram_file;
        Config::ram_file = NO_RAM_FILE;
        Config::save("ram");

    }

    if (Config::slog_on) showMemInfo("Setup finished.");

}

//=======================================================================================
// RESET
//=======================================================================================
void ESPectrum::reset()
{
    // Load romset
    Config::requestMachine(Config::arch, Config::romSet);

    // Ports
    for (int i = 0; i < 128; i++) Ports::port[i] = 0xBF;
    if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) Ports::port[0x1f] = 0; // Kempston
    if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) Ports::port[0x7f] = 0xff; // Fuller

    // Read joystick default definition
    for (int n = 0; n < 24; n++)
        ESPectrum::JoyVKTranslation[n] = (fabgl::VirtualKey) Config::joydef[n];

    MemESP::Reset(); // Reset Memory

    VIDEO::Reset();

    // Reinit disk controller
    if (Config::DiskCtrl == 1 || Z80Ops::isPentagon) {
        // Betadisk.ShutDown();
        // Betadisk.Init();
        Betadisk.EnterIdle();
    }

/*    Tape::tapeFileName = "none";
    if (Tape::tape != NULL) {
        fclose(Tape::tape);
        Tape::tape = NULL;
    }*/
    Tape::tapeStatus = TAPE_STOPPED;
    Tape::tapePhase = TAPE_PHASE_STOPPED;
    Tape::SaveStatus = SAVE_STOPPED;
    Tape::romLoading = false;

    if (Z80Ops::is128) { // Apply pulse length compensation for 128K
        Tape::tapeCompensation = FACTOR128K;
    } else if ((Config::arch=="TK90X" || Config::arch == "TK95") && Config::ALUTK > 0) { // Apply pulse length compensation for Microdigital ALU
        Tape::tapeCompensation = FACTORALUTK;
    } else
        Tape::tapeCompensation = 1;

    // Set block timings if there's a tape loaded and is a .tap
    if (Tape::tape != NULL && Tape::tapeFileType == TAPE_FTYPE_TAP) {
        Tape::TAP_setBlockTimings();
    }

    // Empty audio buffers
    for (int i=0;i<ESP_AUDIO_SAMPLES_PENTAGON;i++) {
        overSamplebuf[i]=0;
        audioBuffer[i]=0;
        AySound::SamplebufAY[i]=0;
    }
    lastaudioBit=0;

    // Set samples per frame and AY_emu flag depending on arch
    int prevAudio_freq = Audio_freq[ESP_delay];
    if (Config::arch == "48K") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_48;
        audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_48;
        audioAYDivider = ESP_AUDIO_AY_DIV_48;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_48;
        AY_emu = Config::AY48;
        Audio_freq[0] = ESP_AUDIO_FREQ_48;
        Audio_freq[1] = ESP_AUDIO_FREQ_48;
        Audio_freq[2] = ESP_AUDIO_FREQ_48_125SPEED;
        Audio_freq[3] = ESP_AUDIO_FREQ_48_150SPEED;
    } else if (Config::arch == "TK90X" || Config::arch == "TK95") {

        switch (Config::ALUTK) {
        case 0:
            samplesPerFrame=ESP_AUDIO_SAMPLES_48;
            audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_48;
            audioAYDivider = ESP_AUDIO_AY_DIV_48;
            audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_48;
            Audio_freq[0] = ESP_AUDIO_FREQ_48;
            Audio_freq[1] = ESP_AUDIO_FREQ_48;
            Audio_freq[2] = ESP_AUDIO_FREQ_48_125SPEED;
            Audio_freq[3] = ESP_AUDIO_FREQ_48_150SPEED;
            break;
        case 1:
            samplesPerFrame=ESP_AUDIO_SAMPLES_TK_50;
            audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_TK_50;
            audioAYDivider = ESP_AUDIO_AY_DIV_TK_50;
            audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_TK_50;
            Audio_freq[0] = ESP_AUDIO_FREQ_TK_50;
            Audio_freq[1] = ESP_AUDIO_FREQ_TK_50;
            Audio_freq[2] = ESP_AUDIO_FREQ_TK_50_125SPEED;
            Audio_freq[3] = ESP_AUDIO_FREQ_TK_50_150SPEED;
            break;
        case 2:
            samplesPerFrame=ESP_AUDIO_SAMPLES_TK_60;
            audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_TK_60;
            audioAYDivider = ESP_AUDIO_AY_DIV_TK_60;
            audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_TK_60;
            Audio_freq[0] = ESP_AUDIO_FREQ_TK_60;
            Audio_freq[1] = ESP_AUDIO_FREQ_TK_60;
            Audio_freq[2] = ESP_AUDIO_FREQ_TK_60_125SPEED;
            Audio_freq[3] = ESP_AUDIO_FREQ_TK_60_150SPEED;
        }

        AY_emu = Config::AY48;

    } else if (Config::arch == "128K") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_128;
        audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_128;
        audioAYDivider = ESP_AUDIO_AY_DIV_128;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_128;
        AY_emu = true;
        Audio_freq[0] = ESP_AUDIO_FREQ_128;
        Audio_freq[1] = ESP_AUDIO_FREQ_128;
        Audio_freq[2] = ESP_AUDIO_FREQ_128_125SPEED;
        Audio_freq[3] = ESP_AUDIO_FREQ_128_150SPEED;
    } else if (Config::arch == "Pentagon") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_PENTAGON;
        audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_PENTAGON;
        audioAYDivider = ESP_AUDIO_AY_DIV_PENTAGON;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_PENTAGON;
        AY_emu = true;
        Audio_freq[0] = ESP_AUDIO_FREQ_PENTAGON;
        Audio_freq[1] = ESP_AUDIO_FREQ_PENTAGON;
        Audio_freq[2] = ESP_AUDIO_FREQ_PENTAGON_125SPEED;
        Audio_freq[3] = ESP_AUDIO_FREQ_PENTAGON_150SPEED;
    }

    if (Config::tape_player) AY_emu = false; // Disable AY emulation if tape player mode is set

    ESPoffset = 0;

    // Readjust output pwmaudio frequency if needed
    if (prevAudio_freq != Audio_freq[ESP_delay]) {
        // printf("Resetting pwmaudio to freq: %d\n",Audio_freq);
        esp_err_t res;
        res = pwm_audio_set_sample_rate(Audio_freq[ESP_delay]);
        if (res != ESP_OK) {
            printf("Can't set sample rate\n");
        }
    }

    // Reset AY emulation
    AySound::init();
    AySound::set_sound_format(Audio_freq[ESP_delay],1,8);
    AySound::set_stereo(AYEMU_MONO,NULL);
    AySound::reset();

    CPU::reset();

}

//=======================================================================================
// KEYBOARD / KEMPSTON
//=======================================================================================
IRAM_ATTR bool ESPectrum::readKbd(fabgl::VirtualKeyItem *Nextkey) {

    bool r = PS2Controller.keyboard()->getNextVirtualKey(Nextkey);
    // Global keys
    if (Nextkey->down) {
        if (Nextkey->vk == fabgl::VK_PRINTSCREEN) { // Capture framebuffer to BMP file in SD Card (thx @dcrespo3d!)
            CaptureToBmp();
            r = false;
        } else
        if (Nextkey->vk == fabgl::VK_SCROLLLOCK) { // Change CursorAsJoy setting
            Config::CursorAsJoy = !Config::CursorAsJoy;
            PS2Controller.keyboard()->setLEDs(false,false,Config::CursorAsJoy);
            if(ps2kbd2)
                PS2Controller.keybjoystick()->setLEDs(false, false, Config::CursorAsJoy);
            Config::save("CursorAsJoy");
            r = false;
        }
    }

    return r;
}

//
// Read second ps/2 port and inject on first queue
//
IRAM_ATTR void ESPectrum::readKbdJoy() {

    if (ps2kbd2) {

        fabgl::VirtualKeyItem NextKey;
        auto KbdJoy = PS2Controller.keybjoystick();

        while (KbdJoy->virtualKeyAvailable()) {
            PS2Controller.keybjoystick()->getNextVirtualKey(&NextKey);
            ESPectrum::PS2Controller.keyboard()->injectVirtualKey(NextKey.vk, NextKey.down, false);
        }

    }

}

fabgl::VirtualKey ESPectrum::JoyVKTranslation[24];
//     fabgl::VK_FULLER_LEFT, // Left
//     fabgl::VK_FULLER_RIGHT, // Right
//     fabgl::VK_FULLER_UP, // Up
//     fabgl::VK_FULLER_DOWN, // Down
//     fabgl::VK_S, // Start
//     fabgl::VK_M, // Mode
//     fabgl::VK_FULLER_FIRE, // A
//     fabgl::VK_9, // B
//     fabgl::VK_SPACE, // C
//     fabgl::VK_X, // X
//     fabgl::VK_Y, // Y
//     fabgl::VK_Z, // Z

fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_FIRE1 = fabgl::VK_NONE;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_FIRE2 = fabgl::VK_NONE;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_TAB = fabgl::VK_TAB;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_GRAVEACCENT = fabgl::VK_GRAVEACCENT;

IRAM_ATTR void ESPectrum::processKeyboard() {
// void ESPectrum::processKeyboard() {

    static uint8_t PS2cols[8] = { 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf };
    static int zxDelay = 0;
    auto Kbd = PS2Controller.keyboard();
    fabgl::VirtualKeyItem NextKey;
    fabgl::VirtualKey KeytoESP;
    bool Kdown;
    bool r = false;
    bool j[10] = { true, true, true, true, true, true, true, true, true, true };
    // bool j1 = true;
    // bool j2 = true;
    // bool j3 = true;
    // bool j4 = true;
    // bool j5 = true;
    // bool j6 = true;
    // bool j7 = true;
    // bool j8 = true;
    // bool j9 = true;
    // bool j0 = true;
    bool jShift = true;

    readKbdJoy();

    while (Kbd->virtualKeyAvailable()) {

        r = readKbd(&NextKey);

        if (r) {

            KeytoESP = NextKey.vk;
            Kdown = NextKey.down;

            if (KeytoESP >= fabgl::VK_JOY1LEFT && KeytoESP <= fabgl::VK_JOY2Z) {
                // printf("KeytoESP: %d\n",KeytoESP);
                ESPectrum::PS2Controller.keyboard()->injectVirtualKey(JoyVKTranslation[KeytoESP - 248], Kdown, false);
                continue;
            }

            if ((Kdown) && ((KeytoESP >= fabgl::VK_F1 && KeytoESP <= fabgl::VK_F12) || KeytoESP == fabgl::VK_PAUSE || KeytoESP == fabgl::VK_VOLUMEUP || KeytoESP == fabgl::VK_VOLUMEDOWN || KeytoESP == fabgl::VK_VOLUMEMUTE)) {

                int64_t osd_start = esp_timer_get_time();

                OSD::do_OSD(KeytoESP, Kbd->isVKDown(fabgl::VK_LCTRL) || Kbd->isVKDown(fabgl::VK_RCTRL), Kbd->isVKDown(fabgl::VK_LSHIFT) || Kbd->isVKDown(fabgl::VK_RSHIFT));

                Kbd->emptyVirtualKeyQueue();

                // Set all zx keys as not pressed
                for (uint8_t i = 0; i < 8; i++) ZXKeyb::ZXcols[i] = 0xbf;
                zxDelay = 15;

                // totalseconds = 0;
                // totalsecondsnodelay = 0;
                // VIDEO::framecnt = 0;

                // Refresh border
                VIDEO::brdnextframe = true;

                ESPectrum::ts_start += esp_timer_get_time() - osd_start;

                return;

            }

            // Reset keys
            if (Kdown && NextKey.LALT) {
                if (NextKey.CTRL) {
                    if (KeytoESP == fabgl::VK_DELETE) {
                        // printf("Ctrl + Alt + Supr!\n");
                        // ESP host reset
                        Config::ram_file = NO_RAM_FILE;
                        Config::save("ram");
                        OSD::esp_hard_reset();
                    } else if (KeytoESP == fabgl::VK_BACKSPACE) {
                        // printf("Ctrl + Alt + backSpace!\n");
                        // Hard
                        if (Config::ram_file != NO_RAM_FILE) {
                            Config::ram_file = NO_RAM_FILE;
                        }
                        Config::last_ram_file = NO_RAM_FILE;
                        ESPectrum::reset();
                        return;
                    }
                } else if (KeytoESP == fabgl::VK_BACKSPACE) {
                    // printf("Alt + backSpace!\n");
                    // Soft reset
                    if (Config::last_ram_file != NO_RAM_FILE) {
                        LoadSnapshot(Config::last_ram_file,"","",0xff);
                        OSD::LoadCheatFile(Config::last_ram_file);
                        Config::ram_file = Config::last_ram_file;
                    } else {
                        // Clear Cheat data
                        CheatMngr::closeCheatFile();
                        ESPectrum::reset();
                    }
                    return;
                }
            }

            if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) Ports::port[0x1f] = 0;
            if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) Ports::port[0x7f] = 0xff;

            if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON) {

                for (int i = fabgl::VK_KEMPSTON_RIGHT; i <= fabgl::VK_KEMPSTON_ALTFIRE; i++)
                    if (Kbd->isVKDown((fabgl::VirtualKey) i))
                        bitWrite(Ports::port[0x1f], i - fabgl::VK_KEMPSTON_RIGHT, 1);

            }

            if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER) {

                // Fuller
                if (Kbd->isVKDown(fabgl::VK_FULLER_RIGHT)) {
                    bitWrite(Ports::port[0x7f], 3, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_FULLER_LEFT)) {
                    bitWrite(Ports::port[0x7f], 2, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_FULLER_DOWN)) {
                    bitWrite(Ports::port[0x7f], 1, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_FULLER_UP)) {
                    bitWrite(Ports::port[0x7f], 0, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_FULLER_FIRE)) {
                    bitWrite(Ports::port[0x7f], 7, 0);
                }

            }

            jShift = !(Kbd->isVKDown(fabgl::VK_LSHIFT) || Kbd->isVKDown(fabgl::VK_RSHIFT));

            if (Config::CursorAsJoy) {

                // Kempston Joystick emulation
                if (Config::joyPS2 == JOYPS2_KEMPSTON) {

                    if (Kbd->isVKDown(fabgl::VK_RIGHT)) {
                        j[8] = jShift;
                        bitWrite(Ports::port[0x1f], 0, j[8]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_LEFT)) {
                        j[5] = jShift;
                        bitWrite(Ports::port[0x1f], 1, j[5]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_DOWN)) {
                        j[6] = jShift;
                        bitWrite(Ports::port[0x1f], 2, j[6]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_UP)) {
                        j[7] = jShift;
                        bitWrite(Ports::port[0x1f], 3, j[7]);
                    }

                // Fuller Joystick emulation
                } else if (Config::joyPS2 == JOYPS2_FULLER) {

                    if (Kbd->isVKDown(fabgl::VK_RIGHT)) {
                        j[8] = jShift;
                        bitWrite(Ports::port[0x7f], 3, !j[8]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_LEFT)) {
                        j[5] = jShift;
                        bitWrite(Ports::port[0x7f], 2, !j[5]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_DOWN)) {
                        j[6] = jShift;
                        bitWrite(Ports::port[0x7f], 1, !j[6]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_UP)) {
                        j[7] = jShift;
                        bitWrite(Ports::port[0x7f], 0, !j[7]);
                    }

                } else if (Config::joyPS2 == JOYPS2_CURSOR) {

                    j[5] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                    j[8] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                    j[7] =  !Kbd->isVKDown(fabgl::VK_UP);
                    j[6] =  !Kbd->isVKDown(fabgl::VK_DOWN);

                } else if (Config::joyPS2 == JOYPS2_SINCLAIR1) { // Right Sinclair

                    if (jShift) {
                        j[9] =  !Kbd->isVKDown(fabgl::VK_UP);
                        j[8] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                        j[7] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                        j[6] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                    } else {
                        j[5] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                        j[8] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                        j[7] =  !Kbd->isVKDown(fabgl::VK_UP);
                        j[6] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                    }

                } else if (Config::joyPS2 == JOYPS2_SINCLAIR2) { // Left Sinclair

                    if (jShift) {
                        j[4] =  !Kbd->isVKDown(fabgl::VK_UP);
                        j[3] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                        j[2] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                        j[1] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                    } else {
                        j[5] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                        j[8] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                        j[7] =  !Kbd->isVKDown(fabgl::VK_UP);
                        j[6] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                    }

                }

            } else {

                // Cursor Keys
                if (Kbd->isVKDown(fabgl::VK_RIGHT)) {
                    jShift = false;
                    j[8] = jShift;
                }

                if (Kbd->isVKDown(fabgl::VK_LEFT)) {
                    jShift = false;
                    j[5] = jShift;
                }

                if (Kbd->isVKDown(fabgl::VK_DOWN)) {
                    jShift = false;
                    j[6] = jShift;
                }

                if (Kbd->isVKDown(fabgl::VK_UP)) {
                    jShift = false;
                    j[7] = jShift;
                }

            }

            // Keypad PS/2 Joystick emulation
            if (Config::joyPS2 == JOYPS2_KEMPSTON) {

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    bitWrite(Ports::port[0x1f], 0, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    bitWrite(Ports::port[0x1f], 1, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    bitWrite(Ports::port[0x1f], 2, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    bitWrite(Ports::port[0x1f], 3, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    bitWrite(Ports::port[0x1f], 4, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_SLASH) || /*Kbd->isVKDown(fabgl::VK_QUESTION) ||*/Kbd->isVKDown(fabgl::VK_RGUI) || Kbd->isVKDown(fabgl::VK_APPLICATION) || Kbd->isVKDown(VK_ESPECTRUM_FIRE2)) {
                    bitWrite(Ports::port[0x1f], 5, 1);
                }

            } else if (Config::joyPS2 == JOYPS2_FULLER) {

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    bitWrite(Ports::port[0x7f], 3, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    bitWrite(Ports::port[0x7f], 2, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    bitWrite(Ports::port[0x7f], 1, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    bitWrite(Ports::port[0x7f], 0, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    bitWrite(Ports::port[0x7f], 7, 0);
                }

            } else if (Config::joyPS2 == JOYPS2_CURSOR) {

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    jShift = true;
                    j[5] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    jShift = true;
                    j[8] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    jShift = true;
                    j[7] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    jShift = true;
                    j[6] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    jShift = true;
                    j[0] = false;
                };

            } else if (Config::joyPS2 == JOYPS2_SINCLAIR1) { // Right Sinclair

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    jShift = true;
                    j[6] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    jShift = true;
                    j[7] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    jShift = true;
                    j[9] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    jShift = true;
                    j[8] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    jShift = true;
                    j[0] = false;
                };

            } else if (Config::joyPS2 == JOYPS2_SINCLAIR2) { // Left Sinclair

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    jShift = true;
                    j[1] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    jShift = true;
                    j[2] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    jShift = true;
                    j[4] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    jShift = true;
                    j[3] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    jShift = true;
                    j[5] = false;
                };

            }

            // Check keyboard status and map it to Spectrum Ports

            bitWrite(PS2cols[0], 0, (jShift)
                & (!Kbd->isVKDown(fabgl::VK_BACKSPACE))
                & (!Kbd->isVKDown(fabgl::VK_CAPSLOCK)) // Caps lock
                &   (!Kbd->isVKDown(VK_ESPECTRUM_GRAVEACCENT)) // Edit
                &   (!Kbd->isVKDown(VK_ESPECTRUM_TAB)) // Extended mode
                &   (!Kbd->isVKDown(fabgl::VK_ESCAPE)) // Break
                ); // CAPS SHIFT
            bitWrite(PS2cols[0], 1, (!Kbd->isVKDown(fabgl::VK_Z)) & (!Kbd->isVKDown(fabgl::VK_z)));
            bitWrite(PS2cols[0], 2, (!Kbd->isVKDown(fabgl::VK_X)) & (!Kbd->isVKDown(fabgl::VK_x)));
            bitWrite(PS2cols[0], 3, (!Kbd->isVKDown(fabgl::VK_C)) & (!Kbd->isVKDown(fabgl::VK_c)));
            bitWrite(PS2cols[0], 4, (!Kbd->isVKDown(fabgl::VK_V)) & (!Kbd->isVKDown(fabgl::VK_v)));

            bitWrite(PS2cols[1], 0, (!Kbd->isVKDown(fabgl::VK_A)) & (!Kbd->isVKDown(fabgl::VK_a)));
            bitWrite(PS2cols[1], 1, (!Kbd->isVKDown(fabgl::VK_S)) & (!Kbd->isVKDown(fabgl::VK_s)));
            bitWrite(PS2cols[1], 2, (!Kbd->isVKDown(fabgl::VK_D)) & (!Kbd->isVKDown(fabgl::VK_d)));
            bitWrite(PS2cols[1], 3, (!Kbd->isVKDown(fabgl::VK_F)) & (!Kbd->isVKDown(fabgl::VK_f)));
            bitWrite(PS2cols[1], 4, (!Kbd->isVKDown(fabgl::VK_G)) & (!Kbd->isVKDown(fabgl::VK_g)));

            bitWrite(PS2cols[2], 0, (!Kbd->isVKDown(fabgl::VK_Q)) & (!Kbd->isVKDown(fabgl::VK_q)));
            bitWrite(PS2cols[2], 1, (!Kbd->isVKDown(fabgl::VK_W)) & (!Kbd->isVKDown(fabgl::VK_w)));
            bitWrite(PS2cols[2], 2, (!Kbd->isVKDown(fabgl::VK_E)) & (!Kbd->isVKDown(fabgl::VK_e)));
            bitWrite(PS2cols[2], 3, (!Kbd->isVKDown(fabgl::VK_R)) & (!Kbd->isVKDown(fabgl::VK_r)));
            bitWrite(PS2cols[2], 4, (!Kbd->isVKDown(fabgl::VK_T)) & (!Kbd->isVKDown(fabgl::VK_t)));

            bitWrite(PS2cols[3], 0, (!Kbd->isVKDown(fabgl::VK_1)) & (!Kbd->isVKDown(fabgl::VK_EXCLAIM))
                                &   (!Kbd->isVKDown(VK_ESPECTRUM_GRAVEACCENT)) // Edit
                                & (j[1]));
            bitWrite(PS2cols[3], 1, (!Kbd->isVKDown(fabgl::VK_2)) & (!Kbd->isVKDown(fabgl::VK_AT))
                                &   (!Kbd->isVKDown(fabgl::VK_CAPSLOCK)) // Caps lock
                                & (j[2])
                                );
            bitWrite(PS2cols[3], 2, (!Kbd->isVKDown(fabgl::VK_3)) & (!Kbd->isVKDown(fabgl::VK_HASH)) & (j[3]));
            bitWrite(PS2cols[3], 3, (!Kbd->isVKDown(fabgl::VK_4)) & (!Kbd->isVKDown(fabgl::VK_DOLLAR)) & (j[4]));
            bitWrite(PS2cols[3], 4, (!Kbd->isVKDown(fabgl::VK_5)) & (!Kbd->isVKDown(fabgl::VK_PERCENT)) & (j[5]));

            bitWrite(PS2cols[4], 0, (!Kbd->isVKDown(fabgl::VK_0)) & (!Kbd->isVKDown(fabgl::VK_RIGHTPAREN)) & (!Kbd->isVKDown(fabgl::VK_BACKSPACE)) & (j[0]));
            bitWrite(PS2cols[4], 1, !Kbd->isVKDown(fabgl::VK_9) & (!Kbd->isVKDown(fabgl::VK_LEFTPAREN)) & (j[9]));
            bitWrite(PS2cols[4], 2, (!Kbd->isVKDown(fabgl::VK_8)) & (!Kbd->isVKDown(fabgl::VK_ASTERISK)) & (j[8]));
            bitWrite(PS2cols[4], 3, (!Kbd->isVKDown(fabgl::VK_7)) & (!Kbd->isVKDown(fabgl::VK_AMPERSAND)) & (j[7]));
            bitWrite(PS2cols[4], 4, (!Kbd->isVKDown(fabgl::VK_6)) & (!Kbd->isVKDown(fabgl::VK_CARET)) & (j[6]));

            bitWrite(PS2cols[5], 0, (!Kbd->isVKDown(fabgl::VK_P)) & (!Kbd->isVKDown(fabgl::VK_p))
                                &   (!Kbd->isVKDown(fabgl::VK_QUOTE)) // Double quote
                                );
            bitWrite(PS2cols[5], 1, (!Kbd->isVKDown(fabgl::VK_O)) & (!Kbd->isVKDown(fabgl::VK_o))
                                &   (!Kbd->isVKDown(fabgl::VK_SEMICOLON)) // Semicolon
                                );
            bitWrite(PS2cols[5], 2, (!Kbd->isVKDown(fabgl::VK_I)) & (!Kbd->isVKDown(fabgl::VK_i)));
            bitWrite(PS2cols[5], 3, (!Kbd->isVKDown(fabgl::VK_U)) & (!Kbd->isVKDown(fabgl::VK_u)));
            bitWrite(PS2cols[5], 4, (!Kbd->isVKDown(fabgl::VK_Y)) & (!Kbd->isVKDown(fabgl::VK_y)));

            bitWrite(PS2cols[6], 0, !Kbd->isVKDown(fabgl::VK_RETURN));
            bitWrite(PS2cols[6], 1, (!Kbd->isVKDown(fabgl::VK_L)) & (!Kbd->isVKDown(fabgl::VK_l)));
            bitWrite(PS2cols[6], 2, (!Kbd->isVKDown(fabgl::VK_K)) & (!Kbd->isVKDown(fabgl::VK_k)));
            bitWrite(PS2cols[6], 3, (!Kbd->isVKDown(fabgl::VK_J)) & (!Kbd->isVKDown(fabgl::VK_j)));
            bitWrite(PS2cols[6], 4, (!Kbd->isVKDown(fabgl::VK_H)) & (!Kbd->isVKDown(fabgl::VK_h)));

            bitWrite(PS2cols[7], 0, !Kbd->isVKDown(fabgl::VK_SPACE)
                            &   (!Kbd->isVKDown(fabgl::VK_ESCAPE)) // Break
            );
            bitWrite(PS2cols[7], 1, (!Kbd->isVKDown(fabgl::VK_LCTRL)) // SYMBOL SHIFT
                                &   (!Kbd->isVKDown(fabgl::VK_RCTRL))
                                &   (!Kbd->isVKDown(fabgl::VK_COMMA)) // Comma
                                &   (!Kbd->isVKDown(fabgl::VK_PERIOD)) // Period
                                &   (!Kbd->isVKDown(fabgl::VK_SEMICOLON)) // Semicolon
                                &   (!Kbd->isVKDown(fabgl::VK_QUOTE)) // Double quote
                                &   (!Kbd->isVKDown(VK_ESPECTRUM_TAB)) // Extended mode
                                ); // SYMBOL SHIFT
            bitWrite(PS2cols[7], 2, (!Kbd->isVKDown(fabgl::VK_M)) & (!Kbd->isVKDown(fabgl::VK_m))
                                &   (!Kbd->isVKDown(fabgl::VK_PERIOD)) // Period
                                );
            bitWrite(PS2cols[7], 3, (!Kbd->isVKDown(fabgl::VK_N)) & (!Kbd->isVKDown(fabgl::VK_n))
                                &   (!Kbd->isVKDown(fabgl::VK_COMMA)) // Comma
                                );
            bitWrite(PS2cols[7], 4, (!Kbd->isVKDown(fabgl::VK_B)) & (!Kbd->isVKDown(fabgl::VK_b)));

        }

    }

    if (ZXKeyb::Exists) { // START - ZXKeyb Exists

        if (zxDelay > 0)
            zxDelay--;
        else
            // Process physical keyboard
            ZXKeyb::process();

        // Detect and process physical kbd menu key combinations
        // CS+SS+<1..0> -> F1..F10 Keys, CS+SS+Q -> F11, CS+SS+W -> F12, CS+SS+S -> Capture screen
        if ((!bitRead(ZXKeyb::ZXcols[0],0)) && (!bitRead(ZXKeyb::ZXcols[7],1))) {

            zxDelay = 15;

            int64_t osd_start = esp_timer_get_time();

            if (!bitRead(ZXKeyb::ZXcols[3],0)) {
                OSD::do_OSD(fabgl::VK_F1,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[3],1)) {
                OSD::do_OSD(fabgl::VK_F2,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[3],2)) {
                OSD::do_OSD(fabgl::VK_F3,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[3],3)) {
                OSD::do_OSD(fabgl::VK_F4,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[3],4)) {
                OSD::do_OSD(fabgl::VK_F5,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],4)) {
                OSD::do_OSD(fabgl::VK_F6,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],3)) {
                OSD::do_OSD(fabgl::VK_F7,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],2)) {
                OSD::do_OSD(fabgl::VK_F8,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],1)) {
                OSD::do_OSD(fabgl::VK_F9,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],0)) {
                OSD::do_OSD(fabgl::VK_F10,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[2],0)) {
                OSD::do_OSD(fabgl::VK_F11,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[2],1)) {
                OSD::do_OSD(fabgl::VK_F12,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[5],0)) { // P -> Pause
                OSD::do_OSD(fabgl::VK_PAUSE,0,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[5],2)) { // I -> Info
                OSD::do_OSD(fabgl::VK_F1,0,true);
            } else
            if (!bitRead(ZXKeyb::ZXcols[2],2)) { // E -> Eject tape
                OSD::do_OSD(fabgl::VK_F6,0,true);
            } else
            // if (!bitRead(ZXKeyb::ZXcols[5],3)) { // U -> Uart test
            //     OSD::do_OSD(fabgl::VK_F5,0,true);
            // } else
            if (!bitRead(ZXKeyb::ZXcols[2],3)) { // R -> Reset to TR-DOS
                OSD::do_OSD(fabgl::VK_F11,true,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[2],4)) { // T -> Turbo
                OSD::do_OSD(fabgl::VK_F2,true,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[7],4)) { // B -> BMP capture
                CaptureToBmp();
            } else
            if (!bitRead(ZXKeyb::ZXcols[5],1)) { // O -> Poke
                OSD::pokeDialog();
            } else
            if (!bitRead(ZXKeyb::ZXcols[5],2)) { // U -> Cheats
                OSD::do_OSD(fabgl::VK_F9,0,true);
            } else
            if (!bitRead(ZXKeyb::ZXcols[7],3)) { // N -> NMI
                Z80::triggerNMI();
            } else
            if (!bitRead(ZXKeyb::ZXcols[6],2)) { // K -> Help / Kbd layout
                OSD::do_OSD(fabgl::VK_F1,true,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[1],1)) { // S -> Save snapshot
                OSD::do_OSD(fabgl::VK_F2,0,true);
            } else
            if (!bitRead(ZXKeyb::ZXcols[0],1)) { // Z -> CenterH
                if (Config::CenterH > -16) Config::CenterH--;
                Config::save("CenterH");
                OSD::osdCenteredMsg("Horiz. center: " + to_string(Config::CenterH), LEVEL_INFO, 375);
            } else
            if (!bitRead(ZXKeyb::ZXcols[0],2)) { // X -> CenterH
                if (Config::CenterH < 16) Config::CenterH++;
                Config::save("CenterH");
                OSD::osdCenteredMsg("Horiz. center: " + to_string(Config::CenterH), LEVEL_INFO, 375);
            } else
            if (!bitRead(ZXKeyb::ZXcols[0],3)) { // C -> CenterV
                if (Config::CenterV > -16) Config::CenterV--;
                Config::save("CenterV");
                OSD::osdCenteredMsg("Vert. center: " + to_string(Config::CenterV), LEVEL_INFO, 375);
            } else
            if (!bitRead(ZXKeyb::ZXcols[0],4)) { // V -> CenterV
                if (Config::CenterV < 16) Config::CenterV++;
                Config::save("CenterV");
                OSD::osdCenteredMsg("Vert. center: " + to_string(Config::CenterV), LEVEL_INFO, 375);
            } else
                zxDelay = 0;

            if (zxDelay) {

                // Set all keys as not pressed
                for (uint8_t i = 0; i < 8; i++) ZXKeyb::ZXcols[i] = 0xbf;

                // Refresh border
                VIDEO::brdnextframe = true;

                ESPectrum::ts_start += esp_timer_get_time() - osd_start;

                return;
            }

        }

        // Combine both keyboards
        for (uint8_t rowidx = 0; rowidx < 8; rowidx++) {
            Ports::port[rowidx] = PS2cols[rowidx] & ZXKeyb::ZXcols[rowidx];
        }

    } else {

        if (r) {
            for (uint8_t rowidx = 0; rowidx < 8; rowidx++) {
                Ports::port[rowidx] = PS2cols[rowidx];
            }
        }

    }

}

//=======================================================================================
// AUDIO
//=======================================================================================
IRAM_ATTR void ESPectrum::audioTask(void *unused) {

    uint8_t rxparam;

    size_t written;

    // PWM Audio Init
    pwm_audio_config_t pac;
    pac.duty_resolution    = LEDC_TIMER_8_BIT;
    pac.gpio_num_left      = SPEAKER_PIN;
    pac.ledc_channel_left  = LEDC_CHANNEL_0;
    pac.gpio_num_right     = -1;
    pac.ledc_channel_right = LEDC_CHANNEL_1;
    pac.ledc_timer_sel     = LEDC_TIMER_0;
    pac.tg_num             = TIMER_GROUP_0;
    pac.timer_num          = TIMER_0;
    pac.ringbuf_len        = 3072; /* 4096; */

    pwm_audio_init(&pac);
    pwm_audio_set_param(Audio_freq[ESP_delay],LEDC_TIMER_8_BIT,1);
    pwm_audio_start();
    pwm_audio_set_volume(aud_volume);

    for (;;) {

        xQueueReceive(audioTaskQueue, &rxparam, portMAX_DELAY);

        pwm_audio_write(audioBuffer, samplesPerFrame, &written, 5 / portTICK_PERIOD_MS);

        xQueueReceive(audioTaskQueue, &rxparam, portMAX_DELAY);

#if 0
        // Finish fill of beeper oversampled audio buffers
        for (uint32_t faudbufcnt = audbufcntover * audioSampleDivider + audioBitbufCount;
                      faudbufcnt < (samplesPerFrame * audioSampleDivider);
                      faudbufcnt++)
        {
            audioBitBuf += faudioBit;
            if (++audioBitbufCount == audioSampleDivider) {
                overSamplebuf[audbufcntover++] = audioBitBuf;
                if ( audbufcntover > samplesPerFrame ) printf("overSamplebuf buffer overflow!\n");
                audioBitBuf = 0;
                audioBitbufCount = 0;
            }
        }
#endif

        if (AY_emu) {
            if (faudbufcntAY < samplesPerFrame) AySound::gen_sound(samplesPerFrame - faudbufcntAY, faudbufcntAY);
//            if (faudbufcntAY < samplesPerFrame)
//                AySound::gen_sound(samplesPerFrame - faudbufcntAY , faudbufcntAY);
            for (int i = 0; i < samplesPerFrame; i++) {
                int beeper = (overSamplebuf[i] / audioSampleDivider) + AySound::SamplebufAY[i];
                audioBuffer[i] = beeper > 255 ? 255 : beeper; // Clamp
            }
        } else {
            for (int i = 0; i < samplesPerFrame; i++) {
                audioBuffer[i] = overSamplebuf[i] / audioSampleDivider;
            }
        }

    }
}

IRAM_ATTR void ESPectrum::BeeperGetSample() {

    // Beeper audiobuffer generation (oversample)
    uint32_t audbufpos = CPU::tstates / audioOverSampleDivider;
    if ( audbufcntover < samplesPerFrame ) { // <-- this don't must be needed
        for (;audbufcnt < audbufpos; audbufcnt++) {
            audioBitBuf += lastaudioBit;
            if (++audioBitbufCount == audioSampleDivider) {
                overSamplebuf[audbufcntover++] = audioBitBuf;
                audioBitBuf = 0;
                audioBitbufCount = 0;
                if (audbufcntover == samplesPerFrame) break; // <-- this don't must be needed
            }
        }
    }

}

IRAM_ATTR void ESPectrum::AYGetSample() {
    // AY audiobuffer generation (oversample)
    uint32_t audbufpos = CPU::tstates / audioAYDivider;
    if (audbufpos > audbufcntAY) {
        AySound::gen_sound(audbufpos - audbufcntAY, audbufcntAY);
        audbufcntAY = audbufpos;
    }
}

//=======================================================================================
// MAIN LOOP
//=======================================================================================

IRAM_ATTR void ESPectrum::loop() {

//    // Video adjustment
//    VIDEO::vga.clear(zxColor(3,0)); // For overscan testing. Remove once adjusted

//    for(;;) {

//        processKeyboard();

//        VIDEO::vga.fillRect(165,126,20,20,zxColor(4,1));
//        VIDEO::vga.fillRect(0,0,20,20,zxColor(4,1));
//        VIDEO::vga.fillRect(331,0,20,20,zxColor(4,1));
//        VIDEO::vga.fillRect(0,251,20,20,zxColor(4,1));
//        VIDEO::vga.fillRect(331,251,20,20,zxColor(4,1));

//        VIDEO::vga.line(331,271,340,271,zxColor(0,0));
//        VIDEO::vga.line(10,271,19,271,zxColor(0,0));
//        VIDEO::vga.line(10,0,19,0,zxColor(0,0));
//        VIDEO::vga.line(331,0,340,0,zxColor(0,0));

//    }

    for(;;) {

        ts_start = esp_timer_get_time();

        // Send audioBuffer to pwmaudio
        if (ESP_delay) xQueueSend(audioTaskQueue, &param, portMAX_DELAY);

        audbufcnt = 0;
        audbufcntover = 0;
        audbufcntAY = 0;

        CPU::loop();

        // --- Process audio buffer
        // Finish fill of beeper oversampled audio buffers
        for (; audbufcnt < (samplesPerFrame * audioSampleDivider); audbufcnt++) {
            audioBitBuf += lastaudioBit;
            if (++audioBitbufCount == audioSampleDivider) {
                overSamplebuf[audbufcntover++] = audioBitBuf;
                audioBitBuf = 0;
                audioBitbufCount = 0;
            }
        }
        faudbufcntAY = audbufcntAY;
        // --- Process audio buffer end

        if (ESP_delay) xQueueSend(audioTaskQueue, &param, portMAX_DELAY);

        processKeyboard();

        // Update stats every 50 frames
        if (VIDEO::OSD && VIDEO::framecnt >= 50) {

            if (VIDEO::OSD & 0x04) {

                // printf("Vol. OSD out -> Framecnt: %d\n", VIDEO::framecnt);

                if (VIDEO::framecnt >= 100) {

                    // Save selected volume if not in tape player mode
                    if (!Config::tape_player) {
                        Config::volume = aud_volume;
                        Config::save("volume");
                    }

                    VIDEO::OSD &= 0xfb;

                    if (VIDEO::OSD == 0) {

                        if (Config::aspect_16_9)
                            VIDEO::Draw_OSD169 = VIDEO::MainScreen;
                        else
                            VIDEO::Draw_OSD43 = Z80Ops::isPentagon ? VIDEO::BottomBorder_Pentagon :  VIDEO::BottomBorder;

                        VIDEO::brdnextframe = true;

                    }

                }

            }

            if ((VIDEO::OSD & 0x04) == 0) {

                if (VIDEO::OSD == 1 && Tape::tapeStatus==TAPE_LOADING) {

                    snprintf(OSD::stats_lin1, sizeof(OSD::stats_lin1), " %-12s %04d/%04d ", Tape::tapeFileName.substr(0 + ESPectrum::TapeNameScroller, 12).c_str(), Tape::tapeCurBlock + 1, Tape::tapeNumBlocks);

                    float percent = (float)((Tape::tapebufByteCount + Tape::tapePlayOffset) * 100) / (float)Tape::tapeFileSize;
                    snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2), " %05.2f%% %07d%s%07d ", percent, Tape::tapebufByteCount + Tape::tapePlayOffset, "/" , Tape::tapeFileSize);

                    if ((++ESPectrum::TapeNameScroller + 12) > Tape::tapeFileName.length()) ESPectrum::TapeNameScroller = 0;

                    OSD::drawStats();

                } else if (VIDEO::OSD == 2) {

                    snprintf(OSD::stats_lin1, sizeof(OSD::stats_lin1), "CPU: %05d / IDL: %05d ", (int)(ESPectrum::elapsed), (int)(ESPectrum::idle));
                    snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2), "FPS:%6.2f / FND:%6.2f ", VIDEO::framecnt / (ESPectrum::totalseconds / 1000000), VIDEO::framecnt / (ESPectrum::totalsecondsnodelay / 1000000));

                    OSD::drawStats();

                }

                totalseconds = 0;
                totalsecondsnodelay = 0;
                VIDEO::framecnt = 0;

            }

        }

        // Flashing flag change
        if (!(VIDEO::flash_ctr++ & 0x0f)) VIDEO::flashing ^= 0x80;

        #ifdef ESPECTRUM_PSRAM
        #ifdef TIME_MACHINE_ENABLED
        // Time machine
        if (Config::TimeMachine) MemESP::Tm_DoTimeMachine();
        #endif
        #endif

        elapsed = esp_timer_get_time() - ts_start;
        idle = target[ESP_delay] - elapsed - ESPoffset; // 100% No turbo
        // idle = 15974 - elapsed - ESPoffset; // +125% 48k
        // idle = 13312 - elapsed - ESPoffset; // +150% 48K
        // idle = 11138 - elapsed - ESPoffset; // +150% TK60

        totalsecondsnodelay += elapsed;

        if (!ESP_delay) {
            totalseconds += elapsed;
            continue;
        }

        if(Config::videomode && !Config::tape_player && ESP_delay == 1) {

            if (sync_cnt++ == 0) {

                if (idle > 0)
                    delayMicroseconds(idle);

            } else {

                // Audio sync (once every 128 frames ~ 2,5 seconds)
                if (sync_cnt & 0x80) {
                    ESPoffset = 128 - pwm_audio_rbstats();
                    sync_cnt = 0;
                }

                // Wait for vertical sync
                for (;;) {
                    if (vsync) break;
                }

                // printf("Vsync!\n");

            }

        } else {

            if (idle > 0)
                delayMicroseconds(idle);
            // else
            //     printf("Elapsed: %d\n",(int)elapsed);

            // Audio sync
            if (++sync_cnt & 0x10) {
                ESPoffset = 128 - pwm_audio_rbstats();
                sync_cnt = 0;
            }

        }

        totalseconds += esp_timer_get_time() - ts_start;
        // printf("Totalsecond: %f\n",double(esp_timer_get_time() - ts_start));

    }

}
