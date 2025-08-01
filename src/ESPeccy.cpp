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
#include <string>
#include <functional>

#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"

#include "ESPeccy.h"
#include "Snapshot.h"
#include "Config.h"
#include "FileUtils.h"
#include "OSD.h"
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
#include "BIOS.h"
#include "ROMLoad.h"
#include "RealTape.h"
#include "KBDLayout.h"
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
#include "esp_ota_ops.h"

#include "CommitDate.h"

//=======================================================================================
// KEYBOARD
//=======================================================================================
fabgl::PS2Controller ESPeccy::PS2Controller;
bool ESPeccy::ps2kbd = false;
bool ESPeccy::ps2kbd2 = false;
bool ESPeccy::ps2mouse = false;

//=======================================================================================
// AUDIO
//=======================================================================================
uint8_t ESPeccy::audioBuffer[ESP_AUDIO_SAMPLES_PENTAGON] = { 0 };
uint8_t ESPeccy::SamplebufCOVOX[ESP_AUDIO_SAMPLES_PENTAGON] = { 0 };
uint32_t ESPeccy::audbufcntCOVOX = 0;
uint32_t ESPeccy::faudbufcntCOVOX = 0;
uint8_t ESPeccy::covoxData[4];
uint16_t ESPeccy::covoxMix;
uint16_t ESPeccy::fcovoxMix;
uint32_t* ESPeccy::overSamplebuf;
signed char ESPeccy::aud_volume = ESP_VOLUME_DEFAULT;
uint32_t ESPeccy::audbufcnt = 0;
uint32_t ESPeccy::audbufcntover = 0;
// uint32_t ESPeccy::faudbufcnt = 0;
uint32_t ESPeccy::faudbufcntover = 0;
uint32_t ESPeccy::audbufcntAY = 0;
uint32_t ESPeccy::faudbufcntAY = 0;
int ESPeccy::lastaudioBit = 0;
int ESPeccy::flastaudioBit = 0;
static int faudioBitBuf = 0;
static unsigned char faudioBitbufCount = 0;
//int ESPeccy::faudioBit = 0;
int ESPeccy::samplesPerFrame;
bool ESPeccy::AY_emu = false;
int ESPeccy::Audio_freq[4];
unsigned char ESPeccy::audioSampleDivider;
unsigned char ESPeccy::audioAYDivider;
unsigned char ESPeccy::audioCOVOXDivider;
unsigned char ESPeccy::audioOverSampleDivider;
int ESPeccy::audioBitBuf = 0;
unsigned char ESPeccy::audioBitbufCount = 0;
QueueHandle_t audioTaskQueue;
TaskHandle_t ESPeccy::audioTaskHandle;
uint8_t param;

int runBios = 0;

uint8_t ESPeccy::aud_active_sources = 0;

bool booting = false;

//=======================================================================================
// TAPE OSD
//=======================================================================================

int ESPeccy::TapeNameScroller = 0;

//=======================================================================================
// BETADISK
//=======================================================================================

bool ESPeccy::trdos = false;
WD1793 ESPeccy::Betadisk;

//=======================================================================================
// RealTape
//=======================================================================================

RealTapeParams ESP_RT_PARMS;
bool ESPeccy::sync_realtape = false;

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

double ESPeccy::totalseconds = 0;
double ESPeccy::totalsecondsnodelay = 0;
int64_t ESPeccy::target[4];
int ESPeccy::sync_cnt = 0;
volatile bool ESPeccy::vsync = false;
int64_t ESPeccy::ts_start;
int64_t ESPeccy::elapsed;
int64_t ESPeccy::idle;
uint8_t ESPeccy::ESP_delay = 1; // EMULATION SPEED: 0-> MAX. SPEED (NO SOUND), 1-> 100% SPEED, 2-> 125% SPEED, 3-> 150% SPEED
int ESPeccy::ESPoffset = 0;

//=======================================================================================
// LOGGING / TESTING
//=======================================================================================

int ESPeccy::ESPtestvar = 0;
int ESPeccy::ESPtestvar1 = 0;
int ESPeccy::ESPtestvar2 = 0;

#define START_MSG_DURATION 20

void ShowStartMsg() {

    VIDEO::clear(zxColor(WHITE, BRIGHT_OFF));

    OSD::drawOSD(false);

    VIDEO::fillRect(OSD::osdInsideX(), OSD::osdInsideY(), OSD_COLS * OSD_FONT_W, 50, zxColor(BLACK, BRIGHT_OFF));

    // Decode Logo in EBF8 format
    int logo_w = (ESPeccy_logo[5] << 8) + ESPeccy_logo[4]; // Get Width
    int logo_h = (ESPeccy_logo[7] << 8) + ESPeccy_logo[6]; // Get Height
    int pos_x = OSD::osdInsideX() + ( OSD_COLS * OSD_FONT_W - logo_w ) / 2;
    int pos_y = OSD::osdInsideY() + ( 50 - logo_h ) / 2;

    OSD::drawCompressedBMP(pos_x, pos_y, ESPeccy_logo);

    OSD::osdAt(7, 1);
    VIDEO::setTextColor(zxColor(WHITE, BRIGHT_OFF), zxColor(BLACK, BRIGHT_OFF));

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
                VIDEO::setTextColor(zxColor(foreint & 0x7, foreint >> 3), zxColor(backint & 0x7, backint >> 3));
                continue;
            }
            VIDEO::print((const char)nextChar);
        } else {
            VIDEO::print("\n");
        }
    }

    char msg[38];
    bool quit = false;
    for (int i=START_MSG_DURATION; i >= 0 && !quit; i--) {
        OSD::osdAt(20, 1);
        sprintf(msg,STARTMSG_CLOSE[Config::lang],i);

        VIDEO::setTextColor(zxColor(WHITE, BRIGHT_OFF), zxColor(BLACK, BRIGHT_OFF));
        VIDEO::print(msg);

        for (int j = 0; j < 200; j++) {
            if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEDIALOG);

            ESPeccy::readKbdJoy();

            if (ESPeccy::PS2Controller.keyboard()->virtualKeyAvailable()) {
                fabgl::VirtualKeyItem Nextkey;
                ESPeccy::readKbd(&Nextkey, KBDREAD_MODEDIALOG);
                if(!Nextkey.down) continue;
                if (Nextkey.vk == fabgl::VK_ESCAPE || Nextkey.vk == fabgl::VK_RETURN || Nextkey.vk == fabgl::VK_JOY1A || Nextkey.vk == fabgl::VK_JOY1B || Nextkey.vk == fabgl::VK_JOY2A || Nextkey.vk == fabgl::VK_JOY2B) {
                    quit = true;
                    break;
                }
            }
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }

    VIDEO::clear(zxColor(WHITE, BRIGHT_OFF));

    // Disable StartMsg
    Config::StartMsg = false;

    // Save all keys after new flash or update
    Config::save();

}

void ESPeccy::showMemInfo(const char* caption) {

    string textout;

    multi_heap_info_t info;

    if (caption) printf("=== %s ===\n", caption);

    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
    textout = " Total free bytes         : " + to_string(info.total_free_bytes) + "\n";
    printf(textout.c_str());

    textout = " Minimum free ever        : " + to_string(info.minimum_free_bytes) + "\n";
    printf(textout.c_str());

    printf("CAPS\n", __FUNCTION__);
    printf("    MALLOC_CAP_8BIT      free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    printf("    MALLOC_CAP_32BIT     free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
    printf("    MALLOC_CAP_32BIT INT free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_32BIT|MALLOC_CAP_INTERNAL), heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT|MALLOC_CAP_INTERNAL), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT|MALLOC_CAP_INTERNAL));
    printf("    MALLOC_CAP_DMA       free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_minimum_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    printf("    MALLOC_CAP_SPIRAM    free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM), heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM), heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    printf("    MALLOC_CAP_INTERNAL  free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    printf("    MALLOC_CAP_DEFAULT   free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT), heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT), heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    printf("    MALLOC_CAP_IRAM_8BIT free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_IRAM_8BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_IRAM_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_IRAM_8BIT));
    printf("    MALLOC_CAP_RETENTION free:%9lu min:%9lu largest free:%9lu\n", heap_caps_get_free_size(MALLOC_CAP_RETENTION), heap_caps_get_minimum_free_size(MALLOC_CAP_RETENTION), heap_caps_get_largest_free_block(MALLOC_CAP_RETENTION));

}

string ESPeccy::getHardwareInfo() {
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

    // Continuar obteniendo información del hardware

    textout += " Chip cores    : " + to_string(chip_info.cores) + "\n";
    textout += " Chip revision : " + to_string(chip_info.revision) + "\n";
    textout += " Flash size    : " + to_string(spi_flash_get_chip_size() / (1024 * 1024)) + (chip_info.features & CHIP_FEATURE_EMB_FLASH ? "MB embedded" : "MB external") + "\n";
    multi_heap_info_t info; heap_caps_get_info(&info, MALLOC_CAP_SPIRAM);
    Config::psramsize = (info.total_free_bytes + info.total_allocated_bytes) >> 10;
    textout += " PSRAM size    : " + (Config::psramsize == 0 ? "N/A or disabled" : to_string(Config::psramsize) + " MB") + "\n";
    textout += " IDF Version   : " + (string)(esp_get_idf_version()) + "\n";

    return textout;
};

//=======================================================================================
// BOOT KEYBOARD
//=======================================================================================
void ESPeccy::bootKeyboard() {

    // printf("Boot kbd!\n");

    bool biosButton = false;

    for (int i = 0; i < 200; i++) {

        if (ZXKeyb::Exists) {
            // Process physical keyboard
            ZXKeyb::process();

            // Detect and process physical kbd menu key combinations
            if (ZXKBD_2) { // 2
                runBios = 1;

            } else
            if (ZXKBD_3) { // 3
                runBios = 3;
            }

        }
        else
        if (!gpio_get_level((gpio_num_t)GPIO_NUM_36)) {
            biosButton = true;
            break;
        }

	    if (ps2kbd) {
            auto Kbd = PS2Controller.keyboard();
            fabgl::VirtualKeyItem NextKey;

            while (Kbd->virtualKeyAvailable()) {

                bool r = Kbd->getNextVirtualKey(&NextKey);

                if (r && NextKey.down) {

                    // Check keyboard status
                    switch (NextKey.vk) {
                        case fabgl::VK_F2:
                        case fabgl::VK_2:
                            runBios = 1;
                            break;

                        case fabgl::VK_F3:
                        case fabgl::VK_3:
                            runBios = 3;
                            break;
                    }

                }
            }
        }

        if (runBios) break;

        delayMicroseconds(1000);

    }

    if (biosButton) {
        runBios = 1;
        /* continue reading for 11 seconds max */
        for (int i = 0; i < 11000 && !gpio_get_level((gpio_num_t)GPIO_NUM_36); i++) {
            if (i >= 10000) { /* 10 seconds, then CRT BIOS */
                runBios = 3;
                break;
            }
            delayMicroseconds(1000);
        }
    }


    if (runBios) {
        Config::videomode = runBios - 1;
        Config::aspect_16_9 = false;
        Config::scanlines = false;
    }

}


//=======================================================================================
// SETUP
//=======================================================================================

void ESPeccy::setup()
{

    booting = true;

    // force get psram size
    ESPeccy::getHardwareInfo();

    if (Config::slog_on) {
        printf("------------------------------------\n");
        printf("| ESPeccy: booting                 |\n");
        printf("------------------------------------\n");
        showMemInfo();
    }

    //=======================================================================================
    // PHYSICAL KEYBOARD (SINCLAIR 8 + 5 MEMBRANE KEYBOARD)
    //=======================================================================================

    ZXKeyb::setup();

    //=======================================================================================
    // PHYSICAL KEYBOARD THROUGH MCP23017
    //=======================================================================================

    // if (!ZXKeyb::Exists) MCPKeyb::setup();

    //=======================================================================================
    // LOAD CONFIG
    //=======================================================================================

    bool config_load_failed = Config::load();

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

    bool was_vreset = false;

    // Set arch if there's no snapshot to load
    if (Config::ram_file == NO_RAM_FILE) {

        if (Config::pref_arch.substr(Config::pref_arch.length()-1) == "R") {

            Config::pref_arch.pop_back();
            Config::save("pref_arch");

            was_vreset = true;

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
            } else if (Config::arch == "+2A") {
                if (Config::pref_romSet_2A != "Last")
                    Config::romSet = Config::pref_romSet_2A;
                else
                    Config::romSet = Config::romSet2A;
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

        }

    }

    printf("Arch: %s, Romset: %s\n",Config::arch.c_str(), Config::romSet.c_str());

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

    // Config::ps2_dev2 = 0; // Force 0 for testing

    if (ZXKeyb::Exists) {

        switch (Config::ps2_dev2) {
            case 0:
                PS2Controller.begin(PS2Preset::zxKeyb, KbdMode::CreateVirtualKeysQueue);
                break;
            case 1:
                PS2Controller.begin(PS2Preset::KeyboardPort0, KbdMode::CreateVirtualKeysQueue);
                // if (PS2Controller.keyboard()->isKeyboardAvailable() == false) {
                //     printf("2nd PS/2 device (Kbd/ESPjoy) not detected\n");
                //     Config::ps2_dev2 = 3;
                //     Config::save("PS2Dev2");
                //     OSD::esp_hard_reset();
                // }
                ps2kbd = true;
                break;
            case 2:
                PS2Controller.begin(PS2Preset::MousePort0, KbdMode::CreateVirtualKeysQueue);
                ps2mouse = PS2Controller.mouse() && PS2Controller.mouse()->isMouseAvailable();
                if (!ps2mouse) {
                    printf("2nd PS/2 device (Kbd/ESPjoy) not detected\n");
                    Config::ps2_dev2 = 4;
                    Config::save("PS2Dev2");
                    OSD::esp_hard_reset();
                }
                break;
            case 3:
            case 4:
                PS2Controller.begin(PS2Preset::zxKeyb, KbdMode::CreateVirtualKeysQueue);
                Config::ps2_dev2 -= 2;
                Config::save("PS2Dev2");
                break;
        }

    } else {

        switch (Config::ps2_dev2) {
            case 0:
                PS2Controller.begin(PS2Preset::KeyboardPort0, KbdMode::CreateVirtualKeysQueue);
                ps2kbd = true;
                break;
            case 1:
                PS2Controller.begin(PS2Preset::KeyboardPort0_KeybJoystickPort1, KbdMode::CreateVirtualKeysQueue);
                ps2kbd  = true;
                ps2kbd2 = true;
                // ps2kbd2 = PS2Controller.keybjoystick()->isKeyboardAvailable();
                // if (!ps2kbd2) {
                //     printf("2nd PS/2 device (Kbd/ESPjoy) not detected\n");
                //     Config::ps2_dev2 = 3;
                //     Config::save("PS2Dev2");
                //     OSD::esp_hard_reset();
                // }
                break;
            case 2:
                PS2Controller.begin(PS2Preset::KeyboardPort0_MousePort1, KbdMode::CreateVirtualKeysQueue);
                ps2kbd = true;
                ps2mouse = PS2Controller.mouse() && PS2Controller.mouse()->isMouseAvailable();
                if (!ps2mouse) {
                    printf("2nd PS/2 device (mouse) not detected\n");
                    Config::ps2_dev2 = 4;
                    Config::save("PS2Dev2");
                    OSD::esp_hard_reset();
                }
                break;
            case 3:
            case 4:
                PS2Controller.begin(PS2Preset::KeyboardPort0, KbdMode::CreateVirtualKeysQueue);
                ps2kbd = true;
                Config::ps2_dev2 -= 2;
                Config::save("PS2Dev2");
                break;
        }

    }

    if (ps2mouse) {
        printf("Mouse detected\n");
        PS2Controller.mouse()->setSampleRate(Config::mousesamplerate);
        PS2Controller.mouse()->setResolution(Config::mousedpi);
        PS2Controller.mouse()->setScaling(Config::mousescaling);
    }

    printf("Waiting boot keys\n");
    bootKeyboard(); // BOOTKEYS: Read keyboard for 200 ms. checking boot keys
    printf("End Waiting boot keys\n");

    // Set Scroll Lock Led as current CursorAsJoy value
    if(ps2kbd)  PS2Controller.keyboard()->setLEDs(false, false, Config::CursorAsJoy);
    if(ps2kbd2) PS2Controller.keybjoystick()->setLEDs(false, false, Config::CursorAsJoy);

    // Set TAB and GRAVEACCENT behaviour
    if (Config::TABasfire1) {
        ESPeccy::VK_ESPECCY_FIRE1 = fabgl::VK_TAB;
        ESPeccy::VK_ESPECCY_FIRE2 = fabgl::VK_GRAVEACCENT;
        ESPeccy::VK_ESPECCY_TAB = fabgl::VK_NONE;
        ESPeccy::VK_ESPECCY_GRAVEACCENT = fabgl::VK_NONE;
    } else {
        ESPeccy::VK_ESPECCY_FIRE1 = fabgl::VK_NONE;
        ESPeccy::VK_ESPECCY_FIRE2 = fabgl::VK_NONE;
        ESPeccy::VK_ESPECCY_TAB = fabgl::VK_TAB;
        ESPeccy::VK_ESPECCY_GRAVEACCENT = fabgl::VK_GRAVEACCENT;
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

    // Load romset
    Config::requestMachine(Config::arch, Config::romSet);

    if (Config::slog_on) showMemInfo("ROMSET loaded");

    MemESP::Init();

    if (Config::slog_on) showMemInfo("MEM Allocated");

    MemESP::Reset();

    if (Config::slog_on) showMemInfo("RAM Initialized");

    //=======================================================================================
    // VIDEO
    //=======================================================================================

    VIDEO::Init();

    if (Config::slog_on) showMemInfo("VIDEO Initialized");

    VIDEO::Reset();

    if (Config::slog_on) showMemInfo("VGA started");

    //=======================================================================================
    // INIT FILESYSTEM
    //=======================================================================================

    FileUtils::initFileSystem();

    if (Config::slog_on) showMemInfo("File system started");

    //=======================================================================================
    // Start Message
    //=======================================================================================

    if (Config::StartMsg) {
        ShowStartMsg(); // Show welcome message
        if (config_load_failed) {
            if (FileUtils::SDReady && !FileUtils::isMountedSDCard()) FileUtils::unmountSDCard();
            if (!FileUtils::SDReady) FileUtils::initFileSystem();
            if (FileUtils::SDReady && Config::backupExistsOnSD()) {
                uint8_t res = OSD::msgDialog("Config backup found on SD.", "Restore it?");
                if (res == DLG_YES) {
                    Config::loadFromSD();
                    Config::save();
                }
            }
        }
    }

    //=======================================================================================
    // BIOS
    //=======================================================================================

    if (runBios) BIOS::run();

    //=======================================================================================
    // Custom Keyboard Layout
    //=======================================================================================

    if ( Config::KBDLayoutFile != "" ) {
        if ( FileUtils::isSDReady() ) {
            FILE *l = fopen(Config::KBDLayoutFile.c_str(), "rb");
            Config::KBDLayoutFile = "";
            Config::save("KBDLayoutFile");
            if (l == NULL) {
                OSD::osdCenteredMsg(ERR_READ_FILE, LEVEL_WARN, 2000);
            } else {
                uint8_t * layout = (uint8_t *)KBDLayout::load(l);
                if (!layout) {
                    OSD::osdCenteredMsg(ERR_READ_FILE, LEVEL_ERROR, 3000);
                } else {
                    esp_err_t res = OSD::updateFirmwareContent(layout, 128);
                    free(layout);
                    OSD::osdCenteredMsg((string)"ERROR Code = " + to_string(res), LEVEL_ERROR, 3000);
                }
                fclose(l);
            }
        }
    }

    KBDLayout::reset();

    //=======================================================================================
    // AUDIO
    //=======================================================================================

    overSamplebuf = (uint32_t *) heap_caps_calloc(ESP_AUDIO_SAMPLES_PENTAGON, sizeof(uint32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);
    if (overSamplebuf == NULL) printf("Can't allocate oversamplebuffer\n");

    // Create Audio task
    audioTaskQueue = xQueueCreate(1, sizeof(uint8_t));
    // Latest parameter = Core. In ESPIF, main task runs on core 0 by default. In Arduino, loop() runs on core 1.
    xTaskCreatePinnedToCore(&ESPeccy::audioTask, "audioTask", 2048 /* 1024 /* 1536 */, NULL, configMAX_PRIORITIES - 1, &audioTaskHandle, 1);

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

    } else if (Config::arch == "128K" || Config::arch == "+2A") {
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

    audioCOVOXDivider = audioAYDivider;
    covoxData[0] = covoxData[1] = covoxData[2] = covoxData[3] = 0;

    if (Config::load_monitor) {
        AY_emu = false; // Disable AY emulation if tape load monitor mode is set
        ESPeccy::aud_volume = ESP_VOLUME_MAX;
    } else
        ESPeccy::aud_volume = Config::volume;

    ESPoffset = 0;

    // AY Sound
    AySound::init();
    AySound::set_sound_format(Audio_freq[ESP_delay],1,8);
    AySound::set_stereo(AYEMU_MONO,NULL);
    AySound::reset();

    aud_active_sources = (Config::Covox & 0x01) | (AY_emu << 1);

    // Init tape
    Tape::Init();
    Tape::tapeFileName = "none";
    Tape::tapeStatus = TAPE_STOPPED;
    Tape::tapeFileType = TAPE_FTYPE_EMPTY;
    Tape::romLoading = false;

    if (Z80Ops::is128 || Z80Ops::is2a3) { // Apply pulse length compensation for 128K
        Tape::tapeCompensation = FACTOR128K;
    } else if ((Config::arch=="TK90X" || Config::arch == "TK95") && Config::ALUTK > 0) { // Apply pulse length compensation for Microdigital ALU
        Tape::tapeCompensation = FACTORALUTK;
    } else
        Tape::tapeCompensation = 1;

    // Init CPU
    Z80::create();

    // Set Ports starting values
    for (int i = 0; i < 128; i++) Ports::port[i] = 0xBF;
    Ports::LastOutTo1FFD = 0;
    if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) Ports::port[0x1f] = 0; // Kempston
    if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) Ports::port[0x7f] = 0xff; // Fuller

    // Read joystick default definition
    for (int n = 0; n < 24; n++)
        ESPeccy::JoyVKTranslation[n] = (fabgl::VirtualKey) Config::joydef[n];

    // Init disk controller
    Betadisk.Init();

    // rvmWD1793Reset(&fdd);

    // Reset cpu
    CPU::reset();

    // Clear Cheat data
    CheatMngr::closeCheatFile();

    Config::last_rom_file = Config::rom_file;

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
        OSD::LoadCheatFile(Config::ram_file);
        Config::ram_file = NO_RAM_FILE;
        Config::save("ram");

    } else {
        // Load ROM if present in Config::rom_file
        if (!was_vreset && Config::rom_file != NO_ROM_FILE) {
            ROMLoad::load(Config::rom_file);
            Config::rom_file = NO_ROM_FILE;
            Config::save("rom");
        }
    }

    // Configura carga de cinta real
    RealTape_realloc_buffers(ESPeccy::Audio_freq[1], ESPeccy::samplesPerFrame, CPU::statesInFrame);

    ESP_RT_PARMS.gpio_num = &Config::realtape_gpio_num;
    ESP_RT_PARMS.global_tstates = &CPU::global_tstates;
    ESP_RT_PARMS.tstates = &CPU::tstates;
    ESP_RT_PARMS.zxkeyb_exists = ZXKeyb::Exists;

    RealTape_init(&ESP_RT_PARMS);

    if (Config::slog_on) showMemInfo("Setup finished.");

    booting = false;

}

//=======================================================================================
// RESET
//=======================================================================================

void ESPeccy::reset()
{
    // Load romset
    Config::requestMachine(Config::arch, Config::romSet);

    // Ports
    for (int i = 0; i < 128; i++) Ports::port[i] = 0xBF;
    Ports::LastOutTo1FFD = 0;
    if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) Ports::port[0x1f] = 0; // Kempston
    if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) Ports::port[0x7f] = 0xff; // Fuller

    // Read joystick default definition
    for (int n = 0; n < 24; n++)
        ESPeccy::JoyVKTranslation[n] = (fabgl::VirtualKey) Config::joydef[n];

    MemESP::Reset(); // Reset Memory

    VIDEO::Reset();

    // Reinit disk controller
    if (Config::DiskCtrl || Z80Ops::isPentagon) {
        // Betadisk.ShutDown();
        // Betadisk.Init();
        Betadisk.EnterIdle();
    }

    Tape::tapeStatus = TAPE_STOPPED;
    Tape::tapePhase = TAPE_PHASE_STOPPED;
    Tape::romLoading = false;

    if (Z80Ops::is128 || Z80Ops::is2a3) { // Apply pulse length compensation for 128K
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

    } else if (Config::arch == "128K" || Config::arch == "+2A") {
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

    audioCOVOXDivider = audioAYDivider;
    covoxData[0] = covoxData[1] = covoxData[2] = covoxData[3] = 0;

    if (Config::load_monitor) AY_emu = false; // Disable AY emulation if tape load monitor mode is set

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

    aud_active_sources = (Config::Covox & 0x01) | (AY_emu << 1);

    CPU::reset();

    // Reconfigura carga de cinta real
    RealTape_realloc_buffers(ESPeccy::Audio_freq[1], ESPeccy::samplesPerFrame, CPU::statesInFrame);

}

//=======================================================================================
// KEYBOARD / KEMPSTON
//=======================================================================================
IRAM_ATTR bool ESPeccy::readKbd(fabgl::VirtualKeyItem *NextKey, uint8_t mode) {

    bool r = PS2Controller.keyboard()->getNextVirtualKey(NextKey);

    // Global keys
    if (NextKey->down) {

        if (ps2kbd) {

            // Start ZXUNOPS2
            if (Config::zxunops2 && !ZXKeyb::Exists) {
                fabgl::VirtualKeyItem akey;
                akey.vk = fabgl::VK_NONE;
                akey.CTRL = false;
                akey.SHIFT = false;

                switch(mode) {
                    case KBDREAD_MODENORMAL:
                        // Detect and process physical kbd menu key combinations
                        // CS+SS+<1..0> -> F1..F10 Keys, CS+SS+Q -> F11, CS+SS+W -> F12, CS+SS+S -> Capture screen
                        if (NextKey->SHIFT && NextKey->CTRL) {
                                 if (NextKey->vk == fabgl::VK_1)                               { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F1; }           // Main Menu
                            else if (NextKey->vk == fabgl::VK_2)                               { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F2; }           // Load State
                            else if (NextKey->vk == fabgl::VK_3)                               { akey.SHIFT = true;  akey.CTRL = false; akey.vk = fabgl::VK_F2; }           // Save State
                            else if (NextKey->vk == fabgl::VK_5)                               { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F5; }           // File Browser
                            else if (NextKey->vk == fabgl::VK_6)                               { akey.SHIFT = true;  akey.CTRL = false; akey.vk = fabgl::VK_F5; }           // Tape Browser
                            else if (NextKey->vk == fabgl::VK_7)                               { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F6; }           // Play/Stop
                            else if (NextKey->vk == fabgl::VK_8)                               { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F8; }           // CPU Stats
                            else if (NextKey->vk == fabgl::VK_9)                               { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F9; }           // Vol-
                            else if (NextKey->vk == fabgl::VK_0)                               { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F10; }          // Vol+
                            else if (NextKey->vk == fabgl::VK_Q || NextKey->vk == fabgl::VK_q) { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F11; }          // Hard Reset
                            else if (NextKey->vk == fabgl::VK_W || NextKey->vk == fabgl::VK_w) { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_F12; }          // ESP32 Reset
                            else if (NextKey->vk == fabgl::VK_P || NextKey->vk == fabgl::VK_p) { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_PAUSE; }        // P -> Pause
                            else if (NextKey->vk == fabgl::VK_I || NextKey->vk == fabgl::VK_i) { akey.SHIFT = true;  akey.CTRL = false; akey.vk = fabgl::VK_F1; }           // I -> INFO
                            else if (NextKey->vk == fabgl::VK_E || NextKey->vk == fabgl::VK_e) { akey.SHIFT = true;  akey.CTRL = false; akey.vk = fabgl::VK_F6; }           // E -> Eject tape
                            else if (NextKey->vk == fabgl::VK_R || NextKey->vk == fabgl::VK_r) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F11; }          // R -> Reset to TR-DOS
                            else if (NextKey->vk == fabgl::VK_T || NextKey->vk == fabgl::VK_t) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F2; }           // T -> Turbo
                            else if (NextKey->vk == fabgl::VK_B || NextKey->vk == fabgl::VK_b) { akey.SHIFT = true;  akey.CTRL = false; akey.vk = fabgl::VK_PRINTSCREEN; }  // B -> BMP capture
                            else if (NextKey->vk == fabgl::VK_O || NextKey->vk == fabgl::VK_o) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F9; }           // O -> Poke
                            else if (NextKey->vk == fabgl::VK_U || NextKey->vk == fabgl::VK_u) { akey.SHIFT = true;  akey.CTRL = false; akey.vk = fabgl::VK_F9; }           // U -> Cheats
                            else if (NextKey->vk == fabgl::VK_N || NextKey->vk == fabgl::VK_n) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F10; }          // N -> NMI
                            else if (NextKey->vk == fabgl::VK_K || NextKey->vk == fabgl::VK_k) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F1; }           // K -> Help / Kbd layout
                            else if (NextKey->vk == fabgl::VK_G || NextKey->vk == fabgl::VK_g) { akey.SHIFT = false; akey.CTRL = false; akey.vk = fabgl::VK_PRINTSCREEN; }  // G -> Capture SCR
                            else if (NextKey->vk == fabgl::VK_Z || NextKey->vk == fabgl::VK_z) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F5; }           // Z -> CenterH
                            else if (NextKey->vk == fabgl::VK_X || NextKey->vk == fabgl::VK_x) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F6; }           // X -> CenterH
                            else if (NextKey->vk == fabgl::VK_C || NextKey->vk == fabgl::VK_c) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F7; }           // C -> CenterV
                            else if (NextKey->vk == fabgl::VK_V || NextKey->vk == fabgl::VK_v) { akey.SHIFT = false; akey.CTRL = true;  akey.vk = fabgl::VK_F8; }           // V -> CenterV
                        }
                        break;

                    case KBDREAD_MODEFILEBROWSER:
                        if (NextKey->SHIFT && !NextKey->CTRL) { // CS + !SS
                                 if (NextKey->vk == fabgl::VK_RETURN)                               { akey.vk = fabgl::VK_JOY1C; } // CS + ENTER -> SPACE / SELECT
                            else if (NextKey->vk == fabgl::VK_SPACE)                                { akey.vk = fabgl::VK_ESCAPE; } // BREAK -> ESCAPE
                            else if (NextKey->vk == fabgl::VK_RIGHTPAREN)                           { akey.vk = fabgl::VK_BACKSPACE; } // CS + 0 -> BACKSPACE
                            else if (NextKey->vk == fabgl::VK_AMPERSAND)                            { akey.vk = fabgl::VK_UP; } // 7 -> VK_UP
                            else if (NextKey->vk == fabgl::VK_CARET)                                { akey.vk = fabgl::VK_DOWN; } // 6 -> VK_DOWN
                            else if (NextKey->vk == fabgl::VK_PERCENT)                              { akey.vk = fabgl::VK_LEFT; } // 5 -> VK_LEFT
                            else if (NextKey->vk == fabgl::VK_ASTERISK)                             { akey.vk = fabgl::VK_RIGHT; } // 8 -> VK_RIGHT
                            // invalid shift keys for filebrowser
                            else if (NextKey->vk == fabgl::VK_EXCLAIM || // 1
                                     NextKey->vk == fabgl::VK_AT || // 2
                                     NextKey->vk == fabgl::VK_HASH || // 3
                                     NextKey->vk == fabgl::VK_DOLLAR || // 4
                                     NextKey->vk == fabgl::VK_LEFTPAREN // 9
                                    )
                                    NextKey->vk = fabgl::VK_NONE;
                        }
                        else
                        if (NextKey->SHIFT && NextKey->CTRL) {
                                 if (NextKey->vk == fabgl::VK_7)                                    { akey.vk = fabgl::VK_PAGEUP; } // 7 -> VK_PAGEUP
                            else if (NextKey->vk == fabgl::VK_6)                                    { akey.vk = fabgl::VK_PAGEDOWN; } // 6 -> VK_PAGEDOWN
                            else if (NextKey->vk == fabgl::VK_5)                                    { akey.SHIFT = true; akey.vk = fabgl::VK_LEFT; } // 5 -> SS + VK_LEFT
                            else if (NextKey->vk == fabgl::VK_8)                                    { akey.SHIFT = true; akey.vk = fabgl::VK_RIGHT; } // 8 -> SS + VK_RIGHT
                            else if (NextKey->vk == fabgl::VK_G || NextKey->vk == fabgl::VK_g)      { akey.vk = fabgl::VK_PRINTSCREEN; } // G -> USE THIS SCR
                            else if (NextKey->vk == fabgl::VK_N || NextKey->vk == fabgl::VK_n)      { akey.vk = fabgl::VK_F2; } // N -> NUEVO / RENOMBRAR
                            else if (NextKey->vk == fabgl::VK_R || NextKey->vk == fabgl::VK_r)      { akey.SHIFT = true; akey.vk = fabgl::VK_F2; } // R -> NUEVO Con ROM
                            else if (NextKey->vk == fabgl::VK_M || NextKey->vk == fabgl::VK_m)      { akey.vk = fabgl::VK_F6; } // M -> MOVE / MOVER
                            else if (NextKey->vk == fabgl::VK_D || NextKey->vk == fabgl::VK_d)      { akey.vk = fabgl::VK_F8; } // D -> DELETE / BORRAR
                            else if (NextKey->vk == fabgl::VK_F || NextKey->vk == fabgl::VK_f)      { akey.vk = fabgl::VK_F3; } // F -> FIND / BUSQUEDA
                            else if (NextKey->vk == fabgl::VK_Z || NextKey->vk == fabgl::VK_z)      { akey.CTRL = true; akey.vk = fabgl::VK_LEFT; } // Z -> CTRL + LEFT
                            else if (NextKey->vk == fabgl::VK_X || NextKey->vk == fabgl::VK_x)      { akey.CTRL = true; akey.vk = fabgl::VK_RIGHT; } // X -> CTRL + RIGHT
                            else if (NextKey->vk == fabgl::VK_C || NextKey->vk == fabgl::VK_c)      { akey.CTRL = true; akey.vk = fabgl::VK_UP; } // C -> CTRL + UP
                            else if (NextKey->vk == fabgl::VK_V || NextKey->vk == fabgl::VK_v)      { akey.CTRL = true; akey.vk = fabgl::VK_DOWN; } // V -> CTRL + DOWN
                        }
                        break;

                    case KBDREAD_MODEINPUT:
                        //akey.CTRL = NextKey->CTRL;
                        //akey.SHIFT = NextKey->SHIFT;

                        if (NextKey->SHIFT && !NextKey->CTRL) { // CS
                                 if (NextKey->vk == fabgl::VK_AMPERSAND)                { akey.vk = fabgl::VK_END; } // 7 -> END
                            else if (NextKey->vk == fabgl::VK_CARET)                    { akey.vk = fabgl::VK_HOME; } // 6 -> HOME
                            else if (NextKey->vk == fabgl::VK_PERCENT)                  { akey.vk = fabgl::VK_LEFT; } // 5 -> LEFT
                            else if (NextKey->vk == fabgl::VK_ASTERISK)                 { akey.vk = fabgl::VK_RIGHT; } // 8 -> RIGHT
                            else if (NextKey->vk == fabgl::VK_RIGHTPAREN)               { akey.vk = fabgl::VK_BACKSPACE; } // CS + 0 -> BACKSPACE
                            else if (NextKey->vk == fabgl::VK_SPACE)                    { akey.vk = fabgl::VK_ESCAPE; } // CS + SPACE && !SS -> ESCAPE
                            // invalid shift keys for input
                            else if (NextKey->vk == fabgl::VK_EXCLAIM ||
                                     NextKey->vk == fabgl::VK_AT ||
                                     NextKey->vk == fabgl::VK_HASH ||
                                     NextKey->vk == fabgl::VK_DOLLAR ||
                                     NextKey->vk == fabgl::VK_LEFTPAREN
                                    )
                                    NextKey->vk = fabgl::VK_NONE;
                        }
                        break;

                    case KBDREAD_MODEINPUTMULTI:
                        if (NextKey->SHIFT && !NextKey->CTRL) { // CS
                                 if (NextKey->vk == fabgl::VK_AMPERSAND)                { akey.vk = fabgl::VK_UP; } // 7 -> VK_UP
                            else if (NextKey->vk == fabgl::VK_CARET)                    { akey.vk = fabgl::VK_DOWN; } // 6 -> VK_DOWN
                            else if (NextKey->vk == fabgl::VK_PERCENT)                  { akey.vk = fabgl::VK_LEFT; } // 5 -> LEFT
                            else if (NextKey->vk == fabgl::VK_ASTERISK)                 { akey.vk = fabgl::VK_RIGHT; } // 8 -> RIGHT
                            else if (NextKey->vk == fabgl::VK_RIGHTPAREN)               { akey.vk = fabgl::VK_BACKSPACE; } // CS + 0 -> BACKSPACE
                            else if (NextKey->vk == fabgl::VK_SPACE)                    { akey.vk = fabgl::VK_ESCAPE; } // CS + SPACE && !SS -> ESCAPE
                            // invalid shift keys for input
                            else if (NextKey->vk == fabgl::VK_EXCLAIM ||
                                     NextKey->vk == fabgl::VK_AT ||
                                     NextKey->vk == fabgl::VK_HASH ||
                                     NextKey->vk == fabgl::VK_DOLLAR ||
                                     NextKey->vk == fabgl::VK_LEFTPAREN
                                    )
                                    NextKey->vk = fabgl::VK_NONE;
                        }
                        break;

                    case KBDREAD_MODEKBDLAYOUT:
                             if (NextKey->vk == fabgl::VK_SPACE)                            { akey.vk = fabgl::VK_ESCAPE; } // SPACE -> ESCAPE
                        else if (NextKey->vk == fabgl::VK_RETURN)                           { akey.vk = fabgl::VK_ESCAPE; } // RETURN -> ESCAPE
                        else if (NextKey->SHIFT && NextKey->CTRL) { // CS + SS
                            if (NextKey->vk == fabgl::VK_K || NextKey->vk == fabgl::VK_k)   { akey.vk = fabgl::VK_ESCAPE; }
                        }
                        else if (NextKey->SHIFT && !NextKey->CTRL) { // CS
                                 if (NextKey->vk == fabgl::VK_PERCENT)                      { akey.vk = fabgl::VK_LEFT;   } // 5 -> LEFT
                            else if (NextKey->vk == fabgl::VK_ASTERISK)                     { akey.vk = fabgl::VK_RIGHT;  } // 8 -> RIGHT
                        }
                        break;

                    case KBDREAD_MODEDIALOG:
                        if (NextKey->SHIFT && !NextKey->CTRL) { // CS
                                 if (NextKey->vk == fabgl::VK_PERCENT)                  { akey.vk = fabgl::VK_LEFT; } // 5 -> LEFT
                            else if (NextKey->vk == fabgl::VK_ASTERISK)                 { akey.vk = fabgl::VK_RIGHT; } // 8 -> RIGHT
                            else if (NextKey->vk == fabgl::VK_SPACE)                    { akey.vk = fabgl::VK_ESCAPE; } // CS + SPACE && !SS -> ESCAPE
                        }
                        else if (NextKey->SHIFT && NextKey->CTRL) { // CS + SS
                            if (NextKey->vk == fabgl::VK_I || NextKey->vk == fabgl::VK_i)   { akey.vk = fabgl::VK_F1; } // For exit from HWInfo
                        }
                        break;

                }

                if (akey.vk != fabgl::VK_NONE) {
                    NextKey->vk = akey.vk;
                    NextKey->CTRL = akey.CTRL;
                    NextKey->SHIFT = akey.SHIFT;
                }

            }
            // End ZXUNOPS2

            if (mode == KBDREAD_MODEBIOS) {
                if (NextKey->SHIFT && !NextKey->CTRL) { // CS
                         if (NextKey->vk == fabgl::VK_AMPERSAND)                { NextKey->SHIFT = false; NextKey->vk = fabgl::VK_UP; } // 7 -> VK_UP
                    else if (NextKey->vk == fabgl::VK_CARET)                    { NextKey->SHIFT = false; NextKey->vk = fabgl::VK_DOWN; } // 6 -> VK_DOWN
                    else if (NextKey->vk == fabgl::VK_PERCENT)                  { NextKey->SHIFT = false; NextKey->vk = fabgl::VK_LEFT; } // 5 -> LEFT
                    else if (NextKey->vk == fabgl::VK_ASTERISK)                 { NextKey->SHIFT = false; NextKey->vk = fabgl::VK_RIGHT; } // 8 -> RIGHT
                    else if (NextKey->vk == fabgl::VK_RIGHTPAREN)               { NextKey->SHIFT = false; NextKey->vk = fabgl::VK_BACKSPACE; } // CS + 0 -> BACKSPACE
                    else if (NextKey->vk == fabgl::VK_SPACE)                    { NextKey->SHIFT = false; NextKey->vk = fabgl::VK_ESCAPE; } // CS + SPACE && !SS -> ESCAPE
                    // invalid shift keys for input
                    else if (NextKey->vk == fabgl::VK_EXCLAIM ||
                             NextKey->vk == fabgl::VK_AT ||
                             NextKey->vk == fabgl::VK_HASH ||
                             NextKey->vk == fabgl::VK_DOLLAR ||
                             NextKey->vk == fabgl::VK_LEFTPAREN
                            )
                            NextKey->vk = fabgl::VK_NONE;
                }
                else
                if (!NextKey->SHIFT && NextKey->CTRL) { // SS
                    if (NextKey->vk == fabgl::VK_S || NextKey->vk == fabgl::VK_s) { NextKey->CTRL = false; NextKey->vk = fabgl::VK_F10; }
                }
            }
        }

        if (NextKey->SHIFT && !NextKey->CTRL && NextKey->vk == fabgl::VK_PRINTSCREEN) { // Capture framebuffer to BMP file in SD Card (thx @dcrespo3d!)
            CaptureToBmp();
            r = false;
        }

        if (NextKey->vk == fabgl::VK_SCROLLLOCK) { // Change CursorAsJoy setting
            Config::CursorAsJoy = !Config::CursorAsJoy;
            if(ps2kbd)
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
IRAM_ATTR void ESPeccy::readKbdJoy() {

    if (ps2kbd2) {

        fabgl::VirtualKeyItem NextKey;
        auto KbdJoy = PS2Controller.keybjoystick();

        while (KbdJoy->virtualKeyAvailable()) {
            PS2Controller.keybjoystick()->getNextVirtualKey(&NextKey);
            ESPeccy::PS2Controller.keyboard()->injectVirtualKey(NextKey.vk, NextKey.down, false);
        }

    }

}

fabgl::VirtualKey ESPeccy::JoyVKTranslation[24];
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

fabgl::VirtualKey ESPeccy::VK_ESPECCY_FIRE1 = fabgl::VK_NONE;
fabgl::VirtualKey ESPeccy::VK_ESPECCY_FIRE2 = fabgl::VK_NONE;
fabgl::VirtualKey ESPeccy::VK_ESPECCY_TAB = fabgl::VK_TAB;
fabgl::VirtualKey ESPeccy::VK_ESPECCY_GRAVEACCENT = fabgl::VK_GRAVEACCENT;

int ESPeccy::zxDelay = 0;

int32_t ESPeccy::mouseX = 0;
int32_t ESPeccy::mouseY = 0;
bool ESPeccy::mouseButtonL = 0;
bool ESPeccy::mouseButtonR = 0;

static uint8_t PS2cols[8] = { 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf };

IRAM_ATTR void ESPeccy::processKeyboard() {

    bool r = false;

    ESPeccy::sync_realtape = false;

    readKbdJoy();

    if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead();
    else
    if (!booting && !gpio_get_level((gpio_num_t)GPIO_NUM_36)) { // Se deshabilita durante el booteo por cualquier posible futuro conflicto

        bool shift_down = false, ctrl_down = false;

        fabgl::VirtualKey injectKey = fabgl::VK_NONE;

        switch(Config::io36button) {
            case BTN_ASSIGN_RESET:
                injectKey = fabgl::VK_F11;
                break;

            case BTN_ASSIGN_NMI:
                ctrl_down = true;
                injectKey = fabgl::VK_F10;
                break;

            case BTN_ASSIGN_CHEATS:
                shift_down = true;
                injectKey = fabgl::VK_F9;
                break;

            case BTN_ASSIGN_POKE:
                ctrl_down = true;
                injectKey = fabgl::VK_F9;
                break;

            case BTN_ASSIGN_STATS:
                injectKey = fabgl::VK_F8;
                break;

            case BTN_ASSIGN_MENU:
                injectKey = fabgl::VK_F1;
                break;

        }

        if (injectKey != fabgl::VK_NONE) {
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
        }
    }

    auto Kbd = PS2Controller.keyboard();
    fabgl::VirtualKeyItem NextKey;
    fabgl::VirtualKey KeytoESP;
    bool j[10] = { true, true, true, true, true, true, true, true, true, true };
    bool jShift = true;

    while (Kbd->virtualKeyAvailable()) {

        r = readKbd(&NextKey);

        if (r) {

            if (!NextKey.SHIFT && !NextKey.CTRL && NextKey.vk == fabgl::VK_PRINTSCREEN) {
                if (Tape::tapeFullPathName=="none") {
                    OSD::osdCenteredMsg(OSD_TAPE_SELECT_ERR[Config::lang], LEVEL_WARN);
                } else {
                    OSD::saveSCR(Tape::tapeFullPathName, (uint32_t *)(MemESP::videoLatch ? MemESP::ram[7] : MemESP::ram[5]));
                }
                continue;
            }

            KeytoESP = NextKey.vk;

            if (KeytoESP >= fabgl::VK_JOY1LEFT && KeytoESP <= fabgl::VK_JOY2Z) {
                // printf("KeytoESP: %d\n",KeytoESP);
                ESPeccy::PS2Controller.keyboard()->injectVirtualKey(JoyVKTranslation[KeytoESP - 248], NextKey.down, false);
                continue;
            }

            if (NextKey.down && ((KeytoESP >= fabgl::VK_F1 && KeytoESP <= fabgl::VK_F12) || KeytoESP == fabgl::VK_PAUSE || KeytoESP == fabgl::VK_VOLUMEUP || KeytoESP == fabgl::VK_VOLUMEDOWN || KeytoESP == fabgl::VK_VOLUMEMUTE)) {

                int64_t osd_start = esp_timer_get_time();

                OSD::do_OSD(KeytoESP, NextKey.CTRL, NextKey.SHIFT);

                // sync real tape is needed
                if (ESPeccy::sync_realtape && RealTape_enabled) {
                    ESPeccy::sync_realtape = false;
                    RealTape_pause();
                    RealTape_start();
                }

                Kbd->emptyVirtualKeyQueue();

                // Set all zx keys as not pressed
                for (uint8_t i = 0; i < 8; i++) ZXKeyb::ZXcols[i] = 0xbf;
                zxDelay = 15;

                // totalseconds = 0;
                // totalsecondsnodelay = 0;
                // VIDEO::framecnt = 0;

                // Refresh border
                VIDEO::brdnextframe = true;

                ESPeccy::ts_start += esp_timer_get_time() - osd_start;

                return;

            }

            // Reset keys
            if (NextKey.down && NextKey.LALT) {
                if (NextKey.CTRL) {
                    if (KeytoESP == fabgl::VK_DELETE) {
                        // printf("Ctrl + Alt + Supr!\n");
                        // ESP host reset
                        Config::rom_file = NO_ROM_FILE;
                        Config::save("rom");

                        Config::ram_file = NO_RAM_FILE;
                        Config::save("ram");
                        OSD::esp_hard_reset();
                    } else if (KeytoESP == fabgl::VK_BACKSPACE) {
                        // printf("Ctrl + Alt + backSpace!\n");
                        // Hard
                        Config::ram_file = NO_RAM_FILE;
                        Config::last_ram_file = NO_RAM_FILE;

                        if (Config::last_rom_file != NO_ROM_FILE) {
                            if ( FileUtils::isSDReady() ) ROMLoad::load(Config::last_rom_file);
                            Config::rom_file = Config::last_rom_file;
                        } else
                            ESPeccy::reset();
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
                        if (Config::last_rom_file != NO_ROM_FILE) {
                            if ( FileUtils::isSDReady() ) ROMLoad::load(Config::last_rom_file);
                            Config::rom_file = Config::last_rom_file;
                        } else
                            ESPeccy::reset();
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

            jShift = !NextKey.SHIFT;

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

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECCY_FIRE1)) {
                    bitWrite(Ports::port[0x1f], 4, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_SLASH) || /*Kbd->isVKDown(fabgl::VK_QUESTION) ||*/Kbd->isVKDown(fabgl::VK_RGUI) || Kbd->isVKDown(fabgl::VK_APPLICATION) || Kbd->isVKDown(VK_ESPECCY_FIRE2)) {
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

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECCY_FIRE1)) {
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

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECCY_FIRE1)) {
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

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECCY_FIRE1)) {
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

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECCY_FIRE1)) {
                    jShift = true;
                    j[5] = false;
                };

            }

            // Check keyboard status and map it to Spectrum Ports

            bitWrite(PS2cols[0], 0, (jShift)
                & (!Kbd->isVKDown(fabgl::VK_BACKSPACE))
                & (!Kbd->isVKDown(fabgl::VK_CAPSLOCK)) // Caps lock
                &   (!Kbd->isVKDown(VK_ESPECCY_GRAVEACCENT)) // Edit
                &   (!Kbd->isVKDown(VK_ESPECCY_TAB)) // Extended mode
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
                                &   (!Kbd->isVKDown(VK_ESPECCY_GRAVEACCENT)) // Edit
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
            bitWrite(PS2cols[7], 1, (!NextKey.CTRL)
                                &   (!Kbd->isVKDown(fabgl::VK_COMMA)) // Comma
                                &   (!Kbd->isVKDown(fabgl::VK_PERIOD)) // Period
                                &   (!Kbd->isVKDown(fabgl::VK_SEMICOLON)) // Semicolon
                                &   (!Kbd->isVKDown(fabgl::VK_QUOTE)) // Double quote
                                &   (!Kbd->isVKDown(VK_ESPECCY_TAB)) // Extended mode
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
        if (Config::realtape_gpio_num == KM_COL_0) {
            if (ZXKBD_SS) { // SS
                if (ZXKBD_CS) { // CS
                    if (ZXKBD_J) { // j
                        // enabled REAL TAPE LOAD mode
                        OSD::osdCenteredMsg("REAL TAPE LOAD ENABLED", LEVEL_INFO, 1000);
                        RealTape_start();
                    }
                } else {
                    if (RealTape_enabled && ZXKBD_S) { // s
                        // disable REAL TAPE LOAD mode (STOP LOADING)
                        OSD::osdCenteredMsg("REAL TAPE LOAD DISABLED", LEVEL_INFO, 1000);
                        RealTape_pause();
                    }
                }
            }
        }
    }

    if (r) {
        for (uint8_t rowidx = 0; rowidx < 8; rowidx++) {
            Ports::port[rowidx] = PS2cols[rowidx];
        }
    }
}


uint8_t ESPeccy::TurboModeSet(int8_t esp_delay) {

    uint8_t current_esp_delay = ESPeccy::ESP_delay;

    if (esp_delay != -1) ESPeccy::ESP_delay = (uint8_t) esp_delay;

    if (ESPeccy::ESP_delay) {

        // Empty audio buffers
        for (int i=0;i<ESP_AUDIO_SAMPLES_PENTAGON;i++) {
            ESPeccy::overSamplebuf[i]=0;
            ESPeccy::audioBuffer[i]=0;
            AySound::SamplebufAY[i]=0;
        }
        ESPeccy::lastaudioBit=0;

        ESPeccy::ESPoffset = 0;

        // printf("Resetting pwmaudio to freq: %d\n",Audio_freq);
        esp_err_t res;
        res = pwm_audio_set_sample_rate(ESPeccy::Audio_freq[ESPeccy::ESP_delay]);
        if (res != ESP_OK) {
            printf("Can't set sample rate\n");
        }

        // Reset AY emulation
        //AySound::init();
        AySound::set_sound_format(ESPeccy::Audio_freq[ESPeccy::ESP_delay],1,8);
        //AySound::set_stereo(AYEMU_MONO,NULL);
        //AySound::reset();
        AySound::prepare_generation();

    }

    return current_esp_delay;

}

//=======================================================================================
// AUDIO
//=======================================================================================

IRAM_ATTR void ESPeccy::audioTask(void *unused) {

    size_t written;

    // // MCP23017 Init
    // mcp23017_test_init();
    // printf("Init done!\n");

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

        xQueueReceive(audioTaskQueue, &param, portMAX_DELAY);

        if (ESP_delay) pwm_audio_write(audioBuffer, samplesPerFrame, &written, 5 / portTICK_PERIOD_MS);

        // task2start = esp_timer_get_time();

        // Process ZX Keyboard
        if (ZXKeyb::Exists) {

            if (zxDelay > 0)

                zxDelay--;

            else {

                // Process physical keyboard
                ZXKeyb::process();

                // Combine both keyboards
                for (uint8_t rowidx = 0; rowidx < 8; rowidx++) {
                    Ports::port[rowidx] = PS2cols[rowidx] & ZXKeyb::ZXcols[rowidx];
                }

            }

        }

        // task2elapsed = esp_timer_get_time() - task2start;

        xQueueReceive(audioTaskQueue, &param, portMAX_DELAY);

        // task2start = esp_timer_get_time();

        if (ESP_delay) {

            // Finish fill of beeper oversampled audio buffers
            if (faudbufcntover < samplesPerFrame)
                overSamplebuf[faudbufcntover++] = faudioBitBuf + (flastaudioBit * (audioSampleDivider - faudioBitbufCount));
            faudioBitBuf = flastaudioBit * audioSampleDivider;
            for(;faudbufcntover < samplesPerFrame;)
                overSamplebuf[faudbufcntover++] = faudioBitBuf;

            switch (aud_active_sources) {
            case 0:
                // Beeper only
                for (int i = 0; i < samplesPerFrame; i++) {
                    int beeper = overSamplebuf[i] / audioSampleDivider;
                    audioBuffer[i] = beeper > 255 ? 255 : beeper;
                }
                break;
            case 1:
                // Beeper + Covox
                for (;faudbufcntCOVOX < samplesPerFrame;)
                    SamplebufCOVOX[faudbufcntCOVOX++] = fcovoxMix;

                for (int i = 0; i < samplesPerFrame; i++) {
                    int beeper = (overSamplebuf[i] / audioSampleDivider) + SamplebufCOVOX[i];
                    audioBuffer[i] = beeper > 255 ? 255 : beeper; // Clamp
                }

                break;
            case 2:
                // Beeper + AY
                if (faudbufcntAY < samplesPerFrame)
                    AySound::gen_sound(samplesPerFrame - faudbufcntAY , faudbufcntAY);

                for (int i = 0; i < samplesPerFrame; i++) {
                    int beeper = (overSamplebuf[i] / audioSampleDivider) + AySound::SamplebufAY[i];
                    audioBuffer[i] = beeper > 255 ? 255 : beeper; // Clamp
                }

                break;
            case 3:
                // Beeper + AY + Covox
                for (;faudbufcntCOVOX < samplesPerFrame;)
                    SamplebufCOVOX[faudbufcntCOVOX++] = fcovoxMix;

                if (faudbufcntAY < samplesPerFrame)
                    AySound::gen_sound(samplesPerFrame - faudbufcntAY , faudbufcntAY);

                for (int i = 0; i < samplesPerFrame; i++) {
                    int beeper = (overSamplebuf[i] / audioSampleDivider) + AySound::SamplebufAY[i] + SamplebufCOVOX[i];
                    audioBuffer[i] = beeper > 255 ? 255 : beeper; // Clamp
                }

                break;
            }

            // if (Config::Covox) {
            //     for (;faudbufcntCOVOX < samplesPerFrame;)
            //         SamplebufCOVOX[faudbufcntCOVOX++] = fcovoxMix;
            // }

            // if (AY_emu) {

            //     if (faudbufcntAY < samplesPerFrame)
            //         AySound::gen_sound(samplesPerFrame - faudbufcntAY , faudbufcntAY);

            //     for (int i = 0; i < samplesPerFrame; i++) {
            //         // uint16_t beeper = (overSamplebuf[i] / audioSampleDivider) + AySound::SamplebufAY[i] + SamplebufCOVOX[i];
            //         // audioBuffer[i] = beeper & 0x00ff; // Clamp
            //         int beeper = (overSamplebuf[i] / audioSampleDivider) + AySound::SamplebufAY[i] + SamplebufCOVOX[i];
            //         audioBuffer[i] = beeper > 255 ? 255 : beeper; // Clamp
            //     }

            // } else

            //     for (int i = 0; i < samplesPerFrame; i++) {
            //         // uint16_t beeper = (overSamplebuf[i] / audioSampleDivider) + SamplebufCOVOX[i];
            //         // audioBuffer[i] = beeper & 0x00ff; // Clamp
            //         int beeper = (overSamplebuf[i] / audioSampleDivider) + SamplebufCOVOX[i];
            //         audioBuffer[i] = beeper > 255 ? 255 : beeper; // Clamp
            //     }

        }

        // task2elapsed = esp_timer_get_time() - task2start;

    }
}

//=======================================================================================
// Beeper audiobuffer generation (oversample)
//=======================================================================================
IRAM_ATTR void ESPeccy::BeeperGetSample() {
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

//=======================================================================================
// AY audiobuffer generation (oversample)
//=======================================================================================
IRAM_ATTR void ESPeccy::AYGetSample() {
    uint32_t audbufpos = CPU::tstates / audioAYDivider;
    if (audbufpos > audbufcntAY) {
        AySound::gen_sound(audbufpos - audbufcntAY, audbufcntAY);
        audbufcntAY = audbufpos;
    }
}

//=======================================================================================
// COVOX audiobuffer generation
//=======================================================================================
IRAM_ATTR void ESPeccy::COVOXGetSample() {
    uint32_t audbufpos = CPU::tstates / audioCOVOXDivider;
    if (audbufpos > audbufcntCOVOX) {
        covoxMix = (covoxData[0] + covoxData[1] + covoxData[2] + covoxData[3]) >> 2;
        for (int i=audbufcntCOVOX;i<audbufpos;i++)
            SamplebufCOVOX[i] = covoxMix;
        audbufcntCOVOX = audbufpos;
    }
}

//=======================================================================================
// MAIN LOOP
//=======================================================================================

IRAM_ATTR void ESPeccy::loop() {

    for(;;) {

        ts_start = esp_timer_get_time();

        // Send audioBuffer to pwmaudio

        xQueueSend(audioTaskQueue, &param, portMAX_DELAY);

        RealTape_prepare_frame();

        Tape::ManageLoading();

        CPU::loop();

        // Copy audio buffer vars for finish buffer code
        faudbufcntover = audbufcntover;
        faudioBitBuf = audioBitBuf;
        faudioBitbufCount = audioBitbufCount;
        faudbufcntAY = audbufcntAY;
        faudbufcntCOVOX = audbufcntCOVOX;
        flastaudioBit = lastaudioBit;
        fcovoxMix = covoxMix;

        xQueueSend(audioTaskQueue, &param, portMAX_DELAY);

        processKeyboard();

        // Update stats every 50 frames
        if (VIDEO::OSD && VIDEO::framecnt >= 50) {

            if (VIDEO::OSD & 0x04) {

                // printf("Vol. OSD out -> Framecnt: %d\n", VIDEO::framecnt);

                if (VIDEO::framecnt >= 100) {

                    // Save selected volume if not in tape load monitor mode
                    if (!Config::load_monitor) {
                        Config::volume = aud_volume;
                        Config::save("volume");
                    }

                    VIDEO::OSD &= 0xfb;

                    if (VIDEO::OSD == 0) {

                        if (Config::aspect_16_9)
                            VIDEO::Draw_OSD169 = Z80Ops::is2a3 ? VIDEO::MainScreen_2A3 : VIDEO::MainScreen;
                        else
                            VIDEO::Draw_OSD43 = Z80Ops::isPentagon ? VIDEO::BottomBorder_Pentagon :  VIDEO::BottomBorder;

                        VIDEO::brdnextframe = true;

                    }

                }

            }

            if ((VIDEO::OSD & 0x04) == 0) {

                if (VIDEO::OSD == 1 && Tape::tapeStatus==TAPE_LOADING) {

                    snprintf(OSD::stats_lin1, sizeof(OSD::stats_lin1), " %-12s %04d/%04d ", Tape::tapeFileName.substr(0 + ESPeccy::TapeNameScroller, 12).c_str(), Tape::tapeCurBlock + 1, Tape::tapeNumBlocks);

                    float percent = (float)((Tape::tapebufByteCount + Tape::tapePlayOffset) * 100) / (float)Tape::tapeFileSize;
                    snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2), " %05.2f%% %07d%s%07d ", percent, Tape::tapebufByteCount + Tape::tapePlayOffset, "/" , Tape::tapeFileSize);

                    if ((++ESPeccy::TapeNameScroller + 12) > Tape::tapeFileName.length()) ESPeccy::TapeNameScroller = 0;

                    OSD::drawStats();

                } else if (VIDEO::OSD == 2) {

                    snprintf(OSD::stats_lin1, sizeof(OSD::stats_lin1), "CPU: %05d / IDL: %05d ", (int)(ESPeccy::elapsed), (int)(ESPeccy::idle));
                    snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2), "FPS:%6.2f / FND:%6.2f ", VIDEO::framecnt / (ESPeccy::totalseconds / 1000000), VIDEO::framecnt / (ESPeccy::totalsecondsnodelay / 1000000));

                    OSD::drawStats();

                }

                totalseconds = 0;
                totalsecondsnodelay = 0;
                VIDEO::framecnt = 0;

            }

        }

        // Flashing flag change
        if (!(VIDEO::flash_ctr++ & 0x0f)) VIDEO::flashing ^= 0x80;

        #ifdef TIME_MACHINE_ENABLED
        // Time machine
        if (Config::TimeMachine) MemESP::Tm_DoTimeMachine();
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

        if(Config::videomode && !Config::load_monitor && ESP_delay == 1) {

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
