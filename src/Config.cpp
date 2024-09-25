/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
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

To Contact the dev team you can write to zxespectrum@gmail.com or
visit https://zxespectrum.speccy.org/contacto

*/

#include <stdio.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <cctype>
#include <algorithm>
#include <locale>
#include "hardconfig.h"
#include "Config.h"
#include "FileUtils.h"
#include "messages.h"
#include "ESPectrum.h"
#include "pwm_audio.h"
#include "roms.h"
#include "OSDMain.h"

string   Config::arch = "48K";
string   Config::romSet = "48K";
string   Config::romSet48 = "48K";
string   Config::romSet128 = "128K";
string   Config::romSetTK90X = "v1es";
string   Config::romSetTK95 = "95es";
string   Config::pref_arch = "48K";
string   Config::pref_romSet_48 = "48K";
string   Config::pref_romSet_128 = "128K";
string   Config::pref_romSet_TK90X = "v1es";
string   Config::pref_romSet_TK95 = "95es";
string   Config::ram_file = NO_RAM_FILE;
string   Config::last_ram_file = NO_RAM_FILE;

bool     Config::slog_on = true;
bool     Config::aspect_16_9 = false;
uint8_t  Config::videomode = 0; // 0 -> SAFE VGA, 1 -> 50HZ VGA, 2 -> 50HZ CRT
uint8_t  Config::esp32rev = 0;
uint8_t  Config::lang = 0;
uint8_t  Config::osd_LRNav = 0;
uint8_t  Config::osd_AltRot = 0;
bool     Config::AY48 = true;
bool     Config::Issue2 = true;
bool     Config::flashload = true;
bool     Config::tape_player = false; // Tape player mode
bool     Config::tape_timing_rg = false; // Rodolfo Guerra ROMs tape timings

uint8_t  Config::joystick1 = JOY_SINCLAIR1;
uint8_t  Config::joystick2 = JOY_SINCLAIR2;
uint16_t Config::joydef[24] = {
    fabgl::VK_6,
    fabgl::VK_7,
    fabgl::VK_9,
    fabgl::VK_8,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_0,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_1,
    fabgl::VK_2,
    fabgl::VK_4,
    fabgl::VK_3,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_5,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE,
    fabgl::VK_NONE
};

uint8_t  Config::joyPS2 = JOY_KEMPSTON;
uint8_t  Config::AluTiming = 0;
uint8_t  Config::ps2_dev2 = 0; // Second port PS/2 device: 0 -> None, 1 -> PS/2 keyboard, 2 -> PS/2 Mouse (TO DO)
bool     Config::CursorAsJoy = false;
int8_t   Config::CenterH = 0;
int8_t   Config::CenterV = 0;

string   Config::SNA_Path = "/";
string   Config::TAP_Path = "/";
string   Config::DSK_Path = "/";

uint16_t Config::SNA_begin_row = 1;
uint16_t Config::SNA_focus = 1;
uint8_t  Config::SNA_fdMode = 0;
string   Config::SNA_fileSearch = "";

uint16_t Config::TAP_begin_row = 1;
uint16_t Config::TAP_focus = 1;
uint8_t  Config::TAP_fdMode = 0;
string   Config::TAP_fileSearch = "";

uint16_t Config::DSK_begin_row = 1;
uint16_t Config::DSK_focus = 1;
uint8_t  Config::DSK_fdMode = 0;
string   Config::DSK_fileSearch = "";

uint8_t Config::scanlines = 0;
uint8_t Config::render = 0;

bool Config::TABasfire1 = false;

bool Config::StartMsg = true;

uint8_t Config::port254default = 0xbf; // For TK90X v1 ROM -> 0xbf: Spanish, 0x3f: Portuguese

uint8_t Config::ALUTK = 1; // TK ALU -> 0 -> Ferranti, 1 -> Microdigital 50hz, 2 -> Microdigital 60hz
uint8_t Config::DiskCtrl = 1; // 0 -> None, 1 -> Betadisk

bool Config::TimeMachine = false;

int8_t Config::volume = ESP_VOLUME_DEFAULT;

// erase control characters (in place)
static inline void erase_cntrl(std::string &s) {
    s.erase(std::remove_if(s.begin(), s.end(),
            [&](char ch)
            { return std::iscntrl(static_cast<unsigned char>(ch));}),
            s.end());
}

enum ConfigType {
    CONFIG_TYPE_STRING,
    CONFIG_TYPE_BOOL,
    CONFIG_TYPE_UINT8,
    CONFIG_TYPE_UINT16,
    CONFIG_TYPE_INT8
};

struct ConfigEntry {
    const char* key;
    ConfigType type;
    void* value;  // Pointer to the configuration value
};

// Define the array of configuration entries
ConfigEntry configEntries[] = {
    {"arch", CONFIG_TYPE_STRING, &Config::arch},
    {"romSet", CONFIG_TYPE_STRING, &Config::romSet},
    {"romSet48", CONFIG_TYPE_STRING, &Config::romSet48},
    {"romSet128", CONFIG_TYPE_STRING, &Config::romSet128},
    {"romSetTK90X", CONFIG_TYPE_STRING, &Config::romSetTK90X},
    {"romSetTK95", CONFIG_TYPE_STRING, &Config::romSetTK95},
    {"pref_arch", CONFIG_TYPE_STRING, &Config::pref_arch},
    {"pref_romSet_48", CONFIG_TYPE_STRING, &Config::pref_romSet_48},
    {"pref_romSet_128", CONFIG_TYPE_STRING, &Config::pref_romSet_128},
    {"pref_romSet_90X", CONFIG_TYPE_STRING, &Config::pref_romSet_TK90X},
    {"pref_romSet_95", CONFIG_TYPE_STRING, &Config::pref_romSet_TK95},
    {"ram", CONFIG_TYPE_STRING, &Config::ram_file},
    {"slog", CONFIG_TYPE_BOOL, &Config::slog_on},
//    {"sdstorage", CONFIG_TYPE_STRING, &Config::sd_storage},
    {"asp169", CONFIG_TYPE_BOOL, &Config::aspect_16_9},
    {"videomode", CONFIG_TYPE_UINT8, &Config::videomode},
    {"language", CONFIG_TYPE_UINT8, &Config::lang},
    {"AY48", CONFIG_TYPE_BOOL, &Config::AY48},
    {"Issue2", CONFIG_TYPE_BOOL, &Config::Issue2},
    {"flashload", CONFIG_TYPE_BOOL, &Config::flashload},
    {"tape_player", CONFIG_TYPE_BOOL, &Config::tape_player},
    {"tape_timing_rg", CONFIG_TYPE_BOOL, &Config::tape_timing_rg},
    {"joystick1", CONFIG_TYPE_UINT8, &Config::joystick1},
    {"joystick2", CONFIG_TYPE_UINT8, &Config::joystick2},
    {"joyPS2", CONFIG_TYPE_UINT8, &Config::joyPS2},
    {"AluTiming", CONFIG_TYPE_UINT8, &Config::AluTiming},
    {"PS2Dev2", CONFIG_TYPE_UINT8, &Config::ps2_dev2},
    {"CursorAsJoy", CONFIG_TYPE_BOOL, &Config::CursorAsJoy},
    {"CenterH", CONFIG_TYPE_INT8, &Config::CenterH},
    {"CenterV", CONFIG_TYPE_INT8, &Config::CenterV},
    {"SNA_Path", CONFIG_TYPE_STRING, &Config::SNA_Path},
    {"TAP_Path", CONFIG_TYPE_STRING, &Config::TAP_Path},
    {"DSK_Path", CONFIG_TYPE_STRING, &Config::DSK_Path},
    {"volume", CONFIG_TYPE_UINT8, &Config::volume},
    {"scanlines", CONFIG_TYPE_UINT8, &Config::scanlines},
    {"render", CONFIG_TYPE_UINT8, &Config::render},

    // Joystick definitions
    {"joydef00", CONFIG_TYPE_UINT16, &Config::joydef[0]},
    {"joydef01", CONFIG_TYPE_UINT16, &Config::joydef[1]},
    {"joydef02", CONFIG_TYPE_UINT16, &Config::joydef[2]},
    {"joydef03", CONFIG_TYPE_UINT16, &Config::joydef[3]},
    {"joydef04", CONFIG_TYPE_UINT16, &Config::joydef[4]},
    {"joydef05", CONFIG_TYPE_UINT16, &Config::joydef[5]},
    {"joydef06", CONFIG_TYPE_UINT16, &Config::joydef[6]},
    {"joydef07", CONFIG_TYPE_UINT16, &Config::joydef[7]},
    {"joydef08", CONFIG_TYPE_UINT16, &Config::joydef[8]},
    {"joydef09", CONFIG_TYPE_UINT16, &Config::joydef[9]},
    {"joydef10", CONFIG_TYPE_UINT16, &Config::joydef[10]},
    {"joydef11", CONFIG_TYPE_UINT16, &Config::joydef[11]},
    {"joydef12", CONFIG_TYPE_UINT16, &Config::joydef[12]},
    {"joydef13", CONFIG_TYPE_UINT16, &Config::joydef[13]},
    {"joydef14", CONFIG_TYPE_UINT16, &Config::joydef[14]},
    {"joydef15", CONFIG_TYPE_UINT16, &Config::joydef[15]},
    {"joydef16", CONFIG_TYPE_UINT16, &Config::joydef[16]},
    {"joydef17", CONFIG_TYPE_UINT16, &Config::joydef[17]},
    {"joydef18", CONFIG_TYPE_UINT16, &Config::joydef[18]},
    {"joydef19", CONFIG_TYPE_UINT16, &Config::joydef[19]},
    {"joydef20", CONFIG_TYPE_UINT16, &Config::joydef[20]},
    {"joydef21", CONFIG_TYPE_UINT16, &Config::joydef[21]},
    {"joydef22", CONFIG_TYPE_UINT16, &Config::joydef[22]},
    {"joydef23", CONFIG_TYPE_UINT16, &Config::joydef[23]},

    // Additional configuration entries
    {"SNA_begin_row", CONFIG_TYPE_UINT16, &Config::SNA_begin_row},
    {"TAP_begin_row", CONFIG_TYPE_UINT16, &Config::TAP_begin_row},
    {"DSK_begin_row", CONFIG_TYPE_UINT16, &Config::DSK_begin_row},
    {"SNA_focus", CONFIG_TYPE_UINT16, &Config::SNA_focus},
    {"TAP_focus", CONFIG_TYPE_UINT16, &Config::TAP_focus},
    {"DSK_focus", CONFIG_TYPE_UINT16, &Config::DSK_focus},
    {"SNA_fdMode", CONFIG_TYPE_UINT8, &Config::SNA_fdMode},
    {"TAP_fdMode", CONFIG_TYPE_UINT8, &Config::TAP_fdMode},
    {"DSK_fdMode", CONFIG_TYPE_UINT8, &Config::DSK_fdMode},
    {"SNA_fileSearch", CONFIG_TYPE_STRING, &Config::SNA_fileSearch},
    {"TAP_fileSearch", CONFIG_TYPE_STRING, &Config::TAP_fileSearch},
    {"DSK_fileSearch", CONFIG_TYPE_STRING, &Config::DSK_fileSearch},
    {"TABasfire1", CONFIG_TYPE_BOOL, &Config::TABasfire1},
    {"StartMsg", CONFIG_TYPE_BOOL, &Config::StartMsg},
    {"ALUTK", CONFIG_TYPE_UINT8, &Config::ALUTK},
    {"DiskCtrl", CONFIG_TYPE_UINT8, &Config::DiskCtrl},
    {"osd_LRNav", CONFIG_TYPE_UINT8, &Config::osd_LRNav},
    {"osd_AltRot", CONFIG_TYPE_UINT8, &Config::osd_AltRot}
};

// Function to load the configuration
void Config::load() {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS
    nvs_handle_t handle;
    err = nvs_open("storage", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle for read!\n", esp_err_to_name(err));
        return;
    }

    // Iterate over the configuration entries and load them
    for (auto& entry : configEntries) {
        switch (entry.type) {
            case CONFIG_TYPE_STRING: {
                size_t required_size = 0;
                err = nvs_get_str(handle, entry.key, NULL, &required_size); // Get required size first
                if (err == ESP_OK && required_size > 0) {
                    std::string* str_value = static_cast<std::string*>(entry.value);
                    char* buffer = new char[required_size];
                    err = nvs_get_str(handle, entry.key, buffer, &required_size);
                    if (err == ESP_OK) {
                        // Asignar el contenido del buffer al string
                        *str_value = std::string(buffer, required_size - 1);  // Se resta 1 para no incluir el carácter nulo
                    }
                    delete[] buffer;  // Free memory
                } else if (err == ESP_ERR_NVS_NOT_FOUND) {
                    printf("Key '%s' not found, skipping...\n", entry.key);
                }
                break;
            }
            case CONFIG_TYPE_BOOL: {
                std::string bool_str;
                size_t required_size = 0;
                err = nvs_get_str(handle, entry.key, NULL, &required_size); // Get size of boolean string
                if (err == ESP_OK && required_size > 0) {
                    char* buffer = new char[required_size];
                    err = nvs_get_str(handle, entry.key, buffer, &required_size);
                    if (err == ESP_OK) {
                        bool_str = buffer;
                        *static_cast<bool*>(entry.value) = (bool_str == "true");
                    }
                    delete[] buffer;
                } else if (err == ESP_ERR_NVS_NOT_FOUND) {
                    printf("Key '%s' not found, skipping...\n", entry.key);
                }
                break;
            }
            case CONFIG_TYPE_UINT8: {
                err = nvs_get_u8(handle, entry.key, static_cast<uint8_t*>(entry.value));
                if (err == ESP_ERR_NVS_NOT_FOUND) {
                    printf("Key '%s' not found, skipping...\n", entry.key);
                }
                break;
            }
            case CONFIG_TYPE_UINT16: {
                err = nvs_get_u16(handle, entry.key, static_cast<uint16_t*>(entry.value));
                if (err == ESP_ERR_NVS_NOT_FOUND) {
                    printf("Key '%s' not found, skipping...\n", entry.key);
                }
                break;
            }
            case CONFIG_TYPE_INT8: {
                err = nvs_get_i8(handle, entry.key, static_cast<int8_t*>(entry.value));
                if (err == ESP_ERR_NVS_NOT_FOUND) {
                    printf("Key '%s' not found, skipping...\n", entry.key);
                }
                break;
            }
        }

        // Print error if there's a problem reading any entry
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            printf("Error (%s) reading key '%s'!\n", esp_err_to_name(err), entry.key);
        }
    }

    // Close NVS
    nvs_close(handle);
}

void Config::save() {
    Config::save("all");
}

// Function to save the configuration
void Config::save(string value) {

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS
    nvs_handle_t handle;
    err = nvs_open("storage", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    // Iterate over the configuration entries and save them
    for (const auto& entry : configEntries) {
        if (value == "all" || value == entry.key) {
            switch (entry.type) {
                case CONFIG_TYPE_STRING:
                    nvs_set_str(handle, entry.key, static_cast<std::string*>(entry.value)->c_str());
                    break;
                case CONFIG_TYPE_BOOL:
                    nvs_set_str(handle, entry.key, *static_cast<bool*>(entry.value) ? "true" : "false");
                    break;
                case CONFIG_TYPE_UINT8:
                    nvs_set_u8(handle, entry.key, *static_cast<uint8_t*>(entry.value));
                    break;
                case CONFIG_TYPE_UINT16:
                    nvs_set_u16(handle, entry.key, *static_cast<uint16_t*>(entry.value));
                    break;
                case CONFIG_TYPE_INT8:
                    nvs_set_i8(handle, entry.key, *static_cast<int8_t*>(entry.value));
                    break;
            }
        }
    }

    // Commit the updates to NVS
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        printf("Error (%s) committing updates to NVS!\n", esp_err_to_name(err));
    }

    // Close NVS
    nvs_close(handle);
}

void Config::requestMachine(string newArch, string newRomSet) {

    arch = newArch;

    port254default = 0xbf; // Default value for port 254

    if (arch == "48K") {

        if (newRomSet=="") romSet = "48K"; else romSet = newRomSet;

        if (newRomSet=="") romSet48 = "48K"; else romSet48 = newRomSet;

        if (romSet48 == "48K")
            MemESP::rom[0] = (uint8_t *) gb_rom_0_sinclair_48k;
        else if (romSet48 == "48Kes")
            MemESP::rom[0] = (uint8_t *) gb_rom_0_48k_es;
        else if (romSet48 == "48Kcs") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_48k_custom;
            MemESP::rom[0] += 8;
        }

    } else if (arch == "128K") {

        if (newRomSet=="") romSet = "128K"; else romSet = newRomSet;

        if (newRomSet=="") romSet128 = "128K"; else romSet128 = newRomSet;

        if (romSet128 == "128K") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_sinclair_128k;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_sinclair_128k;
        } else if (romSet128 == "128Kes") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_128k_es;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_128k_es;
        } else if (romSet128 == "128Kcs") {

            MemESP::rom[0] = (uint8_t *) gb_rom_0_128k_custom;
            MemESP::rom[0] += 8;

            MemESP::rom[1] = (uint8_t *) gb_rom_0_128k_custom;
            MemESP::rom[1] += 16392;

        } else if (romSet128 == "+2") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_plus2;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_plus2;
        } else if (romSet128 == "+2es") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_plus2_es;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_plus2_es;
        } else if (romSet128 == "ZX81+") {
            MemESP::rom[0] = (uint8_t *) gb_rom_0_s128_zx81;
            MemESP::rom[1] = (uint8_t *) gb_rom_1_sinclair_128k;
        }

    } else if (arch == "Pentagon") {

        if (newRomSet=="") romSet = "Pentagon"; else romSet = newRomSet;

        MemESP::rom[0] = (uint8_t *) gb_rom_0_pentagon_128k;
        MemESP::rom[1] = (uint8_t *) gb_rom_1_pentagon_128k;

    } else if (arch == "TK90X") {

        if (newRomSet=="") romSet = "v1es"; else romSet = newRomSet;

        if (newRomSet=="") romSetTK90X = "v1es"; else romSetTK90X = newRomSet;

        if (romSetTK90X == "v1es")
            MemESP::rom[0] = (uint8_t *) rom_0_TK90X_v1;
        else if (romSetTK90X == "v1pt") {
            MemESP::rom[0] = (uint8_t *) rom_0_TK90X_v1;
            port254default = 0x3f;
        } else if (romSetTK90X == "v2es") {
            MemESP::rom[0] = (uint8_t *) rom_0_TK90X_v2;
        } else if (romSetTK90X == "v2pt") {
            MemESP::rom[0] = (uint8_t *) rom_0_TK90X_v2;
            port254default = 0x3f;
        } else if (romSetTK90X == "v3es") {
            MemESP::rom[0] = (uint8_t *) rom_0_TK90X_v3es;
        } else if (romSetTK90X == "v3pt") {
            MemESP::rom[0] = (uint8_t *) rom_0_TK90X_v3pt;
        } else if (romSetTK90X == "v3en") {
            MemESP::rom[0] = (uint8_t *) rom_0_TK90X_v3en;
        } else if (romSetTK90X == "TKcs") {
            MemESP::rom[0] = (uint8_t *) rom_0_tk_custom;
            MemESP::rom[0] += 8;
        }


    } else if (arch == "TK95") {

        if (newRomSet=="") romSet = "95es"; else romSet = newRomSet;

        if (newRomSet=="") romSetTK95 = "95es"; else romSetTK95 = newRomSet;

        if (romSetTK95 == "95es")
            MemESP::rom[0] = (uint8_t *) rom_0_TK95ES;
        else if (romSetTK95 == "95pt") {
            MemESP::rom[0] = (uint8_t *) rom_0_TK95ES;
            port254default = 0x3f;
        }

    }

    MemESP::rom[4] = (uint8_t *) gb_rom_4_trdos_503;

}

void Config::setJoyMap(uint8_t joynum, uint8_t joytype) {

fabgl::VirtualKey newJoy[12];

for (int n=0; n < 12; n++) newJoy[n] = fabgl::VK_NONE;

// Ask to overwrite map with default joytype values
string title = (joynum == 1 ? "Joystick 1" : "Joystick 2");
string msg = OSD_DLG_SETJOYMAPDEFAULTS[Config::lang];
uint8_t res = OSD::msgDialog(title,msg);
if (res == DLG_YES) {

    switch (joytype) {
    case JOY_CURSOR:
        newJoy[0] = fabgl::VK_5;
        newJoy[1] = fabgl::VK_8;
        newJoy[2] = fabgl::VK_7;
        newJoy[3] = fabgl::VK_6;
        newJoy[6] = fabgl::VK_0;
        break;
    case JOY_KEMPSTON:
        newJoy[0] = fabgl::VK_KEMPSTON_LEFT;
        newJoy[1] = fabgl::VK_KEMPSTON_RIGHT;
        newJoy[2] = fabgl::VK_KEMPSTON_UP;
        newJoy[3] = fabgl::VK_KEMPSTON_DOWN;
        newJoy[6] = fabgl::VK_KEMPSTON_FIRE;
        newJoy[7] = fabgl::VK_KEMPSTON_ALTFIRE;
        break;
    case JOY_SINCLAIR1:
        newJoy[0] = fabgl::VK_6;
        newJoy[1] = fabgl::VK_7;
        newJoy[2] = fabgl::VK_9;
        newJoy[3] = fabgl::VK_8;
        newJoy[6] = fabgl::VK_0;
        break;
    case JOY_SINCLAIR2:
        newJoy[0] = fabgl::VK_1;
        newJoy[1] = fabgl::VK_2;
        newJoy[2] = fabgl::VK_4;
        newJoy[3] = fabgl::VK_3;
        newJoy[6] = fabgl::VK_5;
        break;
    case JOY_FULLER:
        newJoy[0] = fabgl::VK_FULLER_LEFT;
        newJoy[1] = fabgl::VK_FULLER_RIGHT;
        newJoy[2] = fabgl::VK_FULLER_UP;
        newJoy[3] = fabgl::VK_FULLER_DOWN;
        newJoy[6] = fabgl::VK_FULLER_FIRE;
        break;
    }

}

// Fill joystick values in Config and clean Kempston or Fuller values if needed
int m = (joynum == 1) ? 0 : 12;

for (int n = m; n < m + 12; n++) {

    bool save = false;
    if (newJoy[n - m] != fabgl::VK_NONE) {
        ESPectrum::JoyVKTranslation[n] = newJoy[n - m];
        save = true;
    } else {

        if (joytype != JOY_KEMPSTON) {
            if (ESPectrum::JoyVKTranslation[n] >= fabgl::VK_KEMPSTON_RIGHT && ESPectrum::JoyVKTranslation[n] <= fabgl::VK_KEMPSTON_ALTFIRE) {
                ESPectrum::JoyVKTranslation[n] = fabgl::VK_NONE;
                save = true;
            }
        }

        if (joytype != JOY_FULLER) {
            if (ESPectrum::JoyVKTranslation[n] >= fabgl::VK_FULLER_RIGHT && ESPectrum::JoyVKTranslation[n] <= fabgl::VK_FULLER_FIRE) {
                ESPectrum::JoyVKTranslation[n] = fabgl::VK_NONE;
                save = true;
            }
        }

    }

    if (save) {
        // Save to config (only changes)
        if (Config::joydef[n] != (uint16_t) ESPectrum::JoyVKTranslation[n]) {
            Config::joydef[n] = (uint16_t) ESPectrum::JoyVKTranslation[n];
            char joykey[9];
            sprintf(joykey,"joydef%02u",n);
            Config::save(joykey);
            // printf("%s %u\n",joykey, joydef[n]);
        }
    }

}

}
