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


#include <functional>

#include "ESPeccy.h"
#include "BIOS.h"
#include "OSD.h"
#include "Video.h"
#include "ZXKeyb.h"
#include "FileUtils.h"
#include "Config.h"

#include "CommitDate.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"

#include "esp_ota_ops.h"

#ifndef FWBUFFSIZE
#define FWBUFFSIZE 512 /* 4096 */
#endif

void BIOS::run() {

    Config::load(); // Restore original config values

    auto Kbd = ESPeccy::PS2Controller.keyboard();
    fabgl::VirtualKeyItem NextKey;

    VIDEO::clear(zxColor(7, 0));

    int base_row = OSD_FONT_H * 4;
    int base_col = OSD_FONT_W * 4;
    int total_rows = OSD::scrH / OSD_FONT_H - 8;
    int total_cols = OSD::scrW / OSD_FONT_W - 8;

    #define PRINT_FILLED_CENTERED(text)  VIDEO::print(string((total_cols - strlen(text)) / 2, ' ').c_str()); VIDEO::print(text); VIDEO::print(string(total_cols - strlen(text) - (total_cols - strlen(text)) / 2, ' ').c_str())
    #define PRINT_FILLED_ROW(text)  VIDEO::print(text); VIDEO::print(string(total_cols - strlen(text), ' ').c_str())
    #define PRINT_FILLED_ROW_ALIGN_RIGHT(text)  VIDEO::print(string(total_cols - strlen(text), ' ').c_str()); VIDEO::print(text)
    #define SET_CURSOR(col,row) VIDEO::setCursor(base_col + (col) * OSD_FONT_W, base_row + (row) * OSD_FONT_H)

    // Opciones del menú
    const char* menuOptions[] = {"Main", "Video", "Keyboard", "Config", "Update", "Exit"};
    const int menuCount = sizeof(menuOptions)/sizeof(menuOptions[0]);

    const char* menuOptionsLily[] = {"Main", "Video", "Keyboard", "Others", "Config", "Update", "Exit"};
    const int menuCountLily = sizeof(menuOptionsLily)/sizeof(menuOptionsLily[0]);

    const char* menuVideo[] = {"Resolution", "Frequency", "Scanlines"};
    const int menuVideoCount = sizeof(menuVideo) / sizeof(menuVideo[0]);

    const char* menuKeyboard[] = {"ZXUnoPS2 (.ZXPure)", "Custom Layout"};
    const int menuKeyboardCount = sizeof(menuKeyboard) / sizeof(menuKeyboard[0]);

    const char* menuOthers[] = {"IO36 Button"};
    const int menuOthersCount = sizeof(menuOthers) / sizeof(menuOthers[0]);

    const char* menuConfig[] = {"Backup Settings", "Restore Settings", "Reset Settings"};
    const int menuConfigCount = sizeof(menuConfig) / sizeof(menuConfig[0]);

    const char* menuExit[] = {"Save Changes & Exit", "Discard Changes & Exit"};
    const int menuExitCount = sizeof(menuExit) / sizeof(menuExit[0]);

    const char* menuOptionsResolution[] = {"320x240 (4:3)", "360x200 (16:9)"};

    const char* menuOptionsFrequency[] = {"60Hz (VGA)", "50Hz (VGA)", "15kHz (CRT)"};

    const char* menuYesNo[] = {"No", "Yes"};

    const char* menuOthersIO36Button[] = {"RESET", "NMI", "CHEATS", "POKE", "STATS", "MENU"};

    const char* menuUpdate[] = {"Firmware Update"};
    const int menuUpdateCount = sizeof(menuUpdate) / sizeof(menuUpdate[0]);

    int selectedOption = 0;

    auto remountSD = [&]() {
        if ( FileUtils::SDReady && !FileUtils::isMountedSDCard() ) FileUtils::unmountSDCard();
        if ( !FileUtils::SDReady ) FileUtils::initFileSystem();
    };


    // Renderizar el menú inicial
    auto renderMenu = [&](int highlight) {
        SET_CURSOR(0, 1);
        int len = 0;
        int limit = ZXKeyb::Exists ? menuCount : menuCountLily;
        for (int i = 0; i < limit; ++i) {
            VIDEO::setTextColor(i == highlight ? zxColor(1, 1) : zxColor(7, 1), i == highlight ? zxColor(7, 1) : zxColor(1, 0));
            VIDEO::print(" ");
            VIDEO::print(ZXKeyb::Exists ? menuOptions[i] : menuOptionsLily[i]);
            VIDEO::print(" ");
            len += strlen(ZXKeyb::Exists ? menuOptions[i] : menuOptionsLily[i]) + 2;
        }
        VIDEO::setTextColor(zxColor(7, 1), zxColor(1, 0));
        VIDEO::print(string(total_cols - len, ' ').c_str());
    };

    auto renderOptions = [&](const char *options[], const char *values[], const int optionsCount, int highlight) {
        SET_CURSOR(1, 3); // Ajustar la posición para el submenú
        for (int i = 0; i < optionsCount; ++i) {
            // Color del texto, resaltado para el elemento seleccionado
            VIDEO::setTextColor(i == highlight ? zxColor(7, 1) : zxColor(1, 0),
                                    i == highlight ? zxColor(0, 0) : zxColor(7, 0));

            // Imprimir el nombre de la opción
            VIDEO::print(" ");
            VIDEO::print(options[i]);

            // Calcular espacios en blanco para alinear valores
            int padding = total_cols - 19 /* Help Column */ - 2 - strlen(options[i]) -
                          (values && values[i] ? strlen(values[i]) : 0) - 2; // espacio antes del valor si existe

            // Añadir los espacios para alineación
            VIDEO::print(string(padding, ' ').c_str());

            // Si hay valores, imprimir el valor alineado a la derecha
            if (values && values[i]) {
                VIDEO::print(values[i]);
            }

            VIDEO::print(" \n");
        }
    };

    auto screen_clear = [&](bool fullwidth = false) {
        int color = zxColor(7, 0);
        for (int y = base_row + OSD_FONT_H * 3; y < base_row + ( total_rows - 2 ) * OSD_FONT_H - OSD_FONT_H / 2; y++)
            for (int x = base_col + OSD_FONT_W - OSD_FONT_W / 2; x < base_col + ( total_cols - ( fullwidth ? 0 : 20 )) * OSD_FONT_W + OSD_FONT_W / 2; x++)
                VIDEO::dotFast(x, y, color);

        const int top = base_row + OSD_FONT_H * 2 + OSD_FONT_H / 2;
        const int buttom = base_row + ( total_rows - 1 ) * OSD_FONT_H - OSD_FONT_H / 2;
        const int left = base_col + OSD_FONT_W / 2;
        const int right = base_col + ( total_cols - 1 ) * OSD_FONT_W + OSD_FONT_W / 2;

        VIDEO::line(  left,    top, right,    top, zxColor(1, 0));
        VIDEO::line(  left, buttom, right, buttom, zxColor(1, 0));
        VIDEO::line(  left,    top,  left, buttom, zxColor(1, 0));
        VIDEO::line( right,    top, right, buttom, zxColor(1, 0));

        VIDEO::line( right - 19 * OSD_FONT_W, top                    , right - 19 * OSD_FONT_W, buttom                 , zxColor(1, 0));
        VIDEO::line( right - 19 * OSD_FONT_W, buttom - 6 * OSD_FONT_H, right                  , buttom - 6 * OSD_FONT_H, zxColor(1, 0));

        SET_CURSOR(total_cols - 19, total_rows - 7);
        VIDEO::setTextColor(zxColor(1, 0), zxColor(7, 0));
        VIDEO::print("\x1A \x1B Select Screen\n");
        VIDEO::print("\x18 \x19 Select Item\n");
        VIDEO::print("Enter: Select/Chg.\n");
        if (ZXKeyb::Exists || Config::zxunops2) {
            VIDEO::print("\x06+S: Save & Exit\n");
            VIDEO::print("BREAK: Exit\n");
        } else {
            VIDEO::print("F10: Save & Exit\n");
            VIDEO::print("ESC: Exit  \n");
        }

    };

    #define BIOS_DLG_ALERT      0
    #define BIOS_DLG_CONFIRM    1
    #define BIOS_DLG_MSG        2

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

        if (pi < p && p - pi + 1 > dialog_width) dialog_width = p - pi + 1;

        dialog_width += 4; // 2 caracteres de margen a cada lado

        // Ajustar el alto total del diálogo
        int dialog_height = message_height + 3 + ((type != BIOS_DLG_MSG) ? 2 : 0); // Incluye el título, márgenes y botones

        int left = (total_cols - dialog_width) / 2;
        int right = left + dialog_width;
        int top = (total_rows - dialog_height) / 2;
        int bottom = top + 2 + dialog_height;

        // Limpiar el área del diálogo
        for (int y = base_row + top * OSD_FONT_H; y < base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2; y++) {
            for (int x = base_col + left * OSD_FONT_W - OSD_FONT_W / 2; x < base_col + right * OSD_FONT_W + OSD_FONT_W / 2; x++) {
                VIDEO::dotFast(x, y, zxColor(7, 0));
            }
        }

        // Dibujar el borde del diálogo
        VIDEO::line(base_col +  left * OSD_FONT_W - OSD_FONT_W / 2, base_row +    top * OSD_FONT_H + OSD_FONT_H / 2, base_col + right * OSD_FONT_W + OSD_FONT_W / 2, base_row +    top * OSD_FONT_H + OSD_FONT_H / 2, zxColor(1, 0));
        VIDEO::line(base_col +  left * OSD_FONT_W - OSD_FONT_W / 2, base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2, base_col + right * OSD_FONT_W + OSD_FONT_W / 2, base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2, zxColor(1, 0));
        VIDEO::line(base_col +  left * OSD_FONT_W - OSD_FONT_W / 2, base_row +    top * OSD_FONT_H + OSD_FONT_H / 2, base_col +  left * OSD_FONT_W - OSD_FONT_W / 2, base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2, zxColor(1, 0));
        VIDEO::line(base_col + right * OSD_FONT_W + OSD_FONT_W / 2, base_row +    top * OSD_FONT_H + OSD_FONT_H / 2, base_col + right * OSD_FONT_W + OSD_FONT_W / 2, base_row + bottom * OSD_FONT_H - OSD_FONT_H / 2, zxColor(1, 0));

        VIDEO::fillRect(base_col + left * OSD_FONT_W, base_row + ( top + 1 ) * OSD_FONT_H, dialog_width * OSD_FONT_W, dialog_height * OSD_FONT_H, zxColor(1,0));

        // Mostrar el título en la primera línea dentro del cuadro
        SET_CURSOR(left + dialog_width / 2 - strlen(title) / 2, top);
        VIDEO::setTextColor(zxColor(1, 0), zxColor(7, 0));
        VIDEO::print(title);

        if ( message_height > 0 ) {
            // Mostrar el mensaje en la tercera línea
            SET_CURSOR(left + 1, top + 2);
            VIDEO::setTextColor(zxColor(7, 1), zxColor(1, 0));
            VIDEO::print(message);
        } else {
            // Mostrar el mensaje en la tercera línea
            SET_CURSOR(left + 1, top + 2);

            int current_line = 0;
            pi = p = message;
            while(*p) {
                if (*p == '\n') {
                    SET_CURSOR(left + dialog_width / 2 - (p - pi) / 2, top + 2 + current_line); // +2 por la línea del título y el margen superior
                    VIDEO::setTextColor(zxColor(7, 1), zxColor(1, 0));
                    for(; pi < p - 1; ++pi) VIDEO::print(*pi);
                    ++current_line;
                }
                ++p;
            }

            if (pi < p) {
                SET_CURSOR(left + dialog_width / 2 - (p - pi) / 2, top + 2 + current_line); // +2 por la línea del título y el margen superior
                VIDEO::setTextColor(zxColor(7, 1), zxColor(1, 0));
                for(; pi < p; ++pi) VIDEO::print(*pi);
            }
        }


        // Mostrar los botones "OK" y "Cancel" en la quinta línea
        int selectedButton = 0; // 0 = OK, 1 = Cancel
        auto renderButtons = [&](int type = BIOS_DLG_ALERT) {
            SET_CURSOR(left + dialog_width / 2 - ( ( type == BIOS_DLG_CONFIRM ) ? 11 : 5 ), top + 4 + message_height);
            VIDEO::setTextColor(selectedButton == 0 ? zxColor(1, 1) : zxColor(7, 0), selectedButton == 0 ? zxColor(7, 1) : zxColor(1, 0));
            VIDEO::print("[   OK   ]");

            if ( type == BIOS_DLG_CONFIRM ) {
                VIDEO::setTextColor(zxColor(7, 1), zxColor(1, 0));
                VIDEO::print("  ");

                VIDEO::setTextColor(selectedButton == 1 ? zxColor(1, 1) : zxColor(7, 0), selectedButton == 1 ? zxColor(7, 1) : zxColor(1, 0));
                VIDEO::print("[ CANCEL ]");
            }
        };

        if (type != BIOS_DLG_MSG) {
            renderButtons(type);
        }

        // No unir este if con el de arriba, si se une, desconozco el motivo, pero el BIOS_DLG_MSG espera por las key...
        if (type != BIOS_DLG_MSG) {
            // Esperar la selección del usuario
            while (true) {
                if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEBIOS);
                while (Kbd->virtualKeyAvailable()) {
                    fabgl::VirtualKeyItem NextKey;
                    if (ESPeccy::readKbd(&NextKey, KBDREAD_MODEBIOS) && NextKey.down) {
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
        }
    };

    // Mostrar información de hardware
    auto showHardwareInfo = [&]() {
        screen_clear();

        // Mostrar información de chip
        SET_CURSOR(1, 3);
        VIDEO::setTextColor(zxColor(1, 0), zxColor(7, 0));

        VIDEO::print(ESPeccy::getHardwareInfo().c_str());
    };

    // Iniciar el menú
    renderMenu(selectedOption);
    showHardwareInfo();

    // Special position
    //VIDEO::setCursor(base_col, base_row - 1); // 0, 0 - 1px
    SET_CURSOR(0,0);
    VIDEO::setTextColor(zxColor(7, 1), zxColor(1, 1));
    PRINT_FILLED_CENTERED("ESPeccy Bios Setup Utility");

    SET_CURSOR(0, total_rows - 1);
    const char* commitDate = getShortCommitDate();
    string footer = "BIOS Date: 20"
                    + std::string(1, commitDate[0]) + commitDate[1] + "/"
                    + std::string(1, commitDate[2]) + commitDate[3] + "/"
                    + std::string(1, commitDate[4]) + commitDate[5] + " "
                    + std::string(1, commitDate[6]) + commitDate[7] + ":"
                    + std::string(1, commitDate[8]) + commitDate[9] + " ";
    VIDEO::setTextColor(zxColor(7, 1), zxColor(1, 0));
    PRINT_FILLED_ROW_ALIGN_RIGHT(footer.c_str());

    // Lógica de navegación del menú
    bool exitMenu = false;

    bool exit_to_main = false;

    auto mainMenuNav = [&](const std::function<void()>& escCancel, const std::function<void()>& f10Cancel) {
        switch (NextKey.vk) {
            case fabgl::VK_RIGHT:
                selectedOption = (selectedOption + 1) % (ZXKeyb::Exists ? menuCount : menuCountLily);
                renderMenu(selectedOption);
                exit_to_main = true;
                break;

            case fabgl::VK_LEFT:
                selectedOption = (selectedOption - 1 + (ZXKeyb::Exists ? menuCount : menuCountLily)) % (ZXKeyb::Exists ? menuCount : menuCountLily);
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

        if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEBIOS);
        while (Kbd->virtualKeyAvailable()) {
            bool r = ESPeccy::readKbd(&NextKey, KBDREAD_MODEBIOS);
            if (r && NextKey.down) mainMenuNav([&showHardwareInfo](){showHardwareInfo();}, [&showHardwareInfo](){showHardwareInfo();});
        }

        if (selectedOption != oldSelectedOptions || exit_to_main ) {
            exit_to_main = false;
            // Acción según la opción seleccionada
            if (selectedOption == 0 ) { // Acción para MAIN
                screen_clear();
                showHardwareInfo();
            }
            else if (selectedOption == 1) { // Acción para Video
                int selectedVideoOption = 0;

                auto renderVideoOptions = [&]() {
                    const char *valuesVideo[] = { menuOptionsResolution[Config::aspect_16_9], menuOptionsFrequency[Config::videomode], menuYesNo[Config::scanlines] };
                    renderOptions(menuVideo, valuesVideo, menuVideoCount, selectedVideoOption);
                };

                // Renderizar menú video
                screen_clear();
                renderVideoOptions();

                // Lógica para el menú video
                while (true) {
                    if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEBIOS);
                    while (Kbd->virtualKeyAvailable()) {
                        bool r = ESPeccy::readKbd(&NextKey, KBDREAD_MODEBIOS);
                        if (r && NextKey.down) {

                            mainMenuNav([&renderVideoOptions](){renderVideoOptions();}, [&renderVideoOptions](){renderVideoOptions();});
                            optionsNav(selectedVideoOption, menuVideoCount, [&renderVideoOptions](){renderVideoOptions();});

                            switch (NextKey.vk) {
                                case fabgl::VK_RETURN:
                                case fabgl::VK_SPACE:
                                    switch (selectedVideoOption) {
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
                                    renderVideoOptions();
                                    break;
                            }
                        }
                    }

                    if (exit_to_main) break;

                    vTaskDelay(100 / portTICK_PERIOD_MS);

                }

                if (!exit_to_main) {
                    screen_clear();
                    renderVideoOptions();
                }
            }
            else if (selectedOption == 2) { // Acción para Keyboard
                int selectedKeyboardOption = 0;

                auto renderKeyboardOptions = [&]() {
                    const char *valuesKeyboard[] = { menuYesNo[Config::zxunops2], menuYesNo[Config::KBDLayoutEnable] };
                    renderOptions(menuKeyboard, valuesKeyboard, menuKeyboardCount, selectedKeyboardOption);
                };

                // Renderizar menú keyboard
                screen_clear();
                renderKeyboardOptions();

                // Lógica para el menú keyboard
                while (true) {
                    if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEBIOS);
                    while (Kbd->virtualKeyAvailable()) {
                        bool r = ESPeccy::readKbd(&NextKey, KBDREAD_MODEBIOS);
                        if (r && NextKey.down) {

                            mainMenuNav([&renderKeyboardOptions](){renderKeyboardOptions();}, [&renderKeyboardOptions](){renderKeyboardOptions();});
                            optionsNav(selectedKeyboardOption, menuKeyboardCount, [&renderKeyboardOptions](){renderKeyboardOptions();});

                            switch (NextKey.vk) {
                                case fabgl::VK_RETURN:
                                case fabgl::VK_SPACE:
                                    switch (selectedKeyboardOption) {
                                        case 0: // Acción para ZXUnoPS2
                                            Config::zxunops2 = !Config::zxunops2;
                                            break;

                                        case 1: // Acción para Custom Layout
                                            Config::KBDLayoutEnable = !Config::KBDLayoutEnable;
                                            break;
                                    }

                                    screen_clear();
                                    renderKeyboardOptions();
                                    break;
                            }
                        }
                    }

                    if (exit_to_main) break;

                    vTaskDelay(100 / portTICK_PERIOD_MS);

                }

                if (!exit_to_main) {
                    screen_clear();
                    renderKeyboardOptions();
                }
            }
            else if (!ZXKeyb::Exists && selectedOption == 3) { // Acción para Others
                int selectedOthersOption = 0;

                auto renderOthersOptions = [&]() {
                    const char *valuesOthes[] = { menuOthersIO36Button[Config::io36button] };
                    renderOptions(menuOthers, valuesOthes, menuOthersCount, selectedOthersOption);
                };

                // Renderizar menú otros
                screen_clear();
                renderOthersOptions();

                // Lógica para el menú otros
                while (true) {
                    if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEBIOS);
                    while (Kbd->virtualKeyAvailable()) {
                        bool r = ESPeccy::readKbd(&NextKey, KBDREAD_MODEBIOS);
                        if (r && NextKey.down) {

                            mainMenuNav([&renderOthersOptions](){renderOthersOptions();}, [&renderOthersOptions](){renderOthersOptions();});
                            optionsNav(selectedOthersOption, menuOthersCount, [&renderOthersOptions](){renderOthersOptions();});

                            switch (NextKey.vk) {
                                case fabgl::VK_RETURN:
                                case fabgl::VK_SPACE:
                                    switch (selectedOthersOption) {
                                        case 0: // Acción para IO36 Button
                                            Config::io36button = (Config::io36button + 1) % (sizeof(menuOthersIO36Button)/sizeof(menuOthersIO36Button[0]));
                                            break;
                                    }

                                    screen_clear();
                                    renderOthersOptions();
                                    break;
                            }
                        }
                    }

                    if (exit_to_main) break;

                    vTaskDelay(100 / portTICK_PERIOD_MS);

                }

                if (!exit_to_main) {
                    screen_clear();
                    renderOthersOptions();
                }
            }
            else if (selectedOption == (ZXKeyb::Exists ? 3 : 4)) { // Acción para CONFIG
                int selectedConfigOption = 0;
                // Renderizar menú de configuración
                screen_clear();
                renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);

                while (true) {
                    if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEBIOS);
                    while (Kbd->virtualKeyAvailable()) {
                        bool r = ESPeccy::readKbd(&NextKey, KBDREAD_MODEBIOS);
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

                if (!exit_to_main) {
                    screen_clear();
                    renderOptions(menuConfig, NULL, menuConfigCount, selectedConfigOption);
                }
            }
            else if (selectedOption == (ZXKeyb::Exists ? 4 : 5)) { // Acción para Update
                int selectedUpdateOption = 0;
                // Renderizar menú de actualización
                screen_clear();
                renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);

                while (true) {
                    if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEBIOS);
                    while (Kbd->virtualKeyAvailable()) {
                        bool r = ESPeccy::readKbd(&NextKey, KBDREAD_MODEBIOS);
                        if (r && NextKey.down) {

                            mainMenuNav([&renderOptions, &menuUpdate, &menuUpdateCount, &selectedUpdateOption](){renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);},
                                        [&renderOptions, &menuUpdate, &menuUpdateCount, &selectedUpdateOption](){renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);});
                            optionsNav(selectedUpdateOption, menuUpdateCount, [&renderOptions, &menuUpdate, &menuUpdateCount, &selectedUpdateOption](){renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);});

                            switch (NextKey.vk) {
                                case fabgl::VK_RETURN:
                                case fabgl::VK_SPACE:
                                    switch (selectedUpdateOption) {
                                        case 0: // Acción para Save Changes & Update
                                            if ( msg_dialog("Confim update firmware", "Are you sure you want to perform\na firmware update?", BIOS_DLG_CONFIRM) ) {

                                                remountSD();

                                                FILE *firmware = fopen("/sd/firmware.upg", "rb");
                                                if (firmware) {

                                                    screen_clear(true);
                                                    renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);
                                                    msg_dialog("Firmware Update", "Update... 0% completed", BIOS_DLG_MSG);

                                                    char ota_write_data[FWBUFFSIZE + 1] = { 0 };

                                                    // get the currently running partition
                                                    const esp_partition_t *partition = esp_ota_get_running_partition();
                                                    if (partition) {
                                                        // Grab next update target
                                                        // const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
                                                        string splabel;
                                                        if (strcmp(partition->label,"esp0")==0) splabel = "esp1"; else splabel= "esp0";

                                                        const esp_partition_t *target = esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_ANY,splabel.c_str());
                                                        if (target) {

                                                            esp_ota_handle_t ota_handle;
                                                            esp_err_t result = esp_ota_begin(target, OTA_SIZE_UNKNOWN, &ota_handle);
                                                            if (result == ESP_OK) {

                                                                size_t bytesread;
                                                                uint32_t byteswritten = 0;

                                                                // Get firmware size
                                                                fseek(firmware, 0, SEEK_END);
                                                                long bytesfirmware = ftell(firmware);
                                                                rewind(firmware);

                                                                while (1) {
                                                                    bytesread = fread(ota_write_data, 1, FWBUFFSIZE, firmware);
                                                                    result = esp_ota_write(ota_handle,(const void *) ota_write_data, bytesread);
                                                                    if (result != ESP_OK) break;

                                                                    byteswritten += bytesread;

                                                                    int percent = (float) 100 / ((float) bytesfirmware / (float) byteswritten);
                                                                    if (percent%10 == 0) {
                                                                        char percent_str[32];
                                                                        sprintf(percent_str, "Update... %d%% completed", percent);
                                                                        msg_dialog("Firmware Update", percent_str, BIOS_DLG_MSG);
                                                                    }

                                                                    // printf("Bytes written: %d\n",byteswritten);
                                                                    if (feof(firmware)) break;
                                                                }

                                                                if (result == ESP_OK) {
                                                                    result = esp_ota_end(ota_handle);
                                                                    if (result == ESP_OK) {
                                                                        result = esp_ota_set_boot_partition(target);
                                                                        if (result == ESP_OK) {

                                                                            screen_clear(true);
                                                                            renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);
                                                                            msg_dialog("Firmware Update", "Update 100% ""completed\nRestarting...", BIOS_DLG_MSG);

                                                                            // Enable StartMsg
                                                                            Config::StartMsg = true;
                                                                            Config::save("StartMsg");

                                                                            delay(2000);

                                                                            // Firmware written: reboot
                                                                            OSD::esp_hard_reset();
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }

                                                screen_clear(true);
                                                renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);
                                                msg_dialog("Firmware Update", "Firmware Update Failed!");

                                            }

                                            screen_clear(true);
                                            renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);
                                            break;
                                    }
                                    break;
                            }
                        }
                    }

                    if (exit_to_main) break;

                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }

                if (!exit_to_main) {
                    screen_clear();
                    renderOptions(menuUpdate, NULL, menuUpdateCount, selectedUpdateOption);
                }

            }
            else if (selectedOption == (ZXKeyb::Exists ? 5 : 6)) { // Acción para EXIT
                int selectedExitOption = 0;
                // Renderizar menú de salida
                screen_clear();
                renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);

                while (true) {
                    if (ZXKeyb::Exists) ZXKeyb::ZXKbdRead(KBDREAD_MODEBIOS);
                    while (Kbd->virtualKeyAvailable()) {
                        bool r = ESPeccy::readKbd(&NextKey, KBDREAD_MODEBIOS);
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

                if (!exit_to_main) {
                    screen_clear();
                    renderOptions(menuExit, NULL, menuExitCount, selectedExitOption);
                }

            }

        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    VIDEO::clear(zxColor(7, 0));
}
