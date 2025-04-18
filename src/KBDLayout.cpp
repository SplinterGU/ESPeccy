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


#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include <vector>
#include <unordered_map>

#include "fabgl.h"

#include "KBDLayout.h"
#include "ESPeccy.h"

using namespace std;

// Tabla de traducción: de "KEY_*" a constante VK_* (entero)
const std::unordered_map<std::string, int> keyToVK = {
    { "KEY_NONE", fabgl::VK_NONE },
    { "KEY_SPACE", fabgl::VK_SPACE },
    { "KEY_0", fabgl::VK_0 },
    { "KEY_1", fabgl::VK_1 },
    { "KEY_2", fabgl::VK_2 },
    { "KEY_3", fabgl::VK_3 },
    { "KEY_4", fabgl::VK_4 },
    { "KEY_5", fabgl::VK_5 },
    { "KEY_6", fabgl::VK_6 },
    { "KEY_7", fabgl::VK_7 },
    { "KEY_8", fabgl::VK_8 },
    { "KEY_9", fabgl::VK_9 },
    { "KEY_KP_0", fabgl::VK_KP_0 },
    { "KEY_KP_1", fabgl::VK_KP_1 },
    { "KEY_KP_2", fabgl::VK_KP_2 },

    { "KEY_KP_3", fabgl::VK_KP_3 },
    { "KEY_KP_4", fabgl::VK_KP_4 },
    { "KEY_KP_5", fabgl::VK_KP_5 },
    { "KEY_KP_6", fabgl::VK_KP_6 },
    { "KEY_KP_7", fabgl::VK_KP_7 },
    { "KEY_KP_8", fabgl::VK_KP_8 },
    { "KEY_KP_9", fabgl::VK_KP_9 },
    { "KEY_a", fabgl::VK_a },
    { "KEY_b", fabgl::VK_b },
    { "KEY_c", fabgl::VK_c },
    { "KEY_d", fabgl::VK_d },
    { "KEY_e", fabgl::VK_e },
    { "KEY_f", fabgl::VK_f },
    { "KEY_g", fabgl::VK_g },
    { "KEY_h", fabgl::VK_h },

    { "KEY_i", fabgl::VK_i },
    { "KEY_j", fabgl::VK_j },
    { "KEY_k", fabgl::VK_k },
    { "KEY_l", fabgl::VK_l },
    { "KEY_m", fabgl::VK_m },
    { "KEY_n", fabgl::VK_n },
    { "KEY_o", fabgl::VK_o },
    { "KEY_p", fabgl::VK_p },
    { "KEY_q", fabgl::VK_q },
    { "KEY_r", fabgl::VK_r },
    { "KEY_s", fabgl::VK_s },
    { "KEY_t", fabgl::VK_t },
    { "KEY_u", fabgl::VK_u },
    { "KEY_v", fabgl::VK_v },
    { "KEY_w", fabgl::VK_w },
    { "KEY_x", fabgl::VK_x },
    { "KEY_y", fabgl::VK_y },
    { "KEY_z", fabgl::VK_z },

    { "KEY_A", fabgl::VK_A },
    { "KEY_B", fabgl::VK_B },
    { "KEY_C", fabgl::VK_C },
    { "KEY_D", fabgl::VK_D },
    { "KEY_E", fabgl::VK_E },
    { "KEY_F", fabgl::VK_F },
    { "KEY_G", fabgl::VK_G },
    { "KEY_H", fabgl::VK_H },
    { "KEY_I", fabgl::VK_I },
    { "KEY_J", fabgl::VK_J },
    { "KEY_K", fabgl::VK_K },
    { "KEY_L", fabgl::VK_L },
    { "KEY_M", fabgl::VK_M },
    { "KEY_N", fabgl::VK_N },
    { "KEY_O", fabgl::VK_O },
    { "KEY_P", fabgl::VK_P },
    { "KEY_Q", fabgl::VK_Q },
    { "KEY_R", fabgl::VK_R },

    { "KEY_S", fabgl::VK_S },
    { "KEY_T", fabgl::VK_T },
    { "KEY_U", fabgl::VK_U },
    { "KEY_V", fabgl::VK_V },
    { "KEY_W", fabgl::VK_W },
    { "KEY_X", fabgl::VK_X },
    { "KEY_Y", fabgl::VK_Y },
    { "KEY_Z", fabgl::VK_Z },
    { "KEY_GRAVEACCENT", fabgl::VK_GRAVEACCENT },
    { "KEY_ACUTEACCENT", fabgl::VK_ACUTEACCENT },
    { "KEY_QUOTE", fabgl::VK_QUOTE },
    { "KEY_QUOTEDBL", fabgl::VK_QUOTEDBL },
    { "KEY_EQUALS", fabgl::VK_EQUALS },
    { "KEY_MINUS", fabgl::VK_MINUS },
    { "KEY_KP_MINUS", fabgl::VK_KP_MINUS },

    { "KEY_PLUS", fabgl::VK_PLUS },
    { "KEY_KP_PLUS", fabgl::VK_KP_PLUS },
    { "KEY_KP_MULTIPLY", fabgl::VK_KP_MULTIPLY },
    { "KEY_ASTERISK", fabgl::VK_ASTERISK },
    { "KEY_BACKSLASH", fabgl::VK_BACKSLASH },
    { "KEY_KP_DIVIDE", fabgl::VK_KP_DIVIDE },
    { "KEY_SLASH", fabgl::VK_SLASH },
    { "KEY_KP_PERIOD", fabgl::VK_KP_PERIOD },
    { "KEY_PERIOD", fabgl::VK_PERIOD },
    { "KEY_COLON", fabgl::VK_COLON },

    { "KEY_COMMA", fabgl::VK_COMMA },
    { "KEY_SEMICOLON", fabgl::VK_SEMICOLON },
    { "KEY_AMPERSAND", fabgl::VK_AMPERSAND },
    { "KEY_VERTICALBAR", fabgl::VK_VERTICALBAR },
    { "KEY_HASH", fabgl::VK_HASH },
    { "KEY_AT", fabgl::VK_AT },
    { "KEY_CARET", fabgl::VK_CARET },
    { "KEY_DOLLAR", fabgl::VK_DOLLAR },
    { "KEY_POUND", fabgl::VK_POUND },
    { "KEY_EURO", fabgl::VK_EURO },
    { "KEY_PERCENT", fabgl::VK_PERCENT },

    { "KEY_EXCLAIM", fabgl::VK_EXCLAIM },
    { "KEY_QUESTION", fabgl::VK_QUESTION },
    { "KEY_LEFTBRACE", fabgl::VK_LEFTBRACE },
    { "KEY_RIGHTBRACE", fabgl::VK_RIGHTBRACE },
    { "KEY_LEFTBRACKET", fabgl::VK_LEFTBRACKET },
    { "KEY_RIGHTBRACKET", fabgl::VK_RIGHTBRACKET },
    { "KEY_LEFTPAREN", fabgl::VK_LEFTPAREN },
    { "KEY_RIGHTPAREN", fabgl::VK_RIGHTPAREN },
    { "KEY_LESS", fabgl::VK_LESS },

    { "KEY_GREATER", fabgl::VK_GREATER },
    { "KEY_UNDERSCORE", fabgl::VK_UNDERSCORE },
    { "KEY_DEGREE", fabgl::VK_DEGREE },
    { "KEY_SECTION", fabgl::VK_SECTION },
    { "KEY_TILDE", fabgl::VK_TILDE },
    { "KEY_NEGATION", fabgl::VK_NEGATION },
    { "KEY_LSHIFT", fabgl::VK_LSHIFT },
    { "KEY_RSHIFT", fabgl::VK_RSHIFT },
    { "KEY_LALT", fabgl::VK_LALT },
    { "KEY_RALT", fabgl::VK_RALT },
    { "KEY_LCTRL", fabgl::VK_LCTRL },
    { "KEY_RCTRL", fabgl::VK_RCTRL },

    { "KEY_LGUI", fabgl::VK_LGUI },
    { "KEY_RGUI", fabgl::VK_RGUI },
    { "KEY_ESCAPE", fabgl::VK_ESCAPE },
    { "KEY_PRINTSCREEN", fabgl::VK_PRINTSCREEN },
    { "KEY_SYSREQ", fabgl::VK_SYSREQ },
    { "KEY_INSERT", fabgl::VK_INSERT },
    { "KEY_KP_INSERT", fabgl::VK_KP_INSERT },
    { "KEY_DELETE", fabgl::VK_DELETE },
    { "KEY_KP_DELETE", fabgl::VK_KP_DELETE },
    { "KEY_BACKSPACE", fabgl::VK_BACKSPACE },
    { "KEY_HOME", fabgl::VK_HOME },
    { "KEY_KP_HOME", fabgl::VK_KP_HOME },
    { "KEY_END", fabgl::VK_END },
    { "KEY_KP_END", fabgl::VK_KP_END },
    { "KEY_PAUSE", fabgl::VK_PAUSE },
    { "KEY_BREAK", fabgl::VK_BREAK },

    { "KEY_SCROLLLOCK", fabgl::VK_SCROLLLOCK },
    { "KEY_NUMLOCK", fabgl::VK_NUMLOCK },
    { "KEY_CAPSLOCK", fabgl::VK_CAPSLOCK },
    { "KEY_TAB", fabgl::VK_TAB },
    { "KEY_RETURN", fabgl::VK_RETURN },
    { "KEY_KP_ENTER", fabgl::VK_KP_ENTER },
    { "KEY_APPLICATION", fabgl::VK_APPLICATION },
    { "KEY_PAGEUP", fabgl::VK_PAGEUP },
    { "KEY_KP_PAGEUP", fabgl::VK_KP_PAGEUP },
    { "KEY_PAGEDOWN", fabgl::VK_PAGEDOWN },
    { "KEY_KP_PAGEDOWN", fabgl::VK_KP_PAGEDOWN },
    { "KEY_UP", fabgl::VK_UP },
    { "KEY_KP_UP", fabgl::VK_KP_UP },

    { "KEY_DOWN", fabgl::VK_DOWN },
    { "KEY_KP_DOWN", fabgl::VK_KP_DOWN },
    { "KEY_LEFT", fabgl::VK_LEFT },
    { "KEY_KP_LEFT", fabgl::VK_KP_LEFT },
    { "KEY_RIGHT", fabgl::VK_RIGHT },
    { "KEY_KP_RIGHT", fabgl::VK_KP_RIGHT },
    { "KEY_KP_CENTER", fabgl::VK_KP_CENTER },
    { "KEY_F1", fabgl::VK_F1 },
    { "KEY_F2", fabgl::VK_F2 },
    { "KEY_F3", fabgl::VK_F3 },
    { "KEY_F4", fabgl::VK_F4 },
    { "KEY_F5", fabgl::VK_F5 },
    { "KEY_F6", fabgl::VK_F6 },
    { "KEY_F7", fabgl::VK_F7 },
    { "KEY_F8", fabgl::VK_F8 },
    { "KEY_F9", fabgl::VK_F9 },
    { "KEY_F10", fabgl::VK_F10 },
    { "KEY_F11", fabgl::VK_F11 },
    { "KEY_F12", fabgl::VK_F12 },

    { "KEY_GRAVE_a", fabgl::VK_GRAVE_a },
    { "KEY_GRAVE_e", fabgl::VK_GRAVE_e },
    { "KEY_ACUTE_e", fabgl::VK_ACUTE_e },
    { "KEY_GRAVE_i", fabgl::VK_GRAVE_i },
    { "KEY_GRAVE_o", fabgl::VK_GRAVE_o },
    { "KEY_GRAVE_u", fabgl::VK_GRAVE_u },
    { "KEY_CEDILLA_c", fabgl::VK_CEDILLA_c },
    { "KEY_ESZETT", fabgl::VK_ESZETT },
    { "KEY_UMLAUT_u", fabgl::VK_UMLAUT_u },

    { "KEY_UMLAUT_o", fabgl::VK_UMLAUT_o },
    { "KEY_UMLAUT_a", fabgl::VK_UMLAUT_a },
    { "KEY_CEDILLA_C", fabgl::VK_CEDILLA_C },
    { "KEY_TILDE_n", fabgl::VK_TILDE_n },
    { "KEY_TILDE_N", fabgl::VK_TILDE_N },
    { "KEY_UPPER_a", fabgl::VK_UPPER_a },
    { "KEY_ACUTE_a", fabgl::VK_ACUTE_a },
    { "KEY_ACUTE_i", fabgl::VK_ACUTE_i },
    { "KEY_ACUTE_o", fabgl::VK_ACUTE_o },
    { "KEY_ACUTE_u", fabgl::VK_ACUTE_u },
    { "KEY_UMLAUT_i", fabgl::VK_UMLAUT_i },
    { "KEY_EXCLAIM_INV", fabgl::VK_EXCLAIM_INV },
    { "KEY_QUESTION_INV", fabgl::VK_QUESTION_INV },

    { "KEY_ACUTE_A", fabgl::VK_ACUTE_A },
    { "KEY_ACUTE_E", fabgl::VK_ACUTE_E },
    { "KEY_ACUTE_I", fabgl::VK_ACUTE_I },
    { "KEY_ACUTE_O", fabgl::VK_ACUTE_O },
    { "KEY_ACUTE_U", fabgl::VK_ACUTE_U },
    { "KEY_GRAVE_A", fabgl::VK_GRAVE_A },
    { "KEY_GRAVE_E", fabgl::VK_GRAVE_E },
    { "KEY_GRAVE_I", fabgl::VK_GRAVE_I },
    { "KEY_GRAVE_O", fabgl::VK_GRAVE_O },
    { "KEY_GRAVE_U", fabgl::VK_GRAVE_U },
    { "KEY_INTERPUNCT", fabgl::VK_INTERPUNCT },
    { "KEY_DIAERESIS", fabgl::VK_DIAERESIS },

    { "KEY_UMLAUT_e", fabgl::VK_UMLAUT_e },
    { "KEY_UMLAUT_A", fabgl::VK_UMLAUT_A },
    { "KEY_UMLAUT_E", fabgl::VK_UMLAUT_E },
    { "KEY_UMLAUT_I", fabgl::VK_UMLAUT_I },
    { "KEY_UMLAUT_O", fabgl::VK_UMLAUT_O },
    { "KEY_UMLAUT_U", fabgl::VK_UMLAUT_U },
    { "KEY_CARET_a", fabgl::VK_CARET_a },
    { "KEY_CARET_e", fabgl::VK_CARET_e },
    { "KEY_CARET_i", fabgl::VK_CARET_i },
    { "KEY_CARET_o", fabgl::VK_CARET_o },
    { "KEY_CARET_u", fabgl::VK_CARET_u },
    { "KEY_CARET_A", fabgl::VK_CARET_A },
    { "KEY_CARET_E", fabgl::VK_CARET_E },

    { "KEY_CARET_I", fabgl::VK_CARET_I },
    { "KEY_CARET_O", fabgl::VK_CARET_O },
    { "KEY_CARET_U", fabgl::VK_CARET_U },


    { "KEY_JOY1LEFT", fabgl::VK_JOY1LEFT },
    { "KEY_JOY1RIGHT", fabgl::VK_JOY1RIGHT },
    { "KEY_JOY1UP", fabgl::VK_JOY1UP },
    { "KEY_JOY1DOWN", fabgl::VK_JOY1DOWN },
    { "KEY_JOY1START", fabgl::VK_JOY1START },
    { "KEY_JOY1MODE", fabgl::VK_JOY1MODE },
    { "KEY_JOY1A", fabgl::VK_JOY1A },
    { "KEY_JOY1B", fabgl::VK_JOY1B },
    { "KEY_JOY1C", fabgl::VK_JOY1C },
    { "KEY_JOY1X", fabgl::VK_JOY1X },
    { "KEY_JOY1Y", fabgl::VK_JOY1Y },
    { "KEY_JOY1Z", fabgl::VK_JOY1Z },
    { "KEY_JOY2LEFT", fabgl::VK_JOY2LEFT },
    { "KEY_JOY2RIGHT", fabgl::VK_JOY2RIGHT },
    { "KEY_JOY2UP", fabgl::VK_JOY2UP },
    { "KEY_JOY2DOWN", fabgl::VK_JOY2DOWN },
    { "KEY_JOY2START", fabgl::VK_JOY2START },
    { "KEY_JOY2MODE", fabgl::VK_JOY2MODE },
    { "KEY_JOY2A", fabgl::VK_JOY2A },
    { "KEY_JOY2B", fabgl::VK_JOY2B },
    { "KEY_JOY2C", fabgl::VK_JOY2C },
    { "KEY_JOY2X", fabgl::VK_JOY2X },
    { "KEY_JOY2Y", fabgl::VK_JOY2Y },
    { "KEY_JOY2Z", fabgl::VK_JOY2Z },
    { "KEY_KEMPSTON_RIGHT", fabgl::VK_KEMPSTON_RIGHT },
    { "KEY_KEMPSTON_LEFT", fabgl::VK_KEMPSTON_LEFT },
    { "KEY_KEMPSTON_DOWN", fabgl::VK_KEMPSTON_DOWN },
    { "KEY_KEMPSTON_UP", fabgl::VK_KEMPSTON_UP },
    { "KEY_KEMPSTON_FIRE", fabgl::VK_KEMPSTON_FIRE },
    { "KEY_KEMPSTON_ALTFIRE", fabgl::VK_KEMPSTON_ALTFIRE },
    { "KEY_FULLER_RIGHT", fabgl::VK_FULLER_RIGHT },
    { "KEY_FULLER_LEFT", fabgl::VK_FULLER_LEFT },
    { "KEY_FULLER_DOWN", fabgl::VK_FULLER_DOWN },
    { "KEY_FULLER_UP", fabgl::VK_FULLER_UP },
    { "KEY_FULLER_FIRE", fabgl::VK_FULLER_FIRE },
    { "KEY_VOLUMEUP", fabgl::VK_VOLUMEUP },        /**< Volume UP */
    { "KEY_VOLUMEDOWN", fabgl::VK_VOLUMEDOWN },      /**< Volume DOWN */
    { "KEY_VOLUMEMUTE", fabgl::VK_VOLUMEMUTE },      /**< MUTE */

    { "KEY_ASCII", fabgl::VK_ASCII }

};

// Estructura para almacenar el mapeo
typedef struct {
    uint8_t      scancode[2];       /**< Raw scancode received from the Keyboard device */
    VirtualKey   virtualKey;        /**< Virtual key generated */
    union {
        struct {
            uint8_t ctrl  : 1;      /**< CTRL needs to be applied. */
            uint8_t lalt  : 1;      /**< LEFT-ALT needs to be applied. */
            uint8_t ralt  : 1;      /**< RIGHT-ALT needs to be applied. */
            uint8_t shift : 1;      /**< SHIFT needs to be applied (OR-ed with capslock). */
            uint8_t       : 4;
        };
        uint8_t modifiers;          /**< Access all modifier bits at once. */
    };
} MappingEntry;

typedef struct {
    char magic[8];
    union {
        uint8_t bytes[sizeof(fabgl::KeyboardLayout)];
        fabgl::KeyboardLayout layout;
    };
} kbdlayout_t;

/* ----------------------- */

#define FILL8   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
#define FILL16  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff

static const kbdlayout_t ESPeccyCustomLayout = {
    { 'E', 'S', 'P', 'C', 'Y', 'K', 'B', 'D' },
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16,
    FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL16, FILL8
};

static long unsigned currentScancode = 0;
static long unsigned currentSpecial = 0;
static long unsigned currentVirtual = 0;
static long unsigned currentExtend = 0;
static long unsigned currentExtendJoy = 0;

static void trim(std::string &s) {
    const char* whitespace = " \t";
    size_t start = s.find_first_not_of(whitespace);
    if(start == std::string::npos) { s = ""; return; }
    size_t end = s.find_last_not_of(whitespace);
    s = s.substr(start, end - start + 1);
}

// Tokeniza una cadena separando por espacios y tabuladores, ignorando tokens que sean exactamente "+"
static std::vector<std::string> lyt_tokenize(const std::string &s) {
    std::vector<std::string> tokens;
    size_t pos = 0, found;
    while((found = s.find_first_of(" \t", pos)) != std::string::npos) {
        if(found > pos) {
            std::string token = s.substr(pos, found - pos);
            if(token != "..")
                tokens.push_back(token);
        }
        pos = found + 1;
    }
    if(pos < s.size()){
        std::string token = s.substr(pos);
        if(token != "..")
            tokens.push_back(token);
    }
    return tokens;
}

static int16_t getHex8(const std::string& token) {
    if (token.size() < 3 || token[0] != '0' || (token[1] != 'x' && token[1] != 'X')) return -1;

    char* endptr = nullptr;
    errno = 0;
    unsigned long value = strtoul(token.c_str(), &endptr, 16);

    if (errno != 0 || endptr == token.c_str() || *endptr != '\0' || value > 0xFF) return -1;

    return static_cast<int16_t>(value);
}

static const int lookupVirtualKey(const std::string &token) {
    auto it = keyToVK.find(token);
    if (it != keyToVK.end()) {
        return it->second;
    }
    return -1;
}

static std::string VKToString(VirtualKey vk) {
    for (const auto& pair : keyToVK) {
        if (pair.second == vk) {
            return pair.first;
        }
    }
    return "UNKNOWN";
}

// Parsea el lado izquierdo y llena la estructura MappingEntry.
static bool parseEntry(const std::vector<std::string> &tokens, MappingEntry &entry) {
    // Reinicia la entrada.
    entry.scancode[0] = entry.scancode[1] = 0;
    //entry.ctrl = entry.lalt = entry.ralt = entry.shift = 0;
    entry.modifiers = 0;
    entry.virtualKey = fabgl::VK_NONE;

    std::vector<std::string> hexTokens;
    std::vector<std::string> keyTokens;

    int currentScancode = 0;
    int16_t h;

    VirtualKey vk;

    // Se procesan los tokens
    for (size_t i = 0; i < tokens.size(); ++i) {
        const std::string &tok = tokens[i];
        if ((h = getHex8(tok)) != -1) {
            if (currentScancode > 1) {
                std::printf("Error: too much hexadecimals tokens\n");
                return false;
            }
            entry.scancode[currentScancode++] = h;
        }
        else
        if(tok == "CTRL")
            entry.ctrl = 1;
        else
        if(tok == "SHIFT")
            entry.shift = 1;
        else
        if(tok == "RALT")
            entry.ralt = 1;
        else
        if(tok == "LALT")
            entry.lalt = 1;
        else if ((vk = static_cast<VirtualKey>(lookupVirtualKey(tok))) != -1) {
            entry.virtualKey = vk;
        }
        else {
            std::printf("Error: unknown token: %s\n", tok.c_str());
            return false;
        }
    }

    if (entry.scancode[0] != 0 && ( entry.virtualKey != fabgl::VK_NONE || entry.modifiers )) {
        std::printf("Error: Mezcla de token hexadecimal y modificador/KEY_ en: ");
        for (auto &t : tokens) std::printf("%s ", t.c_str());
        std::printf("\n");
        return false;
    }

    return true;
}


// Función que parsea una línea del archivo.
static void parseLine(fabgl::KeyboardLayout *customLayout, const std::string &line) {
    // Buscar el '='
    size_t pos = line.find('=');
    if(pos == std::string::npos) {
        std::printf("Error: line without '=': %s\n", line.c_str());
        return;
    }

    std::string left = line.substr(0, pos);
    std::string right = line.substr(pos + 1);
    trim(left);
    trim(right);

    std::vector<std::string> leftTokens = lyt_tokenize(left);
    std::vector<std::string> rightTokens = lyt_tokenize(right);

    MappingEntry entryLeft;
    MappingEntry entryRight;

    if ( !parseEntry(leftTokens, entryLeft) || !parseEntry(rightTokens, entryRight) ) return;

    if (entryLeft.scancode[0] && entryLeft.scancode[1]) {
        if (entryLeft.scancode[0] == 0xE0) {
            if (currentExtend < sizeof(customLayout->exScancodeToVK)/sizeof(customLayout->exScancodeToVK[0])) {
                customLayout->exScancodeToVK[currentExtend].scancode = entryLeft.scancode[1];
                customLayout->exScancodeToVK[currentExtend].virtualKey = entryRight.virtualKey;
                currentExtend++;
            } else {
            printf("Extended %02X %02X -> %s ignored too much entries!\n", entryLeft.scancode[0], entryLeft.scancode[1], VKToString(entryRight.virtualKey).c_str());
            }

        } else
        if (entryLeft.scancode[0] == 0xE2) {
            if (currentExtendJoy < sizeof(customLayout->exJoyScancodeToVK)/sizeof(customLayout->exJoyScancodeToVK[0])) {
                customLayout->exJoyScancodeToVK[currentExtendJoy].scancode = entryLeft.scancode[1];
                customLayout->exJoyScancodeToVK[currentExtendJoy].virtualKey = entryRight.virtualKey;
                currentExtendJoy++;
            } else {
                printf("Extended joy %02X %02X -> %s ignored too much entries!\n", entryLeft.scancode[0], entryLeft.scancode[1], VKToString(entryRight.virtualKey).c_str());
            }

        }
        else
            printf("Invalid multiple scancodes\n");

    }
    else
    if ((entryLeft.scancode[0] || entryLeft.virtualKey != fabgl::VK_NONE) && (entryLeft.modifiers || entryRight.modifiers)) {
        if (entryLeft.scancode[0]) {
            // Special key
            if (currentSpecial < sizeof(customLayout->scancodeToVKCombo)/sizeof(customLayout->scancodeToVKCombo[0])) {
                customLayout->scancodeToVKCombo[currentSpecial].scancode = entryLeft.scancode[0];
                customLayout->scancodeToVKCombo[currentSpecial].virtualKey = entryRight.virtualKey;
                customLayout->scancodeToVKCombo[currentSpecial].modifiers = entryRight.modifiers;
                currentSpecial++;
            } else {
                printf("Special %02X -> %s%s%s%s%s ignored too much entries!\n", entryLeft.scancode[0], entryRight.ctrl ? "CTRL " : "", entryRight.lalt ? "LALT " : "", entryRight.ralt ? "RALT " : "", entryRight.shift ? "SHIFT " : "", VKToString(entryRight.virtualKey).c_str());
            }
        } else {
            // Virtual key
            if (currentVirtual < sizeof(customLayout->alternateVK)/sizeof(customLayout->alternateVK[0])) {
                customLayout->alternateVK[currentVirtual].reqVirtualKey = entryLeft.virtualKey;
                customLayout->alternateVK[currentVirtual].modifiers = entryLeft.modifiers;
                customLayout->alternateVK[currentVirtual].virtualKey = entryRight.virtualKey;
                currentVirtual++;
            } else {
                printf("Virtual %s%s%s%s%s -> %s ignored too much entries!\n", entryLeft.ctrl ? "CTRL " : "", entryLeft.lalt ? "LALT " : "", entryLeft.ralt ? "RALT " : "", entryLeft.shift ? "SHIFT " : "", VKToString(entryLeft.virtualKey).c_str(),  VKToString(entryRight.virtualKey).c_str());
            }
        }

    }
    else
    {
        // Normal Key
        if (currentScancode < sizeof(customLayout->scancodeToVK)/sizeof(customLayout->scancodeToVK[0])) {
            customLayout->scancodeToVK[currentScancode].scancode = entryLeft.scancode[0];
            customLayout->scancodeToVK[currentScancode].virtualKey = entryRight.virtualKey;
            currentScancode++;
        } else {
            printf("Normal %02X -> %s ignored too much entries!\n", entryLeft.scancode[0], VKToString(entryRight.virtualKey).c_str());
        }
    }
}

fabgl::KeyboardLayout * KBDLayout::load(FILE *fp) {

    fabgl::KeyboardLayout *customLayout = (fabgl::KeyboardLayout *) calloc(sizeof(fabgl::KeyboardLayout), 1);
    if (!customLayout) return nullptr;

    char buffer[256];

    customLayout->name = nullptr;
    customLayout->desc = nullptr;
    customLayout->inherited = nullptr;

    currentScancode = 0;
    currentSpecial = 0;
    currentVirtual = 0;
    currentExtend = 0;
    currentExtendJoy = 0;

    while(std::fgets(buffer, sizeof(buffer), fp)) {
        size_t len = std::strlen(buffer);
        if(len > 0 && buffer[len - 1] == '\n')
            buffer[len - 1] = '\0';
        char * c = strchr(buffer, ';');
        if (c) *c = '\0';
        std::string line(buffer);
        trim(line);
        if(line.empty()) continue;
        parseLine(customLayout, line);
    }

    // Clean unused entries

    while (currentExtend < sizeof(customLayout->exScancodeToVK)/sizeof(customLayout->exScancodeToVK[0])) {
        customLayout->exScancodeToVK[currentExtend].scancode = 0x00;
        customLayout->exScancodeToVK[currentExtend].virtualKey = fabgl::VK_NONE;
        currentExtend++;
    }

    while (currentExtendJoy < sizeof(customLayout->exJoyScancodeToVK)/sizeof(customLayout->exJoyScancodeToVK[0])) {
        customLayout->exJoyScancodeToVK[currentExtendJoy].scancode = 0x00;
        customLayout->exJoyScancodeToVK[currentExtendJoy].virtualKey = fabgl::VK_NONE;
        currentExtendJoy++;
    }

    while (currentSpecial < sizeof(customLayout->scancodeToVKCombo)/sizeof(customLayout->scancodeToVKCombo[0])) {
        customLayout->scancodeToVKCombo[currentSpecial].scancode = 0x00;
        customLayout->scancodeToVKCombo[currentSpecial].virtualKey = fabgl::VK_NONE;
        customLayout->scancodeToVKCombo[currentSpecial].modifiers = 0x00;
        currentSpecial++;
    }

    while (currentVirtual < sizeof(customLayout->alternateVK)/sizeof(customLayout->alternateVK[0])) {
        customLayout->alternateVK[currentVirtual].reqVirtualKey = fabgl::VK_NONE;
        customLayout->alternateVK[currentVirtual].modifiers = 0x00;
        customLayout->alternateVK[currentVirtual].virtualKey = fabgl::VK_NONE;
        currentVirtual++;
    }

    while (currentScancode < sizeof(customLayout->scancodeToVK)/sizeof(customLayout->scancodeToVK[0])) {
        customLayout->scancodeToVK[currentScancode].scancode = 0x00;
        customLayout->scancodeToVK[currentScancode].virtualKey = fabgl::VK_NONE;
        currentScancode++;
    }

    return customLayout;

}

void KBDLayout::reset() {
    if (!Config::KBDLayoutEnable) {
        ESPeccy::PS2Controller.keyboard()->setLayout(&fabgl::USLayout);
        return;
    }

    uint8_t * p = (uint8_t *)&ESPeccyCustomLayout.layout;

    int i; for (i = 0; i < sizeof(ESPeccyCustomLayout.layout) && p[i] == 0xff; i++); // check if custom layout have data

    if ( i != sizeof(ESPeccyCustomLayout.layout) )  ESPeccy::PS2Controller.keyboard()->setLayout(&ESPeccyCustomLayout.layout);
    else                                            ESPeccy::PS2Controller.keyboard()->setLayout(&fabgl::USLayout);
}

void KBDLayout::dump_layout(const fabgl::KeyboardLayout *customLayout) {

    unsigned long i = 0;

    printf("Normal\n");
    while (i < sizeof(customLayout->scancodeToVK)/sizeof(customLayout->scancodeToVK[0])) {
        printf("    %02X -> %s\n", customLayout->scancodeToVK[i].scancode, VKToString(customLayout->scancodeToVK[i].virtualKey).c_str());
        i++;
    }

    printf("\n");
    i = 0;
    printf("Virtual\n");
    while (i < sizeof(customLayout->alternateVK)/sizeof(customLayout->alternateVK[0])) {
        printf("    %s%s%s%s%s -> %s\n", customLayout->alternateVK[i].ctrl ? "CTRL " : "",
                                         customLayout->alternateVK[i].lalt ? "LALT " : "",
                                         customLayout->alternateVK[i].ralt ? "RALT " : "",
                                         customLayout->alternateVK[i].shift ? "SHIFT " : "",
                                         VKToString(customLayout->alternateVK[i].reqVirtualKey).c_str(),
                                         VKToString(customLayout->alternateVK[i].virtualKey).c_str());
        i++;
    }

    printf("\n");
    i = 0;
    printf("Extended\n");
    while (i < sizeof(customLayout->exScancodeToVK)/sizeof(customLayout->exScancodeToVK[0])) {
        printf("    0xE0 %02X -> %s\n", customLayout->exScancodeToVK[i].scancode, VKToString(customLayout->exScancodeToVK[i].virtualKey).c_str());
        i++;
    }

    printf("\n");
    i = 0;
    printf("Extended joy\n");
    while (i < sizeof(customLayout->exJoyScancodeToVK)/sizeof(customLayout->exJoyScancodeToVK[0])) {
        printf("    0xE2 %02X -> %s\n", customLayout->exJoyScancodeToVK[i].scancode, VKToString(customLayout->exJoyScancodeToVK[i].virtualKey).c_str());
        i++;
    }

    printf("\n");
    i = 0;
    printf("Special\n");
    while (i < sizeof(customLayout->scancodeToVKCombo)/sizeof(customLayout->scancodeToVKCombo[0])) {
        printf("    %02X -> %s%s%s%s%s\n", customLayout->scancodeToVKCombo[i].scancode,
                                           customLayout->scancodeToVKCombo[i].ctrl ? "CTRL " : "",
                                           customLayout->scancodeToVKCombo[i].lalt ? "LALT " : "",
                                           customLayout->scancodeToVKCombo[i].ralt ? "RALT " : "",
                                           customLayout->scancodeToVKCombo[i].shift ? "SHIFT " : "",
                                           VKToString(customLayout->scancodeToVKCombo[i].virtualKey).c_str());
        i++;
    }

}
