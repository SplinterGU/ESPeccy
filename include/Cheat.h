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


#ifndef CHEAT_H
#define CHEAT_H

#include <string>
#include <vector>
#include <cstdint>

// Estructura para almacenar un POKE
#pragma pack(push, 1)
typedef struct Poke {
    uint8_t bank;               // Banco
    uint16_t address;           // Dirección
    uint8_t value;              // Valor a modificar (POKE)
    uint8_t original;           // Valor original
    union {
        struct {
            bool is_input : 1;  // Si el valor debe ser ingresado por el usuario
            bool orig_mem : 1;  // Indica si el valor original fue recuperado de la memoria
            bool orig_file : 1; // Indica si el valor original provino del archivo .pok
            uint8_t : 5;        // Bits sin usar para completar el byte
        };
        uint8_t flags;          // Acceso directo a los flags
    };
    uint8_t padding[2];         // Relleno manual para alinear a 8 bytes
};
#pragma pack(pop)

// Estructura para almacenar un Cheat
#pragma pack(push, 1)
typedef struct Cheat {
    uint32_t nameOffset;        // Offset del nombre (4 u 8 bytes según la plataforma)
    uint16_t pokeStartIdx;      // Índice de inicio de los POKEs en memoria alineada
    uint16_t pokeCount;         // Cantidad de POKEs asociados
    bool enabled;               // Cheat activado o no
    uint8_t inputCount;         // Cantidad de entradas necesarias
    uint8_t padding[2];         // Relleno para alinear a 32 bits
};
#pragma pack(pop)


// Clase para gestionar los Cheats
class CheatMngr {
public:
    static bool loadCheatFile(const std::string& filename);
    static void clearData();
    static void closeCheatFile();

    static const Cheat getCheat(int index);
    static const Cheat toggleCheat(int index);
    static std::string getCheatName(const Cheat& cheat);
    static std::string getCheatFilename();
    static uint16_t getCheatCount();
    static const Poke getPoke(const Cheat& cheat, size_t pokeIndex);
    static const Poke getInputPoke(const Cheat& cheat, size_t inputIndex);
    static const Poke setPokeValue(const Cheat& cheat, size_t pokeIndex, uint8_t value);

    static const Poke setPokeOriginal(const Cheat& cheat, size_t pokeIndex, uint8_t value);
    static const Poke setPokeOrigMemFetched(const Cheat& cheat, size_t pokeIndex, bool value);
    static const Poke setPokeOrigFileLoaded(const Cheat& cheat, size_t pokeIndex, bool value);
    static void clearAllPokeOrigMem();
    static void fetchCheatOriginalValuesFromMem();
    static void applyCheats();

private:
    static std::string cheatFilename;
    static FILE* cheatFileFP;

    static Cheat* cheats;         // Array de Cheats en memoria alineada
    static Poke* pokes;           // Array de POKEs en memoria alineada
    static uint16_t cheatCount;   // Total de Cheats
    static uint32_t pokeCount;    // Total de POKEs

    static inline void copyPoke(const Poke* src, Poke* dest) {
        const uint32_t* s = reinterpret_cast<const uint32_t*>(src);
        uint32_t* d = reinterpret_cast<uint32_t*>(dest);

        *d++ = *s++;  // Copia los primeros 4 bytes
        *d++ = *s++;  // Copia los segundos 4 bytes
    }

    static inline void copyCheat(const Cheat* src, Cheat* dest) {
        const uint32_t* s = reinterpret_cast<const uint32_t*>(src);
        uint32_t* d = reinterpret_cast<uint32_t*>(dest);

        *d++ = *s++;  // Copia los primeros 4 bytes (nameOffset)
        *d++ = *s++;  // Copia los siguientes 4 bytes (pokeStartIdx y pokeCount)
        *d++ = *s++;  // Copia los últimos 4 bytes (enabled, inputCount, padding)
    }

};

#endif // CheatMngr_H
