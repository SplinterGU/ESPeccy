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


#include "Cheat.h"
#include "MemESP.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_heap_caps.h" // Para heap_caps_malloc

// Variables estáticas
std::string CheatMngr::cheatFilename = "";
FILE* CheatMngr::cheatFileFP = nullptr;
Cheat* CheatMngr::cheats = nullptr;
Poke* CheatMngr::pokes = nullptr;
uint16_t CheatMngr::cheatCount = 0;
uint32_t CheatMngr::pokeCount = 0;

static char line[200]; // Buffer para leer líneas

// Función para liberar todos los datos
void CheatMngr::clearData() {
    if (cheats) heap_caps_free(cheats);
    if (pokes) heap_caps_free(pokes);
    cheats = nullptr;
    pokes = nullptr;
    cheatCount = 0;
    pokeCount = 0;
}

// Función para cerrar el archivo y liberar recursos
void CheatMngr::closeCheatFile() {
    clearData();
    if (cheatFileFP) {
        fclose(cheatFileFP);
        cheatFileFP = nullptr;
    }
}

// Función para contar POKEs y CHEATs
static void countCheatsAndPokes(FILE* file, uint16_t& cheatCount, uint32_t& pokeCount) {
    cheatCount = 0;
    pokeCount = 0;

    rewind(file); // Volver al inicio del archivo

    while (fgets(line, sizeof(line), file)) {
        if (line[0] == 'N') cheatCount++;
        else if (line[0] == 'M' || line[0] == 'Z') pokeCount++;
    }
}

// Función para cargar el archivo .pok
bool CheatMngr::loadCheatFile(const std::string& filename) {
    if (cheatFileFP) fclose(cheatFileFP);

    cheatFileFP = fopen(filename.c_str(), "rb");
    if (!cheatFileFP) {
        printf("Error: Could not open file %s\n", filename.c_str());
        return false;
    }

    cheatFilename = filename;
    clearData(); // Limpiar cualquier dato previo

    // 1. Contar cheats y pokes
    countCheatsAndPokes(cheatFileFP, cheatCount, pokeCount);

    // 2. Reservar memoria alineada para cheats y pokes
    cheats = (Cheat*)heap_caps_malloc(cheatCount * sizeof(Cheat), MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);
    pokes = (Poke*)heap_caps_malloc(pokeCount * sizeof(Poke), MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);

    if (!cheats || !pokes) {
        printf("Error: Failed to allocate memory\n");
        closeCheatFile();
        return false;
    }

    // 3. Segunda pasada para cargar datos
    rewind(cheatFileFP);

    uint32_t pokeIdx = 0;
    uint16_t cheatIdx = 0;
    Cheat currentCheat = {};

    while (fgets(line, sizeof(line), cheatFileFP)) {

        if (line[0] == 'N') {
            if (currentCheat.pokeCount > 0) copyCheat(&currentCheat, &cheats[cheatIdx++]);
            currentCheat = {};
            currentCheat.nameOffset = ftell(cheatFileFP) - strlen(line);
            currentCheat.pokeStartIdx = pokeIdx;
        } else if (line[0] == 'M' || line[0] == 'Z') {
            Poke poke;
            uint16_t val;
            sscanf(line, "%*s %d %d %d %d", &poke.bank, &poke.address, &val, &poke.original);
            poke.is_input = (val == 256);
            poke.orig_file = (poke.original != 0);
            poke.orig_mem = false;
            poke.value = (poke.is_input ? poke.original : (uint8_t) val);
            copyPoke(&poke, &pokes[pokeIdx++]);
            if (poke.is_input) currentCheat.inputCount++;
            currentCheat.pokeCount++;
        }
    }

    copyCheat(&currentCheat, &cheats[cheatIdx++]);

    return true;
}

// Cheat

const Cheat CheatMngr::getCheat(int index) {
    if (index < 0 || index >= cheatCount) return {};
    Cheat c;
    copyCheat(&cheats[index], &c);
    return c;
}

const Cheat CheatMngr::toggleCheat(int index) {
    if (index < 0 || index >= cheatCount) return {};
    Cheat cheat;
    copyCheat(&cheats[index], &cheat);
    cheat.enabled = !cheat.enabled;
    copyCheat(&cheat, &cheats[index]);
    return cheat;
}

// Obtener el nombre del cheat
std::string CheatMngr::getCheatName(const Cheat& cheat) {
    if (!cheatFileFP) return "";
    fseek(cheatFileFP, cheat.nameOffset, SEEK_SET);
    if (fgets(line, sizeof(line), cheatFileFP)) {
        char* p = line;
        while (*p) {
            if (*p == '\r' || *p == '\n') *p = 0;
            ++p;
        }
    }
    return std::string(line + 1);
}

std::string CheatMngr::getCheatFilename() {
    return cheatFilename;
}

uint16_t CheatMngr::getCheatCount() {
    return cheatCount;
}

// Poke

const Poke CheatMngr::getPoke(const Cheat& cheat, size_t pokeIndex) {
    if (pokeIndex >= cheat.pokeCount) return {};
    Poke poke;
    copyPoke(&pokes[cheat.pokeStartIdx + pokeIndex], &poke);
    return poke;
}

const Poke CheatMngr::getInputPoke(const Cheat& cheat, size_t inputIndex) {
    size_t count = 0;
    for (int i = 0; i < cheat.pokeCount; ++i) {
        Poke poke;
        copyPoke(&pokes[cheat.pokeStartIdx + i], &poke);
        if (poke.is_input) {
            if (count == inputIndex) return poke;
            count++;
        }
    }
    return {};
}

const Poke CheatMngr::setPokeValue(const Cheat& cheat, size_t pokeIndex, uint8_t value) {
    if (pokeIndex >= cheat.pokeCount) return {};
    Poke poke;
    copyPoke(&pokes[cheat.pokeStartIdx + pokeIndex], &poke);
    poke.value = value;
    copyPoke(&poke, &pokes[cheat.pokeStartIdx + pokeIndex]);
    return poke;
}

const Poke CheatMngr::setPokeOriginal(const Cheat& cheat, size_t pokeIndex, uint8_t value) {
    if (pokeIndex >= cheat.pokeCount) return {};
    Poke poke;
    copyPoke(&pokes[cheat.pokeStartIdx + pokeIndex], &poke);
    poke.original = value;
    copyPoke(&poke, &pokes[cheat.pokeStartIdx + pokeIndex]);
    return poke;
}

const Poke CheatMngr::setPokeOrigMemFetched(const Cheat& cheat, size_t pokeIndex, bool value) {
    if (pokeIndex >= cheat.pokeCount) return {};
    Poke poke;
    copyPoke(&pokes[cheat.pokeStartIdx + pokeIndex], &poke);
    poke.orig_mem = value;
    copyPoke(&poke, &pokes[cheat.pokeStartIdx + pokeIndex]);
    return poke;
}

const Poke CheatMngr::setPokeOrigFileLoaded(const Cheat& cheat, size_t pokeIndex, bool value) {
    if (pokeIndex >= cheat.pokeCount) return {};
    Poke poke;
    copyPoke(&pokes[cheat.pokeStartIdx + pokeIndex], &poke);
    poke.orig_file = value;
    copyPoke(&poke, &pokes[cheat.pokeStartIdx + pokeIndex]);
    return poke;
}

void CheatMngr::clearAllPokeOrigMem() {
    for (size_t i = 0; i < getCheatCount(); ++i) {
        Cheat cheat = getCheat(i);
        for (size_t j = 0; j < cheat.pokeCount; ++j) {
            setPokeOrigMemFetched(cheat, j, false);
        }
    }
}

void CheatMngr::fetchCheatOriginalValuesFromMem() {
    for (size_t i = 0; i < getCheatCount(); ++i) {
        Cheat cheat = getCheat(i);
        for (size_t j = 0; j < cheat.pokeCount; ++j) {
            Poke poke = getPoke(cheat, j);

            // Solo setear si no fue cargado desde archivo y no fue obtenido previamente
            if (!poke.orig_file && !poke.orig_mem) {
                uint8_t originalValue;

                // Obtener el valor original de la memoria
                if (poke.bank & 0x08) {
                    originalValue = MemESP::ramCurrent[poke.address >> 14][poke.address & 0x3fff];
                } else {
                    originalValue = MemESP::ram[poke.bank & 0x07][poke.address];
                }

                // Setear el valor original y marcar `orig_mem`
                setPokeOrigMemFetched(cheat, j, true);
                setPokeOriginal(cheat, j, originalValue);  // Asegura que se guarde el nuevo valor
            }
        }
    }
}

void CheatMngr::applyCheats() {
    for (size_t i = 0; i < getCheatCount(); ++i) {
        Cheat cheat = getCheat(i);
        for (int j = 0; j < cheat.pokeCount; ++j) {
            Poke poke = getPoke(cheat, j);
            if (cheat.enabled) {
                // Apply poke
                if (poke.bank & 0x08) {
                    // Poke address between 16384 and 65535
                    MemESP::ramCurrent[poke.address >> 14][poke.address & 0x3fff] = poke.value;
                } else {
                    // Poke address in bank
                    MemESP::ram[poke.bank & 0x07][poke.address] = poke.value;
                }
            } else {
                // Revert poke
                if (poke.orig_mem || poke.orig_file) {
                    if (poke.bank & 0x08) {
                        // Poke address between 16384 and 65535
                        MemESP::ramCurrent[poke.address >> 14][poke.address & 0x3fff] = poke.original;
                    } else {
                        // Poke address in bank
                        MemESP::ram[poke.bank & 0x07][poke.address] = poke.original;
                    }
                }
            }
        }
    }
}
