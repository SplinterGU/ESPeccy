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


#ifndef CHARMEM32_H
#define CHARMEM32_H

#include <stdint.h>
#include <stddef.h>

class CharMem32 {
public:
    inline CharMem32(uint8_t* ptr) : wordPtr((uint32_t*)ptr) {}

    class Proxy {
    public:
        inline Proxy(uint32_t* wordPtr, size_t byteIndex)
            : wordPtr(wordPtr),
              wordIndex(byteIndex >> 2),
              byteShift((byteIndex & 0x03) << 3) {}

        inline operator uint8_t() const {
            uint32_t word = wordPtr[wordIndex];
            return (uint8_t)((word >> byteShift) & 0xFF);
        }

        inline Proxy& operator=(uint8_t value) {
            uint32_t& word = wordPtr[wordIndex];
            uint32_t mask = 0xFFu << byteShift;
            word = (word & ~mask) | ((uint32_t)value << byteShift);
            return *this;
        }

        inline Proxy& operator=(const Proxy& other) {
            return *this = (uint8_t)other;
        }

    private:
        uint32_t* wordPtr;
        size_t wordIndex;
        uint32_t byteShift;
    };

    inline Proxy operator[](size_t idx) {
        return Proxy(wordPtr, idx);
    }

    inline Proxy operator[](size_t idx) const {
        return Proxy(wordPtr, idx);
    }

    inline int memcmp(size_t offset1, size_t offset2, size_t size) const {
        for (size_t i = 0; i < size; ++i) {
            uint8_t a = (*this)[offset1 + i];
            uint8_t b = (*this)[offset2 + i];
            if (a - b) return a - b;
        }
        return 0;
    }

    inline int memcmp(const char* ptr, size_t offset, size_t size) const {
        for (size_t i = 0; i < size; ++i) {
            uint8_t c1 = (uint8_t)ptr[i];
            uint8_t c2 = (*this)[offset + i];
            if (c1 - c2) return c1 - c2;
        }
        return 0;
    }

    inline int memcmp(size_t offset, const char* ptr, size_t size) const {
        for (size_t i = 0; i < size; ++i) {
            uint8_t c1 = (*this)[offset + i];
            uint8_t c2 = (uint8_t)ptr[i];
            if (c1 - c2) return c1 - c2;
        }
        return 0;
    }

    inline void memmove(size_t offsetTarget, size_t offsetSource, size_t size) {
        size_t i = 0;
        while (i < size &&
               (((offsetTarget + i) & 0x03) != 0 ||
                ((offsetSource + i) & 0x03) != 0 ||
                (((offsetTarget + i) & 0x03) != ((offsetSource + i) & 0x03)))) {
            (*this)[offsetTarget + i] = (*this)[offsetSource + i];
            ++i;
        }
        while (i + 4 <= size) {
            size_t targetIndex = (offsetTarget + i) >> 2;
            size_t sourceIndex = (offsetSource + i) >> 2;
            wordPtr[targetIndex] = wordPtr[sourceIndex];
            i += 4;
        }
        while (i < size) {
            (*this)[offsetTarget + i] = (*this)[offsetSource + i];
            ++i;
        }
    }

    inline void memmove(char* target, size_t offsetSource, size_t size) const {
        for (size_t i = 0; i < size; ++i) {
            target[i] = (*this)[offsetSource + i];
        }
    }

    inline void memmove(size_t offsetTarget, const char* source, size_t size) const {
        for (size_t i = 0; i < size; ++i) {
            (*this)[offsetTarget + i] = (uint8_t)source[i];
        }
    }

    inline void memset(size_t offset, uint8_t value, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            (*this)[offset + i] = value;
        }
    }

    inline void strcpy(size_t offsetTarget, size_t offsetSource, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            uint8_t c = (*this)[offsetSource + i];
            (*this)[offsetTarget + i] = c;
            if (!c) return;
        }
    }

    inline void strcpy(size_t offsetTarget, const uint8_t* ptr, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            uint8_t c = ptr[i];
            (*this)[offsetTarget + i] = c;
            if (!c) return;
        }
    }

    inline void strcpy(uint8_t* ptr, size_t offsetSource, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            uint8_t c = (*this)[offsetSource + i];
            ptr[i] = c;
            if (!c) return;
        }
    }

    inline int strcmp(size_t offset1, size_t offset2) const {
        size_t i = 0;
        while (true) {
            uint8_t c1 = (*this)[offset1 + i];
            uint8_t c2 = (*this)[offset2 + i];
            if (c1 - c2) return c1 - c2;
            if (!c1) break;
            ++i;
        }
        return 0;
    }

    // Comparar memoria a partir de offset1 con el puntero dado.
    inline int strcmp(size_t offset1, uint8_t *ptr) const {
        size_t i = 0;
        while (true) {
            uint8_t c1 = (*this)[offset1 + i];
            uint8_t c2 = ptr[i];
            if (c1 != c2) return c1 - c2;
            if (c1 == 0) break;
            ++i;
        }
        return 0;
    }

    // Comparar memoria a partir de offset1 con el puntero dado (inverso).
    inline int strcmp(uint8_t *ptr, size_t offset1) const {
        size_t i = 0;
        while (true) {
            uint8_t c1 = ptr[i];
            uint8_t c2 = (*this)[offset1 + i];
            if (c1 != c2) return c1 - c2;
            if (c1 == 0) break;
            ++i;
        }
        return 0;
    }

    inline int strncasecmp(size_t offset1, size_t offset2, size_t size) const {
        for (size_t i = 0; i < size; ++i) {
            uint8_t c1 = (*this)[offset1 + i];
            uint8_t c2 = (*this)[offset2 + i];

            if (c1 >= 'A' && c1 <= 'Z') c1 += 'a' - 'A';
            if (c2 >= 'A' && c2 <= 'Z') c2 += 'a' - 'A';

            if (c1 - c2) return c1 - c2;
            if (!c1) break;
        }
        return 0;
    }

    // Comparar con strcasecmp (ignorando mayúsculas/minúsculas) - offset1 y ptr
    inline int strcasecmp(size_t offset1, uint8_t *ptr) const {
        size_t i = 0;
        while (true) {
            uint8_t c1 = (*this)[offset1 + i];
            uint8_t c2 = ptr[i];

            // Convierte ambos caracteres a minúsculas si son mayúsculas
            if (c1 >= 'A' && c1 <= 'Z') c1 += 'a' - 'A';
            if (c2 >= 'A' && c2 <= 'Z') c2 += 'a' - 'A';

            if (c1 != c2) return c1 - c2;
            if (c1 == 0) break;
            ++i;
        }
        return 0;
    }

    // Comparar con strcasecmp (ignorando mayúsculas/minúsculas) - ptr y offset1
    inline int strcasecmp(uint8_t *ptr, size_t offset1) const {
        size_t i = 0;
        while (true) {
            uint8_t c1 = ptr[i];
            uint8_t c2 = (*this)[offset1 + i];

            // Convierte ambos caracteres a minúsculas si son mayúsculas
            if (c1 >= 'A' && c1 <= 'Z') c1 += 'a' - 'A';
            if (c2 >= 'A' && c2 <= 'Z') c2 += 'a' - 'A';

            if (c1 != c2) return c1 - c2;
            if (c1 == 0) break;
            ++i;
        }
        return 0;
    }

    inline size_t strlen(size_t offset) const {
        size_t len = 0;
        while ((*this)[offset + len]) ++len;
        return len;
    }

private:
    uint32_t* wordPtr;
};

#endif // CHARMEM32_H
