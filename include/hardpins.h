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


#ifndef ESPECCY_HARDPINS_H
#define ESPECCY_HARDPINS_H

// Audio Out
#define SPEAKER_PIN 25

// PS/2 (fabGL library) GPIOs
// Allowed GPIOs: GPIO_NUM_0, GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_15, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33
// Not allowed from GPIO_NUM_34 to GPIO_NUM_39
// #define PIN_CLK_PS2PORT1 GPIO_NUM_33
// #define PIN_DAT_PS2PORT1 GPIO_NUM_32
// #define PIN_CLK_PS2PORT2 GPIO_NUM_26
// #define PIN_DAT_PS2PORT2 GPIO_NUM_27

// Storage mode: pins for external SD card (LILYGO TTGO VGA32 Board and ESPectrum Board)
#define PIN_NUM_MISO_LILYGO_ESPECTRUM GPIO_NUM_2
#define PIN_NUM_MOSI_LILYGO_ESPECTRUM GPIO_NUM_12
#define PIN_NUM_CLK_LILYGO_ESPECTRUM  GPIO_NUM_14
#define PIN_NUM_CS_LILYGO_ESPECTRUM   GPIO_NUM_13

// Storage mode: pins for external SD card (Olimex ESP32-SBC-FABGL Board)
#define PIN_NUM_MISO_SBCFABGL GPIO_NUM_35
#define PIN_NUM_MOSI_SBCFABGL GPIO_NUM_12
#define PIN_NUM_CLK_SBCFABGL GPIO_NUM_14
#define PIN_NUM_CS_SBCFABGL GPIO_NUM_13

// VGA Pins (6 bit)
#define RED_PINS_6B 21, 22
#define GRE_PINS_6B 18, 19
#define BLU_PINS_6B  4,  5
#define HSYNC_PIN 23
#define VSYNC_PIN 15

// Shift Register pins
#define SR_CLK 0
#define SR_LOAD 26
#define SR_DATA 27

// keyboard membrane 5 input pins
#define KM_COL_0 3
#define KM_COL_1 34
#define KM_COL_2 35
#define KM_COL_3 36
#define KM_COL_4 39

#endif // ESPECCY_HARDPINS_H
