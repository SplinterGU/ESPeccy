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


#ifndef REALTAPE_H
#define REALTAPE_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    REALTAPE_AUTO = 0,
    REALTAPE_FORCE_LOAD,
    REALTAPE_FORCE_SAVE
};

// Estructura para inyectar parámetros externos

typedef struct {
    uint8_t *gpio_num;                  // Puntero a Config::realtape_gpio_num
    uint64_t *global_tstates;           // Puntero a CPU::global_tstates
    uint32_t *tstates;                  // Puntero a CPU::tstates
    bool zxkeyb_exists;                 // ZXKeyb::Exists
} RealTapeParams;

extern bool RealTape_enabled;

// Reserva y reasigna el buffer de captura.
void RealTape_realloc_buffers(int audio_freq, int samples_per_frame, uint32_t states_in_frame);

// Inicializa el módulo RealTape (configuración de GPIO y timer).
void RealTape_init(RealTapeParams *params);

// Inicia la captura de datos.
void RealTape_start(void);

// Pausa la captura de datos.
#define RealTape_pause()    RealTape_enabled = false

// Devuelve el nivel de la señal en función de los tstates actuales.
int RealTape_getLevel(void);

// Prepara el frame actual actualizando la posición base de tstates.
void RealTape_prepare_frame(void);

void IRAM_ATTR RealTape_isr_handle(void);

#ifdef __cplusplus
}
#endif

#endif // REALTAPE_H
