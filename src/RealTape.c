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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <driver/adc.h>
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "RealTape.h"

// GPIO
//      GPIO_NUM_3      Villena default
//      GPIO_NUM_32     Villena USB DAT
//      GPIO_NUM_33     Villena USB CLK
//      GPIO_NUM_39     lily pin board & Villena moded
//      GPIO_NUM_34     lily pin board (alternative)
//      GPIO_NUM_26     lily mouse ps/2

/* Variables estáticas internas al módulo */
bool RealTape_enabled = false;
static volatile uint8_t* __capture_buffer = NULL;
static volatile int __capture_buffer_size = 0;
static double __capture_frame_buffer_size = 0;
static volatile int __sample_index = 0;

static double __realtape_factor = 0;
static int __realtape_basepos = 0;
static uint64_t __global_tstates_base = 0;

static int __audio_freq = 0;
static uint32_t __samples_per_frame = 0;

/* Puntero estático a la configuración/parámetros */
static RealTapeParams *rt_params = NULL;

static uint32_t *_gpio_in = NULL, _gpio_mask = 0;

void IRAM_ATTR RealTape_isr_handle(void) {
    if (RealTape_enabled) {
        //__capture_buffer[__sample_index++] = (uint8_t) gpio_get_level((gpio_num_t)*(rt_params->gpio_num));
        __capture_buffer[__sample_index++] = (*_gpio_in & _gpio_mask) != 0;
        if (__sample_index >= __capture_buffer_size) __sample_index = 0;
    }
}

/* Función para liberar el buffer anterior (si existe) y asignar uno nuevo.

 * (int audio_freq               ESPeccy::Audio_freq[1])
 * (int samples_per_frame        ESPeccy::samplesPerFrame)
 * (uint32_t states_in_frame     CPU::statesInFrame)
 */

void RealTape_realloc_buffers(int audio_freq, int samples_per_frame, uint32_t states_in_frame) {
    bool was_enabled = RealTape_enabled;

    // Se pausa la captura para evitar interrupciones durante la reasignación
    if (was_enabled) {
        RealTape_pause();
    }

    // Liberar el buffer anterior, si existe
    if (__capture_buffer != NULL) {
        heap_caps_free((void *)__capture_buffer);
        __capture_buffer = NULL;
    }

    __audio_freq = audio_freq;

    __capture_frame_buffer_size = (double)samples_per_frame;
    __capture_buffer_size = (int)(__capture_frame_buffer_size * 2);

    __capture_buffer = (volatile uint8_t*) heap_caps_malloc(__capture_buffer_size * sizeof(*__capture_buffer), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (__capture_buffer == NULL) {
        printf("Error al asignar el buffer de captura\n");
        return;
    }

    // Factor de conversión: tamaño del buffer de frame / estados por frame
    __realtape_factor = __capture_frame_buffer_size / ((double) states_in_frame);

    // Reiniciamos el índice de captura
    __sample_index = 0;

    __samples_per_frame = samples_per_frame;

    if (was_enabled) {
        RealTape_start();
    }
}

/* Función de inicialización.
 * Se configura el GPIO y el timer.
 */
void RealTape_init(RealTapeParams *params) {

    if (params) rt_params = params;

    if (!rt_params) return;

    if (RealTape_enabled) {
        RealTape_pause();
    }

    // Configuración forzada del GPIO si no se ha establecido
    if (*(rt_params->gpio_num) == 0) {
        if (rt_params->zxkeyb_exists)
            *(rt_params->gpio_num) = GPIO_NUM_3;
        else
            *(rt_params->gpio_num) = GPIO_NUM_39;
    }

    if (*(rt_params->gpio_num) < 32) {
        _gpio_in = &GPIO.in;
        _gpio_mask = 1 << *(rt_params->gpio_num);
     } else {
        _gpio_in = &GPIO.in1;
        _gpio_mask = 1 << (*(rt_params->gpio_num) - 32);
     }

#if 0
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL << *(rt_params->gpio_num);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
#endif

    gpio_set_direction((gpio_num_t)*(rt_params->gpio_num), GPIO_MODE_INPUT);

    // Si no se usa ZXKeyb o el GPIO no es el definido por GPIO_NUM_3, se arranca la captura
    if (!rt_params->zxkeyb_exists || *(rt_params->gpio_num) != GPIO_NUM_3) {
        RealTape_start();
    }
}

/* Función para iniciar la captura.
 * Se guardan algunos valores base (usando rt_params->global_tstates) y se
 * reanuda la captura.
 */
void RealTape_start(void) {
    if (!RealTape_enabled) {
        RealTape_enabled = true;

        __global_tstates_base = *(rt_params->global_tstates);
        __realtape_basepos = 0;

        // Se posiciona el índice 1 frame por delante
        __sample_index = __samples_per_frame;

        //printf("RealTape STARTED\n");
    }
}

/* Función para obtener el nivel actual de la señal capturada.
 */
int IRAM_ATTR RealTape_getLevel(void) {
    int index = (int)((__realtape_basepos + *(rt_params->tstates)) * __realtape_factor) % __capture_buffer_size;
    return __capture_buffer[index];
}

/* Función para preparar el frame actual.
 * Actualiza la posición base de tstates utilizando el valor global.
 */
void RealTape_prepare_frame(void) {
    __realtape_basepos = *(rt_params->global_tstates) - __global_tstates_base;
}
