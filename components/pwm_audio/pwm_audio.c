/* SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "soc/ledc_struct.h"
#include "pwm_audio.h"

#include "../../../include/RealTape.h"

static const char *TAG = "pwm_audio";

#define PWM_AUDIO_CHECK(a, str, ret_val)                          \
    if (!(a))                                                     \
    {                                                             \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val);                                         \
    }

static const char *PWM_AUDIO_PARAM_ADDR_ERROR = "PWM AUDIO PARAM ADDR ERROR";
static const char *PWM_AUDIO_FRAMERATE_ERROR  = "PWM AUDIO FRAMERATE ERROR";
static const char *PWM_AUDIO_STATUS_ERROR     = "PWM AUDIO STATUS ERROR";
static const char *PWM_AUDIO_ALLOC_ERROR      = "PWM AUDIO ALLOC ERROR";
static const char *PWM_AUDIO_RESOLUTION_ERROR = "PWM AUDIO RESOLUTION ERROR";
static const char *PWM_AUDIO_NOT_INITIALIZED  = "PWM AUDIO Uninitialized";
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
static const char *PWM_AUDIO_TG_NUM_ERROR     = "PWM AUDIO TIMER GROUP NUMBER ERROR";
static const char *PWM_AUDIO_TIMER_NUM_ERROR  = "PWM AUDIO TIMER NUMBER ERROR";
#endif

#define BUFFER_MIN_SIZE     (256UL)
#define SAMPLE_RATE_MAX     (48000)
#define SAMPLE_RATE_MIN     (8000)
#define CHANNEL_LEFT_INDEX  (0)
#define CHANNEL_RIGHT_INDEX (1)
#define CHANNEL_LEFT_MASK   (0x01)
#define CHANNEL_RIGHT_MASK  (0x02)
#define VOLUME_0DB          (16)

#ifndef TIMER_BASE_CLK
#define TIMER_BASE_CLK 80000000
#endif
#define TIMER_DIVIDER  3 // 16

/**
 * Debug Configuration
 **/
#define ISR_DEBUG 0  /**< INDEBUG SWITCH */
#define ISR_DEBUG_IO_MASK 0x8000

typedef struct {
    char *buf;                         /**< Original pointer */
    volatile uint32_t head;            /**< ending pointer */
    volatile uint32_t tail;            /**< Read pointer */
    volatile uint32_t size;            /**< Buffer size */
    volatile uint32_t is_give;         /**< semaphore give flag */
    SemaphoreHandle_t semaphore_rb;    /**< Semaphore for ringbuffer */
} ringbuf_handle_t;

typedef struct {
    pwm_audio_config_t    config;                          /**< pwm audio config struct */
    ledc_channel_config_t ledc_channel[PWM_AUDIO_CH_MAX];  /**< ledc channel config */
    ledc_timer_config_t   ledc_timer;                      /**< ledc timer config  */
    ringbuf_handle_t      *ringbuf;                        /**< audio ringbuffer pointer */
    uint32_t              channel_mask;                    /**< channel gpio mask */
    uint32_t              channel_set_num;                 /**< channel audio set number */
    int32_t               framerate;                       /*!< frame rates in Hz */
    int32_t               bits_per_sample;                 /*!< bits per sample (8, 16, 32) */
    int32_t               volume;                          /*!< the volume(-VOLUME_0DB ~ VOLUME_0DB) */
    pwm_audio_status_t status;
} pwm_audio_data_t;

/**< ledc some register pointer */
static volatile uint32_t *g_ledc_left_conf0_val  = NULL;
static volatile uint32_t *g_ledc_left_conf1_val  = NULL;
static volatile uint32_t *g_ledc_left_duty_val   = NULL;
static volatile uint32_t *g_ledc_right_conf0_val = NULL;
static volatile uint32_t *g_ledc_right_conf1_val = NULL;
static volatile uint32_t *g_ledc_right_duty_val  = NULL;

/**< pwm audio handle pointer */
static pwm_audio_data_t *g_pwm_audio_handle = NULL;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static gptimer_handle_t g_gptimer;
#endif

/**
 * Ringbuffer for pwm audio
 */
static esp_err_t rb_destroy(ringbuf_handle_t *rb)
{
    if (rb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (rb->buf) {
        heap_caps_free(rb->buf);
    }

    if (rb->semaphore_rb) {
        vSemaphoreDelete(rb->semaphore_rb);
    }

    heap_caps_free(rb);
    rb = NULL;
    return ESP_OK;
}

static ringbuf_handle_t *rb_create(uint32_t size)
{
    if (size < (BUFFER_MIN_SIZE << 2)) {
        ESP_LOGE(TAG, "Invalid buffer size, Minimum = %"PRIi32"", (int32_t)(BUFFER_MIN_SIZE << 2));
        return NULL;
    }

    ringbuf_handle_t *rb = NULL;
    char *buf = NULL;

    do {
        bool _success =
            (
                (rb             = heap_caps_malloc(sizeof(ringbuf_handle_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL)) &&
                (buf            = heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL))   &&
                (rb->semaphore_rb   = xSemaphoreCreateBinary())

            );

        if (!_success) {
            break;
        }

        rb->is_give = 0;
        rb->buf = buf;
        rb->head = rb->tail = 0;
        rb->size = size;
        return rb;

    } while (0);

    rb_destroy(rb);
    return NULL;
}

static uint32_t IRAM_ATTR rb_get_count(ringbuf_handle_t *rb)
{
    uint32_t tail = rb->tail;

    if (rb->head >= tail) {
        return (rb->head - tail);
    }

    return (rb->size - (tail - rb->head));
}

static uint32_t IRAM_ATTR rb_get_free(ringbuf_handle_t *rb)
{
    /** < Free a byte to judge the ringbuffer direction */
    return (rb->size - rb_get_count(rb) - 1);
}

static esp_err_t rb_flush(ringbuf_handle_t *rb)
{
    rb->tail = rb->head = 0;
    return ESP_OK;
}

static esp_err_t IRAM_ATTR rb_read_byte(ringbuf_handle_t *rb, uint8_t *outdata)
{
    uint32_t tail = rb->tail;

    if (tail == rb->head) {
        return ESP_FAIL;
    }

    // Send a byte from the buffer
    *outdata = rb->buf[tail];

    // Update tail position
    tail++;

    if (tail == rb->size) {
        tail = 0;
    }

    rb->tail = tail;
    return ESP_OK;
}

static esp_err_t rb_write_byte(ringbuf_handle_t *rb, const uint8_t indata)
{
    // Calculate next head
    uint32_t next_head = rb->head + 1;

    if (next_head == rb->size) {
        next_head = 0;
    }

    if (next_head == rb->tail) {
        return ESP_FAIL;
    }

    // Store data and advance head
    rb->buf[rb->head] = indata;
    rb->head = next_head;
    return ESP_OK;
}

static esp_err_t rb_wait_semaphore(ringbuf_handle_t *rb, TickType_t ticks_to_wait)
{
    rb->is_give = 0; /**< As long as it's written, it's allowed to give semaphore again */

    if (xSemaphoreTake(rb->semaphore_rb, ticks_to_wait) == pdTRUE) {
        return ESP_OK;
    }

    return ESP_FAIL;
}

/*
 * Note:
 * In order to improve efficiency, register is operated directly
 */
static inline void ledc_set_left_duty_fast(uint32_t duty_val)
{
    *g_ledc_left_duty_val = (duty_val) << 4; /* Discard decimal part */
    *g_ledc_left_conf0_val |= 0x00000014;
    *g_ledc_left_conf1_val |= 0x80000000;
}
static inline void ledc_set_right_duty_fast(uint32_t duty_val)
{
    *g_ledc_right_duty_val = (duty_val) << 4; /* Discard decimal part */
    *g_ledc_right_conf0_val |= 0x00000014;
    *g_ledc_right_conf1_val |= 0x80000000;
}


volatile uint32_t pwmcount = 0;

/*
 * Timer group ISR handler
 */

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static bool IRAM_ATTR timer_group_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
#else
static void IRAM_ATTR timer_group_isr(void *para)
#endif
{
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    if (handle == NULL) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        return false;
#else
        return;
#endif
    }

    // pwmcount++;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    /* Clear the interrupt */
    timer_group_clr_intr_status_in_isr(handle->config.tg_num, handle->config.timer_num);
    /* After the alarm has been triggered we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(handle->config.tg_num, handle->config.timer_num);

#endif

    // RealTape
    RealTape_isr_handle();

    static uint8_t wave_h, wave_l;
    static uint16_t value;
    ringbuf_handle_t *rb = handle->ringbuf;

// #if (1==ISR_DEBUG)
//     GPIO.out_w1ts = ISR_DEBUG_IO_MASK;
// #endif

    /**
     * It is believed that the channel configured with GPIO needs to output sound
    */
    // if (handle->channel_mask & CHANNEL_LEFT_MASK) {
    //     if (handle->config.duty_resolution > 8) {
    //         if (rb_get_count(rb) > 1) {
    //             rb_read_byte(rb, &wave_l);
    //             rb_read_byte(rb, &wave_h);
    //             value = ((wave_h << 8) | wave_l);
    //             ledc_set_left_duty_fast(value);/**< set the PWM duty */
    //         }
    //     } else {
            if (ESP_OK == rb_read_byte(rb, &wave_h)) {
                ledc_set_left_duty_fast(wave_h);/**< set the PWM duty */
            }
    //     }
    // }

    /**
     * If two gpios are configured, but the audio data has only one channel, copy the data to the right channel
     * Instead, the right channel data is discarded
    */
    // if (handle->channel_mask & CHANNEL_RIGHT_MASK) {
    //     if (handle->channel_set_num == 1) {
    //         if (handle->config.duty_resolution > 8) {
    //             ledc_set_right_duty_fast(value);/**< set the PWM duty */
    //         } else {
    //             ledc_set_right_duty_fast(wave_h);/**< set the PWM duty */
    //         }
    //     } else {
    //         if (handle->config.duty_resolution > 8) {
    //             if (rb_get_count(rb) > 1) {
    //                 rb_read_byte(rb, &wave_l);
    //                 rb_read_byte(rb, &wave_h);
    //                 value = ((wave_h << 8) | wave_l);
    //                 ledc_set_right_duty_fast(value);/**< set the PWM duty */
    //             }
    //         } else {
    //             if (ESP_OK == rb_read_byte(rb, &wave_h)) {
    //                 ledc_set_right_duty_fast(wave_h);/**< set the PWM duty */
    //             }
    //         }
    //     }
    // } else {
    //     if (handle->channel_set_num == 2) {
    //         /**
    //          * Discard the right channel data only if the right channel is configured but the audio data is stereo
    //          * Read buffer but do nothing
    //          */
    //         if (handle->config.duty_resolution > 8) {
    //             if (rb_get_count(rb) > 1) {
    //                 rb_read_byte(rb, &wave_h);
    //                 rb_read_byte(rb, &wave_h);
    //             }
    //         } else {
    //             rb_read_byte(rb, &wave_h);
    //         }
    //     }
    // }

    /**
     * Send semaphore when buffer free is more than BUFFER_MIN_SIZE
     */
    uint32_t free_size = rb_get_free(rb);
    if (0 == rb->is_give && free_size > BUFFER_MIN_SIZE) {
        /**< The execution time of the following code is 2.71 microsecond */
        rb->is_give = 1; /**< To prevent multiple give semaphores */
        BaseType_t xHigherPriorityTaskWoken;
        xSemaphoreGiveFromISR(rb->semaphore_rb, &xHigherPriorityTaskWoken);

        if (pdFALSE != xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }

// #if (1==ISR_DEBUG)
//     GPIO.out_w1tc = ISR_DEBUG_IO_MASK;
// #endif
// #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
//     return true;
// #else
//     return;
// #endif

}

esp_err_t pwm_audio_get_status(pwm_audio_status_t *status)
{
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(NULL != handle, PWM_AUDIO_NOT_INITIALIZED, ESP_ERR_INVALID_STATE);
    *status = handle->status;
    return ESP_OK;
}

esp_err_t pwm_audio_get_param(int *rate, int *bits, int *ch)
{
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(NULL != handle, PWM_AUDIO_NOT_INITIALIZED, ESP_ERR_INVALID_STATE);

    if (NULL != rate) {
        *rate = handle->framerate;
    }

    if (NULL != bits) {
        *bits = (int)handle->bits_per_sample;
    }

    if (NULL != ch) {
        *ch = handle->channel_set_num;
    }

    return ESP_OK;
}

esp_err_t pwm_audio_init(const pwm_audio_config_t *cfg)
{
    ESP_LOGI(TAG, "PWM Audio Version: %d.%d.%d",PWM_AUDIO_VER_MAJOR, PWM_AUDIO_VER_MINOR, PWM_AUDIO_VER_PATCH);
    esp_err_t res = ESP_OK;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    PWM_AUDIO_CHECK(cfg->tg_num < TIMER_GROUP_MAX, PWM_AUDIO_TG_NUM_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(cfg->timer_num < TIMER_MAX, PWM_AUDIO_TIMER_NUM_ERROR, ESP_ERR_INVALID_ARG);
#endif
    PWM_AUDIO_CHECK(cfg != NULL, PWM_AUDIO_PARAM_ADDR_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(cfg->duty_resolution <= 10 && cfg->duty_resolution >= 8, PWM_AUDIO_RESOLUTION_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(NULL == g_pwm_audio_handle, "Already initiate", ESP_ERR_INVALID_STATE);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_LOGI(TAG, "left io: %d | right io: %d | resolution: %dBIT",
             cfg->gpio_num_left, cfg->gpio_num_right, cfg->duty_resolution);
#else
    ESP_LOGI(TAG, "timer: %d:%d | left io: %d | right io: %d | resolution: %dBIT",
             cfg->tg_num, cfg->timer_num, cfg->gpio_num_left, cfg->gpio_num_right, cfg->duty_resolution);
#endif


    pwm_audio_data_t *handle = NULL;

    handle = heap_caps_malloc(sizeof(pwm_audio_data_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    PWM_AUDIO_CHECK(handle != NULL, PWM_AUDIO_ALLOC_ERROR, ESP_ERR_NO_MEM);
    memset(handle, 0, sizeof(pwm_audio_data_t));

    handle->ringbuf = rb_create(cfg->ringbuf_len);
    PWM_AUDIO_CHECK(handle->ringbuf != NULL, PWM_AUDIO_ALLOC_ERROR, ESP_ERR_NO_MEM);

    handle->config = *cfg;
    g_pwm_audio_handle = handle;
#if (1==ISR_DEBUG)
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ISR_DEBUG_IO_MASK;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
#endif

    /**
     * config ledc to generate pwm
     */
    handle->channel_mask = 0;
    handle->ledc_channel[CHANNEL_LEFT_INDEX].gpio_num = -1;
    handle->ledc_channel[CHANNEL_RIGHT_INDEX].gpio_num = -1;

    if (handle->config.gpio_num_left >= 0) {
        handle->ledc_channel[CHANNEL_LEFT_INDEX].channel = handle->config.ledc_channel_left;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].duty = 0;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].gpio_num = handle->config.gpio_num_left;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].speed_mode = LEDC_HIGH_SPEED_MODE; // LEDC_LOW_SPEED_MODE;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].hpoint = 0;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].timer_sel = handle->config.ledc_timer_sel;
        handle->ledc_channel[CHANNEL_LEFT_INDEX].intr_type = LEDC_INTR_DISABLE;
        res = ledc_channel_config(&handle->ledc_channel[CHANNEL_LEFT_INDEX]);
        PWM_AUDIO_CHECK(ESP_OK == res, "LEDC channel left configuration failed", ESP_ERR_INVALID_ARG);
        handle->channel_mask |= CHANNEL_LEFT_MASK;
    }

    if (handle->config.gpio_num_right >= 0) {
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].channel = handle->config.ledc_channel_right;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].duty = 0;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].gpio_num = handle->config.gpio_num_right;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].speed_mode = LEDC_HIGH_SPEED_MODE; // LEDC_LOW_SPEED_MODE;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].hpoint = 0;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].timer_sel = handle->config.ledc_timer_sel;
        handle->ledc_channel[CHANNEL_RIGHT_INDEX].intr_type = LEDC_INTR_DISABLE;
        res = ledc_channel_config(&handle->ledc_channel[CHANNEL_RIGHT_INDEX]);
        PWM_AUDIO_CHECK(ESP_OK == res, "LEDC channel right configuration failed", ESP_ERR_INVALID_ARG);
        handle->channel_mask |= CHANNEL_RIGHT_MASK;
    }

    PWM_AUDIO_CHECK(0 != handle->channel_mask, "Assign at least one channel gpio", ESP_ERR_INVALID_ARG);

#ifdef CONFIG_IDF_TARGET_ESP32S2
    handle->ledc_timer.clk_cfg = LEDC_USE_APB_CLK;
#endif
    handle->ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE; // LEDC_LOW_SPEED_MODE;
    handle->ledc_timer.duty_resolution = handle->config.duty_resolution;
    handle->ledc_timer.timer_num = handle->config.ledc_timer_sel;
    uint32_t freq = (APB_CLK_FREQ / (1 << handle->ledc_timer.duty_resolution));
    handle->ledc_timer.freq_hz = freq - (freq % 1000); // fixed PWM frequency ,It's a multiple of 1000
    res = ledc_timer_config(&handle->ledc_timer);
    PWM_AUDIO_CHECK(res == ESP_OK, "LEDC timer configuration failed", ESP_ERR_INVALID_ARG);

    /**
     * Get the address of LEDC register to reduce the addressing time
     */
    g_ledc_left_duty_val = &LEDC.channel_group[handle->ledc_timer.speed_mode].\
                           channel[handle->ledc_channel[CHANNEL_LEFT_INDEX].channel].duty.val;
    g_ledc_left_conf0_val = &LEDC.channel_group[handle->ledc_timer.speed_mode].\
                            channel[handle->ledc_channel[CHANNEL_LEFT_INDEX].channel].conf0.val;
    g_ledc_left_conf1_val = &LEDC.channel_group[handle->ledc_timer.speed_mode].\
                            channel[handle->ledc_channel[CHANNEL_LEFT_INDEX].channel].conf1.val;
    g_ledc_right_duty_val = &LEDC.channel_group[handle->ledc_timer.speed_mode].\
                            channel[handle->ledc_channel[CHANNEL_RIGHT_INDEX].channel].duty.val;
    g_ledc_right_conf0_val = &LEDC.channel_group[handle->ledc_timer.speed_mode].\
                             channel[handle->ledc_channel[CHANNEL_RIGHT_INDEX].channel].conf0.val;
    g_ledc_right_conf1_val = &LEDC.channel_group[handle->ledc_timer.speed_mode].\
                             channel[handle->ledc_channel[CHANNEL_RIGHT_INDEX].channel].conf1.val;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_BASE_CLK / TIMER_DIVIDER,
    };

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_group_isr,
    };

    res = gptimer_new_timer(&timer_config, &g_gptimer);
    PWM_AUDIO_CHECK(ESP_OK == res, "gptimer configuration failed", res);
    res = gptimer_register_event_callbacks(g_gptimer, &cbs, NULL);
    PWM_AUDIO_CHECK(ESP_OK == res, "gptimer register event callback failed", res);
#else
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .intr_type = TIMER_INTR_LEVEL,
        .auto_reload = 1,
    };
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
    config.clk_src = TIMER_SRC_CLK_APB;  /* ESP32-S2 specific control bit !!!*/
#endif
    res = timer_init(handle->config.tg_num, handle->config.timer_num, &config);
    PWM_AUDIO_CHECK(ESP_OK == res, "Timer group configuration failed", res);
    res = timer_isr_register(handle->config.tg_num, handle->config.timer_num, timer_group_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    PWM_AUDIO_CHECK(ESP_OK == res, "Timer register event callback failed", res);
#endif
    /**< set a initial parameter */
    res = pwm_audio_set_param(16000, 8, 2);
    PWM_AUDIO_CHECK(ESP_OK == res, "Set parameter failed", ESP_FAIL);

    pwm_audio_set_volume(0);

    handle->status = PWM_AUDIO_STATUS_IDLE;

    return ESP_OK;
}

esp_err_t pwm_audio_set_param(int rate, ledc_timer_bit_t bits, int ch)
{
    esp_err_t res = ESP_OK;

    PWM_AUDIO_CHECK(g_pwm_audio_handle->status != PWM_AUDIO_STATUS_BUSY, PWM_AUDIO_STATUS_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(rate <= SAMPLE_RATE_MAX && rate >= SAMPLE_RATE_MIN, PWM_AUDIO_FRAMERATE_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(bits == 32 || bits == 16 || bits == 8, " Unsupported Bit width, only support 8, 16, 32", ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(ch <= 2 && ch >= 1, "Unsupported channel number, only support mono and stereo", ESP_ERR_INVALID_ARG);

    pwm_audio_data_t *handle = g_pwm_audio_handle;

    handle->framerate = rate;
    handle->bits_per_sample = bits;
    handle->channel_set_num = ch;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gptimer_set_raw_count(g_gptimer, 0x00000000ULL);
    /* Configure the alarm value and the interrupt on alarm. */
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = (TIMER_BASE_CLK / TIMER_DIVIDER) / handle->framerate,
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };

    res = gptimer_set_alarm_action(g_gptimer, &alarm_config);
#else
    /* Timer's counter will initially start from value below. Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(handle->config.tg_num, handle->config.timer_num, 0x00000000ULL);
    /* Configure the alarm value and the interrupt on alarm. */
    res = timer_set_alarm_value(handle->config.tg_num, handle->config.timer_num, ((TIMER_BASE_CLK / TIMER_DIVIDER) / handle->framerate) - 1);
#endif
    PWM_AUDIO_CHECK(ESP_OK == res, "pwm_audio set param failed", res);
    return ESP_OK;
}

esp_err_t pwm_audio_set_sample_rate(int rate)
{
    esp_err_t res;
    PWM_AUDIO_CHECK(NULL != g_pwm_audio_handle, PWM_AUDIO_NOT_INITIALIZED, ESP_ERR_INVALID_STATE);
    // PWM_AUDIO_CHECK(g_pwm_audio_handle->status != PWM_AUDIO_STATUS_BUSY, PWM_AUDIO_STATUS_ERROR, ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(rate <= SAMPLE_RATE_MAX && rate >= SAMPLE_RATE_MIN, PWM_AUDIO_FRAMERATE_ERROR, ESP_ERR_INVALID_ARG);

    pwm_audio_data_t *handle = g_pwm_audio_handle;
    handle->framerate = rate;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = (TIMER_BASE_CLK / TIMER_DIVIDER) / handle->framerate,
        .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    res = gptimer_set_alarm_action(g_gptimer, &alarm_config);
#else
    res = timer_set_alarm_value(handle->config.tg_num, handle->config.timer_num, ((TIMER_BASE_CLK / TIMER_DIVIDER) / handle->framerate) - 1);
#endif
    PWM_AUDIO_CHECK(ESP_OK == res, "pwm_audio set sample rate failed", res);
    return ESP_OK;
}

esp_err_t pwm_audio_set_volume(int8_t volume)
{
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(NULL != handle, PWM_AUDIO_NOT_INITIALIZED, ESP_ERR_INVALID_STATE);
    if (volume < 0) {
        PWM_AUDIO_CHECK(-volume <= VOLUME_0DB, "Volume is too small", ESP_ERR_INVALID_ARG);
    } else {
        PWM_AUDIO_CHECK(volume <= VOLUME_0DB, "Volume is too large", ESP_ERR_INVALID_ARG);
    }

    handle->volume = volume + VOLUME_0DB;
    return ESP_OK;
}

esp_err_t pwm_audio_get_volume(int8_t *volume)
{
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(NULL != handle, PWM_AUDIO_NOT_INITIALIZED, ESP_ERR_INVALID_STATE);
    PWM_AUDIO_CHECK(NULL != volume, PWM_AUDIO_PARAM_ADDR_ERROR, ESP_ERR_INVALID_ARG);
    *volume = handle->volume;
    return ESP_OK;
}

esp_err_t IRAM_ATTR pwm_audio_write(uint8_t *inbuf, size_t inbuf_len, size_t *bytes_written, TickType_t ticks_to_wait)
{
    esp_err_t res = ESP_OK;
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(NULL != handle, PWM_AUDIO_NOT_INITIALIZED, ESP_ERR_INVALID_STATE);
    PWM_AUDIO_CHECK(inbuf != NULL && bytes_written != NULL, "Invalid pointer", ESP_ERR_INVALID_ARG);
    PWM_AUDIO_CHECK(inbuf_len != 0, "Length should not be zero", ESP_ERR_INVALID_ARG);

    *bytes_written = 0;
    ringbuf_handle_t *rb = handle->ringbuf;
    TickType_t start_ticks = xTaskGetTickCount();

    while (inbuf_len) {

        uint32_t free = rb_get_free(rb);

        if (ESP_OK == rb_wait_semaphore(rb, ticks_to_wait)) {

            // uint32_t bytes_can_write = inbuf_len;
            // if (inbuf_len > free) {
            //     bytes_can_write = free;
            // }

            uint32_t bytes_can_write = inbuf_len > free ? free : inbuf_len;

            // bytes_can_write &= 0xfffffffc;/**< Aligned data, bytes_can_write should be an integral multiple of 4 */

            // if (0 == bytes_can_write) {
            //     *bytes_written += inbuf_len;  /**< Discard the last misaligned bytes of data directly */
            //     return ESP_OK;
            // }

            /**< Get the difference between PWM resolution and audio samplewidth */
            // int8_t shift = handle->bits_per_sample - handle->config.duty_resolution;
            uint32_t len = bytes_can_write;

            // switch (handle->bits_per_sample) {
            // case 8: {
                // if (shift < 0) {
                //     /**< When the PWM resolution is greater than 8 bits, the value needs to be expanded */
                //     uint16_t value;
                //     uint8_t temp;
                //     shift = -shift;
                //     len >>= 1;
                //     bytes_can_write >>= 1;

                //     for (size_t i = 0; i < len; i++) {
                //         temp = (inbuf[i] * handle->volume / VOLUME_0DB) + 0x7f; /**< offset */
                //         value = temp << shift;
                //         rb_write_byte(rb, value);
                //         rb_write_byte(rb, value >> 8);
                //     }
                // } else {
                    uint8_t value;
                    for (size_t i = 0; i < len; i++) {
                        value = (inbuf[i] * handle->volume / VOLUME_0DB) /*+ 0x7f*/; /**< offset */
                        rb_write_byte(rb, value);
                    }
                // }
            // }
            // break;

            // case 16: {
            //     len >>= 1;
            //     uint16_t *buf_16b = (uint16_t *)inbuf;
            //     static uint16_t value_16b;
            //     int16_t temp;

            //     if (handle->config.duty_resolution > 8) {
            //         for (size_t i = 0; i < len; i++) {
            //             temp = buf_16b[i];
            //             temp = temp * handle->volume / VOLUME_0DB;
            //             value_16b = temp + 0x7fff; /**< offset */
            //             value_16b >>= shift;
            //             rb_write_byte(rb, value_16b);
            //             rb_write_byte(rb, value_16b >> 8);
            //         }
            //     } else {
            //         /**
            //          * When the PWM resolution is 8 bit, only one byte is transmitted
            //          */
            //         for (size_t i = 0; i < len; i++) {
            //             temp = buf_16b[i];
            //             temp = temp * handle->volume / VOLUME_0DB;
            //             value_16b = temp + 0x7fff; /**< offset */
            //             value_16b >>= shift;
            //             rb_write_byte(rb, value_16b);
            //         }
            //     }
            // }
            // break;

            // case 32: {
            //     len >>= 2;
            //     uint32_t *buf_32b = (uint32_t *)inbuf;
            //     uint32_t value;
            //     int32_t temp;

            //     if (handle->config.duty_resolution > 8) {
            //         for (size_t i = 0; i < len; i++) {
            //             temp = buf_32b[i];
            //             temp = temp * handle->volume / VOLUME_0DB;
            //             value = temp + 0x7fffffff; /**< offset */
            //             value >>= shift;
            //             rb_write_byte(rb, value);
            //             rb_write_byte(rb, value >> 8);
            //         }
            //     } else {
            //         /**
            //          * When the PWM resolution is 8 bit, only one byte is transmitted
            //          */
            //         for (size_t i = 0; i < len; i++) {
            //             temp = buf_32b[i];
            //             temp = temp * handle->volume / VOLUME_0DB;
            //             value = temp + 0x7fffffff; /**< offset */
            //             value >>= shift;
            //             rb_write_byte(rb, value);
            //         }
            //     }
            // }
            // break;

            // default:
            //     break;
            // }

            inbuf += bytes_can_write;
            inbuf_len -= bytes_can_write;
            *bytes_written += bytes_can_write;

        } else {
            res = ESP_FAIL;
        }

        if ((xTaskGetTickCount() - start_ticks) >= ticks_to_wait) {
            return res;
        }
    }

    return res;
}

esp_err_t pwm_audio_start(void)
{
    esp_err_t res;
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(NULL != handle, PWM_AUDIO_NOT_INITIALIZED, ESP_ERR_INVALID_STATE);
    PWM_AUDIO_CHECK(handle->status == PWM_AUDIO_STATUS_IDLE, PWM_AUDIO_STATUS_ERROR, ESP_ERR_INVALID_STATE);
    handle->status = PWM_AUDIO_STATUS_BUSY;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    res = gptimer_enable(g_gptimer);
    PWM_AUDIO_CHECK(res == ESP_OK, "gptimer enable fail", res);
    res = gptimer_start(g_gptimer);
    PWM_AUDIO_CHECK(res == ESP_OK, "gptimer start fail", res);
#else
    res = timer_enable_intr(handle->config.tg_num, handle->config.timer_num);
    PWM_AUDIO_CHECK(res == ESP_OK, "timer enable Interrupt fail", res);
    res = timer_start(handle->config.tg_num, handle->config.timer_num);
    PWM_AUDIO_CHECK(res == ESP_OK, "timer start fail", res);
#endif
    return ESP_OK;
}

esp_err_t pwm_audio_stop(void)
{
    esp_err_t res;
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(NULL != handle, PWM_AUDIO_NOT_INITIALIZED, ESP_ERR_INVALID_STATE);
    PWM_AUDIO_CHECK(handle->status == PWM_AUDIO_STATUS_BUSY, PWM_AUDIO_STATUS_ERROR, ESP_ERR_INVALID_STATE);

    /**< just disable timer ,keep pwm output to reduce switching nosie */
    /**< timer disable interrupt */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    res = gptimer_stop(g_gptimer);
    PWM_AUDIO_CHECK(res == ESP_OK, "gptimer stop failed", res);
    res = gptimer_disable(g_gptimer);
    PWM_AUDIO_CHECK(res == ESP_OK, "gptimer disable failed", res);
#else
    res = timer_pause(handle->config.tg_num, handle->config.timer_num);
    PWM_AUDIO_CHECK(res == ESP_OK, "timer pause failed", res);
    res = timer_disable_intr(handle->config.tg_num, handle->config.timer_num);
    PWM_AUDIO_CHECK(res == ESP_OK, "timer disable failed", res);
#endif
    rb_flush(handle->ringbuf);  /**< flush ringbuf, avoid play noise */
    handle->status = PWM_AUDIO_STATUS_IDLE;
    return ESP_OK;
}

esp_err_t pwm_audio_deinit(void)
{
    esp_err_t res;
    pwm_audio_data_t *handle = g_pwm_audio_handle;
    PWM_AUDIO_CHECK(handle != NULL, PWM_AUDIO_PARAM_ADDR_ERROR, ESP_FAIL);
    handle->status = PWM_AUDIO_STATUS_UN_INIT;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    res = gptimer_del_timer(g_gptimer);
    PWM_AUDIO_CHECK(res == ESP_OK, "gptimer del failed", res);
#else
    res = timer_deinit(handle->config.tg_num, handle->config.timer_num);
    PWM_AUDIO_CHECK(res == ESP_OK, "timer deinit failed", res);
#endif

    for (size_t i = 0; i < PWM_AUDIO_CH_MAX; i++) {
        if (handle->ledc_channel[i].gpio_num >= 0) {
            ledc_stop(handle->ledc_channel[i].speed_mode, handle->ledc_channel[i].channel, 0);
        }
    }

    /**< set the channel gpios input mode */
    for (size_t i = 0; i < PWM_AUDIO_CH_MAX; i++) {
        if (handle->ledc_channel[i].gpio_num >= 0) {
            gpio_set_direction(handle->ledc_channel[i].gpio_num, GPIO_MODE_INPUT);
        }
    }

    rb_destroy(handle->ringbuf);
    heap_caps_free(handle);
    g_pwm_audio_handle = NULL;
    return ESP_OK;
}

uint32_t pwm_audio_rbstats(void)
{

    pwm_audio_data_t *handle = g_pwm_audio_handle;

    return rb_get_count(handle->ringbuf);

}

uint32_t pwm_audio_pwmcount(void)
{

    return pwmcount;

}
