/**
 * @file led_strip.c
 * @author Giel Willemsen
 * @brief The API definition for a generic led driver.
 * @version 0.1 2022-09-07 Initial version
 * @date 2022-09-07
 *
 * @copyright
 * MIT License
 *
 * Copyright (c) 2022 Giel Willemsen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <esp_log.h>
#include <driver/rmt.h>
#include "led_strip.h"

#define CALC_COLOR_SIZE(handle) (handle->enable_w_channel ? 4 : 3)
#define BIT_SET(val, bit) (((val) & (1UL << (bit))) == (1UL << (bit)))

typedef struct led_strip
{
    rmt_channel_t channel;
    led_strip_manual_timing_t led_timing;
    led_strip_color_order_t color_order;
    led_strip_color_component_t *pixel_colors;
    led_strip_pixel_index_t led_count;
    bool enable_w_channel;
    bool has_flushed;
} led_strip_t;

static esp_err_t find_empty_channel(rmt_channel_t *channel);
static void mark_channel_free(rmt_channel_t channel);
static void mark_channel_used(rmt_channel_t channel);
static void IRAM_ATTR rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size, size_t wanted_num, size_t *translated_size, size_t *item_num);
static void IRAM_ATTR translate_uint8_to_rmt_samples(rmt_item32_t *items, uint8_t data, led_strip_manual_timing_t timing);
static led_strip_handle_t alloc_led_strip(const led_strip_config_t *config);
static void dealloc_led_strip(led_strip_handle_t);
static led_strip_manual_timing_t map_timing(led_strip_timing_config_t config);
static led_strip_color_t convert_from_rgbw_to_rgb(led_strip_color_t color);
static led_strip_color_t convert_from_rgb_to_rgbw(led_strip_color_t color);
static void set_color_data(led_strip_handle_t handle, led_strip_pixel_index_t index, led_strip_color_t color);

static bool initialized = false;
static bool rmt_channel_used[RMT_CHANNEL_MAX];

extern void led_strip_init(void)
{
    if (initialized)
    {
        return;
    }
    for (int i = 0; i < (int)RMT_CHANNEL_MAX; i++)
    {
        rmt_channel_used[i] = false;
    }
    initialized = true;
}

extern void led_strip_init_config(led_strip_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    config->timing_config.use_manual_timing = false;
    config->timing_config.timing.type = LED_STRIP_TYPE_SK6822;
    config->color_order = LED_STRIP_COLOR_ORDER_RGBW;
    config->gpio_output_num = -1;
    config->led_count = 0;
    config->enable_w_channel = false;
    return;
}

extern esp_err_t led_strip_install(led_strip_handle_t *new_handle, const led_strip_config_t *config)
{
    if (new_handle == NULL || config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    else if (config->led_count == 2)
    {
        return ESP_ERR_NOT_SUPPORTED;
    }
    led_strip_handle_t handle = alloc_led_strip(config);
    if (handle == NULL)
    {
        return ESP_ERR_NO_MEM;
    }
    *new_handle = handle;
    handle->led_timing = map_timing(config->timing_config);
    handle->enable_w_channel = config->enable_w_channel;
    handle->color_order = config->color_order;
    handle->led_count = config->led_count;
    handle->has_flushed = false;

    rmt_channel_t channel = RMT_CHANNEL_MAX;
    esp_err_t err = find_empty_channel(&channel);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        dealloc_led_strip(handle);
        return err;
    }
    mark_channel_used(channel);

    rmt_config_t rmt_channel_config = RMT_DEFAULT_CONFIG_TX(config->gpio_output_num, channel);
    rmt_channel_config.clk_div = LED_STRIP_CLOCK_DIVIDER;
    err = rmt_config(&rmt_channel_config);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        dealloc_led_strip(handle);
        mark_channel_free(channel);
        return err;
    }

    err = rmt_driver_install(rmt_channel_config.channel, 0, 0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        dealloc_led_strip(handle);
        mark_channel_free(channel);
        return err;
    }

    err = rmt_translator_init(rmt_channel_config.channel, rmt_adapter);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        dealloc_led_strip(handle);
        mark_channel_free(channel);
        return err;
    }

    err = rmt_translator_set_context(rmt_channel_config.channel, handle);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        dealloc_led_strip(handle);
        mark_channel_free(channel);
        return err;
    }

    err = rmt_set_tx_loop_mode(rmt_channel_config.channel, false);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        dealloc_led_strip(handle);
        mark_channel_free(channel);
        return err;
    }
    err = rmt_set_tx_intr_en(RMT_CHANNEL_0, false);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        dealloc_led_strip(handle);
        mark_channel_free(channel);
        return err;
    }

    return ESP_OK;
}

extern esp_err_t led_strip_free(led_strip_handle_t handle)
{
    esp_err_t err = rmt_driver_uninstall(handle->channel);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        return err;
    }
    mark_channel_free(handle->channel);
    dealloc_led_strip(handle);
    return ESP_OK;
}

extern esp_err_t led_strip_flush(led_strip_handle_t handle)
{
    bool ready = false;
    esp_err_t err = led_strip_flush_done(handle, &ready);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        return err;
    }
    if (ready == false)
    {
        return ESP_ERR_NOT_FINISHED;
    }

    const size_t data_count = handle->led_count * CALC_COLOR_SIZE(handle);
    err = rmt_write_sample(handle->channel, handle->pixel_colors, data_count, true);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        return err;
    }
    handle->has_flushed = true;
    return ESP_OK;
}

extern esp_err_t led_strip_flush_done(led_strip_handle_t handle, bool *done)
{
    if (handle == NULL || done == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->has_flushed == false)
    {
        *done = true;
        return ESP_OK;
    }
    esp_err_t err = rmt_wait_tx_done(handle->channel, 0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    *done = (err == ESP_OK);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT)
    {
        return err;
    }
    return ESP_OK;
}

extern esp_err_t led_strip_start_flush(led_strip_handle_t handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    bool ready = false;
    esp_err_t err = led_strip_flush_done(handle, &ready);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        return err;
    }
    if (ready == false)
    {
        return ESP_ERR_NOT_FINISHED;
    }

    const size_t data_count = handle->led_count * CALC_COLOR_SIZE(handle);
    err = rmt_write_sample(handle->channel, handle->pixel_colors, data_count, false);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        return err;
    }
    handle->has_flushed = true;
    return ESP_OK;
}

extern esp_err_t led_strip_wait_for_flush_finish(led_strip_handle_t handle)
{
    esp_err_t err = rmt_wait_tx_done(handle->channel, portMAX_DELAY);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

extern esp_err_t led_strip_set_pixel_rgb(led_strip_handle_t handle, led_strip_pixel_index_t index, led_strip_color_component_t r, led_strip_color_component_t g, led_strip_color_component_t b)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    else if (index >= handle->led_count)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    led_strip_color_t color = {
        .r = r,
        .g = g,
        .b = b,
        .w = 0x00,
    };
    if (handle->enable_w_channel)
    {
        color = convert_from_rgb_to_rgbw(color);
    }
    set_color_data(handle, index, color);
    return ESP_OK;
}

extern esp_err_t led_strip_set_pixel_rgbw(led_strip_handle_t handle, led_strip_pixel_index_t index, led_strip_color_component_t r, led_strip_color_component_t g, led_strip_color_component_t b, led_strip_color_component_t w)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    else if (index >= handle->led_count)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    led_strip_color_t color = {
        .r = r,
        .g = g,
        .b = b,
        .w = w,
    };
    if (handle->enable_w_channel == false)
    {
        color = convert_from_rgbw_to_rgb(color);
    }
    set_color_data(handle, index, color);
    return ESP_OK;
}

extern esp_err_t led_strip_fill_rgb(led_strip_handle_t handle, led_strip_color_component_t r, led_strip_color_component_t g, led_strip_color_component_t b)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    led_strip_color_t color = {
        .r = r,
        .g = g,
        .b = b,
        .w = 0x00,
    };
    if (handle->enable_w_channel)
    {
        color = convert_from_rgb_to_rgbw(color);
    }
    for (led_strip_pixel_index_t i = 0; i < handle->led_count; i++)
    {
        set_color_data(handle, i, color);
    }
    return ESP_OK;
}

extern esp_err_t led_strip_fill_rgbw(led_strip_handle_t handle, led_strip_color_component_t r, led_strip_color_component_t g, led_strip_color_component_t b, led_strip_color_component_t w)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    led_strip_color_t color = {
        .r = r,
        .g = g,
        .b = b,
        .w = w,
    };
    if (handle->enable_w_channel == false)
    {
        color = convert_from_rgbw_to_rgb(color);
    }
    for (led_strip_pixel_index_t i = 0; i < handle->led_count; i++)
    {
        set_color_data(handle, i, color);
    }
    return ESP_OK;
}

// Private functions

static esp_err_t find_empty_channel(rmt_channel_t *channel)
{
    if (channel == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < (size_t)RMT_CHANNEL_MAX; i++)
    {
        if (rmt_channel_used[i] == false)
        {
            *channel = (rmt_channel_t)i;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static void mark_channel_free(rmt_channel_t channel)
{
    if (channel != RMT_CHANNEL_MAX)
    {
        rmt_channel_used[(int)channel] = false;
    }
}

static void mark_channel_used(rmt_channel_t channel)
{
    if (channel != RMT_CHANNEL_MAX)
    {
        rmt_channel_used[(int)channel] = true;
    }
}

static void IRAM_ATTR rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size, size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    *translated_size = 0;
    *item_num = 0;
    const uint8_t *raw_data = (const uint8_t *)src;
    void *user_data = NULL;
    if (rmt_translator_get_context(item_num, &user_data) != ESP_OK)
    {
        return; // Just bail out and hope that the driver gets it when translated_size and item_num are zero.
    }
    if (user_data == NULL)
    {
        return; // We need a userdata so we have a handle to get info from.
    }

    led_strip_handle_t handle = (led_strip_handle_t)user_data;
    if (wanted_num < 8)
    {
        // We need at least 8 samples because otherwise we can't convert a whole byte.
        return;
    }

    size_t convertable_bytes = wanted_num / 8; // 1 bits per num (1 period on, 1 period off) and 8 bits per byte.
    if (convertable_bytes > src_size)
    {
        convertable_bytes = src_size;
    }

    size_t rmt_item_offset = 0;
    for (size_t i = 0; i < convertable_bytes; i++)
    {
        translate_uint8_to_rmt_samples(dest + rmt_item_offset, raw_data[i], handle->led_timing);
        rmt_item_offset += 8;
    }

    *translated_size = convertable_bytes;
    *item_num = rmt_item_offset;
}

static void IRAM_ATTR translate_uint8_to_rmt_samples(rmt_item32_t *items, uint8_t data, led_strip_manual_timing_t timing)
{
    for (int i = 0; i < 8; i++)
    {
        int b_index = 7 - i; // MSB first
        if (BIT_SET(data, b_index))
        {
            items[i].duration0 = timing.high_on;
            items[i].duration1 = timing.high_off;
        }
        else
        {
            items[i].duration0 = timing.low_on;
            items[i].duration1 = timing.low_off;
        }
        items[i].level0 = 1;
        items[i].level1 = 0;
    }
}

static led_strip_handle_t alloc_led_strip(const led_strip_config_t *config)
{
    led_strip_handle_t handle = (led_strip_handle_t)heap_caps_calloc(1, sizeof(led_strip_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (handle == NULL)
    {
        return NULL;
    }
    const size_t color_cnt = config->led_count * CALC_COLOR_SIZE(config);
    handle->pixel_colors = (led_strip_color_component_t *)heap_caps_calloc(color_cnt, sizeof(led_strip_color_component_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (handle->pixel_colors == NULL)
    {
        free(handle);
        return NULL;
    }
    return handle;
}

static void dealloc_led_strip(led_strip_handle_t handle)
{
    if (handle != NULL)
    {
        if (handle->pixel_colors != NULL)
        {
            free(handle->pixel_colors);
        }
        free(handle);
    }
}

static led_strip_manual_timing_t map_timing(led_strip_timing_config_t config)
{
    if (config.use_manual_timing)
    {
        return config.timing.manual;
    }
    else
    {
        led_strip_manual_timing_t timing[] = {
            [LED_STRIP_TYPE_SK6822] = {
                .low_on = LED_STRIP_NS_AS_TICKS(300),
                .low_off = LED_STRIP_NS_AS_TICKS(900),
                .high_on = LED_STRIP_NS_AS_TICKS(600),
                .high_off = LED_STRIP_NS_AS_TICKS(600),
                .reset_time = LED_STRIP_US_AS_TICKS(80),
            },
            [LED_STRIP_TYPE_WS281x] = {
                .low_on = LED_STRIP_NS_AS_TICKS(350),
                .low_off = LED_STRIP_NS_AS_TICKS(900),
                .high_on = LED_STRIP_NS_AS_TICKS(900),
                .high_off = LED_STRIP_NS_AS_TICKS(350),
                .reset_time = LED_STRIP_US_AS_TICKS(50),
            },
        };

        switch (config.timing.type)
        {
        case LED_STRIP_TYPE_SK6822:
        case LED_STRIP_TYPE_WS281x:
            return timing[config.timing.type];
        default:
            return timing[LED_STRIP_TYPE_SK6822];
            break;
        }
    }
}

static led_strip_color_t convert_from_rgbw_to_rgb(led_strip_color_t color)
{
    led_strip_color_t col = {
        .r = color.r,
        .g = color.g,
        .b = color.b,
        .w = 0x00,
    };
    return col;
}

static led_strip_color_t convert_from_rgb_to_rgbw(led_strip_color_t color)
{
    led_strip_color_t col = {
        .r = color.r,
        .g = color.g,
        .b = color.b,
        .w = 0x00,
    };
    return col;
}

static void set_color_data(led_strip_handle_t handle, led_strip_pixel_index_t index, led_strip_color_t color)
{
    size_t offset = CALC_COLOR_SIZE(handle) * index;
    size_t r_offset = offset;
    size_t g_offset = offset;
    size_t b_offset = offset;
    size_t w_offset = offset;
    // Weird thing in RMT requires 2nd and third bytes switched.
    switch (handle->color_order)
    {
    case LED_STRIP_COLOR_ORDER_GRBW:
        g_offset = offset;
        r_offset = offset + 2;
        b_offset = offset + 1;
        w_offset = offset + 3;
        break;
    case LED_STRIP_COLOR_ORDER_RGBW:
    default:
        r_offset = offset;
        g_offset = offset + 2;
        b_offset = offset + 1;
        w_offset = offset + 3;
        break;
    }
    if (handle->enable_w_channel)
    {
        handle->pixel_colors[w_offset] = color.w;
    }
    handle->pixel_colors[r_offset] = color.r;
    handle->pixel_colors[g_offset] = color.g;
    handle->pixel_colors[b_offset] = color.b;
}
