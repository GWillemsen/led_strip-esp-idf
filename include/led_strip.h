/**
 * @file led_strip.h
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

#pragma once
#ifndef LED_STRIP_H_
#define LED_STRIP_H_

// Macros
#define LED_STRIP_CLOCK_DIVIDER 8
#define LED_STRIP_NS_PER_SECOND 1e9
#define LED_STRIP_NS_PER_TICK (LED_STRIP_NS_PER_SECOND / (APB_CLK_FREQ / LED_STRIP_CLOCK_DIVIDER))

#define LED_STRIP_ROUND_UP(x, align) ((((x) + (align) - 1) / (align)) * (align))
#define LED_STRIP_NS_AS_TICKS(x) (LED_STRIP_ROUND_UP(x, LED_STRIP_NS_PER_TICK) / LED_STRIP_NS_PER_TICK)
#define LED_STRIP_US_AS_TICKS(x) (LED_STRIP_NS_AS_TICKS((x) * 1000))

// Forward declares
typedef struct led_strip led_strip_t;

// Helper typedefs
typedef led_strip_t* led_strip_handle_t;
typedef uint8_t led_strip_color_component_t;
typedef uint16_t led_strip_pixel_index_t;

// Enums
typedef enum led_strip_type {
    LED_STRIP_TYPE_SK6822 = 0x01,  ///< Use the predefined timing for a SK6822 type LED strip.
    LED_STRIP_TYPE_WS281x = 0x03,  ///< Use the predefined timing for a WS281x type LED strip.
} led_strip_type;

typedef enum led_strip_color_order {
    LED_STRIP_COLOR_ORDER_RGBW,
    LED_STRIP_COLOR_ORDER_GRBW,
} led_strip_color_order_t;

// Structs
typedef struct led_strip_color {
    led_strip_color_component_t r;
    led_strip_color_component_t g;
    led_strip_color_component_t b;
    led_strip_color_component_t w;
} led_strip_color_t;

typedef struct led_strip_manual_timing {
    uint16_t low_on : 15;       ///< The time that 0 bit time is ON, unit is in 100 nanoseconds.
    uint16_t low_off : 15;      ///< The time that 0 bit time is OFF, unit is in 100 nanoseconds.
    uint16_t high_on : 15;      ///< The time that 1 bit time is ON, unit is in 100 nanoseconds.
    uint16_t high_off : 15;     ///< The time that 1 bit time is OFF, unit is in 100 nanoseconds.
    uint16_t reset_time : 15;   ///< Time that the reset signal should be OFF, unit is in 100 nanoseconds.
} led_strip_manual_timing_t;

typedef struct led_strip_timing_config {
    union {
        led_strip_type type;                ///< Use a predefined timing type.
        led_strip_manual_timing_t manual;   ///< Use a custom timing type.
    } timing;
    bool use_manual_timing;
} led_strip_timing_config_t;

typedef struct led_strip_config {
    led_strip_timing_config_t timing_config;
    led_strip_color_order_t color_order;
    int gpio_output_num;
    led_strip_pixel_index_t led_count;
    bool enable_w_channel;
} led_strip_config_t;

// Public functions

/**
 * @brief Initialize the library.
 * 
 */
extern void led_strip_init(void);

/**
 * @brief Default initialize the given config. Does not initialize if parameter is NULL.
 * 
 * @param config The configuration to initialize.
 */
extern void led_strip_init_config(led_strip_config_t *config);

/**
 * @brief Create a new instance of a ledstrip with the given configuration.
 * A led strip size of 2 is NOT supported and will result in ESP_ERR_NOT_SUPPORTED.
 * 
 * @param handle The resulting handle of the instance.
 * @param config The configuration to initialize the led strip with..
 * @return esp_err_t The success code for initializing the led strip.
 */
extern esp_err_t led_strip_install(led_strip_handle_t *handle, const led_strip_config_t* config);

/**
 * @brief Deallocate all resources used for the given led strip.
 * 
 * @param handle The led_strip to deallocate.
 * @return esp_err_t The success code for deallocating and freeing the resources used by the led strip.
 */
extern esp_err_t led_strip_free(led_strip_handle_t handle);

/**
 * @brief Sends the current pixel buffer to the LEDs and wait for the transmission to complete.
 * 
 * @param handle The led strip to send the update for.
 * @return esp_err_t The success code for sending the new frame.
 */
extern esp_err_t led_strip_flush(led_strip_handle_t handle);

/**
 * @brief Check if the given led strip is no longer transmitting.
 * 
 * @param handle The led strip's status to check.
 * @param done Indicator if the LED strip is finished or not.
 * @return esp_err_t The success code for retrieving the status information.
 */
extern esp_err_t led_strip_flush_done(led_strip_handle_t handle, bool *done);

/**
 * @brief Starts with sending the new frame to LEDs but doesn't wait for the transmission to finish (use led_strip_flush_done for that).
 * 
 * @param handle The led strip to send the update for.
 * @return esp_err_t The success code for starting the new transmission.
 */
extern esp_err_t led_strip_start_flush(led_strip_handle_t handle);

/**
 * @brief Blocks as long as there is a ongoing transmission for the led strip.
 * 
 * @param handle The led strip to wait for the current transmission to finish.
 * @return esp_err_t The success code for waiting on the tranmission to finish.
 */
extern esp_err_t led_strip_wait_for_flush_finish(led_strip_handle_t handle);

/**
 * @brief Sets the pixel to the given color. If the led strip uses the W channel as well a conversion calculation will be done.
 * 
 * @param handle The led strip to set the pixel in.
 * @param index The index of the pixel to set the color for (0-based).
 * @param r The red component of the color.
 * @param g The green component of the color.
 * @param b The blue component of the color.
 * @return esp_err_t The success code for setting the color. 
 */
extern esp_err_t led_strip_set_pixel_rgb(led_strip_handle_t handle, led_strip_pixel_index_t index, led_strip_color_component_t r, led_strip_color_component_t g, led_strip_color_component_t b);

/**
 * @brief Sets the pixel to the given color. If the led strip only uses RGB the W component will be dropped.
 * 
 * @param handle The led strip to set the pixel in.
 * @param index The index of the pixel to set the color for (0-based).
 * @param r The red component of the color.
 * @param g The green component of the color.
 * @param b The blue component of the color.
 * @param w The white component of the color.
 * @return esp_err_t The success code for setting the color. 
 */
extern esp_err_t led_strip_set_pixel_rgbw(led_strip_handle_t handle, led_strip_pixel_index_t index, led_strip_color_component_t r, led_strip_color_component_t g, led_strip_color_component_t b, led_strip_color_component_t w);

/**
 * @brief Sets all the pixels in the led strip to the given color. If the led strip uses the W channel as well a conversion calculation will be done.
 * 
 * @param handle The led strip to set the pixel in.
 * @param r The red component of the color.
 * @param g The green component of the color.
 * @param b The blue component of the color.
 * @return esp_err_t The success code for setting the color. 
 */
extern esp_err_t led_strip_fill_rgb(led_strip_handle_t handle, led_strip_color_component_t r, led_strip_color_component_t g, led_strip_color_component_t b);

/**
 * @brief Sets all the pixels in the led strip to the given color. If the led strip only uses RGB the W component will be dropped.
 * 
 * @param handle The led strip to set the pixel in.
 * @param r The red component of the color.
 * @param g The green component of the color.
 * @param b The blue component of the color.
 * @param w The white component of the color.
 * @return esp_err_t The success code for setting the color. 
 */
extern esp_err_t led_strip_fill_rgbw(led_strip_handle_t handle, led_strip_color_component_t r, led_strip_color_component_t g, led_strip_color_component_t b, led_strip_color_component_t w);

#endif // LED_STRIP_H_
