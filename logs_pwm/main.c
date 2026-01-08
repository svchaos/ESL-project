/**
 * Copyright 2021 Evgeniy Morozov
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE
*/

/** @file
 *
 * @defgroup usb_logging_example_main main.c
 * @{
 * @ingroup usb_logging_example
 * @brief Example logging over USB (for nrf52840 dongle or DK).
 *
 * This example contains all the necessary code to build logging over the USB.
 *
 * Example inverts LED colours every second and prints a message to the USB log.
 *
 * It is possible to configure USB stack manually by setting
 * LOG_BACKEND_USB_INIT_STACK to 0 or leave USB init to the stack by setting
 * LOG_BACKEND_USB_INIT_STACK to 1.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "nordic_common.h"
#include "pca10059.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_log_backend_usb.h"

#include "app_usbd.h"
#include "app_usbd_serial_num.h"
#include "nrfx_gpiote.h"
#include "nrfx_systick.h"

const uint8_t led_list[LEDS_NUMBER] = LEDS_LIST;
const uint8_t btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;

void led_off(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    nrfx_gpiote_out_set(led_list[led_idx]);
    // nrf_gpio_pin_write(led_list[led_idx], LEDS_ACTIVE_STATE ? 0 : 1);
}

void led_on(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    nrfx_gpiote_out_clear(led_list[led_idx]);
    // nrf_gpio_pin_write(led_list[led_idx], LEDS_ACTIVE_STATE ? 1 : 0);
}

void leds_off(void)
{
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        led_off(i);
    }
}

void leds_init(void)
{
    nrfx_gpiote_out_config_t config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true);
    
    // gpio_output_voltage_setup();
    for (uint8_t i = 0; i < LEDS_NUMBER; ++i)
    {
        nrfx_gpiote_out_init(led_list[i], &config);
        // nrf_gpio_cfg_output(led_list[i]);
    }
    leds_off();
}

void logs_init()
{
    ret_code_t ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void board_init(void)
{
    logs_init();
    nrfx_gpiote_init();
    leds_init();
    // buttons_init();
    nrfx_systick_init();
    // timer_init();
}

void do_pwm_cycle(uint8_t value, uint8_t led_idx)
{

    for (uint8_t j = 0; j < 20 - value / 10; ++j)
    {
        led_on(led_idx);
        nrfx_systick_delay_us(10*value);

        led_off(led_idx);
        nrfx_systick_delay_us(10*(100 - value));
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // uint32_t value;
    uint32_t time_start;
    uint32_t time_finish;
    nrfx_systick_state_t systick_state;
    board_init();

    NRF_LOG_INFO("Starting up the test project with USB logging");

    // (void) nrf_dfu_trigger_usb_init();
    // bsp_board_init(BSP_INIT_LEDS);

    while (true)
    {
        nrfx_systick_get(&systick_state);
        time_start = systick_state.time;
        // for (uint8_t i = 0; i < 200; ++i)
        // {
        //     value = (i < 100) ? i : (200 - i);
        //     (void)value;

        //     do_pwm_cycle(value, 1);
        // }
        nrfx_systick_get(&systick_state);
        time_finish = systick_state.time;
        NRF_LOG_INFO("ticks start %d, stop %d", 
                            time_start, time_finish);

        LOG_BACKEND_USB_PROCESS();
        NRF_LOG_PROCESS();
    }
}

/**
 *@}
 **/