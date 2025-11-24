/**
 * Copyright 2025 Viktor Strukov
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
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "pca10059.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_usb.h"

#include "app_timer.h"
#include "nrfx_systick.h"
#include "nrfx_pwm.h"

#define DONGLE_ID 4965
const uint8_t led_list[LEDS_NUMBER] = LEDS_LIST;
const uint8_t btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;
bool     m_counter_active = false;
uint8_t  m_counter = 0;

void led_off(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    nrf_gpio_pin_write(led_list[led_idx], LEDS_ACTIVE_STATE ? 0 : 1);
}

void led_on(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    nrf_gpio_pin_write(led_list[led_idx], LEDS_ACTIVE_STATE ? 1 : 0);
}

void leds_off(void)
{
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        led_off(i);
    }
}

void gpio_output_voltage_setup(void)
{
    // Configure UICR_REGOUT0 register only if it is set to default value.
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
        (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        // System reset is needed to update UICR registers.
        NVIC_SystemReset();
    }
}


void leds_init(void)
{
    uint32_t i;
    gpio_output_voltage_setup();
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_cfg_output(led_list[i]);
    }
    leds_off();
}

void buttons_init(void)
{
    uint32_t i;
    for (i = 0; i < BUTTONS_NUMBER; ++i)
    {
        nrf_gpio_cfg_input(btn_list[i], BUTTON_PULL);
    }
}

void board_init(void)
{
    leds_init();
    buttons_init();
    nrfx_systick_init();
}

void pass_delay_when_button_is_pressed(uint32_t delay_ms, uint32_t discretization_step)
{
    int delay = delay_ms;
    uint8_t count = (delay < 0) ? 2 : 1;

    delay = delay_ms / count;
    for (int i = 0; i < count; i++, delay = delay_ms / count)
    while (delay > 0)
    {
        nrf_delay_ms((delay - discretization_step > 0) ? discretization_step : delay);
        if (!nrf_gpio_pin_read(BUTTON_1)) /* Button pressed, active 0 */
        {
            delay -= discretization_step;
        }
    }
}

void logs_init()
{
    ret_code_t ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.
/* Counter timer. */
// APP_TIMER_DEF(m_timer_0);

static volatile bool ready_flag;            // A flag indicating PWM status.
volatile nrfx_systick_state_t systick_state;

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

void timer_handle(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_counter_active)
    {
        m_counter++;
        NRF_LOG_RAW_INFO("counter = %d\n", m_counter);
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // int dongle_id_digit;
    // int dongle_id;
    // int multiplier;
    ret_code_t err_code;
    // ret_code_t ret;
    uint32_t value;

    /* 1-channel PWM, 1kHz, output on Dongle LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(1000L, BSP_LED_2);

    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);


    // ret = app_timer_create(&m_timer_0, APP_TIMER_MODE_REPEATED, timer_handle);
    // APP_ERROR_CHECK(ret);

    // ret = app_timer_start(m_timer_0, APP_TIMER_TICKS(1000), NULL);
    // APP_ERROR_CHECK(ret);

    while (true)
    {
        for (uint8_t i = 0; i < 40; ++i)
        {
            value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);

            ready_flag = false;
            /* Set the duty cycle - keep trying until PWM is ready... */
            while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);

            /* ... or wait for callback. */
            while (!ready_flag);
            APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, value));
            nrfx_systick_delay_ms(25);
        }
    }

    // /* Toggle LEDs. */
    // while (true)
    // {
    //     if (!nrf_gpio_pin_read(BUTTON_1)) /* Button pressed, active 0 */
    //     {
    //         dongle_id = DONGLE_ID;
    //         multiplier = 1000;
    //         for (int i = 0; i < LEDS_NUMBER; i++)
    //         {
    //             dongle_id_digit = dongle_id / multiplier;
    //             for (int j = 0; j < dongle_id_digit << 1; j++)
    //             {
    //                 NRF_LOG_INFO("LED %d.", i);
    //                 LOG_BACKEND_USB_PROCESS();
    //                 NRF_LOG_PROCESS();
    //                 nrf_gpio_pin_toggle(led_list[i]);
    //                 pass_delay_when_button_is_pressed(500,50);
    //             }
    //             dongle_id -= dongle_id_digit * multiplier;
    //             multiplier /= 10;
    //         }
    //     }
    // }
}

/**
 *@}
 **/
