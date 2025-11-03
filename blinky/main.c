/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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

#define DONGLE_ID 4965
const uint8_t led_list[LEDS_NUMBER] = LEDS_LIST;
const uint8_t btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;

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

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    int dongle_id_digit;
    int dongle_id;
    int multiplier;
    /* Configure board. */
    board_init();

    /* Toggle LEDs. */
    while (true)
    {
        if (!nrf_gpio_pin_read(BUTTON_1)) /* Button pressed, active 0 */
        {
            dongle_id = DONGLE_ID;
            multiplier = 1000;
            for (int i = 0; i < LEDS_NUMBER; i++)
            {
                dongle_id_digit = dongle_id / multiplier;
                for (int j = 0; j < dongle_id_digit << 1; j++)
                {
                    nrf_gpio_pin_toggle(led_list[i]);
                    pass_delay_when_button_is_pressed(500,50);
                }
                dongle_id -= dongle_id_digit * multiplier;
                multiplier /= 10;
            }
        }
    }
}

/**
 *@}
 **/
