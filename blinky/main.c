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
#include "nrf_drv_pwm.h"

#include "nrfx_gpiote.h"

#define DONGLE_ID 4965
const uint8_t led_list[LEDS_NUMBER] = LEDS_LIST;
const uint8_t btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;
bool     m_counter_active = false;
uint8_t  m_counter = 0;

/* Counter timer. */
// APP_TIMER_DEF(m_timer_0);
APP_TIMER_DEF(m_timer_1);

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

void button_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (action == NRF_GPIOTE_POLARITY_HITOLO)
    {
        NRF_LOG_RAW_INFO("button pressed");
    }
    else if (action == NRF_GPIOTE_POLARITY_LOTOHI)
    {
        NRF_LOG_RAW_INFO("button released");
    }
    else
    {
        ret_code_t ret;
        if (!nrfx_gpiote_in_is_set(pin))
        {
            ret = app_timer_start(m_timer_1, APP_TIMER_TICKS(500), NULL);
            APP_ERROR_CHECK(ret);
            NRF_LOG_RAW_INFO("\n%d: button toggle, pin 0x%x set\n", app_timer_cnt_get(),pin);
        }
        else
        {
            ret = app_timer_stop(m_timer_1);
            APP_ERROR_CHECK(ret);
            NRF_LOG_RAW_INFO("\n%d: button toggle, pin 0x%x unset\n", app_timer_cnt_get(), pin);
        }
    }
}

void buttons_init(void)
{
    uint8_t i;
    nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    // {
    //     .sense  = NRF_GPIOTE_POLARITY_TOGGLE,
    //     .pull   = NRF_GPIO_PIN_PULLUP,
    //     .is_watcher = false,
    //     .hi_accuracy = false,
    //     .skip_gpio_setup = true
    // };
    config.pull = NRF_GPIO_PIN_PULLUP;
    for (i = 0; i < BUTTONS_NUMBER; ++i)
    {
        nrfx_gpiote_in_init(btn_list[i], &config, button_handler);
        nrfx_gpiote_in_event_enable(btn_list[i], true);
        // nrf_gpio_cfg_input(btn_list[i], BUTTON_PULL);
    }
}


static void timer_handle(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (true)
    {
        m_counter++;
        NRF_LOG_RAW_INFO("\ntimer! counter = %d\n", m_counter);
    }
}


void timer_init(void)
{
    ret_code_t ret;

    app_timer_init();
    ret = app_timer_create(&m_timer_1, APP_TIMER_MODE_SINGLE_SHOT, timer_handle);
    APP_ERROR_CHECK(ret);

 }


void board_init(void)
{
    nrfx_gpiote_init();
    leds_init();
    buttons_init();
    timer_init();
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


static volatile bool ready_flag;            // A flag indicating PWM status.

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
// static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
// static nrf_drv_pwm_t m_pwm2 = NRF_DRV_PWM_INSTANCE(2);

// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
#define USED_PWM(idx) (1UL << idx)
static uint8_t m_used = 0;

static uint16_t const              m_demo1_top  = 2000;
static uint16_t const              m_demo1_step = 4;
static uint16_t                    m_demo1_phase;
static uint8_t                     m_demo1_digit;
static uint8_t                     m_demo1_channel;
static uint16_t                    m_demo1_dongle_id;
static nrf_pwm_values_individual_t m_demo1_seq_values;
static nrf_pwm_sequence_t const    m_demo1_seq =
{
    .values.p_individual = &m_demo1_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_demo1_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

static void demo1_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if ((event_type == NRF_DRV_PWM_EVT_FINISHED) && (!nrf_gpio_pin_read(BUTTON_1)))
    {
        uint8_t channel    = m_demo1_channel;
        bool    down       = m_demo1_phase & 1;
        bool    next_phase = false;

        uint16_t * p_channels = (uint16_t *)&m_demo1_seq_values;
        uint16_t value = p_channels[channel];
        if (down)
        {
            value -= m_demo1_step;
            if (value == 0)
            {
                next_phase = true;
            }
        }
        else
        {
            value += m_demo1_step;
            if (value >= m_demo1_top)
            {
                next_phase = true;
            }
        }
        p_channels[channel] = value;

        if (next_phase)
        {
            if (++m_demo1_phase >= 2 * m_demo1_digit)
            {
                m_demo1_phase = 0;
                m_demo1_channel++;
                m_demo1_channel = m_demo1_channel % NRF_PWM_CHANNEL_COUNT;
                m_demo1_dongle_id   = DONGLE_ID;
                for (int i = 0; i < 3 - m_demo1_channel; i++)
                    m_demo1_dongle_id = m_demo1_dongle_id / 10;
                m_demo1_digit   = m_demo1_dongle_id % 10;
            }
        }
    }
}
static void demo1(void)
{
    NRF_LOG_INFO("Demo 1");

    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            BSP_LED_0 | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            BSP_LED_1 | NRF_DRV_PWM_PIN_INVERTED, // channel 1
            BSP_LED_2 | NRF_DRV_PWM_PIN_INVERTED, // channel 2
            BSP_LED_3 | NRF_DRV_PWM_PIN_INVERTED, // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_2MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = m_demo1_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, demo1_handler));
    m_used |= USED_PWM(0);

    m_demo1_seq_values.channel_0 = 0;
    m_demo1_seq_values.channel_1 = 0;
    m_demo1_seq_values.channel_2 = 0;
    m_demo1_seq_values.channel_3 = 0;
    m_demo1_phase                = 0;
    m_demo1_channel     = 0;
    m_demo1_dongle_id   = DONGLE_ID;
    for (int i = 0; i < 3 - m_demo1_channel; i++)
        m_demo1_dongle_id = m_demo1_dongle_id / 10;
    m_demo1_digit   = m_demo1_dongle_id % 10;

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 1,
                                      NRF_DRV_PWM_FLAG_LOOP);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // int dongle_id_digit;
    // int dongle_id;
    // int multiplier;
    // ret_code_t err_code;
    // ret_code_t ret;
    // uint32_t value;

    // ret = app_timer_create(&m_timer_1, APP_TIMER_MODE_REPEATED, timer_handle);
    // APP_ERROR_CHECK(ret);

    // ret = app_timer_start(m_timer_1, APP_TIMER_TICKS(1000), NULL);
    // APP_ERROR_CHECK(ret);

    logs_init();
    NRF_LOG_INFO("Workshop4 sample started.");
    LOG_BACKEND_USB_PROCESS();
    NRF_LOG_PROCESS();
    board_init();
    demo1();

    // while (true)
    // {
    //     for (uint8_t i = 0; i < 40; ++i)
    //     {
    //         value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);

    //         ready_flag = false;
    //         /* Set the duty cycle - keep trying until PWM is ready... */
    //         while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);

    //         /* ... or wait for callback. */
    //         while (!ready_flag);
    //         APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, value));
    //         nrfx_systick_delay_ms(25);
    //     }
    // }

    // /* Toggle LEDs. */
    while (true)
    {
    //     if (!nrf_gpio_pin_read(BUTTON_1)) /* Button pressed, active 0 */
    //     {
    //         dongle_id = DONGLE_ID;
    //         multiplier = 1000;
            for (int i = 0; i < LEDS_NUMBER; i++)
            {
    //             dongle_id_digit = dongle_id / multiplier;
    //             for (int j = 0; j < dongle_id_digit << 1; j++)
    //             {
                    // NRF_LOG_INFO("test %i", i);
                    LOG_BACKEND_USB_PROCESS();
                    NRF_LOG_PROCESS();
                    // nrf_delay_us(500);
    //                 nrf_gpio_pin_toggle(led_list[i]);
    //                 pass_delay_when_button_is_pressed(500,50);
    //             }
    //             dongle_id -= dongle_id_digit * multiplier;
    //             multiplier /= 10;
    //         }
        }
    }
}

/**
 *@}
 **/
