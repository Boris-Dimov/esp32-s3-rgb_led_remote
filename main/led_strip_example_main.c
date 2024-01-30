/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "device/usbd.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"
#include "driver/timer.h"
#include "driver/timer_types_legacy.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip_encoder.h"
//#include "led_strip_rmt.h"
//#include "led_strip_types.h"
#include "led_strip.h"
#include "portmacro.h"
#include "soc/clk_tree_defs.h"
#include "soc/soc.h"
#include "tinyusb.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

static led_strip_handle_t led_strip;
static int colour = 0;
static QueueHandle_t queue;
static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;
/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0 9000
#define NEC_LEADING_CODE_DURATION_1 4500
#define NEC_PAYLOAD_ZERO_DURATION_0 560
#define NEC_PAYLOAD_ZERO_DURATION_1 560
#define NEC_PAYLOAD_ONE_DURATION_0 560
#define NEC_PAYLOAD_ONE_DURATION_1 1690
#define NEC_REPEAT_CODE_DURATION_0 9000
#define NEC_REPEAT_CODE_DURATION_1 2250
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200 // Tolerance for parsing RMT symbols

static bool IRAM_ATTR changeColour(void* args)
{
  colour = (colour + 1) % 8;
  BaseType_t hptWoken = pdFALSE;
  xQueueSendFromISR(queue, &colour, &hptWoken);
  return true;
}

static bool recvRmt(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* edata, void* user_data)
{
  BaseType_t hptWoken = pdFALSE;
  QueueHandle_t rmtQueue = (QueueHandle_t)user_data;
  // send the received RMT symbols to the parser task
  xQueueSendFromISR(rmtQueue, edata, &hptWoken);
  // return whether any task is woken up
  return hptWoken == pdTRUE;
}

//---START--- of code taken from example (was too lazy to write a parser)
/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
  return (signal_duration < (spec_duration + EXAMPLE_IR_NEC_DECODE_MARGIN)) &&
         (signal_duration > (spec_duration - EXAMPLE_IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t* rmt_nec_symbols)
{
  return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
         nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t* rmt_nec_symbols)
{
  return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
         nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
static bool nec_parse_frame(rmt_symbol_word_t* rmt_nec_symbols)
{
  rmt_symbol_word_t* cur = rmt_nec_symbols;
  uint16_t address = 0;
  uint16_t command = 0;
  bool valid_leading_code =
      nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) && nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
  if (!valid_leading_code)
  {
    return false;
  }
  cur++;
  for (int i = 0; i < 16; i++)
  {
    if (nec_parse_logic1(cur))
    {
      address |= 1 << i;
    }
    else if (nec_parse_logic0(cur))
    {
      address &= ~(1 << i);
    }
    else
    {
      return false;
    }
    cur++;
  }
  for (int i = 0; i < 16; i++)
  {
    if (nec_parse_logic1(cur))
    {
      command |= 1 << i;
    }
    else if (nec_parse_logic0(cur))
    {
      command &= ~(1 << i);
    }
    else
    {
      return false;
    }
    cur++;
  }
  // save address and command
  s_nec_code_address = address;
  s_nec_code_command = command;
  return true;
}

/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
static bool nec_parse_frame_repeat(rmt_symbol_word_t* rmt_nec_symbols)
{
  return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
         nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}
//---END---

void app_main(void)
{
#define BLINK_GPIO 48
#define RECEIVE_GPIO 39

#define TIMER_DIVIDER (16384)                      //  Hardware timer clock divider
#define TIMER_SCALE (APB_CLK_FREQ / TIMER_DIVIDER) // convert counter value to seconds
  timer_config_t tmc;
  tmc.divider = TIMER_DIVIDER;
  tmc.counter_dir = TIMER_COUNT_UP;
  tmc.counter_en = TIMER_PAUSE;
  tmc.auto_reload = TIMER_AUTORELOAD_EN;
  tmc.alarm_en = TIMER_ALARM_EN;
  tmc.clk_src = TIMER_SRC_CLK_APB;
  tmc.intr_type = TIMER_INTR_LEVEL;

  ESP_ERROR_CHECK(timer_init(TIMER_GROUP_1, TIMER_0, &tmc));
  ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 1 * TIMER_SCALE));
  ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, changeColour, NULL, ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM));

  queue = xQueueCreate(10, sizeof(int));
  if (queue == 0)
  {
    printf("Failed to create queue= %p\n", queue);
  }
  else
  {
    puts("Successfully created queue!\n");
  }

  rmt_rx_channel_config_t rmtRxConf;
  rmtRxConf.clk_src = RMT_CLK_SRC_APB;
  rmtRxConf.gpio_num = RECEIVE_GPIO;
  rmtRxConf.mem_block_symbols = 64;      // 64 words 256 bytes
  rmtRxConf.resolution_hz = 1000 * 1000; // 1 MHz for resolution, is that low, is that high?
  rmtRxConf.intr_priority = 0;           // Low priority
  rmtRxConf.flags.with_dma = false;
  rmtRxConf.flags.invert_in = false;
  rmtRxConf.flags.io_loop_back = false;

  rmt_channel_handle_t rmtRxChannel = NULL;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rmtRxConf, &rmtRxChannel));

  // Following code was taken from examples in docs:
  QueueHandle_t receive_queue = xQueueCreate(10, sizeof(rmt_rx_done_event_data_t));
  rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = recvRmt,
  };
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rmtRxChannel, &cbs, receive_queue));
  // The following timing requirement is based on NEC protocol
  rmt_receive_config_t rcvConfig = {
      .signal_range_min_ns = 1250,     // The shortest duration for NEC signal is 560 µs, 1250 ns <
                                       // 560 µs, valid signal is not treated as noise
      .signal_range_max_ns = 12000000, // The longest duration for NEC signal is 9000 µs, 12000000
                                       // ns > 9000 µs, the receive does not stop early
  };
  rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
  //---END---
  ESP_ERROR_CHECK(rmt_enable(rmtRxChannel));

  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = BLINK_GPIO,             // The GPIO that connected to the LED strip's data line
      .max_leds = 1,                            // The number of LEDs in the strip,
      .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
      .led_model = LED_MODEL_WS2812,            // LED strip model
      .flags.invert_out = false,                // whether to invert the output signal (useful
                                                // when your hardware has a level inverter)
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to
                                         // different power consumption
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = false,           // whether to enable the DMA feature
  };

  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

  timer_start(TIMER_GROUP_1, TIMER_0);
  int recvCol = 0;
  int selectedCol = 0;
  uint8_t intensityR = 0;
  uint8_t intensityG = 0;
  uint8_t intensityB = 0;
  int intensity = 16;
  while (true)
  {
    /* Code for timer interrupts:
    xQueueReceive(queue, &recvCol, 50 * portTICK_PERIOD_MS);
    led_strip_set_pixel(led_strip, 0, (recvCol & 4) * 16, (recvCol & 2) * 32,
                        (recvCol & 1) * 64);
    led_strip_refresh(led_strip);
    puts("Did LED task!\n");
    */
    ESP_ERROR_CHECK(rmt_receive(rmtRxChannel, raw_symbols, sizeof(raw_symbols), &rcvConfig));
    rmt_rx_done_event_data_t rx_data;
    xQueueReceive(receive_queue, &rx_data, 500 * portTICK_PERIOD_MS);
    //---START--- of taken code
    // decode RMT symbols
    switch (rx_data.num_symbols)
    {
    case 34: // NEC normal frame
      if (nec_parse_frame(rx_data.received_symbols))
      {
        printf("Command=%04X\r\n", s_nec_code_command);
      }
      // These only apply to my remote, they're NOT standard (I think):
      if (s_nec_code_command == 0xBA45)
      {
        led_strip_clear(led_strip);
        intensityR = 0;
        intensityG = 0;
        intensityB = 0;
        break;
      }
      switch (s_nec_code_command)
      {
      case 0xF20D:
        selectedCol = 4;
        break;
      case 0xF30C:
        selectedCol = 2;
        break;
      case 0xE718:
        selectedCol = 1;
        break;
      case 0xE916:
        intensityR += (selectedCol == 4) * intensity;
        intensityG += (selectedCol == 2) * intensity;
        intensityB += (selectedCol == 1) * intensity;
        break;
      case 0xE619:
        intensityR -= (selectedCol == 4) * intensity;
        intensityG -= (selectedCol == 2) * intensity;
        intensityB -= (selectedCol == 1) * intensity;
        break;
      default:
        break;
      }
      led_strip_set_pixel(led_strip, 0, intensityR, intensityG, intensityB);
      led_strip_refresh(led_strip);
      break;
    case 2: // NEC repeat frame
      if (nec_parse_frame_repeat(rx_data.received_symbols))
      {
        printf("Address=%04X, Command=%04X, repeat\r\n", s_nec_code_address, s_nec_code_command);
      }
      break;
    default:
      printf("Unknown NEC frame\r\n");
      break;
    }

    //---END---
  }
  led_strip_clear(led_strip);
}
