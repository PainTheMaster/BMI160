#ifndef _OLED_H_
#define _OLED_H_

#include <stdio.h>
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/task.h"



#define LEVEL_COMMAND 0
#define LEVEL_DATA 1

#define OLED_HEIGHT_PX 64
#define OLED_WIDTH_PX 96

typedef struct oled_pin_config_struct {
    gpio_num_t pin_gpio_cs;
    gpio_num_t pin_gpio_dc;
    gpio_num_t pin_gpio_res;
    gpio_num_t pin_spi_mosi;
    gpio_num_t pin_spi_clk;

} oled_pin_config_t;

extern void oled_init(oled_pin_config_t *pin_config);
extern esp_err_t sendCommand(uint8_t command);
extern esp_err_t sendDataOneByte(uint8_t data);
extern esp_err_t sendDataContinu(uint8_t *buf, int length);

#endif //_OLED_H_
