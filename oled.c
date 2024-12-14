
#include "oled.h"
#include <string.h>

gpio_num_t pin_gpio_cs,
           pin_gpio_dc,
           pin_gpio_res,
           pin_spi_mosi,
           pin_spi_clk;

spi_device_handle_t handle_spi_oled;

void oled_init(oled_pin_config_t *pin_config){
    //pin config
    pin_gpio_cs = pin_config->pin_gpio_cs;
    pin_gpio_dc = pin_config->pin_gpio_dc;
    pin_gpio_res = pin_config->pin_gpio_res;
    pin_spi_mosi = pin_config->pin_spi_mosi;
    pin_spi_clk = pin_config->pin_spi_clk;

    //GPIO
    gpio_set_direction(pin_gpio_cs, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_gpio_dc, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_gpio_res, GPIO_MODE_OUTPUT);

    //SPI
    spi_bus_config_t bus_spi = {
        .mosi_io_num = (int)pin_spi_mosi,
        .miso_io_num = -1,
        .sclk_io_num = (int)pin_spi_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = OLED_HEIGHT_PX * OLED_WIDTH_PX, //maybe more allowance be appreciated
    };

    spi_bus_initialize(SPI3_HOST, &bus_spi, SPI_DMA_CH_AUTO);


    spi_device_interface_config_t device_oled = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 3,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .clock_speed_hz = 5000000,//5MHz, SSD1661 max spi freq = 7.6 MHz
        .spics_io_num = pin_gpio_cs,
        .queue_size = 3,
    };


    spi_bus_add_device(SPI3_HOST, &device_oled, &handle_spi_oled);


    //Display
    gpio_set_level(pin_gpio_res, 1);
    gpio_set_level(pin_gpio_res, 0);
    vTaskDelay(20/portTICK_PERIOD_MS);
    gpio_set_level(pin_gpio_res, 1);



    sendCommand((uint8_t)0xAE);

    sendCommand((uint8_t)0xA0); //driver remap and color depth
        sendCommand((uint8_t)0b00110010);
    sendCommand((uint8_t)0xAD);
        sendCommand((uint8_t)0b10001110);

    sendCommand((uint8_t)0xAF);
    vTaskDelay(110/portTICK_PERIOD_MS);

    
}

esp_err_t sendCommand(uint8_t command){

    esp_err_t err;
    gpio_set_level(pin_gpio_dc, LEVEL_COMMAND);

    spi_transaction_t transaction; 
    memset(&transaction, 0, sizeof(transaction));
    transaction.length = sizeof(command)*8,
    transaction.tx_buffer = &command,

    err = spi_device_transmit(handle_spi_oled, &transaction);
    return err;

}

esp_err_t sendDataOneByte(uint8_t data){

    esp_err_t err;
    gpio_set_level(pin_gpio_dc, LEVEL_DATA);

    spi_transaction_t transaction = {
        .length = sizeof(data)*8,
        .tx_buffer = &data,
    };

    err = spi_device_transmit(handle_spi_oled, &transaction);
    return err;

}

esp_err_t sendDataContinu(uint8_t* buf, int length){

    esp_err_t err;
    gpio_set_level(pin_gpio_dc, LEVEL_DATA);

    spi_transaction_t transaction = {
        .length = sizeof(*buf)*8*length,
        .tx_buffer = buf,
    };

    err = spi_device_transmit(handle_spi_oled, &transaction);
    return err;

}