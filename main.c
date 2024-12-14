#include <stdio.h>
#include <math.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled.h"
#include "graphic.h"


void app_main(void)
{
    printf("started\n");
    i2c_master_bus_config_t i2c_mst_config ={
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = 4,
        .sda_io_num = 27,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&i2c_mst_config, &bus_handle);

    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, 0x69, -1));

    i2c_device_config_t dev_cfg ={
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x69,
        .scl_speed_hz = 500000,
    };

    oled_pin_config_t pin_config = {
        .pin_gpio_cs=23,
        .pin_gpio_dc=22,
        .pin_gpio_res=21,
        .pin_spi_clk=18,
        .pin_spi_mosi=19
    };
    oled_init(&pin_config);
    set_background(GRAPHIC_BLUE_256);
    display();




    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    printf("config done\n");
    uint8_t tx_buf[6];
    uint8_t rx_buf[0];

    tx_buf[0]=0x7E;
    tx_buf[1]=0xB6;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, tx_buf, 2, 100));
    vTaskDelay(pdMS_TO_TICKS(100));

    tx_buf[0]=0x00;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, tx_buf, 1, rx_buf, 1, 100));
    printf("i2c_communicated\n");
    printf("%x\n", rx_buf[0]);

    tx_buf[0]=0x7E;
    tx_buf[1]=0b00010001;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, tx_buf, 2, 100));
    vTaskDelay(pdMS_TO_TICKS(100));

    tx_buf[0]= 0x04;
    tx_buf[1]= 0x13;
    tx_buf[2]= 0x14;
    tx_buf[3]= 0x15;
    tx_buf[4]= 0x16;
    tx_buf[5]= 0x17;
    uint8_t rx_databuf[6];
    int16_t acc_i[3], acc_i_ini[3];
    float acc_f[3];


    point_t center;
    point_t holiz_l = {.col=0, .row=32};
    point_t holiz_2 = {.col=95, .row=32};
    point_t vert_1 = {.col=48, .row=0};
    point_t vert_2 = {.col=48, .row=64};

    int dx, dy;


    i2c_master_transmit_receive(dev_handle, &tx_buf[0], 1, &rx_databuf[0], 6, 100);
    i2c_master_transmit_receive(dev_handle, &tx_buf[1], 1, &rx_databuf[1], 1, 100);
    i2c_master_transmit_receive(dev_handle, &tx_buf[2], 1, &rx_databuf[2], 1, 100);
    i2c_master_transmit_receive(dev_handle, &tx_buf[3], 1, &rx_databuf[3], 1, 100);
    i2c_master_transmit_receive(dev_handle, &tx_buf[4], 1, &rx_databuf[4], 1, 100);
    i2c_master_transmit_receive(dev_handle, &tx_buf[5], 1, &rx_databuf[5], 1, 100);

    acc_i_ini[0] = (rx_databuf[1] << 8) + rx_databuf[0];
    acc_i_ini[1] = (rx_databuf[3] << 8) + rx_databuf[2];
    acc_i_ini[2] = (rx_databuf[5] << 8) + rx_databuf[4];



    while(1){
        
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &tx_buf[0], 1, &rx_databuf[0], 6, 100));
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &tx_buf[1], 1, &rx_databuf[1], 1, 100));
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &tx_buf[2], 1, &rx_databuf[2], 1, 100));
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &tx_buf[3], 1, &rx_databuf[3], 1, 100));
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &tx_buf[4], 1, &rx_databuf[4], 1, 100));
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &tx_buf[5], 1, &rx_databuf[5], 1, 100));

/*        printf("rdb[0]: %d, rdb[1]: %d\n", rx_databuf[0], rx_databuf[1]);
        printf("rdb[0]: %d, rdb[1]: %d\n", rx_databuf[2], rx_databuf[3]);
        printf("rdb[0]: %d, rdb[1]: %d\n", rx_databuf[4], rx_databuf[5]);*/
        acc_i[0] = (rx_databuf[1] << 8) + rx_databuf[0]-acc_i_ini[0];
        acc_i[1] = (rx_databuf[3] << 8) + rx_databuf[2]-acc_i_ini[1];
        acc_i[2] = (rx_databuf[5] << 8) + rx_databuf[4];
/*
        acc_f[0] = ((float)acc_i[0])/16383.5;
        acc_f[1] = ((float)acc_i[1])/16383.5;
        acc_f[2] = ((float)acc_i[2])/16383.5;        
        printf("x: %f, y: %f, z: %f\n", acc_f[0], acc_f[1], acc_f[2]);
        printf("tot:%f\n",sqrt(acc_f[0]*acc_f[0]+acc_f[1]*acc_f[1]+acc_f[2]*acc_f[2]));
        printf("\n");*/

        if(acc_i[2] < 14189){
            set_background(GRAPHIC_RED_256);
//            display();
        } else {
            set_background(GRAPHIC_BLUE_256);
//            display();
        }

        dx = (1)*(acc_i[1]*32)/8192;
        dy = (1)*(acc_i[0]*32)/8192;

        center.col = 48 + dx;
        center.row = 32 + dy;
        circle(&center, 5, GRAPHIC_GREEN_256,GRAPHIC_GREEN_256);

        line(&holiz_l, &holiz_2, GRAPHIC_WHITE_256);
        line(&vert_1, &vert_2, GRAPHIC_WHITE_256);

        display();


        vTaskDelay(pdMS_TO_TICKS(10));
    }


}
