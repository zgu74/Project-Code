/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "string.h"
#include "APDS9960.h"
#include "pio_i2c.h"
#include "hardware/uart.h"


#define POWER_ENABLE 1
#define PROXIMITY_ENABLE 1
#define ALS_ENABLE 1
#define GESTURE_ENABLE 0

#define ALS_GAIN 0
#define ALS_TIME 219

#define INIT_CONFIG (GESTURE_ENABLE << 6u) | (PROXIMITY_ENABLE << 2u) | (ALS_ENABLE << 1u) | POWER_ENABLE

const int addr = 0x39;
const int max_read = 250;

#define PIN_SDA 22
#define PIN_SCL 23
#define A0 29
#define A2 27


PIO pio = pio0;
uint sm = 0;


void APDS9960_init() {
    // use the "handheld device dynamic" optimal setting (see datasheet)
    uint8_t buf[2];

    // send register number followed by its corresponding value
    buf[0] = ENABLE_REG;
    buf[1] = INIT_CONFIG;
    pio_i2c_write_blocking(pio, sm, addr, buf, 2, false);

    buf[0] = ALS_TIME_REG;
    buf[1] = ALS_TIME;
    pio_i2c_write_blocking(pio, sm, addr, buf, 2, false);
}

void read_proximity(int32_t* proximity) {

    uint8_t buf[1];
    uint8_t reg = PDATA_REG;
    pio_i2c_write_blocking(pio, sm, addr, &reg, 1, true);  // true to keep master control of bus
    pio_i2c_read_blocking(pio, sm, addr, buf, 1, false);  // false - finished with bus

    *proximity = buf[0];
}

void read_rgbc(int32_t* r, int32_t* g, int32_t* b, int32_t* c) {

    uint8_t buf[2];
    uint8_t reg = CDATA_REG_L;
    pio_i2c_write_blocking(pio, sm, addr, &reg, 1, true);  // true to keep master control of bus
    pio_i2c_read_blocking(pio, sm, addr, buf, 8, false);  // false - finished with bus

    *c = (buf[1] << 8) | buf[0];
    *r = (buf[3] << 8) | buf[2];
    *g = (buf[5] << 8) | buf[4];
    *b = (buf[7] << 8) | buf[6];
}

    
int main() {
    stdio_init_all();

    uint offset = pio_add_program(pio, &i2c_program);
    i2c_program_init(pio, sm, offset, PIN_SDA, PIN_SCL);

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    
    //gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    //gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    //gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    //gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);


    sleep_ms(5000);

    APDS9960_init();
    printf("Hello, APDS9960! Reading raw data from module...\n");
    int alarm_flag=0;

     // Initialise UART 0
    uart_init(uart0, 9600);
 
    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(28, GPIO_FUNC_UART);
    gpio_set_function(29, GPIO_FUNC_UART);

    gpio_init(A2);
    gpio_set_dir(A2, GPIO_OUT);
    

    while (true) {
        
        int32_t proximity;
        int32_t r,g,b,c;
        char uart0_read;

        read_proximity(&proximity);
        read_rgbc(&r, &g, &b, &c);

        printf("proximity: %d   \n",proximity);
        if (proximity>100)
        {
            alarm_flag=1;
        }
        if(alarm_flag==1)
        {
            uart_puts(uart0, "warning!\n");
            gpio_put(A2, 1);
            uart0_read=uart_getc(uart0);
            if(uart0_read=='s')
                alarm_flag=0;
        }
        else 
        {
            gpio_put(A2, 0);
        }

        // Wait for data to refresh
        //sleep_ms(1000);

    }
}
