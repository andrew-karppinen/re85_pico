
/*
Ethanol conversion software with cold start detection for rasperry pi pico
Using BMP180 temperature sensor

Using ADC to read potentiometer for base fuel factor on gpio pin 26 (ADC0)

Created by AK
 */




#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17
#define BMP180_ADDR 0x77

#define  ERROR_LED_PIN 28

int16_t MC, MD;
uint16_t AC5, AC6;

#define  BASE_TEMPERATURE_COLD_START 45
#define MAX_FACTOR 170

#define DEFAULT_TEMPRATURE_CELSIUS 15 // if sensor fails

struct Injector {
    bool open;
    int input_gpio_pin; //detect when input is pulled to ground
    int output_gpio_pin; //mosfet control
    uint64_t closed_time;
    uint64_t open_time;
    uint64_t delay_time;
};


void init_ports() {
    stdio_init_all();
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0); // ADC0

    //init i2c
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(ERROR_LED_PIN);
    gpio_set_dir(ERROR_LED_PIN, GPIO_OUT);
}


int16_t bmp180_read_int16(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, BMP180_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, BMP180_ADDR, buf, 2, false);
    return (buf[0] << 8) | buf[1];
}

uint8_t bmp180_read_int8(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(I2C_PORT, BMP180_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, BMP180_ADDR, &val, 1, false);
    return val;
}

bool bmp180_read_calibration() {
    AC5 = bmp180_read_int16(0xB2);
    AC6 = bmp180_read_int16(0xB4);
    MC  = bmp180_read_int16(0xBC);
    MD  = bmp180_read_int16(0xBE);

    uint8_t id = bmp180_read_int8(0xD0);
    return id == 0x55; // BMP180 ID
}

int32_t bmp180_read_temperature() {
    uint8_t cmd[2] = {0xF4, 0x2E};
    i2c_write_blocking(I2C_PORT, BMP180_ADDR, cmd, 2, false);
    sleep_ms(5);

    int32_t UT = bmp180_read_int16(0xF6);

    int32_t X1 = ((UT - AC6) * AC5) >> 15;
    int32_t X2 = (MC << 11) / (X1 + MD);
    int32_t B5 = X1 + X2;

    return (B5 + 8) >> 4; //return temperature
}

void init_injectors_gpio(struct Injector injectors[], int count)
{
    for (int i = 0; i < count; i++) {
        gpio_init(injectors[i].input_gpio_pin);
        gpio_set_dir(injectors[i].input_gpio_pin, GPIO_IN);
        gpio_pull_up(injectors[i].input_gpio_pin);  // Enable built-in pull-up

        gpio_init(injectors[i].output_gpio_pin);
        gpio_set_dir(injectors[i].output_gpio_pin, GPIO_OUT);
        gpio_put(injectors[i].output_gpio_pin, 0); // Ensure output is low initially
    }
}

int scale(int x)
{
    //adc scale
    //sclae raw adc to value 10-35 out = out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min)
    return 10+(x-0)*(35-5)/(4095-0);
}




int main() {

    init_ports();


    struct Injector injectors[] = {
        {false, 2, 18, 0, 0,0 }, //open, input_pin, output_pin, closed_time, open_time, delay_time
        {false, 3, 19, 0, 0,0},
        {false, 4, 20, 0, 0,0},
        {false, 5, 21, 0, 0,0},
        {false, 6, 22, 0, 0,0}
    };

    int count_of_injectors = sizeof(injectors) / sizeof(injectors[0]);

    init_injectors_gpio(injectors, count_of_injectors);

    int base_factor; //10 - 35
    uint16_t raw = adc_read(); // 0–4095
    base_factor = scale(raw);

    int current_factor = base_factor;
    int cold_start_factor = 3;
    int time_factor = 4; //decrase per second

    uint64_t engine_start_time = time_us_64();

    bool engine_running =false;

    uint64_t first_injector_pulse_time = 0;
    uint64_t last_injector_pulse_time = 0;

    float current_temperature;
    if (bmp180_read_calibration()) {
        //read temp from sensor
        int32_t temperature = bmp180_read_temperature();
        current_temperature = temperature / 10.0f; //convert to celsius
    }else { //if BMP180 fail
        current_temperature = DEFAULT_TEMPRATURE_CELSIUS; //default temp
        gpio_put(ERROR_LED_PIN, 1); //turn on error led
    }

    int start_factor;
    if (current_temperature < BASE_TEMPERATURE_COLD_START) {
        start_factor = base_factor + ((BASE_TEMPERATURE_COLD_START - current_temperature) * cold_start_factor);
    }else {
        start_factor = base_factor;
    }

    if (start_factor > MAX_FACTOR) {
        start_factor = MAX_FACTOR;
    }

    current_factor = start_factor;

    while (1) {

        if (engine_running && current_factor > base_factor) {
            current_factor = start_factor - ((time_us_64()-engine_start_time)/1000000) * time_factor;
        }

        if (current_factor < base_factor) {
            current_factor = base_factor;
        }

        if (!engine_running && first_injector_pulse_time != 0) { //first pulse detected
            if ((time_us_64()-last_injector_pulse_time> 190000)) { //no pulses for 190ms
                first_injector_pulse_time = 0; //reset, no start
            }
            else if ((time_us_64()-first_injector_pulse_time)>2000000){ //2 seconds of pulses, engine is running
                engine_running = true;
                engine_start_time = time_us_64();
            }
        }


        //process injectors
        for (int i = 0; i < count_of_injectors; i++) {
            if (gpio_get(injectors[i].input_gpio_pin) == 0) { //input is pulled to ground
                if (injectors[i].open == false) {
                    injectors[i].open_time = time_us_64();
                    last_injector_pulse_time = time_us_64();
                }
                gpio_put(injectors[i].output_gpio_pin, 1); //control mosfet to pull line low
                injectors[i].open = true;

                if (first_injector_pulse_time == 0) { 
                    first_injector_pulse_time = time_us_64();
                }

            } else {
                if (injectors[i].open == true) {
                    injectors[i].closed_time = time_us_64();
                    injectors[i].open = false;

                    //calculate opened time
                    uint64_t opened_time = injectors[i].closed_time - injectors[i].open_time; //temp variable
                    //calculate delay time
                    injectors[i].delay_time = opened_time + (opened_time * current_factor / 100);

                }else if ((time_us_64() - injectors[i].closed_time) > injectors[i].delay_time) { // check if delay time has passed
                    gpio_put(injectors[i].output_gpio_pin, 0); //release line
                }
            }
        }
    }



}
