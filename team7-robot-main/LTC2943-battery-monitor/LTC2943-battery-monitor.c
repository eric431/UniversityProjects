/*
Copyright 2021 Christopher Feghali

Permission is hereby granted, free of charge, to any person obtaining a 
copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation 
the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the 
Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in 
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
DEALINGS IN THE SOFTWARE.
*/

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>

#include "LTC2943/LTC2943.h"

#define INITIAL_CHARGE 1000               //1000 mAh
#define RESISTOR 0.005
#define PRESCALER LTC2943_PRESCALAR_M_64

int main(void){
    
    int fd;
    fd = wiringPiI2CSetup(LTC2943_I2C_ADDRESS);

    if (fd == -1){
        printf("LTC2943 battery monitor not found, exiting...\n");
        return -1;
    }

    printf("LTC2943 battery monitor started at pid %d, press CTRL+C to quit.\n", getpid());

    if (!LTC2943_set_adc_mode(fd, LTC2943_AUTOMATIC_MODE)){
        printf("Could not set ADC mode, exiting...\n");
        return -1;
    }

    if(!LTC2943_set_prescaler(fd, PRESCALER)){
        printf("Could not set prescaler, exiting...\n");
        return -1;        
    }

    if(!LTC2943_set_charge_mAh(fd, INITIAL_CHARGE, RESISTOR, PRESCALER)){
        printf("Could not set initial charge, exiting...\n");
        return -1;        
    }

    while(1){
        //float voltage = LTC2943_get_voltage(fd);
        //float temperature = LTC2943_get_temperature(fd);
        //float current = LTC2943_get_current(fd, RESISTOR);
        float charge = LTC2943_get_charge(fd, RESISTOR, PRESCALER);
        if (charge < 0){
            printf("Battery level error, exiting...\n");
            return -1;
        }
        printf("Battery Level: %f%%\n", 100 * charge / INITIAL_CHARGE);

        delay(10000);
    }

    return 0;
}



