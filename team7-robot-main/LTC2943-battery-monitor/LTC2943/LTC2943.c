/*!
Copyright 2021(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    - Neither the name of Analog Devices, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
    - The use of this software may or may not infringe the patent rights
      of one or more patent holders.  This license does not release you
      from the requirement that you obtain separate licenses from these
      patent holders to use this software.
    - Use of the software either in source or binary form, must be run
      on or directly connected to an Analog Devices Inc. component.
   
THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED.

IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

LTC2943: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement.
LTC2943-1: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement.

@verbatim

The LTC2943 measures battery charge state, battery voltage,
battery current and its own temperature in portable
product applications. The wide input voltage range allows
use with multicell batteries up to 20V. A precision coulomb
counter integrates current through a sense resistor between
the battery’s positive terminal and the load or charger.
Voltage, current and temperature are measured with an
internal 14-bit No Latency ΔΣ™ ADC. The measurements
are stored in internal registers accessible via the onboard
I2C/SMBus Interface

@endverbatim

http://www.linear.com/product/LTC2943
http://www.linear.com/product/LTC2943-1

http://www.linear.com/product/LTC2943#demoboards
http://www.linear.com/product/LTC2943-1#demoboards

REVISION HISTORY
$Revision: 5672 $
$Date: 2016-09-02 11:42:55 -0700 (Fri, 02 Sep 2016) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

//! @defgroup LTC2943 LTC2943: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement

/*! @file
   @ingroup LTC2943
   Library for LTC2943 Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement
*/

#include <stdint.h>
#include <wiringPiI2C.h>
#include "LTC2943.h"

/*!
| Conversion Constants                              |  Value   |
| :------------------------------------------------ | :------: |
| LTC2943_CHARGE_lsb                                | 0.34  mAh|
| LTC2943_VOLTAGE_lsb                               | 1.44   mV|
| LTC2943_CURRENT_lsb                               |  29.3  uV|
| LTC2943_TEMPERATURE_lsb                           | 0.25    C|
| LTC2943_FULLSCALE_VOLTAGE                         |  23.6   V|
| LTC2943_FULLSCALE_CURRENT                         |  60    mV|
| LTC2943_FULLSCALE_TEMPERATURE                     | 510     K|

*/
/*! @name Conversion Constants
@{ */
const float LTC2943_CHARGE_lsb = 0.34E-3;
const float LTC2943_VOLTAGE_lsb = 1.44E-3;
const float LTC2943_CURRENT_lsb = 29.3E-6;
const float LTC2943_TEMPERATURE_lsb = 0.25;
const float LTC2943_FULLSCALE_VOLTAGE = 23.6;
const float LTC2943_FULLSCALE_CURRENT = 60E-3;
const float LTC2943_FULLSCALE_TEMPERATURE = 510;
//! @}

//! @}




// Write an 8-bit code to the LTC2943.
uint8_t LTC2943_write(int fd, uint8_t adc_command, uint8_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  uint8_t ack;

  ack = wiringPiI2CWriteReg8(fd, adc_command, code);
  return(ack);
}


// Write a 16-bit code to the LTC2943.
uint8_t LTC2943_write_16_bits(int fd, uint8_t adc_command, uint16_t code)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  uint8_t ack;

  ack = wiringPiI2CWriteReg16(fd, adc_command, code);
  return(ack);
}

// Reads an 8-bit adc_code from LTC2943
int LTC2943_read(int fd, uint8_t adc_command)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  return wiringPiI2CReadReg8(fd, adc_command);
}

// Reads a 16-bit adc_code from LTC2943
int LTC2943_read_16_bits(int fd, uint8_t adc_command)
// The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
{
  return wiringPiI2CReadReg16(fd, adc_command);
}


float LTC2943_code_to_coulombs(int adc_code, float resistor, uint16_t prescalar)
// The function converts the 16-bit RAW adc_code to Coulombs
{
  float coulomb_charge;
  coulomb_charge =  1000*(float)(adc_code*LTC2943_CHARGE_lsb*prescalar*50E-3)/(resistor*4096);
  coulomb_charge = coulomb_charge*3.6f;
  return(coulomb_charge);
}

float LTC2943_code_to_mAh(int adc_code, float resistor, uint16_t prescalar )
// The function converts the 16-bit RAW adc_code to mAh
{
  float mAh_charge;
  mAh_charge = 1000*(float)(adc_code*LTC2943_CHARGE_lsb*prescalar*50E-3)/(resistor*4096);
  return(mAh_charge);
}

float LTC2943_code_to_voltage(int adc_code)
// The function converts the 16-bit RAW adc_code to Volts
{
  float voltage;
  voltage = ((float)adc_code/(65535))*LTC2943_FULLSCALE_VOLTAGE;
  return(voltage);
}

float LTC2943_code_to_current(int adc_code, float resistor)
// The function converts the 16-bit RAW adc_code to Amperes
{
  float current;
  current = (((float)adc_code-32767)/(32767))*((float)(LTC2943_FULLSCALE_CURRENT)/resistor);
  return(current);
}

float LTC2943_code_to_kelvin_temperature(int adc_code)
// The function converts the 16-bit RAW adc_code to Kelvin
{
  float temperature;
  temperature = adc_code*((float)(LTC2943_FULLSCALE_TEMPERATURE)/65535);
  return(temperature);
}

float LTC2943_code_to_celcius_temperature(int adc_code)
// The function converts the 16-bit RAW adc_code to Celcius
{
  float temperature;
  temperature = adc_code*((float)(LTC2943_FULLSCALE_TEMPERATURE)/65535) - 273.15;
  return(temperature);
}

// Used to set and clear bits in a control register.  bits_to_set will be bitwise OR'd with the register.
// bits_to_clear will be inverted and bitwise AND'd with the register so that every location with a 1 will result in a 0 in the register.
uint8_t LTC2943_register_set_clear_bits(int fd, uint8_t register_address, uint8_t bits_to_set, uint8_t bits_to_clear)
{
  uint8_t register_data;
  int8_t ack = 0;

  register_data = LTC2943_read(fd, register_address);
  register_data = register_data & (~bits_to_clear);
  register_data = register_data | bits_to_set;
  ack |= LTC2943_write(fd, register_address, register_data);
  return(ack);
}

// LTC2943_AUTOMATIC_MODE                  0xC0
// LTC2943_SCAN_MODE                       0x80
// LTC2943_MANUAL_MODE                     0x40
// LTC2943_SLEEP_MODE                      0x00
uint8_t LTC2943_set_adc_mode(int fd, uint8_t mode)
{
  uint8_t code;
  uint8_t mask = 0x3F; //00111111

  code = LTC2943_read(fd, LTC2943_CONTROL_REG);

  //printf("control register code is %d\n", (int8_t)code);

  if ((int8_t)code == -1){
    return 0;
  }

  code = (code & mask) | mode;

  if(LTC2943_write(fd, LTC2943_CONTROL_REG, code)){
    return 1;
  }

  return 0;
}

// LTC2943_PRESCALAR_M_1                   0x00
// LTC2943_PRESCALAR_M_4                   0x08
// LTC2943_PRESCALAR_M_16                  0x10
// LTC2943_PRESCALAR_M_64                  0x18
// LTC2943_PRESCALAR_M_256                 0x20
// LTC2943_PRESCALAR_M_1024                0x28
// LTC2943_PRESCALAR_M_4096                0x30
uint8_t LTC2943_set_prescaler(int fd, uint8_t prescaler)
{  
  uint8_t code;
  uint8_t mask = 0xC7; //11000111

  code = LTC2943_read(fd, LTC2943_CONTROL_REG);

  if ((int8_t)code == -1){
    return 0;  
  }

  code = (code & mask) | prescaler;

  if(LTC2943_write(fd, LTC2943_CONTROL_REG, code)){
    return 1;
  }

  return 0;
}

// #define LTC2943_STATUS_REG                          0x00
uint8_t LTC2943_get_status(int fd)
{
  return LTC2943_read(fd, LTC2943_STATUS_REG);
}


// #define LTC2943_ACCUM_CHARGE_MSB_REG                0x02
// #define LTC2943_ACCUM_CHARGE_LSB_REG                0x03
uint8_t LTC2943_set_charge_mAh(int fd, float charge, float resistor, uint16_t prescalar)
{
  /* 
  before writing to the accumulated charge registers, 
  the analog section should be temporarily shut down by 
  setting B[0] to 1
  */
  uint8_t code;
  uint8_t mask = 0xFE; //11111110
  uint16_t charge_code;

  code = LTC2943_read(fd, LTC2943_CONTROL_REG);

  if ((int8_t)code == -1){
    return 0;  
  }

  code = (code & mask) | 1;

  if(!LTC2943_write(fd, LTC2943_CONTROL_REG, code)){
    return 0;
  }

  charge_code = (uint16_t)((charge * resistor * 4096) / (1000 * prescalar * LTC2943_CHARGE_lsb * 50E-3));

  if(!LTC2943_write_16_bits(fd, LTC2943_ACCUM_CHARGE_MSB_REG, charge_code)){
    return 0;
  }

  code = (code & mask) | 0;

  if(LTC2943_write(fd, LTC2943_CONTROL_REG, code)){
    return 1;
  }

  return 0;
}

// #define LTC2943_ACCUM_CHARGE_MSB_REG                0x02
// #define LTC2943_ACCUM_CHARGE_LSB_REG                0x03
float LTC2943_get_charge(int fd, float resistor, uint16_t prescalar)
{
  return LTC2943_code_to_mAh(LTC2943_read_16_bits(fd, LTC2943_ACCUM_CHARGE_MSB_REG),resistor, prescalar);
}

// #define LTC2943_VOLTAGE_MSB_REG                     0x08
// #define LTC2943_VOLTAGE_LSB_REG                     0x09
float LTC2943_get_voltage(int fd)
{
  return LTC2943_code_to_voltage(LTC2943_read_16_bits(fd, LTC2943_VOLTAGE_MSB_REG));
}

// #define LTC2943_CURRENT_MSB_REG                     0x0E
// #define LTC2943_CURRENT_LSB_REG                     0x0F
float LTC2943_get_current(int fd, float resistor)
{
  return LTC2943_code_to_current(LTC2943_read_16_bits(fd, LTC2943_CURRENT_MSB_REG), resistor); 
}

// #define LTC2943_TEMPERATURE_MSB_REG                 0x14
// #define LTC2943_TEMPERATURE_LSB_REG                 0x15
float LTC2943_get_temperature(int fd)
{
  return LTC2943_code_to_celcius_temperature(LTC2943_read_16_bits(fd, LTC2943_TEMPERATURE_MSB_REG));
}











