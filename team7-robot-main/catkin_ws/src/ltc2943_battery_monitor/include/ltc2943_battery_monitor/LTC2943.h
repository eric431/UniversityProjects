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
I2C/SMBus Interface.

I2C DATA FORMAT (MSB FIRST):

Data Out:
Byte #1                                    Byte #2                     Byte #3

START  SA6 SA5 SA4 SA3 SA2 SA1 SA0 W SACK  C7  C6 C5 C4 C3 C2 C1 C0 SACK D7 D6 D5 D4 D3 D2 D1 D0 SACK  STOP

Data In:
Byte #1                                    Byte #2                                    Byte #3

START  SA6 SA5 SA4 SA3 SA2 SA1 SA0 W SACK  C7  C6  C5 C4 C3 C2 C1 C0 SACK  Repeat Start SA6 SA5 SA4 SA3 SA2 SA1 SA0 R SACK

Byte #4                                   Byte #5
MSB                                       LSB
D15 D14  D13  D12  D11  D10  D9 D8 MACK   D7 D6 D5 D4 D3  D2  D1  D0  MNACK  STOP

START       : I2C Start
Repeat Start: I2c Repeat Start
STOP        : I2C Stop
SAx         : I2C Address
SACK        : I2C Slave Generated Acknowledge (Active Low)
MACK        : I2C Master Generated Acknowledge (Active Low)
MNACK       : I2c Master Generated Not Acknowledge
W           : I2C Write (0)
R           : I2C Read  (1)
Cx          : Command Code
Dx          : Data Bits
X           : Don't care



Example Code:

Read charge, current, and voltage.

    adc_command = LTC2943_SENSE_MONITOR | LTC2943_AUTOMATIC_MODE; // Builds commands to set LTC2943 to automatic mode
    ack |= LTC2943_write(LTC2943_I2C_ADDRESS, LTC2943_CONTROL_REG, adc_command);   // Sets the LTC2943 to automatic mode

    resistor = .1; // Resistor Value On Demo Board

    ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_CHARGE_MSB_REG, &charge_code);  // Reads the ADC registers that contains charge value
    charge = LTC2943_code_to_coulombs(charge_code, resistor, prescalarValue); // Calculates charge from charge code, resistor and prescalar

    ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_CURRENT_MSB_REG, &current_code); // Reads the voltage code across sense resistor
    current = LTC2943_code_to_current(current_code, resistor); // Calculates current from current code, resistor value.

    ack |= LTC2943_read_16_bits(LTC2943_I2C_ADDRESS, LTC2943_VOLTAGE_MSB_REG, &voltage_code);   // Reads voltage voltage code
    VIN = LTC2943_VIN_code_to_voltage(voltage_code);  // Calculates VIN voltage from VIN code and lsb


@endverbatim

http://www.linear.com/product/LTC2943
http://www.linear.com/product/LTC2943-1

http://www.linear.com/product/LTC2943#demoboards
http://www.linear.com/product/LTC2943-1#demoboards

*/

/*! @file
    @ingroup LTC2943
    Header for LTC2943: Multicell Battery Gas Gauge with Temperature, Voltage and Current Measurement
*/


#ifndef LTC2943_H
#define LTC2943_H


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
| LTC2943 I2C Address Assignment    | Value |
| :-------------------------------- | :---: |
| LTC2943_I2C_ADDRESS               | 0xC8  |
*/

/*! @name LTC2943 I2C Address
@{ */

#define LTC2943_I2C_ADDRESS 0x64
#define LTC2943_I2C_ALERT_RESPONSE  0x0C
//! @}


/*!
| Name                                              | Value |
| :------------------------------------------------ | :---: |
| LTC2943_STATUS_REG                                | 0x00  |
| LTC2943_CONTROL_REG                               | 0x01  |
| LTC2943_ACCUM_CHARGE_MSB_REG                      | 0x02  |
| LTC2943_ACCUM_CHARGE_LSB_REG                      | 0x03  |
| LTC2943_CHARGE_THRESH_HIGH_MSB_REG                | 0x04  |
| LTC2943_CHARGE_THRESH_HIGH_LSB_REG                | 0x05  |
| LTC2943_CHARGE_THRESH_LOW_MSB_REG                 | 0x06  |
| LTC2943_CHARGE_THRESH_LOW_LSB_REG                 | 0x07  |
| LTC2943_VOLTAGE_MSB_REG                           | 0x08  |
| LTC2943_VOLTAGE_LSB_REG                           | 0x09  |
| LTC2943_VOLTAGE_THRESH_HIGH_MSB_REG               | 0x0A  |
| LTC2943_VOLTAGE_THRESH_HIGH_LSB_REG               | 0x0B  |
| LTC2943_VOLTAGE_THRESH_LOW_MSB_REG                | 0x0C  |
| LTC2943_VOLTAGE_THRESH_LOW_LSB_REG                | 0x0D  |
| LTC2943_CURRENT_MSB_REG                           | 0x0E  |
| LTC2943_CURRENT_LSB_REG                           | 0x0F  |
| LTC2943_CURRENT_THRESH_HIGH_MSB_REG               | 0x10  |
| LTC2943_CURRENT_THRESH_HIGH_LSB_REG               | 0x11  |
| LTC2943_CURRENT_THRESH_LOW_MSB_REG                | 0x12  |
| LTC2943_CURRENT_THRESH_LOW_LSB_REG                | 0x13  |
| LTC2943_TEMPERATURE_MSB_REG                       | 0x14  |
| LTC2943_TEMPERATURE_LSB_REG                       | 0x15  |
| LTC2943_TEMPERATURE_THRESH_HIGH_REG               | 0x16  |
| LTC2943_TEMPERATURE_THRESH_LOW_REG                | 0x17  |
*/

/*! @name Registers
@{ */
// Registers
#define LTC2943_STATUS_REG                          0x00
#define LTC2943_CONTROL_REG                         0x01
#define LTC2943_ACCUM_CHARGE_MSB_REG                0x02
#define LTC2943_ACCUM_CHARGE_LSB_REG                0x03
#define LTC2943_CHARGE_THRESH_HIGH_MSB_REG          0x04
#define LTC2943_CHARGE_THRESH_HIGH_LSB_REG          0x05
#define LTC2943_CHARGE_THRESH_LOW_MSB_REG           0x06
#define LTC2943_CHARGE_THRESH_LOW_LSB_REG           0x07
#define LTC2943_VOLTAGE_MSB_REG                     0x08
#define LTC2943_VOLTAGE_LSB_REG                     0x09
#define LTC2943_VOLTAGE_THRESH_HIGH_MSB_REG         0x0A
#define LTC2943_VOLTAGE_THRESH_HIGH_LSB_REG         0x0B
#define LTC2943_VOLTAGE_THRESH_LOW_MSB_REG          0x0C
#define LTC2943_VOLTAGE_THRESH_LOW_LSB_REG          0x0D
#define LTC2943_CURRENT_MSB_REG                     0x0E
#define LTC2943_CURRENT_LSB_REG                     0x0F
#define LTC2943_CURRENT_THRESH_HIGH_MSB_REG         0x10
#define LTC2943_CURRENT_THRESH_HIGH_LSB_REG         0x11
#define LTC2943_CURRENT_THRESH_LOW_MSB_REG          0x12
#define LTC2943_CURRENT_THRESH_LOW_LSB_REG          0x13
#define LTC2943_TEMPERATURE_MSB_REG                 0x14
#define LTC2943_TEMPERATURE_LSB_REG                 0x15
#define LTC2943_TEMPERATURE_THRESH_HIGH_REG         0x16
#define LTC2943_TEMPERATURE_THRESH_LOW_REG          0x17
//! @}

/*!
| Command Codes                                 | Value     |
| :-------------------------------------------- | :-------: |
| LTC2943_AUTOMATIC_MODE                        | 0xC0      |
| LTC2943_SCAN_MODE                             | 0x80      |
| LTC2943_MANUAL_MODE                           | 0x40      |
| LTC2943_SLEEP_MODE                            | 0x00      |
| LTC2943_PRESCALAR_M_1                         | 0x00      |
| LTC2943_PRESCALAR_M_4                         | 0x08      |
| LTC2943_PRESCALAR_M_16                        | 0x10      |
| LTC2943_PRESCALAR_M_64                        | 0x18      |
| LTC2943_PRESCALAR_M_256                       | 0x20      |
| LTC2943_PRESCALAR_M_1024                      | 0x28      |
| LTC2943_PRESCALAR_M_4096                      | 0x30      |
| LTC2943_PRESCALAR_M_4096_2                    | 0x31      |
| LTC2943_ALERT_MODE                            | 0x04      |
| LTC2943_CHARGE_COMPLETE_MODE                  | 0x02      |
| LTC2943_DISABLE_ALCC_PIN                      | 0x00      |
| LTC2943_SHUTDOWN_MODE                         | 0x01      |
*/

/*! @name Command Codes
@{ */
// Command Codes
#define LTC2943_AUTOMATIC_MODE                  0xC0
#define LTC2943_SCAN_MODE                       0x80
#define LTC2943_MANUAL_MODE                     0x40
#define LTC2943_SLEEP_MODE                      0x00

#define LTC2943_PRESCALAR_M_1                   0x00
#define LTC2943_PRESCALAR_M_4                   0x08
#define LTC2943_PRESCALAR_M_16                  0x10
#define LTC2943_PRESCALAR_M_64                  0x18
#define LTC2943_PRESCALAR_M_256                 0x20
#define LTC2943_PRESCALAR_M_1024                0x28
#define LTC2943_PRESCALAR_M_4096                0x30
#define LTC2943_PRESCALAR_M_4096_2              0x31

#define LTC2943_ALERT_MODE                      0x04
#define LTC2943_CHARGE_COMPLETE_MODE            0x02

#define LTC2943_DISABLE_ALCC_PIN                0x00
#define LTC2943_SHUTDOWN_MODE                   0x01

//! @}

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
// const float LTC2943_CHARGE_lsb = 0.34E-3;
// const float LTC2943_VOLTAGE_lsb = 1.44E-3;
// const float LTC2943_CURRENT_lsb = 29.3E-6;
// const float LTC2943_TEMPERATURE_lsb = 0.25;
// const float LTC2943_FULLSCALE_VOLTAGE = 23.6;
// const float LTC2943_FULLSCALE_CURRENT = 60E-3;
// const float LTC2943_FULLSCALE_TEMPERATURE = 510;
//! @}

//! @}


//! Write an 8-bit code to the LTC2943.
//! @return The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
uint8_t LTC2943_write(int fd, //!< Register address for the LTC2943
                     uint8_t adc_command, //!< The "command byte" for the LTC2943
                     uint8_t code         //!< Value that will be written to the register.
                    );

//! Write a 16-bit code to the LTC2943.
//! @return The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
uint8_t LTC2943_write_16_bits(int fd, //!< Register address for the LTC2943
                             uint8_t adc_command, //!< The "command byte" for the LTC2943
                             uint16_t code        //!< Value that will be written to the register.
                            );


//! Reads an 8-bit adc_code from LTC2943
//! @return The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
int LTC2943_read(int fd, //!< Register address for the LTC2943
                    uint8_t adc_command //!< The "command byte" for the LTC2943
                   );

//! Reads a 16-bit adc_code from LTC2943
//! @return The function returns the state of the acknowledge bit after the I2C address write. 0=acknowledge, 1=no acknowledge.
int LTC2943_read_16_bits(int fd, //!< Register address for the LTC2943
                            uint8_t adc_command //!< The "command byte" for the LTC2943
                           );


//! Calculate the LTC2943 charge in Coulombs
//! @return Returns the Coulombs of charge in the ACR register.
float LTC2943_code_to_coulombs(int adc_code,        //!< The RAW ADC value
                               float resistor,         //!< The sense resistor value
                               uint16_t prescalar      //!< The prescalar value
                              );

//! Calculate the LTC2943 charge in mAh
//! @return Returns the Coulombs of charge in the ACR register.
float LTC2943_code_to_mAh(int adc_code,            //!< The RAW ADC value
                          float resistor,       //!< The sense resistor value
                          uint16_t prescalar    //!< The prescalar value
                         );

//! Calculate the LTC2943 SENSE+ voltage
//! @return Returns the SENSE+ Voltage in Volts
float LTC2943_code_to_voltage(int adc_code              //!< The RAW ADC value
                             );

//! Calculate the LTC2943 current with a sense resistor
//! @return Returns the current through the sense resisor
float LTC2943_code_to_current(int adc_code,                //!< The RAW ADC value
                              float resistor                   //!< The sense resistor value
                             );

//! Calculate the LTC2943 temperature
//! @return Returns the temperature in Kelvin
float LTC2943_code_to_kelvin_temperature(int adc_code           //!< The RAW ADC value
                                        );

//! Calculate the LTC2943 temperature
//! @return Returns the temperature in Celcius
float LTC2943_code_to_celcius_temperature(int adc_code          //!< The RAW ADC value
                                         );

// Used to set and clear bits in a control register.  bits_to_set will be bitwise OR'd with the register.
// bits_to_clear will be inverted and bitwise AND'd with the register so that every location with a 1 will result in a 0 in the register.
uint8_t LTC2943_register_set_clear_bits(int fd, uint8_t register_address, uint8_t bits_to_set, uint8_t bits_to_clear);


//Extra functions
uint8_t LTC2943_set_adc_mode(int fd, uint8_t mode);

uint8_t LTC2943_get_status(int fd);

uint8_t LTC2943_set_prescaler(int fd, uint8_t prescaler);

uint8_t LTC2943_set_charge_mAh(int fd, float charge, float resistor, uint16_t prescalar);

float LTC2943_get_temperature(int fd);

float LTC2943_get_charge(int fd, float restistor, uint16_t prescalar);

float LTC2943_get_current(int fd, float resistor);

float LTC2943_get_voltage(int fd);



#ifdef __cplusplus
}
#endif

#endif  // LTC2943_H
