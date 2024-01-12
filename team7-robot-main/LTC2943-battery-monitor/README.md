# LTC2943-battery-monitor

## Description

LTC2943-battery-monitor uses the LTC2943 IC to monitor the battery level of LIPO's
Communication via I2C and the battery level is printed to `stdout`.

## Installation

raspi-battery-monitor requires wiringPi. If you are using Raspbian OS, you probably already have it.
Check your version with `gpio -v`. If you get `Unknown17`, then you'll have to update your version executing 
the following commands (taken from https://learn.sparkfun.com/tutorials/raspberry-gpio/all):

```
sudo apt-get purge wiringpi
hash -r
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
git pull origin
./build
```

Run `make` to build.


You can run the program with `./LTC294-battery-monitor`

## Usage

LTC2943-battery-monitor prints the battery level every 10 seconds.
