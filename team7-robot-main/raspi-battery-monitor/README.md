# raspi-battery-monitor

## Description

raspi-battery-monitor works by connecting 4 wires to GPIO pins 17, 27, 22, 23 
that correspond to 100%, 75%, 50%, and 25% battery levels. Some logic is performed and the battery level is printed to `stdout`.

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

After this, you should be able to compile the `c` files.


```
gcc -o battery-monitor battery-monitor.c -l wiringPi
gcc -o battery-monitor2 battery-monitor2.c -l wiringPi
```

You can run the programs with
```
./battery-monitor
./battery-monitor2
```

## Usage

battery-monitor prints the battery level every 1 second.

battery-monitor2 prints the battery level whenever a change in level occurs which it does with interrupts.

