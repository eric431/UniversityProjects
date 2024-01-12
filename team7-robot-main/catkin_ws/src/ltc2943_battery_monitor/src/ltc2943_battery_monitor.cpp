#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>

#include "ltc2943_battery_monitor/LTC2943.h"
#include "ros/ros.h"
#include "battery_level_msg/battery_level.h"

#define INITIAL_CHARGE 1000               //1000 mAh
#define RESISTOR 0.005
#define PRESCALER LTC2943_PRESCALAR_M_64

int main (int argc, char **argv){

    ros::init(argc, argv, "ltc2943_battery_monitor");
    ros::NodeHandle n;
    ros::Publisher LTC2943_battery_monitor_pub = n.advertise<battery_level_msg::battery_level>("ltc2943_battery_level",1);
    ros::Rate loop_rate(0.1); //0.1 Hz
    battery_level_msg::battery_level msg;

    int fd;
    fd = wiringPiI2CSetup(LTC2943_I2C_ADDRESS);

    if (fd == -1){
        ROS_FATAL("LTC2943 battery monitor not found, exiting...\n");
    }

    if (!LTC2943_set_adc_mode(fd, LTC2943_AUTOMATIC_MODE)){
        ROS_FATAL("Could not set ADC mode, exiting...\n");
        return -1;
    }

    if(!LTC2943_set_prescaler(fd, PRESCALER)){
        ROS_FATAL("Could not set prescaler, exiting...\n");
        return -1;
    }

    if(!LTC2943_set_charge_mAh(fd, INITIAL_CHARGE, RESISTOR, PRESCALER)){
        ROS_FATAL("Could not set initial charge, exiting...\n");
        return -1;
    }

    while (ros::ok()){
        float voltage = LTC2943_get_voltage(fd);
        float temperature = LTC2943_get_temperature(fd);
        float current = LTC2943_get_current(fd, RESISTOR);
        float charge = LTC2943_get_charge(fd, RESISTOR, PRESCALER);
        
        ROS_DEBUG("Voltage: %f", voltage);
        ROS_DEBUG("Temperature: %f", temperature);
        ROS_DEBUG("Current: %f", current);
        ROS_DEBUG("Charge: %f", charge);

        if (charge < 0){
            ROS_FATAL("Battery level error, exiting...\n");
            return -1;
        }
        
        msg.battery_level = 100 * charge / INITIAL_CHARGE;
        

        ROS_INFO("%f", msg.battery_level);
        LTC2943_battery_monitor_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}