#include "ros/ros.h"
#include "battery_level_msg/battery_level.h"

static const double startingCapacity = 10;     // 10 Ah
static const double currentUsage = 1.22;       // 1.22 A
static const double rate = 0.1;                // 0.1 Hz -> 10 seconds
static const int hourToSecond = 3600;

int main (int argc, char **argv){
	double presentCapacity = startingCapacity;
    ros::init(argc, argv, "raspi_battery_monitor");
    ros::NodeHandle n;
    ros::Publisher raspi_battery_monitor_pub = n.advertise<battery_level_msg::battery_level>("raspi_battery_level",1);
    ros::Rate loop_rate(rate); //0.1 Hz
    battery_level_msg::battery_level msg;


    while (ros::ok()){
		
		presentCapacity = presentCapacity - currentUsage / (rate * hourToSecond);
		msg.battery_level = 100 * presentCapacity / startingCapacity;

		ROS_INFO("%f", msg.battery_level);
		raspi_battery_monitor_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}