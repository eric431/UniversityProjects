#include <cstdlib>
#include <string>

#include "ros/ros.h"
#include "cmd_runner/CmdRun.h"

bool cmdRun(cmd_runner::CmdRun::Request &req,
            cmd_runner::CmdRun::Response &res)
{
    std::string command = req.command;
    int returnVal =  std::system(command.c_str());
    res.success = returnVal == 0 ? 1 : 0; 
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_runner_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("cmd_runner", cmdRun);
    ros::spin();

    return 0;
}
