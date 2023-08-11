#include <ros/ros.h>    
#include <std_msgs/String.h> 

#include <thread>   

#include "husky_controller/husky_controller.h"
#include "husky_controller/mujoco_interface.h"

#define TorqueControl 1
#define PositionControl 0

/**********************************************************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_controller");   
    ros::NodeHandle nh; 
    DataContainer dc;

    // int control_mode = TorqueControl;
    int control_mode = PositionControl;

    MujocoInterface mujoco_interface(nh, dc);
    HuskyController husky_controller(nh, dc, control_mode);

    std::thread thread[3];

    thread[0] = std::thread(&MujocoInterface::stateUpdate, &mujoco_interface);
    thread[1] = std::thread(&HuskyController::compute, &husky_controller);
    thread[2] = std::thread(&MujocoInterface::sendCommand, &mujoco_interface, control_mode);

    for (int i = 0; i < 3; i++)
        thread[i].join();   

    return 0;
}
