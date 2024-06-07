#include "ros_interface/amerigo_hardware_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amerigo_hardware_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    AmerigoHardwareInterface Amerigo(nh);
    spinner.spin();
    return 0;
}