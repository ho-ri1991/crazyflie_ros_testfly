#include"ros/ros.h"
#include"crazyflie.hpp"
#include"crazyflie_ros_testfly/SetThrust.h"
#include<exception>
#include<string>

int main(int argc, char** argv){
    ros::init(argc, argv, "set_thrust_client");
    if(argc < 2){
        ROS_INFO("usage: set_thrust thrust_inc");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<crazyflie_ros_testfly::SetThrust>(testfly::Crazyflie::service_set_thrust);

    crazyflie_ros_testfly::SetThrust req;
    try{
        req.request.thrust = std::stoi(argv[1]);
    }catch(std::exception& e){
        ROS_INFO("%s", e.what());
    }
    
    if(client.call(req)){
        ROS_INFO("thrust: %d", static_cast<int>(req.response.thrust));
    }else{
        ROS_ERROR("fail to set thrust");
        return 1;
    }
    return 0;
}