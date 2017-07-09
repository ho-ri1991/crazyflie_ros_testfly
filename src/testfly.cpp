#include"ros/ros.h"
#include"crazyflie_driver/AddCrazyflie.h"
#include"crazyflie_driver/RemoveCrazyflie.h"
#include"geometry_msgs/Twist.h"

const char* const prefix = "/test";

int main(int argc, char** argv){
    ros::init(argc, argv, "testflie");

    if(argc < 1){
        ROS_INFO("usage: testfly crazyflie_uri");
        return 1;
    }

    const char* const crazyflieURI = argv[0];

    ros::NodeHandle n;

    ros::ServiceClient addCrazyflieClient = n.serviceClient<crazyflie_driver::AddCrazyflie>("add_crazyflie");
    ros::ServiceClient removeCrazyflieClient = n.serviceClient<crazyflie_driver::RemoveCrazyflie>("remove_crazyflie");

    crazyflie_driver::AddCrazyflie addReq;
    addReq.request.uri = crazyflieURI;
    addReq.request.tf_prefix = prefix;

    if(addCrazyflieClient.call(addReq)){
        ROS_INFO("connected");
        crazyflie_driver::RemoveCrazyflie removeReq;
        removeReq.request.uri = crazyflieURI;
        if(removeCrazyflieClient.call(removeReq)){
            ROS_INFO("disconnected");
        }else{
            ROS_INFO("Failed to disconnect");
            return 1;
        }
    }else{
        ROS_INFO("Failed to connect to crazyflie");
        return 1;
    }

    return 0;
}
