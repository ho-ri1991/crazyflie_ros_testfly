#include"ros/ros.h"
#include"crazyflie_driver/AddCrazyflie.h"
#include"crazyflie_driver/RemoveCrazyflie.h"
#include"geometry_msgs/Twist.h"
#include<string>
#include<exception>

const char* const prefix = "/test";

int main(int argc, char** argv){
    ros::init(argc, argv, "testfly");

    if(argc < 2){
        ROS_INFO("usage: testfly crazyflie_uri initial_thrust");
        return 1;
    }

    const char* const crazyflieURI = argv[1];
    int thrust = 0;
    if(argc > 2){
        try{
            thrust = std::stoi(argv[2]);
        }catch(std::exception& e){
            std::cout<<e.what()<<std::endl;
        }
    }

    ros::NodeHandle n;

    ros::ServiceClient addCrazyflieClient = n.serviceClient<crazyflie_driver::AddCrazyflie>("add_crazyflie");
    ros::ServiceClient removeCrazyflieClient = n.serviceClient<crazyflie_driver::RemoveCrazyflie>("remove_crazyflie");

    crazyflie_driver::AddCrazyflie addReq;
    addReq.request.uri = crazyflieURI;
    addReq.request.tf_prefix = "";
    addReq.request.roll_trim = 0;
    addReq.request.pitch_trim = 0;
    addReq.request.use_ros_time = true;
    addReq.request.enable_logging = true;
    addReq.request.enable_logging_imu = true;
    addReq.request.enable_logging_battery = true;
    addReq.request.enable_logging_temperature = true;
    addReq.request.enable_logging_magnetic_field = true;
    addReq.request.enable_logging_pressure = true;

    if(addCrazyflieClient.call(addReq)){
        ROS_INFO("connected");

        ros::Publisher testflyPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = thrust;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;

        ros::Rate loop_rate(50);

        while(ros::ok()){
            testflyPub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

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
