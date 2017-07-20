#ifndef TESTFLY_CRAZYFLIE_HPP
#define TESTFLY_CRAZYFLIE_HPP

#include"ros/ros.h"
#include"crazyflie_driver/AddCrazyflie.h"
#include"crazyflie_driver/RemoveCrazyflie.h"
#include"geometry_msgs/Twist.h"
#include<string>
#include<exception>
#include"crazyflie_ros_testfly/SetThrust.h"


namespace testfly{
    class Crazyflie{
    public:
        static constexpr const char* topic_control = "/cmd_vel";
        static constexpr const char* topic_add_crazyflie = "add_crazyflie";
        static constexpr const char* topic_remove_crazyflie = "remove_crazyflie";
        static constexpr const char* service_set_thrust = "set_thrust";
        static constexpr int topic_control_buffer_size = 10;
    private:
        //
        const std::string prefix;
        const std::string uri;
        geometry_msgs::Twist state;
        bool isConnected;
        ros::NodeHandle nodeHandler;
        //clients for add/remove crazyflie server
        ros::ServiceClient addCrazyflieClient;
        ros::ServiceClient removeCrazyflieClient;
        //
        ros::ServiceServer setThrustServive;
        //controller
        ros::Publisher controller;
    private:
        bool setThrust(crazyflie_ros_testfly::SetThrust::Request& req,
                        crazyflie_ros_testfly::SetThrust::Response& res){
            state.linear.z += req.thrust;
            res.thrust = state.linear.z;
            return true;
        }
    public:
        bool connect(){
            crazyflie_driver::AddCrazyflie addReq;
            addReq.request.uri = uri;
            addReq.request.tf_prefix = prefix;
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
                isConnected = true;
                return false;
            }else{
                ROS_INFO("failed to connect");
                isConnected = false;
                return true;
            }
        }
        bool disconnect(){
            if(!isConnected)return false;
            crazyflie_driver::RemoveCrazyflie removeReq;
            removeReq.request.uri = uri;
            if(removeCrazyflieClient.call(removeReq)){
                ROS_INFO("disconnected");
                isConnected = false;
                return false;
            }else{
                ROS_INFO("Failed to disconnect");
                return true;
            }            
        }
    public:
        Crazyflie(const std::string& prefix_, const std::string& uri_, const geometry_msgs::Twist& initial_state)
            : prefix(prefix_)
            , uri(uri_)
            , state(initial_state)
            , isConnected(false)
            , nodeHandler()
            , addCrazyflieClient(nodeHandler.serviceClient<crazyflie_driver::AddCrazyflie>(topic_add_crazyflie))
            , removeCrazyflieClient(nodeHandler.serviceClient<crazyflie_driver::RemoveCrazyflie>(topic_remove_crazyflie))
            , setThrustServive(nodeHandler.advertiseService(service_set_thrust, &Crazyflie::setThrust, this))
            , controller(nodeHandler.advertise<geometry_msgs::Twist>(prefix + topic_control, topic_control_buffer_size)){

        }
        ~Crazyflie(){
            if(isConnected){
                disconnect();
            }
        }
    public:
        void work(){
            controller.publish(state);
        }
    };
}

#endif