#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/char.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class IMU_subscriber:public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;

    geometry_msgs::msg::Vector3 imu_linear_accl;
    void topic_callback(const sensor_msgs::msg::Imu & msg);

public:
    IMU_subscriber();
    geometry_msgs::msg::Vector3 get_imu_linear_acc();
    
};



class Offboard_publisher:public rclcpp::Node{
private:
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr publisher;

    mavros_msgs::msg::AttitudeTarget msg;
    
    rclcpp::TimerBase::SharedPtr timer;  // 定时器用于定时发布话题
    
    void Offboard_publisher_timer_callback();
public:
    // 先调用父类的构造函数，然后再执行大括号内的自定义的构造函数
    Offboard_publisher();
};
