#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/char.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <libevdev/libevdev.h>
#include <fcntl.h>

#include "key_input.hpp"
#include "mavros_offboard.hpp"

// 绑定函数占位符
using std::placeholders::_1;
// 时间相关
using namespace std::chrono_literals;


IMU_subscriber::IMU_subscriber():Node("IMU_subscriber"){
    subscription = this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data_raw", rclcpp::QoS(10).best_effort(), std::bind(&IMU_subscriber::topic_callback, this, _1));
}

void IMU_subscriber::topic_callback(const sensor_msgs::msg::Imu & msg) {

    imu_linear_accl.x = msg.linear_acceleration.x;
    imu_linear_accl.y = msg.linear_acceleration.y;
    imu_linear_accl.z = msg.linear_acceleration.z;
}

geometry_msgs::msg::Vector3 IMU_subscriber::get_imu_linear_acc() {
    return imu_linear_accl;
}


Offboard_publisher::Offboard_publisher():Node("Offboard_publisher"){
    publisher = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
    timer = this->create_wall_timer(100ms, std::bind(&Offboard_publisher::Offboard_publisher_timer_callback, this)); // 定时器
}

void Offboard_publisher::Offboard_publisher_timer_callback(){   // 回调函数用于发送offboard消息
    msg.thrust = 0.5;
    msg.type_mask = 7;      // 这里的body_rate表示角速度还是角度，取决于type_mask的值，如果为7表示角度
    msg.body_rate.x = 1;    // 单位度每秒或度
    msg.body_rate.y = 8;
    msg.body_rate.z = 3;

    publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(),"AttitudeTarget published!");
}


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    const char* dev_path = "/dev/input/event6";  // 输入设备路径，根据实际情况修改
    int fd = open(dev_path, O_RDONLY);
    if (fd == -1) {
        perror("Open device error");
        return 0;
    }

    if (libevdev_new_from_fd(fd, &dev) != 0) {
        perror("Libevdev init error");
        close(fd);
        return 0;
    }
    std::cout << "device name: " << libevdev_get_name(dev) << std::endl;
    auto node = std::make_shared<KeyboardReader>();

    // 示例：在主循环中获取并使用结构体中的键盘输入
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

    }
    rclcpp::shutdown();
    return 0; 
}
// int main(int argc, char ** argv)
// {
//     rclcpp::init(argc,argv);
//     auto node=rclcpp::Node::make_shared("helloworld_node_cpp");
//     RCLCPP_INFO(node->get_logger(),"hello world");
//     rclcpp::spin(std::make_shared<Offboard_publisher>());

//     rclcpp::shutdown();
//     return 0;
// }
