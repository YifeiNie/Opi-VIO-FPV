#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/char.hpp"
#include "mavROS_test.hpp"

using std::placeholders::_1;

class IMU_Subscriber:public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;
    void topic_callback(const sensor_msgs::msg::Imu & msg) const{
        
        imu_linear_accl = msg.linear_acceleration;
        RCLCPP_INFO(this->get_logger(), "Received IMU data: Acceleration x: %f, y: %f, z: %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    }

    geometry_msgs::msg::Vector3 get_linear_acc_vector() const {

    }

public:
    // 构造函数
    IMU_Subscriber():Node("IMU_Subscriber"){
        subscription = this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data_raw", rclcpp::QoS(10).best_effort(), std::bind(&IMU_Subscriber::topic_callback, this, _1));
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node=rclcpp::Node::make_shared("helloworld_node_cpp");
    RCLCPP_INFO(node->get_logger(),"hello world");
    rclcpp::spin(std::make_shared<IMU_Subscriber>());

    rclcpp::shutdown();
    return 0;
}
