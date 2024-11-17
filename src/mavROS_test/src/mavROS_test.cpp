#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/char.hpp"
#include "mavROS_test.hpp"

// 绑定函数占位符
using std::placeholders::_1;
// 时间相关
using namespace std::chrono_literals;

class IMU_subscriber:public rclcpp::Node{
private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;

    geometry_msgs::msg::Vector3 imu_linear_accl;

    // 不能使用const修饰函数，否则无法修改成员变量
    void topic_callback(const sensor_msgs::msg::Imu & msg) {
    
        imu_linear_accl.x = msg.linear_acceleration.x;
        imu_linear_accl.y = msg.linear_acceleration.y;
        imu_linear_accl.z = msg.linear_acceleration.z;
        
    }

public:
    // 构造函数
    IMU_subscriber():Node("IMU_subscriber"){
        subscription = this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data_raw", rclcpp::QoS(10).best_effort(), std::bind(&IMU_subscriber::topic_callback, this, _1));
    }
    // get函数
    geometry_msgs::msg::Vector3 get_imu_linear_acc() const {
        return imu_linear_accl;
    }
    
};

class Offboard_publisher:public rclcpp::Node{
private:
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr publisher;

    mavros_msgs::msg::AttitudeTarget msg;
    
    rclcpp::TimerBase::SharedPtr timer;  // 定时器用于定时发布话题
   
    void Offboard_publisher_timer_callback(){   // 回调函数用于发送offboard消息
        msg.thrust = 0.5;
        msg.type_mask = 7;      // 这里的body_rate表示角速度还是角度，取决于type_mask的值，如果为7表示角度
        msg.body_rate.x = 0;    // 单位度每秒或度
        msg.body_rate.y = 8;
        msg.body_rate.z = 0;

        publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(),"AttitudeTarget published!");
    }
public:
    // 先调用父类的构造函数，然后再执行大括号内的自定义的构造函数
    Offboard_publisher():Node("Offboard_publisher"){
        publisher = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
        timer = this->create_wall_timer(100ms, std::bind(&Offboard_publisher::Offboard_publisher_timer_callback, this)); // 定时器
    }
    
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    auto node=rclcpp::Node::make_shared("helloworld_node_cpp");
    RCLCPP_INFO(node->get_logger(),"hello world");
    rclcpp::spin(std::make_shared<Offboard_publisher>());

    rclcpp::shutdown();
    return 0;
}
