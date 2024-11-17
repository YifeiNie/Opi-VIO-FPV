#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>

// 结构体用于存储键盘输入
struct KeyInput {
    char key;
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
};

// 创建一个类，用于读取键盘输入并保存到结构体中
class KeyboardReader : public rclcpp::Node{
public:
    KeyboardReader() : Node("keyboard_reader"){
        RCLCPP_INFO(this->get_logger(), "Keyboard reader node has been started.");
        // 配置终端，使其能够读取单个字符而无需按 Enter
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio_ = old_tio_;
        new_tio_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);

        // 启动一个定时器，每隔100毫秒读取键盘输入
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&KeyboardReader::timer_callback, this));
    }


    ~KeyboardReader(){
        // 恢复终端的原始设置
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

// 获取最新的键盘输入
    KeyInput get_last_key_input() const {
        return last_key_input_;
    }
private:
    void timer_callback(){
        char key;
        if (read(STDIN_FILENO, &key, 1) > 0){
            // 更新结构体中的键盘输入信息
            last_key_input_.key = key;
            last_key_input_.timestamp = std::chrono::steady_clock::now();

            RCLCPP_INFO(this->get_logger(), "Key pressed: '%d'", key);
        }
    }
    KeyInput last_key_input_; // 保存最近一次的键盘输入
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios old_tio_, new_tio_;
};

// int main(int argc, char *argv[]){
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<KeyboardReader>();

//     // 示例：在主循环中获取并使用结构体中的键盘输入
//     while (rclcpp::ok()) {
//         rclcpp::spin_some(node);

//         // 获取最近一次的键盘输入
//         KeyInput input = node->get_last_key_input();
//         std::cout << "Last key pressed: " << input.key << std::endl;

//         // 添加延迟，避免打印太快
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));

//     }
//     rclcpp::shutdown();
//     return 0; 
// }