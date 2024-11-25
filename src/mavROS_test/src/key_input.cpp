#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <libevdev/libevdev.h>

#include "key_input.hpp"

struct libevdev* dev = nullptr;

// 简单的打印
void printKeyStatus(bool* key_status) {
    std::cout << "W: " << (key_status[W] ? "Pressed" : "Released") << ", ";
    std::cout << "A: " << (key_status[A] ? "Pressed" : "Released") << ", ";
    std::cout << "S: " << (key_status[S] ? "Pressed" : "Released") << ", ";
    std::cout << "D: " << (key_status[D] ? "Pressed" : "Released") << ", ";
    std::cout << "Up: " << (key_status[UP] ? "Pressed" : "Released") << ", ";
    std::cout << "Down: " << (key_status[DOWN] ? "Pressed" : "Released") << ", ";
    std::cout << "Left: " << (key_status[LEFT] ? "Pressed" : "Released") << ", ";
    std::cout << "Right: " << (key_status[RIGHT] ? "Pressed" : "Released") << std::endl;
    usleep(1000);
}

// 节点类的构造函数
KeyboardReader::KeyboardReader() : Node("keyboard_reader"){
    RCLCPP_INFO(this->get_logger(), "Keyboard reader node has been started.");
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&KeyboardReader::timer_callback, this));
}

// 更新键盘输入状态
void KeyboardReader::update_key_status(uint8_t key_index){
    if (ev.value == 1) {         // 按下
        key_status[key_index] = true;
    } else if (ev.value == 0) {  // 松开
        key_status[key_index]= false;
    } else if (ev.value == 2) {  // 按住
        // 持续按住时不需要改变状态
    }
}

// get函数
bool* KeyboardReader::get_key_input(){
    return key_status;
}

// 检查按键事件，并更新相应的标志位
void KeyboardReader::timer_callback(){
    if (libevdev_next_event(dev, LIBEVDEV_READ_FLAG_BLOCKING, &ev) == 0) {
        if (ev.type == EV_KEY) {
            switch (ev.code) {
                case KEY_W:
                    update_key_status(W);
                    break;
                case KEY_A:
                    update_key_status(A);
                    break;
                case KEY_S:
                    update_key_status(S);
                    break;
                case KEY_D:
                    update_key_status(D);
                    break;
                case KEY_UP:
                    update_key_status(UP);
                    break;
                case KEY_DOWN:
                    update_key_status(DOWN);
                    break;
                case KEY_LEFT:
                    update_key_status(LEFT);
                    break;
                case KEY_RIGHT:
                    update_key_status(RIGHT);
                    break;
                default:
                    break;
            }
        }
        printKeyStatus(key_status);
    }
}