#pragma once

#define INPUT_KEYS_NUM 6
#define W     0
#define A     1
#define S     2
#define D     3
#define UP    4
#define DOWN  4
#define LEFT  6
#define RIGHT 7

extern struct libevdev* dev;

class KeyboardReader : public rclcpp::Node{
public:
    KeyboardReader();
    void update_key_status(uint8_t key_index);
    bool* get_key_input();

private:
    bool key_status[INPUT_KEYS_NUM];
    struct input_event ev;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
};

void printKeyStatus(bool* key_status);


