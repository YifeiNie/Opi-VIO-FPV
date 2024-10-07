# Opi-VIO-FPV
## 介绍
 这是一个基于视觉里程计的自主小型无人机项目，期望目标是实现高速行进中的避障与路径规划，计划使用事件相机来解决高速状态下RGB相机的动态模糊问题，一边学习一边开发中...
## 硬件型号
机架：3.5寸，螺丝孔全通孔  
螺丝：M2、M3及其对应的普通和自紧螺母  
电机：PACER P1604，3800KV，upside down放置以增大空间和推力  
飞控：Kakute H7 Mini，孔距20mm  
电调：Tekko32 F4 4in1 Mini 45A ESC，孔距20mm  
电脑：香橙派5 pro/酷派，CPU均为RK3588  
桨叶：Gemfan 3.5寸三叶桨  
接收机：乐迪R12DSM（SBUS有线协议，无线电协议未开源且当前无适配遥控器，计划改为自制ELRS无线电协议和CRSF有线协议接收机）  
## 改动记录
### 2024.10.02 by Nyf --机架公差问题
- 机架上电机的卡簧孔受公差影响较大，导致旋转时摩擦卡簧增大阻尼，通过粗十字螺丝刀插进去旋转打磨使得电机能流畅旋转  
### 2024.10.04 by Nyf --安装ROS2和MavROS
- 使用命令`wget http://fishros.com/install -O fishros&& . fishros`安装会方便很多，我安装的是ROS2，ROS1资料多，支持老Ubuntu版本而较新版本用不了，但将于2025年停止更新，ROS2功能更多，需要结合需求和系统版本进行选择
- 先后在两个终端分别使用`ros2 run turtlesim turtlesim_node`和`ros2 run turtlesim turtle_teleop_key`来测试ROS2的安装，具体为方向键控制一个小乌龟
- 先后使用`sudo apt-get install ros-jazzy-mavros`和`sudo apt-get install ros-jazzy-mavros-extras`安装MavROS工具，其中jazzy是我安装的ROS2的版本EGO 
### 2024.10.05 by Nyf --瞎折腾
- 尝试编译Ego-Planner，发现其基于ROS，使用catkin进行构建，而我安装的ROS2（jazzy）使用colcon进行构建，最终尝试失败，放弃，Planner的事情后面再说，先计划嗯看源码
- 拿到了修改后的BF固件源码，开始尝试参考[这里](https://blog.csdn.net/zhengyangliu123/article/details/54783443)进行编译，失败，明天再说把
### 2024.10.06 by Nyf
- 今天的教训是网络上的各类教程都必须先弄清楚版本上的差异，各种依赖的使用和安装在不同版本的Ubuntu下有很大差别，比如
    - 我使用的Ubuntu24.04已经不支持ROS，只支持ROS2,
    - ARM工具链gcc-arm-none-eabi在以前需要lsb-core工具才能正常使用，但是在24.04（noble）版本下已经不需要这个工具，直接使用`sudo apt install gcc-arm-none-eabi`即可完成安装，详见[这里](https://askubuntu.com/questions/1519420/lsb-was-removed-on-noble)
- 在使用apt进行安装时，如果出现报错：没有可用的软件包...，说明该版本Ubuntu不支持这个包了，上网找当前版本下的解决方法
- git操作时出现permission denied，使用sudo才能解决，是因为初次clone时使用了root，导致权限出现问题，可以使用`sudo chown -R "$USER" -- /path/to/your/local/repo/folder`，详见[这里](https://stackoverflow.com/questions/73580646/why-cant-i-use-git-without-sudo)
- 源码编译时记得在Opi-VIO-FPV/make/tools.mk里修改符合你安装工具链的版本号
- 发现bf地面站不支持arm64的Ubuntu，所以后面计划在OPi上编程和烧录，参考[这里](https://www.carliatronics.eu/stm32-development-and-debug-with-vscode-on-ubuntu-2404/)
### 2024.10.07 by Nyf --尝试通过mavlink和mavROS打通飞控与电脑的通信
- 给我自己买的Inav固件的F405飞控改刷了BF固件。需要用到zagig工具配置飞控的bootloader，然后按住boot键给飞控上电先刷固件，再进入bf地面站命令从文件加载MPU6500的配置即可，操作步骤见[bf文档](https://betaflight.com/docs/wiki/guides/current/installing-betaflight)，操作视频见[这里](https://www.bilibili.com/video/BV1824y1v7JB/?spm_id_from=333.788.top_right_bar_window_history.content.click&vd_source=3a6242b3cb9435a95f7d4a98159f0607)，配置文件见[这里](https://github.com/YifeiNie/F405Firmware-IMU-configFile)
- 简单了解了ROS2