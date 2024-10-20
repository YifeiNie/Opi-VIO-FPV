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
- 先后在两个终端分别使用`ros2 run turtlesim turtlesim_node`和`ros2 run turtlesim turtle_teleop_key`来测试ROS2的安装，具体为用方向键控制一个小乌龟，注意光标必须放在第二个命令所在的终端才能使用方向键控制乌龟方向
- 先后使用`sudo apt-get install ros-jazzy-mavros`和`sudo apt-get install ros-jazzy-mavros-extras`安装MavROS工具，其中jazzy是我安装的ROS2的版本，安装路径为`/opt/ros/jazzy/share`
### 2024.10.05 by Nyf --瞎折腾
- 尝试编译Ego-Planner，发现其基于ROS，使用catkin进行构建，而我安装的ROS2（jazzy）使用colcon进行构建，最终尝试失败，放弃，Planner的事情后面再说，先计划嗯看源码
- 拿到了修改后的BF固件源码，开始尝试参考[这里](https://blog.csdn.net/zhengyangliu123/article/details/54783443)进行编译，失败，明天再说把
### 2024.10.06 by Nyf
- 今天的教训是网络上的各类教程都必须先弄清楚版本上的差异，各种依赖的使用和安装在不同版本的Ubuntu下有很大差别，比如
    - 我使用的Ubuntu24.04已经不支持ROS，只支持ROS2,
    - ARM工具链gcc-arm-none-eabi在以前需要lsb-core工具才能正常使用，但是在24.04（noble）版本下已经不需要这个工具，直接使用`sudo apt install gcc-arm-none-eabi`即可完成安装，详见[这里](https://askubuntu.com/questions/1519420/lsb-was-removed-on-noble)
- 在使用apt进行安装时，如果出现报错：没有可用的软件包...，在排除源和网络问题后仍不能解决，说明该版本Ubuntu不支持这个包了，上网找当前版本下的解决方法
- git操作时出现permission denied，使用sudo才能解决，是因为初次clone时使用了root，导致权限出现问题，可以使用`sudo chown -R "$USER" -- /path/to/your/local/repo/folder`，详见[这里](https://stackoverflow.com/questions/73580646/why-cant-i-use-git-without-sudo)
- 源码编译时记得在Opi-VIO-FPV/make/tools.mk里修改符合你安装工具链的版本号
- 发现bf地面站不支持arm64的Ubuntu，所以后面计划在OPi上编程和烧录，参考[这里](https://www.carliatronics.eu/stm32-development-and-debug-with-vscode-on-ubuntu-2404/)
### 2024.10.07 by Nyf --尝试通过mavlink和mavROS打通飞控与电脑的通信
- 给我自己买的Inav固件的F405飞控改刷了BF固件。需要用到zagig工具配置飞控的bootloader，然后按住boot键给飞控上电先刷固件，再进入bf地面站的CLI命令从文件加载MPU6500的配置文件即可，操作步骤见[bf文档](https://betaflight.com/docs/wiki/guides/current/installing-betaflight)，操作视频见[这里](https://www.bilibili.com/video/BV1824y1v7JB/?spm_id_from=333.788.top_right_bar_window_history.content.click&vd_source=3a6242b3cb9435a95f7d4a98159f0607)，配置文件见[这里](https://github.com/YifeiNie/F405Firmware-IMU-configFile)
- 简单了解了ROS2
### 2024.10.09 by Nyf --配置win下的飞控的编译环境
- 由于前面说的，bf地面站不支持arm64的OPi，我又没有一台Linux电脑，故尝试在win下写飞控代码并编译。折腾了大半天各方面都想到了始终无法make，最终发现`make`命令一直在调用Delphi的`make.exe`！！！且关键词检索，发现有个[论坛](http://www.qtcn.org/bbs/read-htm-tid-1075-page-1.html)也有人遇到相同问题。问题在删除它后解决。我的Matlab，Vitis，MRS等需要调用make的软件都没有这种情况，只有Delphi有，不知道为啥，红温了
- Win的命令行下使用`where make`查找当前使用`make`命令调用的make.exe的路径配置Win下的STM32编译环境
- 配置Win环境下的STM32编译工具，先下载工具链[rm-none-eabi-gcc](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)(具体是下载arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi.zip
即可),解压后将bin文件夹的路径添加到环境变量。然后下载[MinGW](https://zenlayer.dl.sourceforge.net/project/mingw/Installer/mingw-get-setup.exe?viasf=1)，安装包的时候安装第2，5，7（即最后一个）个包，并添加bin路径的环境变量，最后将`mingw32-make.exe`改为`make.exe`，一定要等一会，或者重启，之后使用`-v`命令检测前面的工具链和后面的MinGW是否安装完毕。整个流程参考[这里](https://www.cnblogs.com/bangbangzoutianya/p/17402641.html)
- 千辛万苦，终于在Win下过编，明天上电！
- 新版的MinGW好像支持一些linux的命令，移植过来时不用在makefile里修改替代了（比如linux的`rm`和win的`del`）
### 2024.10.10 by Nyf
- BF_NeSC成功在我的405飞控上运行并成功读到MAVLink数据，前提是需要通过BF地面站的CLI命令行修改一下端口，具体操作见我10.07日的笔记
### 2024.10.11 by Nyf
- 由于使用PX4.launch启动mavROS，所以PX4飞控上的固件ArduPilot中回传的GPS数据包需要一个地图数据集才能被OPi解析（其实英国不用管那个报错也可以，而且有人说这里的“error”其实是警告，不影响mavROS的运行，[参见](https://github.com/mavlink/mavros/issues/246)）具体解决：运行`sudo geographiclib-get-magnetic emm2015`命令下载，然后在路径`/opt/ros/jazzy/lib/mavros`下运行shell脚本：`sudo bash install_geographiclib_datasets.sh`。注意，这个的安装非常慢且终端一直卡在一个语句上，直到对应的三个数据集都被成功安装才可以使用，在我的OPi上花费了大约十来分钟
- 成功在Opi上运行mavROS，但不知道是什么原因飞控和Pi一直连不上，前者一直在发数据，后者的ROS结点也都开启了，有可能是波特率不对，白天在调一调
- 飞控与OPi成功连接！以下是步骤
    - OPi上下载CuteCom作为串口调试助手`sudo apt install cutecom`
    - 飞控bf地面站左边栏目 “端口-遥测输出-选择mavlink，波特率115200”，“接收机-遥测”输出打开
    - 使用CH340连接飞控与OPi，切记只连接tx，rx，g，不要连接5v，飞控上电，发现ch340上RXD字样边红灯长亮，说明有数据在发送，说明飞控配置完毕
    - OPi上输入`sudo chmod 777 /dev/tty*`命令对串口赋予权限，然后打开CuteCom，点击连接会直接发现有源源不断的数据被接收，因为Linux上自带了CH340驱动。如果没有数据收到，请：
        - 使用`dmesg | tail`检查USB是否被连接，如果输入命令后打印出`interface 0 claimed by ch341 while 'brltty' sets config #1`，说明软件brltty（这是个盲人辅助软件）影响了USB的连接，使用`sudo apt remove brltty`删除它。然后一定要重复赋予串口权限的操作，再进行连接即可
    - 使用`dmesg | grep tty*`或者直接查看cutecom中的串口端口名，使用`sudo gedit /opt/ros/jazzy/share/mavros/launch/px4.launch`打开启动配置文件，修改fcu_url为`/dev/ttyCH341USB0:2000000`，其中CH341USB0是我的端口名，需要根据具体修改
    - 使用`ros2 launch mavros px4.launch`启动，等一会儿后发现终端上源源不断打印类似`[mavros_node-1] [INFO] [1728627432.538089052] [mavros.mavros_router]: link[1000] removed stale remote address 194.121`的内容，说明飞控与OPi成功连接
### 2024.10.12 by Nyf
- 学C++，笔记见[这里](https://github.com/YifeiNie/Cpp-tutorial.git)
### 2024.10.12 by Nyf
- 弄了一天，还是mavlink还是连不到mavros，不知道要怎么搞了，预计会卡在这里几天
### 2024.10.16 by Nyf --测试失败
- 折腾三天，结果如下：
    - 使用`ros2 topic list`查看已发布话题，mavros在Opi上正常运行，话题全部启动
    - 使用`/uas1/mavlink_source`查看mavlink消息，正常，说明飞控到ros连接建立，在`Opi-VIO-FPV/src/main/telementry/mavlink.c`路径里的`processMAVLinkTelemetry()`函数修改消息内容，并通过cutecom读取，可以修改并被读取
    - 使用`ros2 topic echo /mavros/state`打印飞控与mavros连接情况，disconnected，说明是mavlink到mavros这个环节出问题
    - 使用诊断`ros2 topic echo /diagnostics`查看，发现大多数消息都是level 2（错误），除了`'mavros_router: endpoint 1001: /uas1'`，`name: 'mavros_router: endpoint 1000: /dev/ttyCH341USB0:2000000'`，`'mavros_router: MAVROS Router'`话题为level 0（正常）
- 期间尝试各种方法，有说改传输速率的，失败；修改固件，失败；更换版本，失败；尝试修改mavlink源码，失败...
- 发现我的mavros版本是2.8.0,但是最新版是2.9.0,且根据github中的issue中提到2.8.0有诸多bug在刚刚发布的2.9.0中被修复，然而其release显示ustable，见[这里](https://github.com/mavlink/mavros/issues?q=is%3Aissue+is%3Aopen)
- 尝试更新到2.9.0,发现apt的最新版本是2.8.0，尝试源码更新，出错，折腾一天，失败，见[这里](https://github.com/mavlink/mavros/issues/2003)
- 后面的计划是：使用px4仿真环境再尝试mavros通信，如果成功，继续查找问题；如果失败，则放弃ROS2和mavros 2.8.0直到2.9.0正式在源中发布再做尝试，并暂时先使用Ubuntu 20.04和ROS1
<<<<<<< HEAD
### 2024.10.17 by Nyf 
- 折腾一天，给新笔记本装了Unbuntu 20.04
    - 其中网卡驱动非常折磨，后发现问题是我安装的系统Linux内核版本太老，且网络配置文件`sudo gedit /etc/NetworkManager/NetworkManager.conf`有一行要改为'true'，具体见[这里](https://blog.csdn.net/taoxicun/article/details/133200526)
    - 下载的Chrome，clash等双击无法运行并报错:'没有安装处理"shared library"文件的程序'，但在命令行里可以，这是因为双击的路径是系统路径，而命令行则是在文件所在路径，故需要配置环境变量
    - 但是我手动配置环境变量后，开机后会在输密码处无限循环，网上说就是因为改了环境变量的问题，说可以ctrl+alt+F1到F6任意按键以使用命令行输入密码登录，但我试了，全部失败，放弃，就用命令行挺好的
- 安装了ROS Noetic，对应的mavros版本是1.19.0，成功，读到了飞控的imu等数据，且操作相同，进一步说明ROS2的mavros还存在一些问题，mavros的contributor回复说2.9.0 in testing，能正式通过apt安装还需等到下一次同步。
=======
### 2024.10.19 by Nyf
- 血泪教训，在配置clash时，一定要在代理页面选择全局代理，不要选择直连！！

