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
### 2024.10.17 by Nyf 
- 折腾一天，给新笔记本装了Unbuntu 20.04
    - 其中网卡驱动非常折磨，后发现问题是我安装的系统Linux内核版本太老，且网络配置文件`sudo gedit /etc/NetworkManager/NetworkManager.conf`有一行要改为'true'，具体见[这里](https://blog.csdn.net/taoxicun/article/details/133200526)
    - 下载的Chrome，clash等双击无法运行并报错:'没有安装处理"shared library"文件的程序'，但在命令行里可以，这是因为双击的路径是系统路径，而命令行则是在文件所在路径，故需要配置环境变量
    - 但是我手动配置环境变量后，开机后会在输密码处无限循环，网上说就是因为改了环境变量的问题，说可以ctrl+alt+F1到F6任意按键以使用命令行输入密码登录，但我试了，全部失败，放弃，就用命令行挺好的
- 安装了ROS Noetic，对应的mavros版本是1.19.0，成功，读到了飞控的imu等数据，且操作相同，进一步说明ROS2的mavros还存在一些问题，mavros的contributor回复说2.9.0 in testing，能正式通过apt安装还需等到下一次同步。
### 2024.10.19 by Nyf
- 血泪教训，在配置clash时，一定要在代理页面选择全局代理，不要选择直连！！
### 2024.10.20-22 by Nyf
- 这两天一直在看源码，由于调度问题，mavlink发送imu数据的频率往往会低于设定频率，通过修改srv/main/fc/task.c中的`TASK_TELEMETRY`优先级可解决此问题，同样在下面也可以修改imu数据的更新频率，进而解决了最初需要解决的问题
- 下一步计划编写飞控的mavlink接收函数，并搭建控制器的框架，但目前仍不太清楚其回调的原理，需要进一步对serial.c文件里的openSerialPort()函数进行分析
- 另发现可以使用`cliPrint`函数来在BF地面站打印调试信息，用法和C语言的printf一样，便于后续的调试  
### 2024.10.25 -by Nyf
- 成功安装d435驱动，读取到了点云数据
- 使用F4飞控组装一架穿越机，电机正常工作，计划今日试飞
- 计划使用cool Pi作为上位，使用d435作为VIO，使用VINS进行数据融合进行视觉导航
### 2024.10.26 -by Nyf
- `roscore`是ros节点运行前必须执行的，在运行后卡死在`Done checking log file disk usage.Usage is <1GB`，然后等很久会报错`unable to contact my own server at xxxxxx`，这说明此时该ros运行的电脑是从机并且没有连上主机，可以通过`sudo gedit ~/.bashrc`打开shell配置文件，并将对应的内容修改为`export ROS_HOSTNAME=localhost`和`export ROS_MASTER_URI=http://localhost:11311`，然后一定要重启，再次运行，发现roscore可以运行，使用`rviz`进行测试发现可以打开了，参考[这里](https://blog.csdn.net/qq_42535748/article/details/125818486)。
- 成功使用CoolPi运行planner仿真，但是卡成PPT，不知道是Rviz的问题还是本身CoolPi就跑不了，初步怀疑是前者
- 简单学习了Fusion360，计划给Pi和相机画一个架子
### 2024.10.27 -by Nyf - 发现源码bug
- mavlink源码有bug，会导致发送频率自动除以2，见mavlink.c文件里的`mavlinkStreamTrigger`函数及我加的注释
### 2024.10.28 -by Nyf
- ros中有关realsense的启动文件rs_camera.launch有两个，一个在路径`/package/catkin_ws/src/realsense-ros/realsense2_camera/launch`下，也是cool pi使用的，另一个在`/opt/ros/noetic/share/realsense2_camera/launch`，这个是默认的安装位置。在使用时，需要修改其中的参数来设置相机发布的话题，比如左，右目的图像，深度图像等等
- 直接驱动realsense，没有任何问题，D435图像读取正常，但是启用vins时报错`/opt/ros/noetic/lib/nodelet/nodelet: symbol lookup error: /home/coolpi/Code/Fast-Drone-250-master/devel/lib//librealsense2_camera.so: undefined symbol: _ZN20ddynamic_reconfigure19DDynamicReconfigureC1ERKN3ros10NodeHandleE
[camera/realsense2_camera_manager-2] process has died [pid 27915, exit code 127, cmd /opt/ros/noetic/lib/nodelet/nodelet manager __name:=realsense2_camera_manager __log:=/home/coolpi/.ros/log/09d5dc34-952f-11ef-9e4f-e0752666340d/camera-realsense2_camera_manager-2.log]`，注意到关键词ddynamic_reconfigure
    - 经检查，除了apt默认的/ros/noetic/share路径下，在别的工作空间也安装了这个包，而新部署的planner在编译时，会引用原有的.bashrc中source的内容，并写入到其环境变量配置脚本devel/setup.bash中。而使用这个planner，必须要启动它的setup.bash，但由于它对.bashrc引用，会导致两个ddynamic_reconfigure相互冲突。因此也容易得到解决办法，参考[这里](https://github.com/IntelRealSense/realsense-ros/issues/838)：
        - 注释掉全部.bashrc中的source
        - 使用`source ~/.bashrc`更新
        - 重新编译planner，得到新的不含任何引用setup.bash
        - 为了防止影响此电脑上的其他程序，取消注释
        - 使用`source ~/.bashrc`更新，该问题解决
- 但vins还是启动不了，一直在等待imu数据，估计是数据格式发送仍然不对，需要进一步debug
### 2024.10.29 -by Nyf
- 找了一个px4飞控来进行对比，需要注意要使用`rosservice call /mavros/set_stream_rate 0 10 1`来开启数据流，第一个参数'0'表示所有传感器，第二个参数'10'表示数据流的频率单位hz，第三个参数'1'表示开启数据流，参考[这里](https://blog.csdn.net/m0_73885374/article/details/140937715?spm=1001.2014.3001.5502)
- BF固件中，可以在accgyro_mpu6500.c中看到这样的定义`int accel_range = INV_FSR_16G`，而在后面又有`busWriteRegister(dev, MPU_RA_ACCEL_CONFIG, accel_range << 3)`的程序，所以相当于量程设置为±16×2^3 G，而MPU6500的精度是16位，也即-32768到32767，所以假设读取的16位有符号整型的数据为1000，则说明真实的加速度是1000*128G/32768 G
- px4固件中的很多相关mavlink的函数、变量都是通过.xml和.msg自动生成C++/Py代码，所以直接搜函数会发现在工程中没有定义
- 通过查看mavros源码，终于发现问题所在：尽管各个数据已经完全一致，但是mavros源码`imu.cpp`的`handle_raw_imu`函数中有定义，不对非apm或px4飞控发送的imu_raw数据进行数据处理就直接发布，这样就导致量纲不对。虽然之前就可以直接通过强行修改和找规律得到争取的mavros消息，但是我不想这样。
- 下一步计划在进一步修改bf源码，使其可以模拟px4发送mavlink数据
### 2024.10.30 -by Nyf -mavros有bug
- mavros bug：
    - PX4固件使用mavlink发送的imu_raw中的线加速度单位是mG，mavros发布的（也是vins需要的）线加速度单位是m/s^2
    - 按理说二者之间的转换应该是乘上系数9.80665/1000，但是mavros中对px4的转换仅仅有/1000，而对APM固件的处理才是正确的，对此他给的解释是`APM send SCALED_IMU data as RAW_IMU`，我不知道ArduPilot是如何，但是PX4发布imu_raw数据的话题抬头就是SCALED_IMU（见px4固件的头文件`SCALED_IMU.hpp`）
    - 然而他判断飞控类型是三类，先判断是否为ArduPilot固件，然后再else if PX4固件，对其他的不做数据转换处理
    - 目前市面上的PX4能够正常使用vins，其实是因为mavros会错误的将PX4判断为ArduPilot固件，然后就不进行后面的else判断了，进而负负得正，乘以了正确的系数
    - 最后的问题终于找到，mavros默认系统id是1，bf固件默认系统id的是0，所以可以修改固件中所有mavlink打包函数（例如mavlink_msg_heartbeat_pack()）中的uint8_t system_id为1，也可以修改px4.launch中的tgt_system值为0:&lt;arg name="tgt_system" default="0" /&gt;。至此，可以通过修改mavlink_msg_heartbeat_pack()函数中的参数uint8_t autopilot，来使得mavros将飞控解析为各种类型，但由于前面说的bug，故需要设置为MAV_AUTOPILOT_ARDUPILOTMEGA
- planner运行起来了，但是大概率由于px4飞控的imu是倒着安装的，各个轴的对应目前还在调，有可能出现轴反向的情况，体现在测试中就是位置点原地就开始大幅漂移，程序运行一段时间后就会崩溃，报错信息显示内存越界，该报错相对符合前面的推论，有待进一步debug
### 2024.10.30 -by Nyf
- 本项目的视觉部分短暂搁置，计划优先对飞控的部分进行修改
### 2024.11.03 -by Nyf
- 飞控或者mavros源码中找不到的头文件或者声名，大概率出现在`opt/ros/noetic/include`里
- 关于两个话题mavros/imu/data和mavros/imu/data_raw的合并问题，对于ROS2，这个bug已经在最新的2.9.0版本改了（其中开头的'2'表示适配ROS2的版本），但是对ROS1最新的1.20.0版本还是没改，我去反映一下
### 2024.11.08 -by Nyf
- 按照前几次笔记的方法，成功适配ROS2 noble，说明还真不是bug的问题，但是这仍然是一个好消息
- 这两天在调雷达时由于新买的nuc的cpu版本太新，进入设置-关于，里面的显卡显示为llvmpipe (LLVM 12.0.0, 256 bits)，也即纯软件渲染无显卡，说明核显不在工作，这也导致跑FAST-LIO2并运行Rviz实时查看点云时及其卡顿。
- 经检查发现是内核版本过低，而如果要升级内核，ubuntu20.04又不再支持了，故以后又全部转为ubuntu20.04和ROS2，且不再改变了，人不能总是留在过去，必须向前看。
