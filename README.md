## 该分支用于测试mavROS，实现BF固件的F405飞控与Opi的通信
### 软硬件依赖
- 硬件
    - 香橙派5-Pro 64G
    - Aocoda-RC F405飞控 MPU6500版
    - Ch340 USB转串口
- 软件
    - Ubuntu 24.04 (noble)
    - Betaflight V4.3.2
    - ROS2 (jazzy)
    - ROS2使用C++工具包 
### 2024.10.08 bt Nyf --node helloworld
- 实现了使用ROS2节点在终端中打印helloworld
    - 首先是初始化，创建一个工作空间（名字自取）和子目录src，进入工作空间并执行`colcon build`，然后会发现工作空间多了build，install和log文件夹
    - 使用`ros2 pkg create Name1 --build-type ament_cmake --dependencies rclcpp --node-name $(Name2)`创建功能包，其中Name1是src下的文件夹，Name2既是将自动创建的cpp文件名，也是最后生成可执行文件的名字
    - 编写程序
    - 用scode作文本编辑时，出现`rclcpp.hpp no such file or directory`报错，但不影响colcon的编译和最后的运行只是红波浪线看着难受，通过在vscode工作区添加.vscode文件夹并编辑json文件修复该报错
    - 然后返回工作区目录并下使用`colcon build`进行编译
    - 使用`. install/setup.bash`命令
    - 使用`ros2 run Name1 Name2`执行前面编译好的可执行文件
- 针对官方的还是自定义的功能包，都可以在.home目录下的隐藏文件bashrc中的最后一行添加命令，`source /opt/ros/jazzy/setup.bash source` 和 `source /home/nyf/code/ROS2_workspace/hello_world/install/setup.bash`，前者是官方，后者是自定义的，当然后者的路径要匹配，这样就可以省略倒数第二步

