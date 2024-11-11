## Betaflight4.4.0固件的二次开发指南 -- Nyf
Remark: 这里只对我相对于源码做出的改动予以说明，详细的修改原因和修改后的效果等请查看主文件夹下的README.md文件
### Mavlink消息
- 在文件mavlink.c中，`mavlink_msg_xxxxx_pack`函数定义了各种不同类型的mavlink消息，这些消息会被上位机的mavros转化为各种ROS topic以供订阅
- 在文件mavlink.c中，修改了`mavlinkStreamTrigger`函数，修复了源码中mavlink数据发送频率自动减半的bug
- 在文件mavlink.c中，添加了`mavlinkSendImuRawData`函数，用于发送IMU的角速度和线加速度
- 在文件mavlink.c中，添加了`mavlink_msg_heartbeat_pack`函数，使得mavros可以将Betaflight飞控认为是Ardupilot的autopilot，避免被认为是Generic的autopilot
- 在文件mavlink.c中修改了`processMAVLinkTelemetry`函数，使得上面我定义的mavlink消息能和默认的消息一样被发送
- 在文件task.c中，修改了文件里的[TASK_TELEMETRY]任务的参数，使得mavlink数据的发送频率进一步提高

### IMU数据
- 在文件mavlink.c中，添加的函数`mavlinkSendImuRawData`中涉及一些看起来"莫名其妙"缩放因数，这是因为我测试的飞控使用的IMU是MPU6500，其意义如下
    - BF固件中，可以在accgyro_mpu6500.c中看到这样的定义`int accel_range = INV_FSR_16G`.注意在后面有`busWriteRegister(dev, MPU_RA_ACCEL_CONFIG, accel_range << 3)`的程序，这是因为寄存器写入需要位移，并不代表量程放大了2^3倍，而MPU6500的精度是16位，也即-32768到32767，所以假设读取的16位有符号整型的数据为1000，则说明真实的加速度是1000 \* 16G/32768G，或者说是1000/2048G
- 针对不同的飞控，需要根据其IMU型号来修改这些参数，但是一定要保证：
    - mavlink发送出去的加速度的单位G/1000，也即'毫重力加速度'
    - mavlink发送出去的角速度单位是毫弧度/秒，即millirad/s

### PID控制器
#### 这里的PID控制器是串级的，但是外环Kp=1，Ki和Kd都为0
- 当飞行模式处于角度模式或GPS救援模式时，控制对象是角度，此时角速度环的期望值是遥控器通道值-当前角度  
    - 但是通常理解应该是期望值就是遥控器的通道值，如果减去当前角度不就变成误差了吗？
    - 实际上这里可以理解为变成了外环角度环，内环角速度环的串级控制器，只是角度环的期望角度值是0（这两个模式下要自动保持水平），Kp=1，Ki和Kd=0
    - 众所周知外环的输出是内环的期望值，而对于这种情况，外环的输出是：Kp \* (遥控器通道值-0)+Ki \* ∫(遥控器通道值-0)+Kd \* d(遥控器通道值-0)/dt
    - 而Ki和Kd都是0，所以角度环的输出就变成1 \* (遥控器通道值-0)=遥控器通道值，并输入了角速度环作为期望值
    - 而在这两个模式下，期望的角速度是0.所以内环的输出是Kp \* (期望值-当前值)+Ki \* ∫(期望值-当前值)+Kd \* d(期望值-当前值)/dt
    - = Kp \* (遥控器通道值-当前角速度)+Ki \ \*  ∫(遥控器通道值-当前角速度)+Kd \* d(遥控器通道值-当前角速度)/dt
- 当飞行模式处于HORIZON模式时，飞机同时具备角度模式下的稳定和角速度模式下的可操控性，控制对象是角速度和角度的加权，
    - 此时遥控器的输入既是角速度期望值，也在归一化后转化为了角度期望值，角度期望值同样作用与没有Ki和Kd，Kp=1的外环，即角度环
    - 角速度期望值和外环的输出作加权和，作为角速度环的输入，使得飞机具备前面所说的控制性能

### Debug
- 修改了两个文件：在cli_debug_print.h和rx.h前都加上`#define USE_CLI_DEBUG_PRIN`，解锁cli打印debug的功能
- 在cli.c文件里可以进行打印debug，建议在`#if defined(USE_BOARD_INFO)`后的条件编译作用域内进行，我在那里有一个example
- cli不支持浮点数打印，所以如果进行debug，可以转为int类型
- *也可以在引用头文件后在程序的其他地方打印，或者直接在task.c里新建一个打印任务，但不建议这么做

### 接收机与遥控器


### offboard模式

ENABLE_FLIGHT_MODE

IS_RC_MODE_ACTIVE
isRangeActive
