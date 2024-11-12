## Betaflight4.4.0固件的二次开发指南 -- Nyf
Remark: 这里只对我相对于源码做出的改动予以说明，详细的修改原因和修改后的效果等请查看主文件夹下的README.md文件
### Mavlink消息
- <u>**在文件mavlink.c中**</u>，`mavlink_msg_xxxxx_pack`函数定义了各种不同类型的mavlink消息，这些消息会被上位机的mavros转化为各种ROS topic以供订阅
- <u>**在文件mavlink.c中**</u>，修改了`mavlinkStreamTrigger`函数，修复了源码中mavlink数据发送频率自动减半的bug
- <u>**在文件mavlink.c中**</u>，添加了`mavlinkSendImuRawData`函数，用于发送IMU的角速度和线加速度
- <u>**在文件mavlink.c中**</u>，添加了`mavlink_msg_heartbeat_pack`函数，使得mavros可以将Betaflight飞控认为是Ardupilot的autopilot，避免被认为是Generic的autopilot
- <u>**在文件mavlink.c中**</u>，修改了`processMAVLinkTelemetry`函数，使得上面我定义的mavlink消息能和默认的消息一样被发送
- <u>**在文件task.c中**</u>，修改了文件里的[TASK_TELEMETRY]任务的参数，使得mavlink数据的发送频率进一步提高

### IMU数据
- <u>**在文件mavlink.c中**</u>，添加的函数`mavlinkSendImuRawData`中涉及一些看起来"莫名其妙"缩放因数，这是因为我测试的飞控使用的IMU是MPU6500，其意义如下
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
- <u>**在文件cli_debug_print.h和rx.h中**</u>，都加上`#define USE_CLI_DEBUG_PRIN`，解锁cli打印debug的功能
- 在cli.c文件里可以进行打印debug，建议在`#if defined(USE_BOARD_INFO)`后的条件编译作用域内进行，我在那里有一个example
- cli不支持浮点数打印，所以如果进行debug，可以转为int类型
- *也可以在引用头文件后在程序的其他地方打印，或者直接在task.c里新建一个打印任务，但不建议这么做

### 接收机与遥控器

### MSP
- MSP就是连接飞控和电脑的那根线用的协议，电脑上的地面站配置飞控就基于该协议

### 模式
- Betaflight的所谓模式，就是遥控器上各个按键位于特定位置时表达的特定含义，也可以理解为允许某些状态共存的状态机，每个状态只有处于或不处于两种情况。这些模式在re_modes.h的枚举变量`boxId_e`中列出，分为以下三类
    - 1.起飞模式：只有一种，顾名思义
    - 2.飞行模式：说明飞机的运动控制情况，比如BOXANGLE即前面说的角度模式，再比如BOXGPSRESCUE即前面说的GPS救援模式
    - 3.RC模式：将某些键位或通道设置控制某些外设比如舵机，黑盒子等
- 模式唯一的设置方法是在`case MSP_SET_MODE_RANGE:`中，也即通过MSP，不过二次开发时也可以在这里编程以增加自定义的模式比如offboard模式
- 一个模式由一个特殊的数据结构管理————结构体modeActivationCondition_t，在re_modes.h里定义，其包含的成员
    - boxId_e modeId，模式名称
    - uint8_t auxChannelIndex，控制该模式的遥控器通道
    - channelRange_t range，在通道的哪些范围内时该模式生效（或者说处于该模式）
    - boxId_e linkedTo，与该模式关联的其他模式
    - modeLogic_e modeLogic，该模式与关联模式之间的关系，只能是and或者or
    - 其关系通过and掩码和or掩码管理，管理逻辑在rc_mode.c的`updateMasksForMac`函数中体现，不在具体解释
- 模式的定义与管理非常复杂，简而言之就是通过一个宏定义，这个宏定义了元素为上述数据结构的数组的同时，也定义了一些访问该数组的函数，这些函数有的只读（modeActivationConditions），有的可读可写（modeActivationConditionsMutable）
- Betaflight提供了四个用户模式（BOXUSERx），可以通过在msp_box.c文件的`initActiveBoxIds`函数里调用宏宏`BME(BOXUSER1)`来启用，然后打开Betaflight地面站，就可以在"模式"一栏中找到USER1模式，同样的，也可以如此设置USER2~USER4模式

### offboard模式
- <u>**在文件runtime_config.h中**</u>，枚举变量flightModeFlags_e中添加了OFFBOARD_MODE
- <u>**在target文件夹下F405和H743的target.h中**</u>，都添加宏定义`#define USE_OFFBOARD_MODE`
- <u>**在文件core.c中**</u>，添加`#ifdef USE_OFFBOARD_MODE...`，使得BOXUSER1可以控制offboard模式的开启与关闭
- <u>**在文件msp_box.c中**</u>，添加`#ifdef USE_OFFBOARD_MODE...`，使得可以通过地面站设置控制offboard模式的通道和范围等参数



ENABLE_FLIGHT_MODE

IS_RC_MODE_ACTIVE
isRangeActive
