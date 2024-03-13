# 1、介绍
### betaflight源代码适配于可支持betaflight固件的飞控，可用于无人机的需求功能开发。
# 2、操作流程
## 2.1 编译流程
### 首先在ubuntu20.04下clone源码，并下载好gcc交叉编译器
```
  git clone https://github.com/NeSC-ZJU/BetaFlight_Nesc.git
```
### 交叉编译器的下载版本头
```
  arm-none-eabi-gcc
```
### [gcc源码包](https://developer.arm.com/downloads/-/gnu-rm) 
### gcc编译器使用操作步骤 [gcc使用方法](https://blog.csdn.net/zhengyangliu123/article/details/54783443)
### 然后我们需要在工程目录下进行
```
  make clean
```
### 最后按照以下格式输入,其中STM32H743为使用芯片的型号
```
  make TARGET=STM32H743
```
# 3、获得编译后的固件
### 固件的格式
```
  xxx.hex
```
