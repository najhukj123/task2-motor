# STM32 电机控制：第二阶段

基于 STM32F103 的直流减速电机控制工程。目前包含：

- TIM1 PWM 驱动电机
- TIM2 编码器模式读取速度与圈数
- 串口命令与 VOFA+ 波形输出
- 速度位置式 PI 控制
- 位置 PD 控制
- 旋转编码器在线调整参数

## 目前重点

先完善速度 PI 的代码结构与调参流程，再继续位置控制和旋钮参数映射。

## 关键文件

- `Core/Src/main.c`：控制逻辑、PID、串口解析和旋钮处理
- `Core/Src/tim.c`：TIM1、TIM2、TIM3 配置
- `Core/Src/usart.c`：串口配置
- `Core/Src/gpio.c`：GPIO 与外部中断配置
- `task2_motor.ioc`：STM32CubeMX 工程配置
- `docs/TABLET_STUDY.md`：在平板上学习和改代码的方法

## 使用原则

1. CubeMX 自动生成区域之外的代码不要随意修改。
2. 每次只改一个小功能，并单独提交。
3. 平板上只能做代码阅读、注释和逻辑修改；编译、烧录、波形测试回到电脑后进行。
4. 未经硬件验证的提交保留在 `tablet-study` 分支，通过测试后再合并到 `main`。

