# STM32F103_TIM_Encode
STM32F103C8T6 利用定时器的编码模式识别转速和方向 ，MDK工程

# 编码器说明
 - 使用的编码器型号为: amron E6B2-CWZ3E
 - 各颜色线定义如下表
            | 线色 | 端子名称 |
            | ---- | -------- |
            | 褐色 | 电源+VCC |
            | 黑色 | 输出A相  |
            | 白色 | 输出B相  |
            | 橙色 | 输出Z相  |
            | 蓝色 | GND      |

 - 编码器需要5~12V的直流输入

# STM32CBT6开发板
 - 利用STM32F103 定时器TIM3的编码器模式识别编码器的旋转角度。 输入引脚 PA0、 PA1 分别识别编码器A、 B 相的输出。
 - PA2配置外部中断模式，捕获Z相的脉冲;
 - 通过SEGGER JLINK组件打印调试信息。
    - 搜索map文件中的_SEGGER_RTT的RAM地址,
    - 打开J-LINK RTT Viewer，如下图设置，上一步查到的_SEGGER_RTT填入最后的 RTT Control block框中，点击OK即可查看调试信息
    - ![JLINK RTT Viewer](D:\Projects\STM32_TIMEncoder\STM32F103_TIM_Encode\doc\JLINK RTT Viewer.png)
 - 开发板上有5V输出引脚，该引脚与编码器的电源VCC相连即可。
 - 目前编码器A/B相的输入是与开发板的PA0 PA1直接相连接的。而编码器A/B相的输出电压是5V, 开发板识别的电压是3.3V.所以有所隐患，但是实际测试可以正常运行。