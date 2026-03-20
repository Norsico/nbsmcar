# 智能车软件开发

在逐飞科技 STC32G144K 100Pin 单片机开源库的基础上进行修改，智能车开发。

## 硬件特性

- **芯片**: STC32G144K246 (100Pin)
- **主频**: 最大 72MHz
- **工作电压**: 3.3V

## 项目结构

```
├── 00参考代码/          # 参考示例
│   ├── 逐飞官方示例/
│   ├── 华北理工镜头组STC全国第一开源5ms/
│   ├── 杭科院STC2024国一第五开源/
│   └── 路边野生开源代码/
├── 01Tools/            # 工具
├── 02libraries/        # ZF 库
│   ├── zf_common/      # 公共基础（时钟、调试、中断、类型定义、FIFO等）
│   ├── zf_components/  # 组件（USB库、逐飞助手）
│   ├── zf_device/      # 设备驱动（屏幕、摄像头、IMU、WiFi等）
│   └── zf_driver/      # 底层驱动（GPIO、UART、PWM、SPI、ADC等）
└── project/            # 用户项目代码
    ├── 00example/      # 测试后的模块参考示例
    ├── code/           # 核心代码
    ├── mdk/            # Keil 工程
    │   └── out_file/   # 编译输出
    └── user/           # 应用层（按键、显示、中断等）
        └── app/        # 应用模块
```

## 目录说明

### 00参考代码 - 参考示例

测试后的模块参考示例，包含逐飞官方示例和各高校开源代码。

### 02libraries - ZF 库

| 目录 | 说明 |
|------|------|
| `zf_common/` | 公共基础：时钟、调试、中断、类型定义、FIFO、字体等 |
| `zf_driver/` | 底层驱动：GPIO、UART、PWM、SPI、ADC、EXTI、定时器、PIT、EEPROM、延时等 |
| `zf_device/` | 设备驱动：屏幕(IPS/TFT)、摄像头(MT9V03X)、IMU(IMU660RA/IMU963RA)、WiFi、蓝牙等 |
| `zf_components/` | 组件：STC32G USB库、逐飞助手协议 |

### project - 用户代码

| 目录 | 说明 |
|------|------|
| `00example/` | 测试后的模块参考示例 |
| `code/` | 核心代码（车辆控制、PID、寻线等） |
| `user/` | 应用层（按键、显示等） |
| `mdk/` | Keil MDK 工程文件 |

## 分支说明

- `main` - N 开发分支
- `develop` - C 开发分支

## 编译说明

1. 使用 Keil MDK 打开 `project/mdk/seekfree.uvproj`
2. 编译选项建议使用 `-O7` 优化级别
3. 编译输出位于 `project/mdk/out_file/`

## 注意事项

- 循环变量必须在函数开头声明，禁止在 for 循环中声明变量
- 中断处理函数中禁止执行耗时操作
- 使用项目标准数据类型（`uint8`、`uint16`、`int16` 等）
