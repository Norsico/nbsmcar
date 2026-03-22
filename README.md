# NBSMCar - 智能车软件开发

基于逐飞科技 STC32G144K 100Pin 单片机开源库进行智能车软件开发。

## 硬件特性

- **芯片**: STC32G144K246 (100Pin)
- **主频**: 最大 96MHz
- **工作电压**: 3.3V

## 项目结构

```
NBSMCar/
├── 00参考代码/                  # 参考示例
│   ├── 逐飞官方示例/            # Coreboard & Motherboard 示例
│   ├── 华北理工镜头组STC开源/   # 5ms 周期开源代码
│   ├── 杭科院STC开源/          # 2024 国一第五开源
│   └── 路边野生开源代码/
│
├── 01Tools/                    # 开发工具
│
├── 02libraries/                # ZF 逐飞库
│   ├── zf_common/             # 公共基础
│   ├── zf_components/         # 组件（USB、逐飞助手）
│   ├── zf_device/             # 设备驱动
│   └── zf_driver/             # 底层驱动
│
└── project/                     # 用户代码
    ├── 00example/              # 模块参考示例
    ├── code/                   # 核心代码
    │   └── dev/                # 硬件驱动（电机、舵机、编码器、IMU等）
    ├── user/                   # 应用层
    │   └── app/                # 应用模块（按键、显示）
    └── mdk/                    # Keil 工程
        └── out_file/           # 编译输出
```

## 核心代码 (project/code/)

| 文件 | 说明 |
|------|------|
| `dev_motor.c/h` | 电机驱动 |
| `dev_servo.c/h` | 舵机驱动 |
| `dev_encoder.c/h` | 编码器驱动 |
| `dev_imu.c/h` | 陀螺仪驱动 |
| `dev_wheel.c/h` | 轮子控制 |
| `dev_wifi.c/h` | WiFi 通信 |
| `dev_other.c/h` | 蜂鸣器、激光笔 |
| `pid_control.c/h` | PID 控制器 |
| `SearchLine.c/h` | 寻线算法 |
| `system_state.c/h` | 系统状态管理 |
| `tuning_param.c/h` | 调参参数 |

## 应用层 (project/user/)

| 文件 | 说明 |
|------|------|
| `app_key.c/h` | 按键处理（短按/长按） |
| `app_display.c/h` | 屏幕显示 |
| `app_display_pro.c/h` | 高级显示 |
| `isr.c/h` | 中断服务程序 |
| `main.c` | 主程序入口 |

## 分支说明

- `main` - 主开发分支
- `develop` - 功能开发分支

## 编译说明

1. 使用 Keil MDK 打开 `project/mdk/seekfree.uvproj`
2. 建议使用 `-O7` 优化级别
3. 编译输出位于 `project/mdk/out_file/`

## 开发规范

- 循环变量须在函数开头声明
- 中断处理函数中禁止执行耗时操作
- 使用项目标准数据类型（`uint8`、`uint16`、`int16` 等）
