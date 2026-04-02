# AGENTS.md

本文件对整个仓库生效，内容保持精简，只记录长期有用的协作信息。

## 1. 易错事项

- `.c` 和 `.h` 默认统一使用 UTF-8 编码。批量修改编码前，先执行干跑检查：
  `python3 01Tools/convert_c_headers_to_utf8.py . --dry-run --summary-only`
- 默认把 `project/` 视为当前正在开发的主工程；`00参考代码/` 主要用于查实现、对比方案、借鉴接口，不要默认把里面的代码当成当前生效代码。
- 除非明确需要，不要改动 `project/mdk/out_file/`、参考工程里的 `Out_File/` 这类生成目录。
- `project/mdk/seekfree.uvproj` 属于工程配置文件；只有在增删源码、调整分组或修正工程路径时才改。
- `project/code/` 或 `project/user/` 下新增 `.c` 文件后，要同步检查 `project/mdk/seekfree.uvproj` 是否已加入对应编译分组，否则 Keil 不会参与编译。
- 修改或重构现有代码时，不要删除用户原有注释；新增或整理代码时，沿用仓库现有中文注释风格，至少给主流程、状态切换、关键配置和硬件相关逻辑补上简洁注释。
- 注释要简洁、直接，优先说明关键行为或原因，不要写无意义注释。
- 用户后续需要手动调整的宏定义，必须逐项写清楚含义、范围和步进，不能只留一行总注释。
- 手调范围不能拍脑袋乱填：能直接从底层注释/驱动约束拿到的，就按来源写；拿不到明确范围的，要在注释里写清楚这是工程侧推定范围，以及推定依据。
- 不要乱加宏定义。模块内局部宏放对应 `.c` 文件顶部；跨文件共用宏放对应 `.h` 或统一配置头文件，不要散落在业务逻辑中间。
- 不要乱加标志位。标志位优先收口到明确的状态管理文件，例如 `project/code/system_state.*`，不要分散写在多个业务文件里。
- Keil C251 按 C89 规则编译，局部变量必须放在函数或代码块开头，不能写在执行语句后面。
- 发现新的仓库级长期注意事项时，直接追加到本文件，不再单独建记忆文件。

## 2. 仓库结构

```text
smcar_code/
├── .vscode/
│   └── settings.json                 # VS Code 工作区配置
├── 00参考代码/                        # 参考工程，不是当前主工程
│   ├── 逐飞官方示例/
│   │   ├── Coreboard_Demo/
│   │   └── Motherboard_Demo/
│   ├── 华北理工镜头组STC全国第一开源5ms/
│   │   └── Project/
│   ├── 杭科院STC2024国一第五开源/
│   │   └── Project/
│   └── 路边野生开源代码/
├── 01Tools/
│   ├── check_enc.py
│   └── convert_c_headers_to_utf8.py
├── 02libraries/                      # 逐飞底层库
│   ├── doc/
│   ├── zf_common/                    # 公共头文件、启动文件、基础工具
│   ├── zf_driver/                    # ADC/UART/PWM/SPI 等底层驱动
│   ├── zf_device/                    # 屏幕、IMU、摄像头、WIFI SPI 等设备封装
│   └── zf_components/
│       ├── seekfree_assistant/       # 逐飞助手通信与调参
│       └── stc32g_usb_lib/           # USB 相关组件
├── project/                          # 当前主工程
│   ├── 00example/                    # 模块/初始化示例
│   ├── user/
│   │   ├── main.c
│   │   ├── isr.c
│   │   ├── isr.h
│   │   └── app/                      # 应用层模块（显示、按键、巡线入口等）
│   ├── code/
│   │   ├── SearchLine.c
│   │   ├── ackerman.c
│   │   ├── pid_control.c
│   │   ├── system_state.c
│   │   ├── tuning_param.c
│   │   ├── wifi_tuning_config.h
│   │   ├── 本文件夹作用.txt
│   │   └── dev/                      # 设备/执行器驱动封装
│   └── mdk/                          # Keil 工程文件与输出目录
│       ├── seekfree.uvproj
│       ├── seekfree.uvopt
│       ├── seekfree.uvgui.lenovo
│       ├── MDK删除临时文件.bat
│       └── out_file/
├── README.md
├── CLAUDE.md
└── AGENTS.md
```

- `project/code/dev/` 主要放电机、舵机、编码器、IMU、显示、WiFi 等底层封装。
- `project/user/app/` 主要放业务层模块，例如显示菜单、按键逻辑、巡线入口。
- `project/00example/` 是当前主工程内的示例代码，可参考初始化和模块调用方式，但不等于主流程一定会执行。

## 3. 历史记忆

### 2026-03-11

- 仓库内 `.c` 和 `.h` 已统一转换为 UTF-8；`.vscode/settings.json` 当前默认编码为 `utf8`。
- 如果 Keil 报 `unprintable character`，先检查源码注释里是否混入控制字符，不要先假设是编译器不支持 UTF-8。
- 仓库顶层目录当前使用 `01Tools/` 和 `02libraries/`，Keil 工程路径也需要保持同步。
- 当前车辆是四轮结构：前轮由舵机控制转向，后轮由两路直流电机驱动。
- 已确认 `RIGHT_MOTOR` 映射到 `IO_P75` / `PWMB_CH1_P74` 这一组。
- 当前前轮舵机默认使用 `PWME_CH3P_PA4`；以 `project/code/dev/dev_servo.h` 中的限幅宏为准，当前为 `80 / 90 / 110`。
- 当前 `MT9V03X` 图像最右侧最后一列按历史约定视为无效黑边；底层宽度保持 `188`，业务层按前 `187` 列作为有效图像处理和显示。
- 高速 `WIFI SPI` 图传与 `IPS200` 屏幕不能同时作为主要显示链路使用，调试图传时应停掉屏幕整图刷新。

### 2026-04-02

- `Param Config` 页面已经做过一轮轻量化调整：去掉多余横线和加粗尝试，当前方案以“左侧短色条 + 当前项文字变色”为主，优先保证刷新速度。
- 当前 UI 键位映射为：`B2/PB2` 返回，`B3/PB3` 上，`B4/PB4` 下，`P32` 确定。
- 调参界面已补上长按加速逻辑：当前由 `B3/PB3` 和 `B4/PB4` 负责连续加减；短按按正常步进，长按约 `800ms` 后按 `10x step` 连发，连发间隔约 `100ms`。
- `B2/PB2` 长按会直接返回主菜单；所有回到主菜单的路径都会把主菜单光标重置到第一个条目。
- 当前这套 UI 调整主要集中在 `project/user/app/app_display.c`，长按调参逻辑主要在 `project/user/app/app_key.c`。
