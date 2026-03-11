# AGENTS.md

本文件对整个仓库生效。

## 注意事项

- `.c` 和 `.h` 默认统一使用 UTF-8 编码。
- 批量修改编码前，先执行干跑检查：
  `python3 01Tools/convert_c_headers_to_utf8.py . --dry-run --summary-only`
- 除非用户明确要求，否则不要改动 `project/mdk/`、`Out_File/` 这类生成目录。
- `project/code/` 可以放业务代码，但新增 `.c` 后要同步加入 `project/mdk/seekfree.uvproj`，否则 Keil 不会参与编译。
- 发现新的仓库级长期注意事项时，直接追加到本文件，不再单独建记忆文件。

## 仓库结构

```text
smcar_code/
├── .vscode/
│   └── settings.json                 # VS Code 工作区配置
├── 02libraries/                      # 逐飞底层库
│   ├── zf_common/                    # 公共头文件、启动文件、基础工具
│   ├── zf_driver/                    # ADC/UART/PWM/SPI 等驱动
│   ├── zf_device/                    # 屏幕、IMU、摄像头、WIFI SPI 等设备封装
│   └── zf_components/
│       ├── seekfree_assistant/       # 逐飞助手通信与调参
│       └── stc32g_usb_lib/           # USB 相关组件
├── project/                          # 当前主工程
│   ├── user/
│   │   ├── main.c
│   │   ├── isr.c
│   │   └── isr.h
│   ├── code/
│   │   └── 本文件夹作用.txt
│   └── mdk/                          # Keil 工程文件与输出目录
│       ├── seekfree.uvproj
│       ├── seekfree.uvopt
│       └── out_file/
├── 00参考代码/                        # 参考工程，不是当前主工程
│   ├── 逐飞官方示例/
│   │   ├── Coreboard_Demo/
│   │   └── Motherboard_Demo/
│   ├── 华北理工镜头组STC全国第一开源5ms/
│   └── 杭科院STC2024国一第五开源/
├── 01Tools/
│   └── convert_c_headers_to_utf8.py # 批量统一 `.c/.h` 编码
└── AGENTS.md
```

- 默认把 `project/` 视为当前正在开发的主工程。
- `00参考代码/` 主要用于查实现、对比方案、借鉴接口，不要默认把里面的代码当成当前生效代码。

## 记忆

### 2026-03-11

- 仓库内 `.c` 和 `.h` 已统一转换为 UTF-8。
- VS Code 工作区默认编码已在 `.vscode/settings.json` 中设置为 `utf8`。
- 如果 Keil 报 `unprintable character`，先检查源码注释里是否混入了控制字符，不要先假设是编译器不支持 UTF-8。
- `project/code/` 下新增的 `.c` 文件需要同时登记到 `project/mdk/seekfree.uvproj` 的 `code` 组。
- 仓库顶层目录当前使用 `01Tools/` 和 `02libraries/`，Keil 工程路径也需要保持同步。
