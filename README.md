# 920demo：用 ESP32-S3 控制云台电机 FZYT-28

**目标**：用微雪 ESP32-S3-DEV-KIT-N8R8 替代原来的 RV1126B 主板，通过串口控制 FZYT-28 云台电机。

- **代码仓库**：https://github.com/CodingMcnugget/920demo
- **在线控制页面**（GitHub Pages，HTTPS，Chrome/Edge 打开后蓝牙连接电机）：**https://codingmcnugget.github.io/920demo/webapp/**
  推送 `main` 分支后 GitHub Pages 自动重新发布，约 1 分钟生效。

## 当前状态（2026-09-18）

| 项 | 状态 |
|---|---|
| 开发板资料（官方文档、原理图、引脚分析） | ✅ |
| 硬件连接方案（接线、供电、电平、物料） | ✅ 已按实物接线验证 |
| **电机协议** | ✅ **已实测破解**：飞特 SCS 总线舵机格式，1 Mbps，出厂 ID = 2 |
| **转到指定角度 / 读取当前角度 / 扭矩开关** | ✅ 真机验证通过（90° 往返） |
| 固件（`st` 命令控制电机 + 联调工具） | ✅ 已编译、已烧录、已实测 |
| **持续转动 / 速度控制** | ✅ 由 ESP32 推进目标位置实现（`st run ±度/秒`），实测 60°/s × 2s = 120.0°。电机自身的速度寄存器仍未知 |
| 电机自身的速度 / 加速度寄存器、电压电流读取 | 🔴 地址与标准飞特表不同，需向厂家要寄存器表 |
| **蓝牙 + 网页控制**（Web Bluetooth，Chrome/Edge） | ✅ 已实测：扫描、连接、按住持续转、点动、位置回显 |
| **舵机（GPIO4）+ 灯调光（GPIO5）** | ✅ 固件命令 `servo` / `led`，网页滑条；接线见 docs/01 第 6 节。硬件未接，等实物验证 |
| 第二个电机、多电机总线 | ⏳ 未测。需要改 ID（出厂都是 2，会冲突） |

## 关键结论

1. **原 RV1126B 固件里的 `FC 80 …` 协议不是电机的协议**。那是发给原云台控制板的，FZYT-28 电机本身完全不响应。
2. **电机用飞特 SCS 格式**：`FF FF ID LEN INSTR PARAM CHK`，1,000,000 波特率，16 位寄存器**大端**。详见 [docs/02](docs/02-资料与出处.md) 第 4 节。
3. **出厂扭矩关闭**：上电不转、不锁、手拧是松的。固件默认开启"上电保持"（`st hold on`），ESP32 一开机就把扭矩打开并锁住当前位置，所以现在通电就有劲、推了会回弹。
4. **位置 4096 计数 = 一圈**，限位出厂 0–4095。

## 目录

```
920demo/
├── README.md
├── docs/
│   ├── 01-硬件连接方案.md                   接线图、供电、电平、物料清单、上电检查
│   ├── 02-资料与出处.md                     官方链接、原理图要点、规格书摘要、协议实测结果
│   ├── 03-烧录与控制步骤.md                  烧录、st 命令控制电机、联调工具、向厂家要什么
│   └── ESP32-S3-DEV-KIT-N8R8-schematic.pdf  微雪官方原理图
├── index.html                             跳转到 webapp/（给 GitHub Pages 用）
├── webapp/
│   └── index.html                         网页控制端：Chrome 打开，蓝牙连接 FZYT-28-xxxx
└── firmware/
    └── fzyt28_bringup/                    工程目录（PlatformIO 和 Arduino IDE 通用）
        ├── platformio.ini                 PlatformIO 配置
        ├── fzyt28_bringup.ino             主程序：命令行、电机控制、BLE 服务、联调工具
        ├── scs_protocol.h                 电机协议（飞特 SCS 格式）：组包、解析、已验证寄存器
        ├── ptz_protocol.h                 旧云台控制板协议（电机不认，仅供参考）
        └── board_config.h                 引脚、波特率
```

## 快速开始

1. **接线**：看 [docs/01](docs/01-硬件连接方案.md)。四根线：GPIO17→电机 RX、GPIO18←电机 TX、共地、电机独立供电。
2. **烧录**：看 [docs/03](docs/03-烧录与控制步骤.md) 第 1 节。
3. **控制**：串口监视器（115200）依次输入：
   ```
   st ping          → ID 2 在线
   st pos           → 当前角度
   st move 90       → 转到 90°（自动开扭矩）
   st torque off    → 关扭矩
   ```
4. **网页控制**：Chrome 打开 https://codingmcnugget.github.io/920demo/webapp/ （或本地 `webapp/index.html`）→ 连接电机 → 选 `FZYT-28-xxxx`。

## 接线速查

| ESP32-S3 | | FZYT-28 |
|---|---|---|
| GPIO17 | ———————— | PIN3 RX |
| GPIO18 | ———————— | PIN4 TX |
| GND | ———————— | PIN1 GND（与电机电源负极共地） |
| （不接） | | PIN2 VDD ← 独立电源 **5–10V**（实测 8.1V 正常；**12V 直连会烧**，5V 太贴下限） |

| ESP32-S3 | | 外设 |
|---|---|---|
| GPIO4 | ———————— | 舵机信号线（舵机 VCC 接独立 5V，GND 共地） |
| GPIO5 | —— 330Ω —— | 小 LED 正极（负极接 GND）；灯带经 MOSFET 模块，见 docs/01 第 6 节 |
