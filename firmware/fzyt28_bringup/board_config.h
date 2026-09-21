#pragma once
// ============================================================================
// 板级配置 —— 微雪 ESP32-S3-DEV-KIT-N8R8
// 依据：docs/ESP32-S3-DEV-KIT-N8R8-schematic.pdf（微雪官方原理图）
// ============================================================================
//
// 已被板载电路或模组占用、不要拿来接电机的 GPIO：
//   GPIO19/20  原生 USB（经 CH334F 集线器接到 Type-C）
//   GPIO43/44  UART0（U0TXD/U0RXD，接 CH343P 串口芯片，即 Arduino 的 Serial0）
//   GPIO38     板载 WS2812B RGB 灯
//   GPIO35~37  N8R8 模组内部的 Octal PSRAM（乐鑫文档：不建议他用）
//   GPIO0/3/45/46  启动配置引脚（strapping），上电时电平会影响启动
//   GPIO0      同时是板上 BOOT 键，本固件用它退出透传模式
//
// 电机串口选 GPIO17/18：普通 IO，不与上面任何功能冲突，
// 在排针 P1 上相邻（第 9、10 脚），接线方便。

#define MOTOR_TX_PIN        17      // ESP32 TX  →  FZYT-28 PIN3 (RX)
#define MOTOR_RX_PIN        18      // ESP32 RX  ←  FZYT-28 PIN4 (TX)

// FZYT-28 实测波特率：1,000,000 8N1（飞特 SCS 协议，见 scs_protocol.h）
// 旧 RV1126B 云台控制板协议（ptz 命令）用的是 115200，需要时用 baud 115200 切换
#define MOTOR_DEFAULT_BAUD  1000000

// 是否在上电时发旧云台协议的 MasterOn 帧（FC 80 11 09 00 96）。
// 已确认电机本身不认这套协议，默认关闭。
#define PTZ_MASTER_ON_AT_BOOT  0

// 舵机（标准 50Hz 脉宽舵机，如 SG90 / MG90S / MG996R）
#define SERVO_PIN           4       // 排针 P1 第 3 脚，信号线（3.3V 电平，舵机都认）
#define SERVO_MIN_US        500     // 默认"全开(0°)"脉宽（可用 servo setopen 标定后存 NVS 覆盖）
#define SERVO_MAX_US        2500    // 默认"全关(180°)"脉宽（可用 servo setclosed 标定）
#define SERVO_MAX_DEG       180
// 标定探索用的脉宽硬限（比 500~2500 略宽，方便找到机构真正的全开/全关端点；
// 多数 MG/SG 舵机能接受，若在极限处发出嗡嗡声/发烫说明顶到机械限位，往回退一点）
// MG90S 标准行程约 180°（500~2500µs），转不过自己的机械限位。开合对齐靠半齿轮机械自调，
// 固件给标准 0-180° 即可。这两个是脉宽硬限（= 0°/180°），也是标定端点的默认值。
#define SERVO_EXPLORE_MIN_US SERVO_MIN_US
#define SERVO_EXPLORE_MAX_US SERVO_MAX_US

// 灯（PWM 调光）：小 LED 经 330Ω 直接驱动；灯带/大功率灯经 MOSFET 模块
#define LED_PWM_PIN         5       // 排针 P1 第 4 脚
#define LED_PWM_FREQ        5000    // 5kHz，肉眼和相机都看不到闪烁
#define LED_PWM_BITS        12      // 0~4095

#define RGB_LED_PIN         38      // 板载 WS2812B
#define BOOT_KEY_PIN        0       // 板载 BOOT 键，按下为低电平

// 串口 scan 命令依次尝试的波特率
static const uint32_t SCAN_BAUDS[] = {
    9600, 19200, 38400, 57600, 115200, 230400, 250000,
    460800, 500000, 921600, 1000000,
};
