// ============================================================================
// FZYT-28 云台电机 —— ESP32-S3 串口控制 + 联调固件
//
// 目标板：微雪 ESP32-S3-DEV-KIT-N8R8
// 目标电机：FANGZHOU FZYT-28（FOC 闭环无刷，串口控制，5~10V）
//           即原 on-device 自研云台（RV1126B UART7 直连）所用的电机
//
// 电机协议（2026-09-18 真机实测）：飞特 SCS 总线舵机格式，1 Mbps，见 scs_protocol.h
//   出厂 ID = 2，扭矩默认关闭，位置 4096 计数 = 一圈。
//
// 旧协议（ptz/jog 命令）：原 RV1126B 云台驱动 librp_mw.so 反汇编得到的 6 字节帧
//   FC 80 <b2> <b3> 00 <sum8>，115200 8N1，只发不收。
//   已确认 FZYT-28 电机本身不响应这套协议，它是发给原云台控制板的。保留仅供参考。
//
// 命令：
//     st      —— 电机控制（ping / 读写寄存器 / 扭矩 / 读位置 / 转到角度 / 扫 ID）
//     ptz     —— 旧云台控制板协议（up/down/left/right/stop/reset/on/off …）
//     jog     —— 旧协议：朝某方向转指定毫秒后自动停止
//     tx      —— 发任意十六进制字节并抓回应，用来验证厂家给的新指令
//     scan    —— 逐个波特率监听，用"帧错误计数"判断电机实际波特率
//     listen  —— 抓电机主动发出的数据
//     bridge  —— USB 透传：厂家上位机经 ESP32 控制电机，另一个 COM 口看双向报文
//     selftest—— TX/RX 短接自检，先排除 ESP32 这一侧的问题
//
// Arduino IDE 设置（微雪官方说明 + 本板硬件）：
//   开发板：ESP32S3 Dev Module
//   USB CDC On Boot：Enabled         （必须，否则编译报错）
//   USB Mode：Hardware CDC and JTAG  （默认值）
//   Flash Size：8MB   PSRAM：OPI PSRAM
//
// 两个 COM 口（板上 CH334F 集线器，一根 Type-C 出两个口）：
//   原生 USB 口 = Serial   ：命令行；bridge 模式下变成透传口
//   CH343 口    = Serial0  ：命令行（与上面镜像）；bridge 模式下显示抓包
// 串口助手行尾选 CR 或 LF 均可，波特率 115200。
// ============================================================================

#include <Arduino.h>
#include <driver/gpio.h>
#include <atomic>
#include "board_config.h"
#include "ptz_protocol.h"
#include "scs_protocol.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_mac.h>

#if !ARDUINO_USB_CDC_ON_BOOT
#error "请在 Arduino IDE 的 工具 → USB CDC On Boot 选择 Enabled"
#endif

HardwareSerial MotorUart(1);

static uint32_t g_baud = MOTOR_DEFAULT_BAUD;
static uint32_t g_waitMs = 200;       // tx 命令发完后等待回应的时间
static bool     g_monitor = true;     // 空闲时是否打印电机主动发来的数据
static bool     g_bridge = false;

// 当前默认操作的电机 ID。开机时 detectMotorId() 自动探测（不同批次电机出厂 ID 可能
// 是 1 或 2，甚至被改过），命令省略 [id] 时就用这个。默认先用出厂值 2 兜底。
static uint8_t  g_motorId = scs::DEFAULT_ID;

// 软件零点：home 处的原始计数，存在 ESP32 的 NVS 里（蓝牙一条命令即可设，不写电机 EEPROM，
// 掉电不丢）。所有角度都相对它显示/下发（左负右正）。
// g_target 是当前命令目标(原始计数)，点动在它上面累加、不必每次先读当前位置——连点也能连续
// 平滑地跟（这是"丝滑"的关键）。g_targetSynced=false 表示还没和电机实际位置对齐过。
static int32_t g_zeroOffset = 0;
static long    g_target = 0;
static bool    g_targetSynced = false;

// 由 UART 事件任务（另一个 FreeRTOS 任务）累加，主循环读取
static std::atomic<uint32_t> g_frameErr{0};
static std::atomic<uint32_t> g_breakErr{0};
static std::atomic<uint32_t> g_otherErr{0};
static uint32_t g_rxTotal = 0;

// ---------------------------------------------------------------------------
// BLE：Nordic UART 服务。网页/手机把文本命令写到 RX 特征，输出经 TX 特征通知回去。
// 命令与串口命令行完全相同（st move 90 / st jog -15 / st torque off ...）。
// ---------------------------------------------------------------------------
#define BLE_SVC_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_RX_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   // 手机 → ESP32（write）
#define BLE_TX_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // ESP32 → 手机（notify）

struct FrameSplitter;   // 前置声明：Arduino 自动生成的原型会插在下面第一个函数之前

static BLECharacteristic *g_bleTx = nullptr;
static volatile bool g_bleConnected = false;
static String g_bleRxBuf;                       // BLE 任务写入、主循环读取
static portMUX_TYPE g_bleMux = portMUX_INITIALIZER_UNLOCKED;
static String g_bleTxBuf;                       // 攒到换行或 100 字节再 notify

static void bleFlushTx() {
    if (!g_bleConnected || !g_bleTx || !g_bleTxBuf.length()) { g_bleTxBuf = ""; return; }
    g_bleTx->setValue((uint8_t *)g_bleTxBuf.c_str(), g_bleTxBuf.length());
    g_bleTx->notify();
    g_bleTxBuf = "";
    delay(3);   // 连续 notify 之间留一点间隔，避免手机端丢包
}

// ---------------------------------------------------------------------------
// 输出：平时两个 COM 口同时输出；bridge 模式下 Serial 在透传，只输出到 Serial0；
// 有 BLE 连接时同时通知给手机
// ---------------------------------------------------------------------------
class ConsoleOut : public Print {
public:
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buf, size_t n) override {
        if (!g_bridge) Serial.write(buf, n);
        Serial0.write(buf, n);
        if (g_bleConnected) {
            for (size_t i = 0; i < n; i++) {
                g_bleTxBuf += (char)buf[i];
                if (buf[i] == '\n' || g_bleTxBuf.length() >= 100) bleFlushTx();
            }
        }
        return n;
    }
};
static ConsoleOut out;

class BleServerCb : public BLEServerCallbacks {
    void onConnect(BLEServer *) override { g_bleConnected = true; }
    void onDisconnect(BLEServer *s) override {
        g_bleConnected = false;
        s->getAdvertising()->start();   // 断开后继续广播，允许再次连接
    }
};

class BleRxCb : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        // Arduino-ESP32 3.x 返回 String，2.x（PlatformIO 官方平台）返回 std::string，
        // 两者都有 c_str()/length()，这样写两边都能编
        auto v = c->getValue();
        taskENTER_CRITICAL(&g_bleMux);
        if (g_bleRxBuf.length() + v.length() < 1024) g_bleRxBuf.concat(v.c_str(), v.length());
        taskEXIT_CRITICAL(&g_bleMux);
    }
};

static String g_bleName;

static void bleBegin() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char name[24];
    snprintf(name, sizeof(name), "FZYT-28-%02X%02X", mac[4], mac[5]);
    g_bleName = name;

    BLEDevice::init(name);
    BLEDevice::setMTU(247);
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new BleServerCb());
    BLEService *svc = server->createService(BLE_SVC_UUID);
    g_bleTx = svc->createCharacteristic(BLE_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    BLECharacteristic *rx = svc->createCharacteristic(
        BLE_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(new BleRxCb());
    svc->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SVC_UUID);
    adv->setScanResponse(true);
    adv->start();
}

// 从 BLE 接收缓冲取出一整行命令；没有完整行返回 false
static bool blePollLine(String &line) {
    taskENTER_CRITICAL(&g_bleMux);
    int nl = g_bleRxBuf.indexOf('\n');
    if (nl < 0) nl = g_bleRxBuf.indexOf('\r');
    if (nl < 0) {
        // 手机端可能不带换行：一次 write 就是一条命令
        if (g_bleRxBuf.length()) { line = g_bleRxBuf; g_bleRxBuf = ""; taskEXIT_CRITICAL(&g_bleMux); return true; }
        taskEXIT_CRITICAL(&g_bleMux);
        return false;
    }
    line = g_bleRxBuf.substring(0, nl);
    g_bleRxBuf.remove(0, nl + 1);
    taskEXIT_CRITICAL(&g_bleMux);
    return true;
}

// ---------------------------------------------------------------------------
// 帧切分：按字节间空闲时间切分。间隔 > 3.5 个字符时间（至少 1.5ms）视为新帧。
// 这是启发式的，协议未知时用来让十六进制输出更易读。
// ---------------------------------------------------------------------------
struct FrameSplitter {
    uint8_t  buf[256];
    size_t   len = 0;
    uint32_t lastUs = 0;

    void push(uint8_t b) {
        buf[len++] = b;
        lastUs = micros();
    }
    bool full() const { return len >= sizeof(buf); }
    bool due() const {
        uint32_t gapUs = max<uint32_t>(35000000UL / g_baud, 1500);
        return len && (micros() - lastUs) > gapUs;
    }
};

static uint8_t xor8(const uint8_t *d, size_t n) {
    uint8_t s = 0;
    for (size_t i = 0; i < n; i++) s ^= d[i];
    return s;
}

// ---------------------------------------------------------------------------
// 接收日志：电机发来的每个字节都记下时间和当时的波特率，存在板子里。
// USB 链路掉线（例如电机上电时的电流冲击）期间 listen/scan 的输出会丢失，
// 重连后用 dump 命令把这段时间收到的内容读出来。
// （放在 FrameSplitter 之后：Arduino 会把自动生成的函数原型插到第一个函数
//   定义之前，若本函数在 FrameSplitter 之前，pumpMotor 的原型会引用未定义类型）
// ---------------------------------------------------------------------------
struct RxLogEntry {
    uint32_t ms;
    uint32_t baud;
    uint8_t  b;
};
static RxLogEntry g_rxLog[512];
static size_t   g_rxLogHead = 0;   // 下一个写入位置
static size_t   g_rxLogCount = 0;  // 有效条数（最多 512，满了覆盖最旧）

static uint8_t motorRead() {
    uint8_t b = (uint8_t)MotorUart.read();
    g_rxLog[g_rxLogHead] = {millis(), g_baud, b};
    g_rxLogHead = (g_rxLogHead + 1) % 512;
    if (g_rxLogCount < 512) g_rxLogCount++;
    g_rxTotal++;
    return b;
}

static void printFrame(Print &p, const char *tag, const uint8_t *d, size_t n) {
    p.printf("[%8lu ms] %s len=%u:", (unsigned long)millis(), tag, (unsigned)n);
    for (size_t i = 0; i < n; i++) p.printf(" %02X", d[i]);

    // 协议摸底提示：末字节是否恰好是前面字节的累加和 / 异或
    if (n >= 3) {
        if (sum8(d, n - 1) == d[n - 1]) p.print("  [末字节=sum8]");
        if (xor8(d, n - 1) == d[n - 1]) p.print("  [末字节=xor8]");
    }

    bool text = n > 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t c = d[i];
        if ((c < 0x20 || c > 0x7E) && c != '\r' && c != '\n') { text = false; break; }
    }
    if (text) {
        p.print("  \"");
        for (size_t i = 0; i < n; i++) {
            if (d[i] == '\r') p.print("\\r");
            else if (d[i] == '\n') p.print("\\n");
            else p.write(d[i]);
        }
        p.print("\"");
    }
    p.println();
}

// ---------------------------------------------------------------------------
// LED：蓝=空闲  绿闪=收到电机数据  紫=透传  红=自检失败
// ---------------------------------------------------------------------------
static uint32_t g_ledRestoreAt = 0;

// Arduino-ESP32 3.x 叫 rgbLedWrite，2.x（PlatformIO 官方平台）叫 neopixelWrite
static void ledSet(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    rgbLedWrite(RGB_LED_PIN, r, g, b);
#else
    neopixelWrite(RGB_LED_PIN, r, g, b);
#endif
}

static void ledMode() {
    if (g_bridge) ledSet(12, 0, 12);
    else ledSet(0, 0, 12);
}

static void ledBlinkRx() {
    ledSet(0, 24, 0);
    g_ledRestoreAt = millis() + 60;
}

// ---------------------------------------------------------------------------
// 电机串口
// ---------------------------------------------------------------------------
static void resetErrors() {
    g_frameErr = 0;
    g_breakErr = 0;
    g_otherErr = 0;
}

static void flushMotorRx() {
    while (MotorUart.available()) MotorUart.read();
}

static void setBaud(uint32_t baud) {
    MotorUart.updateBaudRate(baud);
    g_baud = baud;
    delay(20);
    flushMotorRx();
    resetErrors();
}

static void motorBegin() {
    MotorUart.setRxBufferSize(2048);
    MotorUart.begin(g_baud, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
    // 电机未上电或 TX 没接时 RX 悬空，上拉避免噪声被当成数据
    gpio_pullup_en((gpio_num_t)MOTOR_RX_PIN);
    MotorUart.onReceiveError([](hardwareSerial_error_t e) {
        if (e == UART_FRAME_ERROR) g_frameErr++;
        else if (e == UART_BREAK_ERROR) g_breakErr++;
        else g_otherErr++;
    });
}

// 把当前收到的电机数据灌进切分器；够一帧就打印
static size_t pumpMotor(FrameSplitter &sp, const char *tag, Print *p) {
    size_t frames = 0;
    while (MotorUart.available()) {
        sp.push(motorRead());
        if (sp.full()) {
            if (p) printFrame(*p, tag, sp.buf, sp.len);
            sp.len = 0;
            frames++;
        }
    }
    if (sp.due()) {
        if (p) printFrame(*p, tag, sp.buf, sp.len);
        sp.len = 0;
        frames++;
        ledBlinkRx();
    }
    return frames;
}

// 在 ms 毫秒内收集并打印电机数据，返回收到的帧数
static size_t captureFor(uint32_t ms, const char *tag) {
    FrameSplitter sp;
    size_t frames = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        frames += pumpMotor(sp, tag, &out);
        delay(1);
    }
    delay(5);
    frames += pumpMotor(sp, tag, &out);
    if (sp.len) {
        printFrame(out, tag, sp.buf, sp.len);
        frames++;
    }
    return frames;
}

// ---------------------------------------------------------------------------
// 十六进制解析：接受 "FC 80 01"、"FC8001"、"0xFC,0x80" 等写法
// ---------------------------------------------------------------------------
static bool parseHex(const String &s, uint8_t *dst, size_t cap, size_t &n) {
    n = 0;
    String tok;
    auto flushTok = [&]() -> bool {
        if (!tok.length()) return true;
        if (tok.startsWith("0x") || tok.startsWith("0X")) tok = tok.substring(2);
        if (tok.length() == 0 || tok.length() % 2) return false;
        for (size_t i = 0; i < tok.length(); i++)
            if (!isxdigit((unsigned char)tok[i])) return false;
        for (size_t i = 0; i < tok.length(); i += 2) {
            if (n >= cap) return false;
            dst[n++] = (uint8_t)strtoul(tok.substring(i, i + 2).c_str(), nullptr, 16);
        }
        tok = "";
        return true;
    };
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (isxdigit((unsigned char)c) || c == 'x' || c == 'X') tok += c;
        else if (!flushTok()) return false;
    }
    return flushTok() && n > 0;
}

// ---------------------------------------------------------------------------
// 命令
// ---------------------------------------------------------------------------
static void cmdHelp() {
    out.println();
    out.println("FZYT-28 云台控制固件 —— 命令列表");
    out.printf ("电机控制（飞特 SCS 协议，1 Mbps；[id] 省略时用开机自动识别的 ID=%u）：\n", g_motorId);
    out.println("  st ping [id]                 探测电机");
    out.println("  st scan                      在当前波特率下扫描 ID 0~253，找出所有在线电机");
    out.println("  st pos [id]                  读当前位置（计数和角度）");
    out.println("  st torque on|off [id]        扭矩开/关（出厂默认关，不开不会动）");
    out.println("  st move <角度> [id]          转到绝对角度 0~360（自动打开扭矩），并打印 1.5 秒轨迹");
    out.println("  st go <角度> [id]            转到绝对角度，只回一行结果（网页按钮用）");
    out.println("  st jog <±角度> [id]          相对当前位置转动，例: st jog -15");
    out.println("  st run <±度/秒> [id]         持续转动（每 <700ms 重发一次当心跳，到边界自动停）");
    out.println("  st stop                      停止持续转动/巡航");
    out.println("  st sweep <角A> <角B> [°/s]   头部连环左右巡航（绝对角度；st sweep off 停）");
    out.println("  st cruise <幅度°> [°/s]      以当前位置为中心±幅度来回巡航（st cruise off 停）");
    out.println("  st hold on|off               上电保持：ESP32 开机就锁住电机，扭矩掉了自动补（存 NVS，默认开）");
    out.println("  st read <addr> <n> [id]      读寄存器，例: st read 0x38 2");
    out.println("  st write <addr> <hex..> [id=N] [force]  写寄存器（16 位大端），例: st write 0x2A 07 24");
    out.println("  st home [id]                 软件归零：当前位置设为 home（存 ESP32 NVS，掉电不丢，不写电机）");
    out.println("  st zero confirm [id]         把当前位置记为零位（写电机 EEPROM 0x16，一般用 st home 就够）");
    out.printf ("舵机（GPIO%d，50Hz）与灯（GPIO%d，PWM 调光）：\n", SERVO_PIN, LED_PWM_PIN);
    out.println("  servo <0~180> [ms]           转到角度：0=全开 180=全关 90=半开（按标定端点映射）");
    out.println("  servo us <500~2500> [ms]     直接给脉宽（MG90S 标准范围）");
    out.println("  servo setopen / setclosed    把当前脉宽标定为 全开(0°) / 全关(180°)（存 NVS，掉电不丢）");
    out.println("  servo cal                    显示当前开合标定；servo off 松开；servo speed <°/s> 改限速");
    out.println("  servo sweep <角A> <角B> [°/s] 舵机连环来回巡航（固件自主跑；servo sweep off 停）");
    out.println("  led <0~100> [渐变ms]         灯亮度百分比，可选渐变时间；led on / led off");
    out.println("  led breathe [周期ms] [最小%] [最大%]  呼吸灯（固件自主跑；led off 停）");
    out.println("旧云台控制板协议（115200，电机本身不认，仅供参考）：");
    out.println("  ptz <cmd>            on off up down left right stop stopv stoph reset aion aioff privon privoff");
    out.println("  jog <方向> <ms>      朝 up/down/left/right 转 ms 毫秒后自动停止");
    out.println("调试工具：");
    out.println("  status               当前波特率、引脚、收包与错误计数");
    out.println("  baud <n>             设置电机串口波特率");
    out.println("  scan [ms]            逐个波特率监听（默认每档 1500ms），找电机实际波特率");
    out.println("  listen [ms]          监听电机数据（默认 5000ms）");
    out.println("  mon on|off           空闲时是否自动打印电机发来的数据");
    out.println("  dump [clear]         打印板上缓存的接收日志（USB 掉线期间收到的也在里面）；clear 清空");
    out.println("  tx <hex...>          原样发送字节，并在 wait 毫秒内抓回应");
    out.println("  txsum <hex...>       发送前自动在末尾追加 sum8 校验字节");
    out.println("  wait <ms>            设置 tx 后等待回应的时间（默认 200）");
    out.println("  selftest             TX/RX 短接自检（需拔掉电机、用杜邦线短接 GPIO17-18）");
    out.println("  bridge               USB 透传模式：原生 USB 口<->电机，CH343 口显示抓包；按 BOOT 键或在 CH343 口输入 exit 退出");
    out.println();
}

// 把接收日志按时间间隔切成帧打印。gap 用记录时的波特率算 3.5 字符时间，至少 1.5ms
static void cmdDump(bool clear) {
    if (clear) {
        g_rxLogHead = g_rxLogCount = 0;
        out.println("接收日志已清空");
        return;
    }
    out.printf("接收日志：%u 条（最多保留 512 条，累计收到 %lu 字节）\n",
               (unsigned)g_rxLogCount, (unsigned long)g_rxTotal);
    size_t start = (g_rxLogHead + 512 - g_rxLogCount) % 512;
    uint8_t frame[64];
    size_t n = 0;
    uint32_t frameMs = 0, frameBaud = 0, lastMs = 0;
    auto flush = [&]() {
        if (!n) return;
        out.printf("  [%8lu ms @%lu]", (unsigned long)frameMs, (unsigned long)frameBaud);
        for (size_t i = 0; i < n; i++) out.printf(" %02X", frame[i]);
        out.println();
        n = 0;
    };
    for (size_t i = 0; i < g_rxLogCount; i++) {
        const RxLogEntry &e = g_rxLog[(start + i) % 512];
        uint32_t gapMs = max<uint32_t>(35000UL / max<uint32_t>(e.baud, 1), 2);
        if (n && (e.baud != frameBaud || e.ms - lastMs > gapMs || n >= sizeof(frame))) flush();
        if (!n) { frameMs = e.ms; frameBaud = e.baud; }
        frame[n++] = e.b;
        lastMs = e.ms;
    }
    flush();
}

static void cmdStatus() {
    out.printf("电机串口: UART1  TX=GPIO%d  RX=GPIO%d  %lu 8N1  当前默认电机 ID=%u\n",
               MOTOR_TX_PIN, MOTOR_RX_PIN, (unsigned long)g_baud, g_motorId);
    out.printf("BLE: %s，%s\n", g_bleName.c_str(), g_bleConnected ? "已连接" : "未连接（广播中）");
    out.printf("累计收到: %lu 字节\n", (unsigned long)g_rxTotal);
    out.printf("错误计数: frame=%lu  break=%lu  other=%lu\n",
               (unsigned long)g_frameErr, (unsigned long)g_breakErr, (unsigned long)g_otherErr);
    out.printf("RX 引脚当前电平: %s\n", digitalRead(MOTOR_RX_PIN) ? "高（空闲正常）" : "低（电机未上电 / TX 未接 / 电平异常）");
    if (g_breakErr)
        out.println("提示: break 错误 = RX 线被长时间拉低，先查电机供电、共地和 TX 线");
    if (g_frameErr)
        out.println("提示: frame 错误 = 波特率不对或电平/干扰问题，试试 scan");
}

static void cmdScan(uint32_t msPerBaud) {
    uint32_t original = g_baud;
    uint32_t best = 0;
    size_t bestBytes = 0;

    out.printf("开始扫描，每档监听 %lu ms。扫描期间不要发命令。\n", (unsigned long)msPerBaud);
    for (uint32_t b : SCAN_BAUDS) {
        setBaud(b);
        FrameSplitter sp;
        size_t bytes = 0, frames = 0, solid = 0;
        uint8_t sample[24];
        size_t sampleLen = 0;

        // 单独一个 FF/FE/00 字节多半是电平毛刺（电机上电瞬间 TX 抖一下），
        // 高波特率下会被当成字节收进来，不算作有效数据
        auto isGlitch = [](const FrameSplitter &f) {
            return f.len == 1 && (f.buf[0] == 0xFF || f.buf[0] == 0xFE || f.buf[0] == 0x00);
        };

        uint32_t t0 = millis();
        while (millis() - t0 < msPerBaud) {
            while (MotorUart.available()) {
                sp.push(motorRead());
                bytes++;
                if (sp.full()) { frames++; solid++; sp.len = 0; }
            }
            if (sp.due()) {
                if (!isGlitch(sp)) {
                    solid++;
                    if (!sampleLen) {
                        sampleLen = min(sp.len, sizeof(sample));
                        memcpy(sample, sp.buf, sampleLen);
                    }
                }
                frames++;
                sp.len = 0;
            }
            delay(1);
        }
        delay(20);  // 让错误事件任务把计数更新完

        out.printf("%8lu: %6u 字节 %4u 帧（有效 %u）  frame_err=%-4lu break=%-4lu",
                   (unsigned long)b, (unsigned)bytes, (unsigned)frames, (unsigned)solid,
                   (unsigned long)g_frameErr, (unsigned long)g_breakErr);
        if (sampleLen) {
            out.print("  样例:");
            for (size_t i = 0; i < sampleLen; i++) out.printf(" %02X", sample[i]);
        }
        out.println();

        if (solid && bytes > bestBytes && g_frameErr == 0 && g_breakErr == 0) {
            best = b;
            bestBytes = bytes;
        }
    }

    setBaud(original);
    if (best) {
        out.printf("结论: %lu 收到数据且无帧错误，很可能是电机波特率。执行 baud %lu 切换。\n",
                   (unsigned long)best, (unsigned long)best);
    } else {
        out.println("结论: 没有哪一档同时满足\"有有效数据且无错误\"。");
        out.println("  全部 0 字节或只有毛刺 → 电机不主动上报（FZYT-28 就是这样，用 st ping 主动探测），或供电/接线问题（看 status 的 RX 电平）");
        out.println("  各档都有错误 → 电平不匹配、没共地，或 TX/RX 接反");
    }
    out.printf("已恢复为 %lu\n", (unsigned long)original);
}

static void sendAndCapture(const uint8_t *d, size_t n) {
    flushMotorRx();
    printFrame(out, "ESP->M", d, n);
    MotorUart.write(d, n);
    MotorUart.flush();
    size_t frames = captureFor(g_waitMs, "M->ESP");
    if (!frames) out.printf("（%lu ms 内无回应）\n", (unsigned long)g_waitMs);
}

static void cmdTx(const String &args, bool appendSum) {
    uint8_t buf[250];
    size_t n = 0;
    if (!parseHex(args, buf, sizeof(buf) - 1, n)) {
        out.println("十六进制格式错误，例: tx FC 80 11 09 00 96");
        return;
    }
    if (appendSum) {
        buf[n] = sum8(buf, n);
        n++;
    }
    sendAndCapture(buf, n);
}

static void cmdSelftest() {
    out.println("自检：请确认已拔掉电机、GPIO17 与 GPIO18 已用杜邦线短接。");
    flushMotorRx();
    uint8_t pattern[256];
    for (int i = 0; i < 256; i++) pattern[i] = (uint8_t)i;
    MotorUart.write(pattern, sizeof(pattern));
    MotorUart.flush();

    uint8_t got[256];
    size_t n = 0;
    uint32_t t0 = millis();
    while (n < sizeof(got) && millis() - t0 < 1500) {
        if (MotorUart.available()) got[n++] = (uint8_t)MotorUart.read();
        else delay(1);
    }

    size_t mismatch = 0;
    for (size_t i = 0; i < n; i++)
        if (got[i] != pattern[i]) mismatch++;

    if (n == sizeof(got) && mismatch == 0) {
        out.printf("自检通过：%lu 波特率下 256 字节收发一致。ESP32 串口这一侧没问题。\n",
                   (unsigned long)g_baud);
        ledMode();
    } else {
        out.printf("自检失败：收到 %u/256 字节，%u 字节不一致。检查短接线是否接在 GPIO17 和 GPIO18。\n",
                   (unsigned)n, (unsigned)mismatch);
        ledSet(24, 0, 0);
    }
}

// ---------------------------------------------------------------------------
// 云台控制（协议定义见 ptz_protocol.h）
// ---------------------------------------------------------------------------

// 只发不等回应：原协议没有应答，需要精确计时的场合（jog）用这个
static void sendPtzNow(uint8_t b2, uint8_t b3) {
    uint8_t f[6];
    buildPtz(b2, b3, f);
    MotorUart.write(f, sizeof(f));
    MotorUart.flush();
    printFrame(out, "ESP->M", f, sizeof(f));
}

// 停止：两个轴的停止帧都发。原驱动只给"正在转的轴"发，两个都发效果相同且更稳妥
static void ptzStopAll() {
    sendPtzNow(0x11, 0x07);
    sendPtzNow(0x11, 0x08);
}

static void cmdPtz(String name) {
    name.toLowerCase();
    if (name == "stop") {
        ptzStopAll();
        return;
    }
    const PtzCmd *c = findPtz(name);
    if (!c) {
        out.println("未知 ptz 命令，输入 help 查看");
        return;
    }
    uint8_t f[6];
    buildPtz(c->b2, c->b3, f);
    sendAndCapture(f, sizeof(f));
}

static void cmdJog(const String &args) {
    int sp = args.indexOf(' ');
    String dir = sp < 0 ? args : args.substring(0, sp);
    long ms = sp < 0 ? 0 : args.substring(sp + 1).toInt();
    dir.toLowerCase();

    const PtzCmd *c = findPtz(dir);
    bool isDir = dir == "up" || dir == "down" || dir == "left" || dir == "right";
    if (!c || !isDir || ms <= 0 || ms > 10000) {
        out.println("用法: jog <up|down|left|right> <1~10000 毫秒>，例: jog left 800");
        return;
    }
    sendPtzNow(c->b2, c->b3);
    delay((uint32_t)ms);
    ptzStopAll();
}

// ---------------------------------------------------------------------------
// 飞特 SCS 协议收发
// ---------------------------------------------------------------------------

// 发一条指令，等应答。写指令没有应答（expectReply=false）。返回是否收到有效应答
static bool scsTransact(uint8_t id, uint8_t instr, const uint8_t *params, size_t n,
                        scs::Reply *reply, bool expectReply, bool verbose) {
    uint8_t tx[80];
    size_t txLen = scs::build(tx, id, instr, params, n);
    flushMotorRx();
    if (verbose) printFrame(out, "ESP->M", tx, txLen);
    MotorUart.write(tx, txLen);
    MotorUart.flush();
    if (!expectReply) return true;

    uint8_t rx[80];
    size_t rxLen = 0;
    uint32_t t0 = millis(), lastByte = millis();
    while (millis() - t0 < 60) {
        while (MotorUart.available() && rxLen < sizeof(rx)) {
            rx[rxLen++] = motorRead();
            lastByte = millis();
        }
        size_t want = scs::expectedLength(rx, rxLen);
        if (want && rxLen >= want) break;
        if (rxLen && millis() - lastByte > 5) break;
        delay(1);
    }
    if (verbose && rxLen) printFrame(out, "M->ESP", rx, rxLen);
    if (!rxLen) {
        if (verbose) out.println("（无应答）");
        return false;
    }
    scs::Reply local;
    scs::Reply &r = reply ? *reply : local;
    if (!scs::parse(rx, rxLen, r)) {
        if (verbose) out.println("应答解析失败（长度或校验和不对）");
        return false;
    }
    if (r.err && verbose) out.printf("电机错误码: 0x%02X\n", r.err);
    return true;
}

static bool scsRead(uint8_t id, uint8_t addr, uint8_t n, scs::Reply &r, bool verbose) {
    uint8_t p[2] = {addr, n};
    return scsTransact(id, scs::INST_READ, p, 2, &r, true, verbose) && r.len == n;
}

// settle=true：写完等 20ms，保证紧接着的读回拿到新值。连续推目标位置时用 false
static bool scsWrite(uint8_t id, uint8_t addr, const uint8_t *data, size_t n, bool verbose, bool settle = true) {
    uint8_t p[64];
    if (n + 1 > sizeof(p)) return false;
    p[0] = addr;
    for (size_t i = 0; i < n; i++) p[1 + i] = data[i];
    bool ok = scsTransact(id, scs::INST_WRITE, p, n + 1, nullptr, false, verbose);
    if (settle) delay(20);  // 写指令无应答；电机处理需要几毫秒，紧接着读回会读到旧值
    return ok;
}

static bool scsWrite16(uint8_t id, uint8_t addr, uint16_t v, bool verbose, bool settle = true) {
    uint8_t d[2] = {(uint8_t)(v >> 8), (uint8_t)v};
    return scsWrite(id, addr, d, 2, verbose, settle);
}

// ---------------------------------------------------------------------------
// 连续转动（st run / st stop）：电机没有可用的速度寄存器，所以由 ESP32 每 20ms
// 把目标位置往前推一小步来实现匀速。需要客户端每 <700ms 重发一次 st run 当心跳，
// 断线或忘记 stop 会自动停；到 0°/360° 边界自动停。
// ---------------------------------------------------------------------------
static float    g_runDps = 0;        // 度/秒，0 = 不在转
static float    g_runGoal = 0;       // 当前推进中的目标（计数，浮点累加）
static uint8_t  g_runId = g_motorId;
static uint32_t g_runLastCmdMs = 0;  // 最近一次 st run 的时间（心跳）
static uint32_t g_runLastStepMs = 0;

// 头部巡航（连环左右旋转）：固件本地在两个角度间来回平滑推进，软件只发一次开始/停止，
// 不需要心跳、不用每帧发命令。停止用 st sweep off 或 st stop。
static bool     g_sweepOn = false;
static long     g_sweepLo = 0, g_sweepHi = 0;   // 原始计数，两端点
static float    g_sweepDps = 60;
static int      g_sweepDir = 1;
static uint32_t g_sweepLastMs = 0;

static void runStop(const char *why) {
    if (g_runDps == 0) return;
    g_runDps = 0;
    uint16_t p = 0;
    scsReadU16(g_runId, scs::REG_PRESENT_POSITION, p);
    g_target = p; g_targetSynced = true;   // 停下后把点动目标对齐到实际停的位置
    out.printf("STOP (%s) at %.1f° (%u)\n", why, rawToUserDeg(p), p);
}

static void runStart(uint8_t id, float dps) {
    uint32_t now = millis();
    if (g_runDps == 0 || id != g_runId) {
        uint16_t cur = 0;
        if (!scsReadU16(id, scs::REG_PRESENT_POSITION, cur)) { out.printf("ID %u 无应答\n", id); return; }
        uint8_t te = 0;
        if (scsReadU8(id, scs::REG_TORQUE_ENABLE, te) && !te) {
            uint8_t one = 1;
            scsWrite(id, scs::REG_TORQUE_ENABLE, &one, 1, false);
        }
        g_runGoal = cur;
        g_runId = id;
        g_runLastStepMs = now;
        out.printf("RUN %+.0f°/s from %.1f°\n", dps, scs::countsToDeg(cur));
    }
    g_runDps = dps;
    g_runLastCmdMs = now;
}

// ---------------------------------------------------------------------------
// 上电保持（st hold）：电机出厂上电扭矩关闭、软趴趴的。开启后 ESP32 开机就给电机
// 开扭矩并锁在当前位置；之后每 2s 检查一次，扭矩掉了（电机单独断电重启过）就自动
// 重新锁定。设置存在 NVS 里，ESP32 重启仍有效。st torque off 会临时松开，直到下一次
// 运动命令或 st torque on。
// ---------------------------------------------------------------------------
static Preferences g_prefs;
static bool     g_holdMode = true;        // NVS "hold"，默认开
static bool     g_holdSuspended = false;  // st torque off 之后为 true
static uint32_t g_holdLastCheckMs = 0;

static bool holdArm(uint8_t id, bool verbose) {
    uint16_t cur = 0;
    if (!scsReadU16(id, scs::REG_PRESENT_POSITION, cur)) return false;
    scsWrite16(id, scs::REG_GOAL_POSITION, cur, false);   // 先把目标对齐当前位置，开扭矩时不会跳
    g_target = cur; g_targetSynced = true;
    uint8_t one = 1;
    scsWrite(id, scs::REG_TORQUE_ENABLE, &one, 1, false);
    uint8_t te = 0;
    bool ok = scsReadU8(id, scs::REG_TORQUE_ENABLE, te) && te == 1;
    if (verbose) out.printf("保持: ID %u 锁定在 %.1f° (%u) %s\n", id, rawToUserDeg(cur), cur, ok ? "OK" : "失败");
    return ok;
}

static void holdService() {
    if (!g_holdMode || g_holdSuspended || g_runDps != 0 || g_sweepOn) return;
    uint32_t now = millis();
    if (now - g_holdLastCheckMs < 2000) return;
    g_holdLastCheckMs = now;
    uint8_t te = 0;
    if (!scsReadU8(g_motorId, scs::REG_TORQUE_ENABLE, te)) return;  // 电机不在线，下次再试
    if (!te) holdArm(g_motorId, true);
}

// ---------------------------------------------------------------------------
// 舵机与灯：ESP32 的 LEDC 硬件 PWM。
// Arduino-ESP32 3.x 用 ledcAttach(pin,...)，2.x（PlatformIO 官方平台）用通道号 API，这里两边都兼容
// ---------------------------------------------------------------------------
static const int SERVO_FREQ = 50, SERVO_BITS = 14;     // 20ms 周期分 16384 步，约 1.2µs/步
static const int SERVO_CH = 0, LED_CH = 2;             // 2.x 需要通道号；两路用不同定时器组

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwmAttach(int pin, int ch, int freq, int bits) { ledcAttach(pin, freq, bits); }
static void pwmWrite(int pin, int ch, uint32_t duty) { ledcWrite(pin, duty); }
static void pwmDetach(int pin, int ch) { ledcDetach(pin); pinMode(pin, INPUT); }
#else
static void pwmAttach(int pin, int ch, int freq, int bits) { ledcSetup(ch, freq, bits); ledcAttachPin(pin, ch); }
static void pwmWrite(int pin, int ch, uint32_t duty) { ledcWrite(ch, duty); }
static void pwmDetach(int pin, int ch) { ledcDetachPin(pin); pinMode(pin, INPUT); }
#endif

static bool     g_servoAttached = false;
static int      g_servoUs = 0;          // 当前实际输出的脉宽
static float    g_servoCurUs = 0;       // 限速推进用的浮点当前值
static int      g_servoTargetUs = 0;
static float    g_servoSpeedDps = 300;  // 默认限速 300°/s（180° 用 0.6s）
static float    g_servoMoveUsPerMs = 0; // 本次移动的速度
static uint32_t g_servoLastMs = 0;

// 舵机巡航（连环来回）：固件本地在两端点间自主往返，软件只发一次开始/停止
static bool g_servoSweepOn = false;
static int  g_servoSweepAUs = 0, g_servoSweepBUs = 0;

// 舵机开合标定：0°(全开)/180°(全关) 对应的实际脉宽，存 NVS。角度按这两点线性映射，
// 于是 0°=全开、180°=全关、90°=正好半开。默认用 board_config 的 500/2500。
static int  g_servoOpenUs = SERVO_MIN_US;
static int  g_servoClosedUs = SERVO_MAX_US;

// 角度 <-> 脉宽：按标定的全开(0°)/全关(180°)端点线性映射
static float servoUsToDeg(int us) {
    if (g_servoClosedUs == g_servoOpenUs) return 0;
    return (us - g_servoOpenUs) * (float)SERVO_MAX_DEG / (g_servoClosedUs - g_servoOpenUs);
}
static int servoDegToUs(float deg) {
    deg = constrain(deg, 0.0f, (float)SERVO_MAX_DEG);   // MG90S 只有 0~180°
    return g_servoOpenUs + (int)roundf(deg * (g_servoClosedUs - g_servoOpenUs) / SERVO_MAX_DEG);
}

// 直接输出脉宽（不限速）
static void servoOutputUs(int us) {
    us = constrain(us, SERVO_EXPLORE_MIN_US, SERVO_EXPLORE_MAX_US);
    if (!g_servoAttached) { pwmAttach(SERVO_PIN, SERVO_CH, SERVO_FREQ, SERVO_BITS); g_servoAttached = true; }
    uint32_t duty = (uint32_t)((uint64_t)us * ((1UL << SERVO_BITS) - 1) / 20000UL);
    pwmWrite(SERVO_PIN, SERVO_CH, duty);
    g_servoUs = us;
    g_servoCurUs = us;
}

// 限速移动：一步跳到位时舵机堵转电流最大（接在板子 5V 脚上会把 USB 拉掉），
// 改成每 20ms 推进一点。durationMs=0 时按 g_servoSpeedDps 算时间
static void servoMoveUs(int targetUs, uint32_t durationMs) {
    targetUs = constrain(targetUs, SERVO_EXPLORE_MIN_US, SERVO_EXPLORE_MAX_US);
    if (!g_servoAttached) {            // 刚上电不知道舵机在哪，第一次直接输出
        servoOutputUs(targetUs);
        g_servoTargetUs = targetUs;
        g_servoMoveUsPerMs = 0;
        return;
    }
    float distDeg = fabsf(servoUsToDeg(targetUs) - servoUsToDeg(g_servoUs));
    if (durationMs == 0) durationMs = (uint32_t)(distDeg / g_servoSpeedDps * 1000.0f);
    g_servoTargetUs = targetUs;
    if (durationMs < 20 || distDeg < 0.5f) { servoOutputUs(targetUs); g_servoMoveUsPerMs = 0; return; }
    g_servoMoveUsPerMs = fabsf((float)targetUs - g_servoCurUs) / durationMs;
    g_servoLastMs = millis();
}

static void servoService() {
    // 巡航：到端点(不再移动)后自动翻转到另一端
    if (g_servoSweepOn && g_servoAttached && g_servoMoveUsPerMs == 0) {
        servoMoveUs(g_servoTargetUs == g_servoSweepAUs ? g_servoSweepBUs : g_servoSweepAUs, 0);
    }
    if (g_servoMoveUsPerMs == 0 || !g_servoAttached) return;
    uint32_t now = millis();
    if (now - g_servoLastMs < 20) return;
    float step = g_servoMoveUsPerMs * (now - g_servoLastMs);
    g_servoLastMs = now;
    float next = g_servoCurUs < g_servoTargetUs ? min(g_servoCurUs + step, (float)g_servoTargetUs)
                                                : max(g_servoCurUs - step, (float)g_servoTargetUs);
    servoOutputUs((int)(next + 0.5f));
    g_servoCurUs = next;
    if ((int)(next + 0.5f) == g_servoTargetUs) g_servoMoveUsPerMs = 0;
}

// 舵机停止输出脉冲：普通舵机会松开（能用手拧）；数字舵机可能保持位置
static void servoRelease() {
    g_servoSweepOn = false;
    g_servoMoveUsPerMs = 0;
    if (!g_servoAttached) return;
    pwmDetach(SERVO_PIN, SERVO_CH);
    g_servoAttached = false;
    g_servoUs = 0;
}

static bool     g_ledAttached = false;
static float    g_ledPct = 0;        // 当前亮度 0~100
static float    g_ledTarget = 0;     // 渐变目标
static float    g_ledStepPerMs = 0;  // 渐变速度
static uint32_t g_ledLastMs = 0;

// 呼吸特效：固件本地按帧生成正弦亮度，软件只发一次开始/停止，不用每帧发命令
static bool     g_ledBreathe = false;
static uint32_t g_ledBrPeriod = 3000;   // 一个完整呼吸周期(ms)
static float    g_ledBrMin = 0, g_ledBrMax = 100;
static uint32_t g_ledBrT0 = 0, g_ledBrLast = 0;

// 人眼亮度感知是对数的，做一个 gamma 2.2，这样 50% 看起来才是一半亮
static void ledApply(float pct) {
    if (!g_ledAttached) { pwmAttach(LED_PWM_PIN, LED_CH, LED_PWM_FREQ, LED_PWM_BITS); g_ledAttached = true; }
    float norm = constrain(pct, 0.0f, 100.0f) / 100.0f;
    uint32_t maxDuty = (1UL << LED_PWM_BITS) - 1;
    uint32_t duty = (uint32_t)(powf(norm, 2.2f) * maxDuty + 0.5f);
    if (norm >= 1.0f) duty = maxDuty;
    pwmWrite(LED_PWM_PIN, LED_CH, duty);
    g_ledPct = pct;
}

static void ledSet(float pct, uint32_t fadeMs) {
    pct = constrain(pct, 0.0f, 100.0f);
    if (fadeMs == 0) { g_ledTarget = pct; g_ledStepPerMs = 0; ledApply(pct); return; }
    g_ledTarget = pct;
    g_ledStepPerMs = fabsf(pct - g_ledPct) / fadeMs;
    g_ledLastMs = millis();
}

static void ledService() {
    if (g_ledBreathe) {
        uint32_t now = millis();
        if (now - g_ledBrLast < 16) return;   // ~60fps 足够顺
        g_ledBrLast = now;
        float ph = ((now - g_ledBrT0) % g_ledBrPeriod) / (float)g_ledBrPeriod;  // 0~1
        float k = 0.5f - 0.5f * cosf(6.2831853f * ph);                          // 平滑 0~1
        ledApply(g_ledBrMin + (g_ledBrMax - g_ledBrMin) * k);
        return;
    }
    if (g_ledStepPerMs == 0 || g_ledPct == g_ledTarget) return;
    uint32_t now = millis();
    float step = g_ledStepPerMs * (now - g_ledLastMs);
    if (step < 0.05f) return;
    g_ledLastMs = now;
    float next = g_ledPct < g_ledTarget ? min(g_ledPct + step, g_ledTarget) : max(g_ledPct - step, g_ledTarget);
    ledApply(next);
    if (next == g_ledTarget) g_ledStepPerMs = 0;
}

static void cmdServo(const String &args) {
    String tok[4];
    int n = splitArgs(args, tok, 4);
    if (!n) {
        if (g_servoAttached) out.printf("舵机: %d µs = %.1f°（GPIO%d）\n", g_servoUs, servoUsToDeg(g_servoUs), SERVO_PIN);
        else out.printf("舵机: 未输出（GPIO%d）。用法: servo <0~%d> | servo us <%d~%d> | servo off\n", SERVO_PIN, SERVO_MAX_DEG, SERVO_MIN_US, SERVO_MAX_US);
        return;
    }
    String sub = tok[0]; sub.toLowerCase();
    if (sub == "off" || sub == "release") { servoRelease(); out.println("舵机: 已停止输出（松开）"); return; }
    if (sub == "speed") {
        if (n >= 2) g_servoSpeedDps = constrain(tok[1].toFloat(), 10.0f, 3000.0f);
        out.printf("舵机默认限速 = %.0f°/s\n", g_servoSpeedDps);
        return;
    }
    // 开合端点标定：转到物理全开位置后 servo setopen，转到全关后 servo setclosed
    if (sub == "setopen") {
        if (!g_servoAttached) { out.println("先用 servo us <脉宽> 或 servo <角度> 把舵机转到全开位置，再 setopen"); return; }
        g_servoOpenUs = g_servoTargetUs; g_prefs.putInt("svo", g_servoOpenUs);   // 用目标脉宽,避免限速途中没到位
        out.printf("OK 已标定 全开(0°) = %d µs\n", g_servoOpenUs);
        return;
    }
    if (sub == "setclosed") {
        if (!g_servoAttached) { out.println("先把舵机转到全关位置，再 setclosed"); return; }
        g_servoClosedUs = g_servoTargetUs; g_prefs.putInt("svc", g_servoClosedUs);
        out.printf("OK 已标定 全关(180°) = %d µs\n", g_servoClosedUs);
        return;
    }
    if (sub == "cal") {
        out.printf("舵机标定: 全开0°=%dµs 全关180°=%dµs 当前=%dµs(%.0f°)\n",
                   g_servoOpenUs, g_servoClosedUs, g_servoUs, servoUsToDeg(g_servoUs));
        return;
    }
    if (sub == "sweep") {
        if (n >= 2 && (tok[1] == "off" || tok[1] == "0")) { g_servoSweepOn = false; out.println("舵机巡航停止"); return; }
        if (n < 3) { out.println("用法: servo sweep <角A> <角B> [°/s]（连环来回，servo sweep off 停）"); return; }
        g_servoSweepAUs = servoDegToUs(tok[1].toFloat());
        g_servoSweepBUs = servoDegToUs(tok[2].toFloat());
        if (n > 3) g_servoSpeedDps = constrain(tok[3].toFloat(), 10.0f, 3000.0f);
        g_servoSweepOn = true;
        servoMoveUs(g_servoSweepAUs, 0);
        out.printf("舵机巡航: %.0f° <-> %.0f° @ %.0f°/s（servo sweep off 停）\n", tok[1].toFloat(), tok[2].toFloat(), g_servoSpeedDps);
        return;
    }
    g_servoSweepOn = false;   // 任何手动舵机移动都先停巡航
    int targetUs;
    uint32_t ms;
    if (sub == "us") {
        if (n < 2) { out.println("用法: servo us <µs> [ms]"); return; }
        targetUs = tok[1].toInt();
        ms = n > 2 ? (uint32_t)max(0L, tok[2].toInt()) : 0;
    } else {
        targetUs = servoDegToUs(tok[0].toFloat());
        ms = n > 1 ? (uint32_t)max(0L, tok[1].toInt()) : 0;
    }
    servoMoveUs(targetUs, ms);
    out.printf("舵机 -> %.1f° (%d µs)%s\n", servoUsToDeg(g_servoTargetUs), g_servoTargetUs,
               g_servoMoveUsPerMs ? "，限速移动中" : "");
}

static void cmdLed(const String &args) {
    String tok[4];
    int n = splitArgs(args, tok, 4);
    if (!n) { out.printf("灯: %.0f%%（GPIO%d）。用法: led <0~100> [渐变ms] | led on | led off | led breathe [周期ms] [最小%%] [最大%%]\n", g_ledPct, LED_PWM_PIN); return; }
    String sub = tok[0]; sub.toLowerCase();
    if (sub == "breathe") {
        if (n >= 2 && (tok[1] == "off" || tok[1] == "0")) { g_ledBreathe = false; out.println("灯: 呼吸关"); return; }
        g_ledBrPeriod = n >= 2 ? (uint32_t)max(200L, tok[1].toInt()) : 3000;
        g_ledBrMin = n >= 3 ? constrain(tok[2].toFloat(), 0.0f, 100.0f) : 0;
        g_ledBrMax = n >= 4 ? constrain(tok[3].toFloat(), 0.0f, 100.0f) : 100;
        g_ledBreathe = true; g_ledBrT0 = millis(); g_ledBrLast = 0;
        out.printf("灯: 呼吸中 周期%lums %.0f%%~%.0f%%（led off 停）\n", (unsigned long)g_ledBrPeriod, g_ledBrMin, g_ledBrMax);
        return;
    }
    g_ledBreathe = false;   // 任何手动灯命令都先关呼吸
    float pct;
    if (sub == "on") pct = 100;
    else if (sub == "off") pct = 0;
    else pct = tok[0].toFloat();
    uint32_t fade = n > 1 ? (uint32_t)max(0L, tok[1].toInt()) : 0;
    ledSet(pct, fade);
    if (fade) out.printf("灯 -> %.0f%%，%lu ms 渐变\n", pct, (unsigned long)fade);
    else out.printf("灯 -> %.0f%%\n", pct);
}

// 每次 loop 调用；到时间就推进一步
static void runService() {
    if (g_runDps == 0) return;
    uint32_t now = millis();
    if (now - g_runLastCmdMs > 700) { runStop("心跳超时"); return; }
    if (now - g_runLastStepMs < 20) return;
    float dt = (now - g_runLastStepMs) / 1000.0f;
    g_runLastStepMs = now;
    g_runGoal += g_runDps * dt * scs::COUNTS_PER_REV / 360.0f;
    bool hitEdge = false;
    if (g_runGoal <= 0) { g_runGoal = 0; hitEdge = true; }
    if (g_runGoal >= scs::COUNTS_PER_REV - 1) { g_runGoal = scs::COUNTS_PER_REV - 1; hitEdge = true; }
    scsWrite16(g_runId, scs::REG_GOAL_POSITION, (uint16_t)(g_runGoal + 0.5f), false, false);
    if (hitEdge) runStop("到达边界");
}

// 头部巡航：在 g_sweepLo~g_sweepHi 之间来回平滑推进目标，到端点自动反向。自主运行，无需心跳。
static void sweepService() {
    if (!g_sweepOn) return;
    uint32_t now = millis();
    if (now - g_sweepLastMs < 20) return;
    float dt = (now - g_sweepLastMs) / 1000.0f;
    g_sweepLastMs = now;
    g_target += lroundf(g_sweepDir * g_sweepDps * dt * scs::COUNTS_PER_REV / 360.0f);
    if (g_target >= g_sweepHi) { g_target = g_sweepHi; g_sweepDir = -1; }
    if (g_target <= g_sweepLo) { g_target = g_sweepLo; g_sweepDir = 1; }
    g_targetSynced = true;
    scsWrite16(g_motorId, scs::REG_GOAL_POSITION, (uint16_t)clampCounts(g_target), false, false);
}

static bool scsReadU8(uint8_t id, uint8_t addr, uint8_t &v) {
    scs::Reply r;
    if (!scsRead(id, addr, 1, r, false)) return false;
    v = r.data[0];
    return true;
}

static bool scsReadU16(uint8_t id, uint8_t addr, uint16_t &v) {
    scs::Reply r;
    if (!scsRead(id, addr, 2, r, false)) return false;
    v = scs::be16(r.data);
    return true;
}

// 开机自动探测在线电机的 ID，写入 g_motorId。
// 顺序：先 ping 出厂值 2、再 ping 1（最常见的两种出厂 ID），都没有再扫 0~253。
// 探测不到就保留默认值，交给用户手动 st ping / st scan（可能是电机比 ESP32 晚上电）。
static void detectMotorId() {
    scs::Reply r;
    const uint8_t prefer[] = { scs::DEFAULT_ID, 1 };
    for (uint8_t id : prefer) {
        if (scsTransact(id, scs::INST_PING, nullptr, 0, &r, true, false)) {
            g_motorId = id;
            out.printf("电机自动识别: ID %u\n", g_motorId);
            return;
        }
    }
    for (int id = 0; id <= 253; id++) {
        if (scsTransact((uint8_t)id, scs::INST_PING, nullptr, 0, &r, true, false)) {
            g_motorId = (uint8_t)id;
            out.printf("电机自动识别: ID %u（扫描找到）\n", g_motorId);
            return;
        }
    }
    out.printf("未探测到电机，暂用默认 ID %u（可能电机还没上电，稍后可手动 st ping / st scan）\n", g_motorId);
}

// 确保速度环 D 已整定：出厂 0x24=4 阻尼过低，带配重会 ~4Hz 自激振荡（"晃头"）。
// 厂家建议调 D，实测加到 24 消除振荡、到位准、保持稳。只在值不对时才写，避免每次开机写 EEPROM。
// 换同款新电机（出厂同样 D=4）时开机会自动补上。
static void ensureMotorTuning() {
    uint8_t d = 0;
    if (!scsReadU8(g_motorId, scs::REG_SPD_D, d)) {
        out.println("整定: 电机无应答，跳过速度环 D 检查");
        return;
    }
    if (d == scs::SPD_D_TUNED) {
        out.printf("整定: 速度环 D(0x24) 已是 %u，无需修改\n", d);
        return;
    }
    uint8_t v = scs::SPD_D_TUNED;
    scsWrite(g_motorId, scs::REG_SPD_D, &v, 1, false);
    uint8_t rb = 0;
    bool ok = scsReadU8(g_motorId, scs::REG_SPD_D, rb) && rb == scs::SPD_D_TUNED;
    out.printf("整定: 速度环 D(0x24) %u -> %u（消除自激振荡）%s\n", d, scs::SPD_D_TUNED, ok ? "OK" : "失败");
}

// 解析 "0x38" / "56" 这类数字
static long parseNum(const String &s, long dflt) {
    String t = s;
    t.trim();
    if (!t.length()) return dflt;
    return (t.startsWith("0x") || t.startsWith("0X")) ? strtol(t.c_str() + 2, nullptr, 16) : t.toInt();
}

// 把 args 按空格拆开，最多 maxTok 个
static int splitArgs(const String &args, String *tok, int maxTok) {
    int n = 0, start = 0;
    while (n < maxTok) {
        int sp = args.indexOf(' ', start);
        String t = sp < 0 ? args.substring(start) : args.substring(start, sp);
        t.trim();
        if (t.length()) tok[n++] = t;
        if (sp < 0) break;
        start = sp + 1;
    }
    return n;
}

// 软件零点换算：原始计数 <-> 相对 home 的角度（带符号，左负右正）
static float rawToUserDeg(long raw) { return (raw - g_zeroOffset) * 360.0f / scs::COUNTS_PER_REV; }
static long  userDegToRaw(float deg) { return g_zeroOffset + lroundf(deg * scs::COUNTS_PER_REV / 360.0f); }
// 只夹到原始编码器范围 0~4095（= 物理一圈），不是人为数字限位，只为避免越界取模造成反向乱转
static long  clampCounts(long c) { if (c < 0) c = 0; if (c > scs::COUNTS_PER_REV - 1) c = scs::COUNTS_PER_REV - 1; return c; }
// 把命令目标对齐到电机当前实际位置（开扭矩 / 保持 / 手动搬动后调用，避免下一次点动跳变）
static void syncTarget(uint8_t id) {
    uint16_t cur = 0;
    if (scsReadU16(id, scs::REG_PRESENT_POSITION, cur)) { g_target = cur; g_targetSynced = true; }
}

static void cmdSt(const String &args) {
    String tok[12];
    int n = splitArgs(args, tok, 12);
    if (!n) {
        out.println("用法见 help 的 st 部分");
        return;
    }
    String sub = tok[0];
    sub.toLowerCase();

    if (sub == "ping") {
        uint8_t id = (uint8_t)parseNum(n > 1 ? tok[1] : "", g_motorId);
        scs::Reply r;
        if (scsTransact(id, scs::INST_PING, nullptr, 0, &r, true, true))
            out.printf("ID %u 在线，错误码 0x%02X\n", r.id, r.err);
        else
            out.printf("ID %u 无应答（检查波特率是否 1000000、电机供电、TX/RX 是否交叉）\n", id);
        return;
    }

    if (sub == "scan") {
        out.printf("在 %lu 波特率下扫描 ID 0~253...\n", (unsigned long)g_baud);
        int found = 0;
        for (int id = 0; id <= 253; id++) {
            scs::Reply r;
            if (scsTransact((uint8_t)id, scs::INST_PING, nullptr, 0, &r, true, false)) {
                out.printf("  ID %d 在线\n", id);
                found++;
            }
        }
        out.printf("扫描完成，%d 个电机在线\n", found);
        return;
    }

    if (sub == "pos") {
        uint8_t id = (uint8_t)parseNum(n > 1 ? tok[1] : "", g_motorId);
        uint16_t pos;
        if (scsReadU16(id, scs::REG_PRESENT_POSITION, pos))
            out.printf("ID %u 当前位置 = %u 计数 = %.1f°\n", id, pos, rawToUserDeg(pos));
        else
            out.printf("ID %u 无应答\n", id);
        return;
    }

    if (sub == "torque") {
        if (n < 2) { out.println("用法: st torque on|off [id]"); return; }
        bool on = tok[1] == "on" || tok[1] == "1";
        uint8_t id = (uint8_t)parseNum(n > 2 ? tok[2] : "", g_motorId);
        g_sweepOn = false;
        runStop("torque");
        g_holdSuspended = !on;   // 手动关扭矩后，自动保持不再抢着打开
        if (on) {
            uint16_t cur = 0;    // 开扭矩前把目标对齐当前位置，否则会跳回上次的目标
            if (scsReadU16(id, scs::REG_PRESENT_POSITION, cur)) { scsWrite16(id, scs::REG_GOAL_POSITION, cur, false); g_target = cur; g_targetSynced = true; }
        }
        uint8_t v = on ? 1 : 0;
        scsWrite(id, scs::REG_TORQUE_ENABLE, &v, 1, true);
        uint8_t rb;
        if (scsReadU8(id, scs::REG_TORQUE_ENABLE, rb))
            out.printf("ID %u 扭矩 = %s\n", id, rb ? "开" : "关");
        else
            out.printf("ID %u 无应答\n", id);
        return;
    }

    if (sub == "move") {
        if (n < 2) { out.println("用法: st move <角度> [id]"); return; }
        g_sweepOn = false;
        float deg = tok[1].toFloat();
        uint8_t id = (uint8_t)parseNum(n > 2 ? tok[2] : "", g_motorId);
        uint8_t te = 0;
        if (!scsReadU8(id, scs::REG_TORQUE_ENABLE, te)) {
            out.printf("ID %u 无应答\n", id);
            return;
        }
        if (!te) {
            uint8_t one = 1;
            scsWrite(id, scs::REG_TORQUE_ENABLE, &one, 1, false);
            out.println("扭矩已自动打开");
        }
        uint16_t target = (uint16_t)clampCounts(userDegToRaw(deg));
        uint16_t before = 0;
        scsReadU16(id, scs::REG_PRESENT_POSITION, before);
        scsWrite16(id, scs::REG_GOAL_POSITION, target, true);
        g_target = target; g_targetSynced = true;
        out.printf("目标 %.1f° (%u)，出发位置 %.1f° (%u)\n",
                   rawToUserDeg(target), target, rawToUserDeg(before), before);
        for (int i = 0; i < 6; i++) {
            delay(250);
            uint16_t pos;
            if (scsReadU16(id, scs::REG_PRESENT_POSITION, pos))
                out.printf("  t+%.2fs  %.1f° (%u)\n", (i + 1) * 0.25f, rawToUserDeg(pos), pos);
        }
        return;
    }

    if (sub == "zero") {
        // 实测：往 0x16 写任意值 → 电机把当前位置记入 0x16-0x17（2 字节）。
        // 是否影响 0x38 的读数需重启后验证，见 docs/02 第 4 节
        if (n < 2 || tok[1] != "confirm") {
            out.println("st zero 会把电机当前位置记为零位（写 EEPROM，改动出厂值）。确认请输入: st zero confirm [id]");
            return;
        }
        uint8_t id = (uint8_t)parseNum(n > 2 ? tok[2] : "", g_motorId);
        uint8_t z = 0;
        scsWrite(id, scs::REG_ZERO_SET, &z, 1, true);
        uint16_t rec = 0;
        if (scsReadU16(id, scs::REG_ZERO_SET, rec)) out.printf("零位寄存器 0x16 现在 = %u\n", rec);
        return;
    }

    // 软件归零：把当前位置记为 home（存 ESP32 NVS，不写电机 EEPROM，蓝牙即可用，掉电不丢）
    if (sub == "home") {
        g_sweepOn = false;
        uint8_t id = (uint8_t)parseNum(n > 1 ? tok[1] : "", g_motorId);
        uint16_t cur = 0;
        if (!scsReadU16(id, scs::REG_PRESENT_POSITION, cur)) { out.printf("ID %u 无应答\n", id); return; }
        g_zeroOffset = cur;
        g_prefs.putInt("zero", g_zeroOffset);
        g_target = cur; g_targetSynced = true;
        out.printf("OK home 已设为当前位置（原始 %u 计数），现在读数 = %.1f°。掉电不丢。\n", cur, rawToUserDeg(cur));
        return;
    }

    // go / jog：给网页按钮用的精简版，只发目标、不打印轨迹，回一行结果
    if (sub == "hold") {
        if (n >= 2) {
            g_holdMode = (tok[1] == "on" || tok[1] == "1");
            g_prefs.putBool("hold", g_holdMode);
            g_holdSuspended = false;
            if (g_holdMode) holdArm(g_motorId, true);
        }
        out.printf("上电保持 = %s%s\n", g_holdMode ? "开" : "关", g_holdSuspended ? "（已被 st torque off 临时松开）" : "");
        return;
    }

    if (sub == "run") {
        g_holdSuspended = false;
        g_sweepOn = false;
        if (n < 2) { out.println("用法: st run <±度/秒> [id]（需每 <700ms 重发一次作为心跳）"); return; }
        float dps = tok[1].toFloat();
        uint8_t id = (uint8_t)parseNum(n > 2 ? tok[2] : "", g_motorId);
        if (dps == 0) { runStop("run 0"); return; }
        if (dps > 720) dps = 720;
        if (dps < -720) dps = -720;
        runStart(id, dps);
        return;
    }

    if (sub == "stop") {
        bool did = false;
        if (g_sweepOn) { g_sweepOn = false; out.println("巡航停止"); did = true; }
        if (g_runDps != 0) { runStop("stop"); did = true; }
        if (!did) out.println("STOP (未在转动)");
        return;
    }

    if (sub == "sweep") {
        if (n >= 2 && (tok[1] == "off" || tok[1] == "0")) { g_sweepOn = false; out.println("巡航停止"); return; }
        if (n < 3) { out.println("用法: st sweep <角A> <角B> [°/s]（在两角度间连环来回，st sweep off 停）"); return; }
        float a = tok[1].toFloat(), b = tok[2].toFloat();
        float dps = n > 3 ? tok[3].toFloat() : 60;
        if (dps < 1) dps = 1;  if (dps > 720) dps = 720;
        uint8_t id = g_motorId;
        uint8_t te = 0;
        if (!scsReadU8(id, scs::REG_TORQUE_ENABLE, te)) { out.printf("ID %u 无应答\n", id); return; }
        if (!te) { uint8_t one = 1; scsWrite(id, scs::REG_TORQUE_ENABLE, &one, 1, false); }
        long ca = clampCounts(userDegToRaw(a)), cb = clampCounts(userDegToRaw(b));
        g_sweepLo = min(ca, cb);  g_sweepHi = max(ca, cb);
        g_sweepDps = dps;
        runStop("sweep");                 // 停掉可能在跑的 run
        syncTarget(id);                   // 从当前实际位置开始
        g_sweepDir = (g_target < (g_sweepLo + g_sweepHi) / 2) ? 1 : -1;
        g_holdSuspended = true;           // 巡航期间不抢着保持
        g_sweepLastMs = millis();
        g_sweepOn = true;
        out.printf("巡航: %.1f° <-> %.1f° @ %.0f°/s（st sweep off 停）\n", a, b, dps);
        return;
    }

    // 以当前位置为中心 ±幅度 来回巡航。固件自己同步读当前位置，网页不必先读角度，避免算错中心。
    if (sub == "cruise") {
        if (n >= 2 && (tok[1] == "off" || tok[1] == "0")) { g_sweepOn = false; out.println("巡航停止"); return; }
        if (n < 2) { out.println("用法: st cruise <幅度°> [°/s]（以当前位置为中心±幅度来回，st cruise off 停）"); return; }
        float amp = fabsf(tok[1].toFloat());
        float dps = n > 2 ? tok[2].toFloat() : 60;
        if (dps < 1) dps = 1;  if (dps > 720) dps = 720;
        uint8_t id = g_motorId;
        uint8_t te = 0;
        if (!scsReadU8(id, scs::REG_TORQUE_ENABLE, te)) { out.printf("ID %u 无应答\n", id); return; }
        if (!te) { uint8_t one = 1; scsWrite(id, scs::REG_TORQUE_ENABLE, &one, 1, false); }
        uint16_t cur = 0;
        if (!scsReadU16(id, scs::REG_PRESENT_POSITION, cur)) { out.printf("ID %u 无应答\n", id); return; }
        long ampc = lroundf(amp * scs::COUNTS_PER_REV / 360.0f);
        g_sweepLo = clampCounts((long)cur - ampc);
        g_sweepHi = clampCounts((long)cur + ampc);
        g_sweepDps = dps;
        runStop("cruise");
        g_target = cur; g_targetSynced = true;
        g_sweepDir = 1;
        g_holdSuspended = true;
        g_sweepLastMs = millis();
        g_sweepOn = true;
        out.printf("巡航(当前±%.0f°): %.1f° <-> %.1f° @ %.0f°/s（st cruise off 停）\n",
                   amp, rawToUserDeg(g_sweepLo), rawToUserDeg(g_sweepHi), dps);
        return;
    }

    if (sub == "go" || sub == "jog") {
        runStop("新指令");
        g_sweepOn = false;
        g_holdSuspended = false;
        if (n < 2) { out.printf("用法: st %s <角度> [id]\n", sub.c_str()); return; }
        float deg = tok[1].toFloat();
        uint8_t id = (uint8_t)parseNum(n > 2 ? tok[2] : "", g_motorId);
        uint8_t te = 0;
        if (!scsReadU8(id, scs::REG_TORQUE_ENABLE, te)) { out.printf("ID %u 无应答\n", id); return; }
        if (!te) { uint8_t one = 1; scsWrite(id, scs::REG_TORQUE_ENABLE, &one, 1, false); syncTarget(id); }
        long target;
        if (sub == "jog") {
            // 相对点动：在"命令目标"上累加，不必每次以当前位置为基准——连点能连续平滑地跟。
            // 首次点动先把目标对齐当前实际位置，避免跳变。
            if (!g_targetSynced) syncTarget(id);
            target = clampCounts(g_target + lroundf(deg * scs::COUNTS_PER_REV / 360.0f));
        } else {
            // 绝对角度：相对 home，带符号（左负右正）。只夹到物理一圈 0~4095，不做人为限位。
            target = clampCounts(userDegToRaw(deg));
        }
        g_target = target; g_targetSynced = true;
        scsWrite16(id, scs::REG_GOAL_POSITION, (uint16_t)target, false);
        out.printf("OK %s -> %.1f° (%ld)\n", sub.c_str(), rawToUserDeg(target), target);
        return;
    }

    if (sub == "read") {
        if (n < 3) { out.println("用法: st read <addr> <n> [id]"); return; }
        uint8_t addr = (uint8_t)parseNum(tok[1], 0);
        uint8_t cnt = (uint8_t)parseNum(tok[2], 1);
        uint8_t id = (uint8_t)parseNum(n > 3 ? tok[3] : "", g_motorId);
        scs::Reply r;
        if (scsRead(id, addr, cnt, r, true)) {
            out.printf("寄存器 0x%02X:", addr);
            for (size_t i = 0; i < r.len; i++) out.printf(" %02X", r.data[i]);
            if (r.len == 2) out.printf("   (大端 = %u)", scs::be16(r.data));
            if (r.len == 1) out.printf("   (= %u)", r.data[0]);
            out.println();
        }
        return;
    }

    if (sub == "write") {
        if (n < 3) { out.println("用法: st write <addr> <hex...> [id]，最后一个参数若 <= 253 且前面已有数据则视为 id"); return; }
        uint8_t addr = (uint8_t)parseNum(tok[1], 0);
        // 数据 = tok[2..]，全部按十六进制；如果最后一个 token 以 "id=" 开头则是 ID，
        // 末尾带 "force" 才允许写 0x16/0x17
        uint8_t id = g_motorId;
        int last = n;
        bool force = false;
        if (tok[last - 1] == "force") { force = true; last--; }
        if (last > 2 && tok[last - 1].startsWith("id=")) { id = (uint8_t)parseNum(tok[last - 1].substring(3), g_motorId); last--; }
        if ((addr == scs::REG_ZERO_SET || addr == scs::REG_ZERO_SET + 1) && !force) {
            out.println("拒绝：0x16 不是飞特表里的 D 参数，实测写入任何值都会把当前位置记为零位（原出厂值 8）。");
            out.println("      要设零位请用 st zero；确实要直接写请在命令末尾加 force");
            return;
        }
        uint8_t data[32];
        size_t dn = 0;
        for (int i = 2; i < last && dn < sizeof(data); i++) data[dn++] = (uint8_t)strtoul(tok[i].c_str(), nullptr, 16);
        if (!dn) { out.println("没有数据"); return; }
        scsWrite(id, addr, data, dn, true);
        scs::Reply r;
        if (scsRead(id, addr, (uint8_t)dn, r, false)) {
            out.printf("读回 0x%02X:", addr);
            for (size_t i = 0; i < r.len; i++) out.printf(" %02X", r.data[i]);
            out.println();
        }
        return;
    }

    out.println("未知 st 子命令，输入 help 查看");
}

static void enterBridge() {
    out.println("进入透传：原生 USB 口 <-> 电机串口；本口（CH343）显示抓包。按 BOOT 键或输入 exit 退出。");
    Serial0.flush();
    // 丢掉命令行残留的 "\n"，避免被当成数据透传给电机
    delay(50);
    while (Serial.available()) Serial.read();
    g_bridge = true;
    ledMode();
}

static void exitBridge() {
    g_bridge = false;
    ledMode();
    out.println("已退出透传");
}

static void runCommand(String line) {
    line.trim();
    if (!line.length()) return;

    int sp = line.indexOf(' ');
    String cmd = sp < 0 ? line : line.substring(0, sp);
    String args = sp < 0 ? "" : line.substring(sp + 1);
    args.trim();
    cmd.toLowerCase();

    if (cmd == "help" || cmd == "?") cmdHelp();
    else if (cmd == "status") cmdStatus();
    else if (cmd == "baud") {
        long b = args.toInt();
        if (b < 1200 || b > 3000000) out.println("用法: baud 115200");
        else { setBaud((uint32_t)b); out.printf("波特率 = %ld\n", b); }
    }
    else if (cmd == "scan") cmdScan(args.length() ? (uint32_t)args.toInt() : 1500);
    else if (cmd == "listen") {
        uint32_t ms = args.length() ? (uint32_t)args.toInt() : 5000;
        out.printf("监听 %lu ms...\n", (unsigned long)ms);
        size_t frames = captureFor(ms, "M->ESP");
        out.printf("监听结束，共 %u 帧\n", (unsigned)frames);
    }
    else if (cmd == "mon") {
        g_monitor = (args == "on");
        out.printf("自动监听 %s\n", g_monitor ? "开" : "关");
    }
    else if (cmd == "tx") cmdTx(args, false);
    else if (cmd == "txsum") cmdTx(args, true);
    else if (cmd == "wait") {
        g_waitMs = (uint32_t)max(0L, args.toInt());
        out.printf("回应等待 = %lu ms\n", (unsigned long)g_waitMs);
    }
    else if (cmd == "dump") cmdDump(args == "clear");
    else if (cmd == "selftest") cmdSelftest();
    else if (cmd == "bridge") enterBridge();
    else if (cmd == "st") cmdSt(args);
    else if (cmd == "servo") cmdServo(args);
    else if (cmd == "led") cmdLed(args);
    else if (cmd == "ptz") cmdPtz(args);
    else if (cmd == "jog") cmdJog(args);
    else out.println("未知命令，输入 help 查看");
}

// ---------------------------------------------------------------------------
// 行输入（两个 COM 口各一份缓冲）
// ---------------------------------------------------------------------------
struct LineReader {
    String buf;
    bool poll(Stream &s, String &line) {
        while (s.available()) {
            char c = (char)s.read();
            if (c == '\r' || c == '\n') {
                if (buf.length()) {
                    line = buf;
                    buf = "";
                    return true;
                }
                continue;
            }
            if (c == 8 || c == 127) {
                if (buf.length()) buf.remove(buf.length() - 1);
                continue;
            }
            if (buf.length() < 512) buf += c;
        }
        return false;
    }
};
static LineReader g_usbLine, g_uartLine;

// ---------------------------------------------------------------------------
// 透传
// ---------------------------------------------------------------------------
static FrameSplitter g_pcToMotor, g_motorToPc;

static void bridgeLoop() {
    uint8_t buf[256];

    size_t n = Serial.available();
    if (n) {
        n = Serial.readBytes(buf, min(n, sizeof(buf)));
        MotorUart.write(buf, n);
        for (size_t i = 0; i < n; i++) {
            g_pcToMotor.push(buf[i]);
            if (g_pcToMotor.full()) { printFrame(Serial0, "PC->M", g_pcToMotor.buf, g_pcToMotor.len); g_pcToMotor.len = 0; }
        }
    }

    n = MotorUart.available();
    if (n) {
        n = min(n, sizeof(buf));
        for (size_t i = 0; i < n; i++) buf[i] = motorRead();
        Serial.write(buf, n);
        for (size_t i = 0; i < n; i++) {
            g_motorToPc.push(buf[i]);
            if (g_motorToPc.full()) { printFrame(Serial0, "M->PC", g_motorToPc.buf, g_motorToPc.len); g_motorToPc.len = 0; }
        }
    }

    if (g_pcToMotor.due()) { printFrame(Serial0, "PC->M", g_pcToMotor.buf, g_pcToMotor.len); g_pcToMotor.len = 0; }
    if (g_motorToPc.due()) { printFrame(Serial0, "M->PC", g_motorToPc.buf, g_motorToPc.len); g_motorToPc.len = 0; ledBlinkRx(); }

    String line;
    if (g_uartLine.poll(Serial0, line)) {
        line.trim();
        if (line.equalsIgnoreCase("exit")) exitBridge();
        else Serial0.println("透传中，只接受 exit");
    }
    if (digitalRead(BOOT_KEY_PIN) == LOW) {
        delay(30);
        if (digitalRead(BOOT_KEY_PIN) == LOW) {
            exitBridge();
            while (digitalRead(BOOT_KEY_PIN) == LOW) delay(10);
        }
    }
}

// ---------------------------------------------------------------------------
static FrameSplitter g_idle;

void setup() {
    Serial.setTxTimeoutMs(0);  // 电脑没打开原生 USB 口时不阻塞
    Serial.begin(115200);
    Serial0.begin(115200);
    pinMode(BOOT_KEY_PIN, INPUT_PULLUP);
    ledMode();

    motorBegin();
    bleBegin();
    delay(300);

    out.println();
    out.println("==== FZYT-28 云台控制固件 / Waveshare ESP32-S3-DEV-KIT-N8R8 ====");
    out.printf("BLE 广播名: %s（Nordic UART 服务，网页 webapp/index.html 可连接）\n", g_bleName.c_str());
    detectMotorId();
    ensureMotorTuning();
    cmdStatus();
#if PTZ_MASTER_ON_AT_BOOT
    out.println("发送 MasterOn（与原 RV1126B 驱动初始化行为一致）");
    sendPtzNow(0x11, 0x09);
#endif
    g_prefs.begin("fzyt", false);
    g_holdMode = g_prefs.getBool("hold", true);
    g_zeroOffset = g_prefs.getInt("zero", 0);   // 软件零点，掉电不丢
    g_servoOpenUs = g_prefs.getInt("svo", SERVO_MIN_US);      // 舵机开合标定
    g_servoClosedUs = g_prefs.getInt("svc", SERVO_MAX_US);
    out.printf("上电保持: %s（st hold on|off 修改）\n", g_holdMode ? "开" : "关");
    if (g_holdMode) {
        // 电机可能比 ESP32 晚上电，多试几次；仍不行就交给 holdService 每 2s 重试
        for (int i = 0; i < 3 && !holdArm(g_motorId, true); i++) delay(300);
    }
    out.println("输入 help 查看命令");
}

void loop() {
    if (g_ledRestoreAt && millis() > g_ledRestoreAt) {
        g_ledRestoreAt = 0;
        ledMode();
    }
    if (!g_bridge) { runService(); sweepService(); holdService(); }
    ledService();
    servoService();

    if (g_bridge) {
        bridgeLoop();
        return;
    }

    String line;
    if (g_usbLine.poll(Serial, line)) runCommand(line);
    if (!g_bridge && g_uartLine.poll(Serial0, line)) runCommand(line);
    if (!g_bridge && blePollLine(line)) {
        runCommand(line);
        bleFlushTx();
    }

    if (!g_bridge) {
        if (g_monitor) pumpMotor(g_idle, "M->ESP", &out);
        else flushMotorRx();
    }
    delay(1);
}
