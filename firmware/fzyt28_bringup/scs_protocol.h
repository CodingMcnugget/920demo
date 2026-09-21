#pragma once
// ============================================================================
// FZYT-28 实际使用的串口协议：飞特 SCS 系列总线舵机格式（2026-09-18 真机实测）
//
//   串口：1,000,000 bps，8N1，独立 TX/RX 两根线
//   指令包：FF FF ID LEN INSTR PARAM... CHK      LEN = 参数个数 + 2
//   应答包：FF xx ID LEN ERR  DATA...  CHK      LEN = 数据个数 + 2
//   CHK = ~(ID + LEN + INSTR/ERR + 参数/数据之和) 的低 8 位
//   16 位寄存器为大端（高字节在前）。写指令没有应答，读指令有。
//   实测应答包的第二个头字节固定为 F5 而不是 FF，解析时不校验这一位。
//
// 寄存器地址来自厂家上位机 swj v4.03（云台执行器控制系统）逆向 + 真机验证：
//   0x05 ID              0x09 最小角限位(2B)  0x0B 最大角限位(2B) 出厂 0~4095
//   0x07 堵转电流阈值(2B) 0x0D 温度保护  0x0E 电压上限  0x0F 电压下限
//   0x10 最大扭矩(2B)     出厂 30000（不是 1000！上位机界面 0~1000 只是输入框范围）
//   0x14 中位调整(2B)     0x16 预设位/零位(2B) 写任意值 = 把当前位置记为零位（出厂 8）
//   0x1C 工作模式         0x1D 电机方向（0 = 正）
//   0x1F/0x20/0x21 位置环 P/I/D（s8，出厂 0/0/0）
//   0x22/0x23/0x24 速度环 P/I/D（s8，出厂 0/0/4）
//   0x28 扭矩使能（出厂 0，必须写 1 才会动） 0x2A 目标位置(2B) 0x38 当前位置(2B)
//   0x3A 当前速度
//   ⚠ 早期把 0x15 当 P、0x21 当模式都是误判：真正的 PID 在 0x1F/0x22。
//
// 【云台自激振荡的根因与解法（2026-09-21 真机确认）】
//   出厂速度环 D(0x24)=4 阻尼过低 → 带 66g 配重时位置环欠阻尼，移动/保持时约 4Hz 自激
//   振荡（"疯狂晃头"）。厂家建议"调 D"：把 0x24 加到 24，振荡消失，到位精准、保持峰峰
//   ≈0.1°。见固件 ensureMotorTuning()。0x10=30000 是出厂正常值，无需改。
//   0x06=恢复出厂指令（INST_RESET，会把 EEPROM 复位为出厂默认，ID/波特率保留）。
// ============================================================================

#include <Arduino.h>

namespace scs {

enum : uint8_t {
    INST_PING = 0x01, INST_READ = 0x02, INST_WRITE = 0x03,
    INST_REG_WRITE = 0x04, INST_ACTION = 0x05, INST_RESET = 0x06,
    INST_SYNC_WRITE = 0x83,
};

enum : uint8_t {
    REG_ID = 0x05, REG_MIN_ANGLE = 0x09, REG_MAX_ANGLE = 0x0B,
    REG_MAX_TORQUE = 0x10,  // 2 字节，出厂 30000（正常，别改）
    REG_ZERO_SET = 0x16,    // ⚠ 2 字节。写任意值 = 把当前位置记为零位（出厂 8）
    REG_POS_P = 0x1F, REG_POS_I = 0x20, REG_POS_D = 0x21,   // 位置环 PID（s8，出厂 0/0/0）
    REG_SPD_P = 0x22, REG_SPD_I = 0x23, REG_SPD_D = 0x24,   // 速度环 PID（s8，出厂 0/0/4）
    REG_TORQUE_ENABLE = 0x28, REG_GOAL_POSITION = 0x2A,
    REG_PRESENT_POSITION = 0x38, REG_PRESENT_SPEED = 0x3A,
};

// 速度环 D 的目标值：出厂 4 欠阻尼会自激振荡，加到这个值消除（见头部说明）
constexpr uint8_t SPD_D_TUNED = 24;

constexpr uint8_t  DEFAULT_ID = 2;
constexpr uint16_t COUNTS_PER_REV = 4096;

inline uint8_t checksum(const uint8_t *body, size_t n) {
    uint8_t s = 0;
    for (size_t i = 0; i < n; i++) s += body[i];
    return (uint8_t)~s;
}

// 组指令包，返回总长度。out 至少要有 params 长度 + 6 字节
inline size_t build(uint8_t *out, uint8_t id, uint8_t instr, const uint8_t *params, size_t n) {
    out[0] = 0xFF;
    out[1] = 0xFF;
    out[2] = id;
    out[3] = (uint8_t)(n + 2);
    out[4] = instr;
    for (size_t i = 0; i < n; i++) out[5 + i] = params[i];
    out[5 + n] = checksum(out + 2, n + 3);
    return n + 6;
}

struct Reply {
    uint8_t id = 0;
    uint8_t err = 0;
    uint8_t data[64];
    size_t  len = 0;
};

// 应答总长度 = LEN + 4；LEN 未知时返回 0
inline size_t expectedLength(const uint8_t *buf, size_t n) {
    return n >= 4 ? (size_t)buf[3] + 4 : 0;
}

// 解析应答包；校验和错返回 false
inline bool parse(const uint8_t *buf, size_t n, Reply &r) {
    if (n < 6 || buf[0] != 0xFF) return false;
    size_t total = expectedLength(buf, n);
    if (total < 6 || total > n || total - 6 > sizeof(r.data)) return false;
    if (checksum(buf + 2, total - 3) != buf[total - 1]) return false;
    r.id = buf[2];
    r.err = buf[4];
    r.len = total - 6;
    for (size_t i = 0; i < r.len; i++) r.data[i] = buf[5 + i];
    return true;
}

inline uint16_t be16(const uint8_t *d) { return (uint16_t)((d[0] << 8) | d[1]); }
inline float countsToDeg(uint16_t c) { return c * 360.0f / COUNTS_PER_REV; }
inline uint16_t degToCounts(float deg) {
    while (deg < 0) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return (uint16_t)(deg * COUNTS_PER_REV / 360.0f + 0.5f) % COUNTS_PER_REV;
}

}  // namespace scs
