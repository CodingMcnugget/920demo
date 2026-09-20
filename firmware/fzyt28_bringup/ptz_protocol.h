#pragma once
// ============================================================================
// 云台串口协议：FC 80 <b2> <b3> 00 <sum8>，115200 8N1，只发不收
//
// 来源：RV1126B ODM 固件 librp_mw.so 反汇编（RpMwPtzControl* 系列函数）
//   帧表位于 .rodata 0xa570，每项 5 字节，第 6 字节为前 5 字节累加和。
//   原驱动初始化发 MasterOn，关闭时发 MasterOff；停止时只给正在转的轴发停止帧。
//
// b2 推测为电机 ID：0x01 = 俯仰电机，0x10 = 水平电机，0x11 = 全部（待真机验证）
// ============================================================================

#include <Arduino.h>

inline uint8_t sum8(const uint8_t *d, size_t n) {
    uint8_t s = 0;
    for (size_t i = 0; i < n; i++) s += d[i];
    return s;
}

struct PtzCmd {
    const char *name;
    uint8_t b2, b3;
};

static const PtzCmd PTZ_CMDS[] = {
    {"up", 0x01, 0x00},    {"down", 0x01, 0x01},   {"left", 0x10, 0x02},
    {"right", 0x10, 0x03}, {"stopv", 0x11, 0x07},  {"stoph", 0x11, 0x08},
    {"aion", 0x10, 0x04},  {"aioff", 0x11, 0x05},  {"reset", 0x11, 0x06},
    {"on", 0x11, 0x09},    {"off", 0x11, 0x0A},    {"privon", 0x11, 0x0B},
    {"privoff", 0x11, 0x0C},
};

inline const PtzCmd *findPtz(const String &name) {
    for (const auto &c : PTZ_CMDS)
        if (name == c.name) return &c;
    return nullptr;
}

inline void buildPtz(uint8_t b2, uint8_t b3, uint8_t f[6]) {
    f[0] = 0xFC;
    f[1] = 0x80;
    f[2] = b2;
    f[3] = b3;
    f[4] = 0x00;
    f[5] = sum8(f, 5);
}
