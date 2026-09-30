#pragma once

// =====================================================================
// ECUE 双模式固件 —— 全局可调参数（单一来源）
//
// 口径依据（三方冻结，2026-09-30）：
//   - 采样窗口 30ms（与标定 CSV 同口径，阈值 150/600/1500 免重标）
//   - 消抖 40ms（沿用实测现码）
//   - 亮度帽 40%（30 灯 WS2812 全白峰值约 1.8A，USB 供电必压降）
// 修改本文件后需重新编译烧录（pio run）。
// =====================================================================

// ---- 灯条 ----
constexpr uint8_t LED_PIN = 7;
constexpr uint8_t LED_COUNT = 30;               // 30 颗口径，勿改（与 Esaber 31 颗结论无关）
// 亮度帽：255=全亮（约 1.8A，仅外接 5V>=2A 时可用）；100=约 40%（USB 直供安全值）。
constexpr uint8_t LED_BRIGHTNESS_MAX = 100;

// ---- 按钮 ----
// 接线口径：按钮一端 GPIO、另一端 GND，固件内部上拉，按下=低，无需外部电阻。
// 若实物另一端接 3.3V：把 BUTTON_ACTIVE_LOW 改为 false 即可（自动改用下拉），语义不变。
constexpr uint8_t BTN_EFFECT = 0;               // btn1：预设灯效模式内循环效果
constexpr uint8_t BTN_MODE = 10;                // btn2：传感器/预设互切；长按 2s 待机
constexpr bool BUTTON_ACTIVE_LOW = true;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
constexpr uint32_t BUTTON_LONGPRESS_MS = 2000;

// ---- 压电引擎（标定程序 piezo_test 与运行时固件共用口径）----
constexpr uint8_t PIEZO_ADC_PIN = 4;            // ADC1_CH4（勿把 btn1/IO0 加入 ADC 扫描）
constexpr int PIEZO_TRIGGER = 150;              // 触发阈值（码），约 92mV
constexpr int IMPACT_LEVEL1 = 600;              // swing >= 此值 -> 黄
constexpr int IMPACT_LEVEL2 = 1500;             // swing >= 此值 -> 红
constexpr uint32_t PIEZO_WINDOW_US = 30000;     // 触发后高速采样窗口 30ms
constexpr uint32_t PIEZO_REFRACT_MS = 200;      // 窗口结束后的静默期，防余振连发
constexpr int PIEZO_DEAD_ZONE = 40;             // 静噪死区（码）
constexpr uint32_t PIEZO_POLL_US = 500;         // 平时轮询间隔，约 2kHz
constexpr uint32_t PIEZO_REST_UPDATE_US = 2000; // 静息电平 EMA 更新节奏
constexpr int PIEZO_REST_SHIFT = 6;             // 静息 EMA 步长 1/64

// ---- 模式切换反馈（交互规范 V1 §2/§3）----
constexpr uint32_t FLASH_ON_MS = 60;            // 确认闪：亮 60ms
constexpr uint32_t FLASH_OFF_MS = 60;           // 确认闪：灭 60ms，2 下共 240ms
constexpr uint32_t STANDBY_FADE_MS = 300;       // 进入待机：300ms 线性渐灭
constexpr uint32_t HANDOVER_SILENCE_MS = 200;   // 进传感器模式交接静默（与确认闪并行）
