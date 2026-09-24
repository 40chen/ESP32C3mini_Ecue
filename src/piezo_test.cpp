#include <Arduino.h>
#include "piezo_test.h"

// =====================================================================
// 压电撞击观察程序（改进版，直接替换原 piezo_test.cpp）
//
// 板子: ESP32-C3 mini
// 信号链: 压电传感器 -> AD8605 电荷放大器(R16=10M, C12=10p) -> GPIO4
//
// 针对的问题（见对话总结）:
//  1) AD8605 输出长期贴在 0V 或满量程轨上，数据大部分不是敲击产生的
//  2) 旧代码 20ms 采样(50Hz)抓不到 ms 级撞击脉冲
//  3) 基线被饱和值"带跑"，产生大量假峰值
//  4) 贴轨时的 ±31 码噪声(约±20mV)被当成有效信号
//  5) 碰传感器数据"减小"——有效信号是负向跳变（贴上轨时向下）
//
// 设计思路:
//  - 平时低速轮询(约2.5kHz)，维护"静息电平" rest（EMA，跟随任何贴轨状态）
//  - |raw - rest| 超过阈值 -> 进入 30ms 高速窗口，全速连续采样
//  - 窗口内同时记录最小/最大值：贴上轨时撞击向下、贴下轨时向上，双向都能抓
//  - 每次撞击输出一行 CSV：swing,min,max,rest,hi,lo,satH,satL
//  - 窗口结束后有 200ms 静默期，防止余振反复触发
//  - 长时间无事件且贴轨时，每 5s 打印一行 # 状态，方便确认电路仍在饱和
//
// 重要提示: ESP32-C3 在 ADC_11db 下满量程约 2.5V（不是 3.3V），1码 ≈ 0.61mV。
// 需要标定过的毫伏值时，把 readAdc() 换成 analogReadMilliVolts(PIEZO_PIN)。
// =====================================================================

namespace {

constexpr uint8_t PIEZO_PIN = 4;   // ADC1_CH4，与 WiFi 不冲突（ADC2 才会冲突）

// ---------------- 可调参数 ----------------
constexpr uint32_t IDLE_INTERVAL_US  = 400;    // 平时轮询间隔，约 2.5kHz
constexpr uint32_t REST_UPDATE_US    = 2000;   // 静息电平更新周期（EMA 时间常数约 128ms）
constexpr int      TRIGGER_THRESHOLD = 150;    // 触发阈值(码)。误触发多 -> 调大；轻敲不触发 -> 调小
constexpr int      DEAD_ZONE         = 40;     // 静噪死区(码)。滤掉 ±31 码的贴轨噪声，仍有零星行 -> 调到 50~60
constexpr uint32_t WINDOW_US         = 30000;  // 触发后高速采样窗口 30ms
constexpr uint32_t REFRACTORY_MS     = 200;    // 窗口结束后的静默期
constexpr uint32_t STUCK_STATUS_MS   = 5000;   // 无事件时贴轨状态行的打印周期
constexpr int      BASELINE_SHIFT    = 6;      // rest 的 EMA 步长 = 1/64

constexpr int ADC_MAX   = 4095;                // 12bit
constexpr int RAIL_HIGH = ADC_MAX - 10;        // >=4085 视为贴上轨（ADC 满量程，约 2.5V）
constexpr int RAIL_LOW  = 10;                  // <=10   视为贴下轨（约 0V，削底）

// ---------------- 内部状态 ----------------
int      rest           = 0;      // 静息电平（可以是 0V 附近的下轨，也可以是贴死的上轨）
uint32_t lastIdleUs     = 0;
uint32_t lastRestUs     = 0;
bool     inWindow       = false;
uint32_t windowStartUs  = 0;
int      wMin = 0, wMax = 0;
bool     wSatH = false, wSatL = false;
uint32_t refractUntilMs = 0;
uint32_t lastStuckMs    = 0;

inline int readAdc() {
  return analogRead(PIEZO_PIN);
}
}  // namespace

void piezoTestSetup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PIEZO_PIN, INPUT);
  analogSetPinAttenuation(PIEZO_PIN, ADC_11db);

  // 初始化静息电平：连续读 128 次（约 256ms）取平均
  long sum = 0;
  for (int i = 0; i < 128; ++i) {
    sum += readAdc();
    delay(2);
  }
  rest = static_cast<int>(sum / 128);

  Serial.println();
  Serial.println(F("[piezo] impact observer started | GPIO4, 115200"));
  Serial.println(F("[piezo] note: ESP32-C3 ADC_11db full-scale ~2.5V, 1 LSB ~ 0.61mV"));
  Serial.print(F("[piezo] rest level init = "));
  Serial.println(rest);
  Serial.println(F("[piezo] csv: swing,min,max,rest,hi,lo,satH,satL"));
  Serial.println(F("[piezo] # lines are status, not impacts"));
}

void piezoTestLoop() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // ---------- 窗口期：全速连续采样 ----------
  if (inWindow) {
    const int raw = readAdc();
    if (raw < wMin) wMin = raw;
    if (raw > wMax) wMax = raw;
    if (raw >= RAIL_HIGH) wSatH = true;
    if (raw <= RAIL_LOW)  wSatL = true;

    if (nowUs - windowStartUs >= WINDOW_US) {
      inWindow       = false;
      refractUntilMs = nowMs + REFRACTORY_MS;

      const int hi = (wMax - rest > 0) ? (wMax - rest) : 0;  // 相对静息的正向幅度
      const int lo = (rest - wMin > 0) ? (rest - wMin) : 0;  // 相对静息的负向幅度
      int swing = wMax - wMin;                               // 峰峰幅度（推荐作为力度相对指标）
      if (swing < DEAD_ZONE) swing = 0;

      // swing      本次撞击峰峰值(码)。0.61mV/码换算毫伏
      // hi/lo      相对静息电平的正/负向幅度，谁大说明当前贴哪条轨
      // satH/satL  1 表示窗口内顶到上/下轨，力度超出量程，该行不能用于力度对比
      Serial.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",
                    swing, wMin, wMax, rest, hi, lo,
                    wSatH ? 1 : 0, wSatL ? 1 : 0);
    }
    return;  // 窗口期内不做基线更新、不检测新触发
  }

  // ---------- 静默期：不更新 rest，不触发 ----------
  if (nowMs < refractUntilMs) {
    return;
  }

  // ---------- 平时：低速轮询 ----------
  if (nowUs - lastIdleUs < IDLE_INTERVAL_US) {
    return;
  }
  lastIdleUs = nowUs;

  const int raw = readAdc();

  // 周期性更新静息电平：即使输出贴死在某条轨上，rest 也会跟随过去，
  // 这样"从上轨往下掉"或"从下轨往上跳"都能触发
  if (nowUs - lastRestUs >= REST_UPDATE_US) {
    lastRestUs = nowUs;
    rest += (raw - rest) / (1 << BASELINE_SHIFT);
  }

  // 长时间无事件且贴轨：周期提示，避免"为什么没数据"的困惑
  if (nowMs - lastStuckMs >= STUCK_STATUS_MS &&
      (rest >= RAIL_HIGH || rest <= RAIL_LOW)) {
    lastStuckMs = nowMs;
    Serial.printf("#stuck at %s rail, rest=%d (circuit saturated, hardware fix needed)\n",
                  (rest >= RAIL_HIGH) ? "HIGH" : "LOW", rest);
  }

  // 双向触发：不再只等"数据变大"
  const int d = raw - rest;
  if (d > TRIGGER_THRESHOLD || d < -TRIGGER_THRESHOLD) {
    inWindow      = true;
    windowStartUs = nowUs;
    wMin  = raw;
    wMax  = raw;
    wSatH = (raw >= RAIL_HIGH);
    wSatL = (raw <= RAIL_LOW);
  }
}
