#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

#include "cue_controller.h"
#include "piezo_config.h"

// =====================================================================
// ECUE 运行时双模式固件（M2）
//
// 模式：传感器（压电引擎+撞击响应）/ 预设灯效（五效果循环）/ 待机（全灭）
// 按键：btn2(IO10) 短按互切模式、长按 2s 待机；
//       btn1(IO0)  传感器模式内循环撞击风格（M2）/ 预设模式内循环效果（M1）；
//                  长按语义已预留，现场重标定随 M3 落地。
// 待机中任意短按唤醒回传感器模式。
// 开机：恒进预设灯效模式（C 语义，V2 §5.6），NVS 恢复 effect/style 档位
//       索引（§5.5），开机无确认闪——灯效即反馈。
// 参数见 piezo_config.h；交互口径见《ECUE_交互规范_设计匠人_20260930》V2。
// =====================================================================

namespace {

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---- 运行时模式 ----
enum class RuntimeMode : uint8_t {
  Sensor,   // 传感器模式：压电引擎 + 撞击响应
  Preset,   // 预设灯效模式：五效果循环
  Standby   // 待机：全灭
};

RuntimeMode mode = RuntimeMode::Sensor;       // 真实开机模式由 setup 内 enterPresetMode
                                              // 设定（V2 §5.6，唯一非按键调用点）

constexpr bool DEBUG_IMPACT = true;           // 触发时打印 swing，标定完可改 false

uint8_t presetIndex = 0;                      // 预设域档位记忆（来回切模式不重置）
constexpr uint8_t PRESET_COUNT = 5;           // 五效果，无黑档；Off 语义移交 btn2 长按
uint32_t lastPresetUpdate = 0;                // 预设效果 25ms 渲染节流
uint8_t styleIndex = 0;                       // 传感器域撞击风格档位（M2 四态循环）

// ---- 压电引擎状态 ----
uint32_t piezoLastPollUs = 0;
uint32_t piezoLastRestUs = 0;
int      piezoRest = 0;
bool     piezoInWindow = false;
uint32_t piezoWindowStartUs = 0;
int      piezoMin = 0;
int      piezoMax = 0;
uint32_t piezoRefractUntil = 0;               // 复用为触发后静默 + 交接静默

// ---- 撞击响应（M2 多风格：渲染参数在触发时快照，渲染中切档不跳变）----
uint32_t effectStart = 0;
uint32_t effectUntil = 0;
uint32_t effectColor = 0;
uint8_t  effectSpan = 8;                      // S2 条长快照（离散档表值）
uint16_t effectSpeed = 60;                    // S3 波速快照（灯/s，离散档表值）
uint16_t effectSwing = 0;                     // 连续模式 swing 快照（§5.3 渲染侧现算）
uint8_t  effectStyle = 0;                     // 风格快照（0=M1 脉冲 → S1/S2/S3）

// ---- §5.4 风格切换提示（微缩演示版，非阻塞，S2/S3 复用撞击绘制函数）----
uint8_t  hintStyle = 0xFF;                    // 0xFF=无提示在播
uint32_t hintStart = 0;

// ---- §5.3 连续映射公式（STRIKE_CONTINUOUS=1 时参与编译）----
// N=8+round((swing-150)/1350×22) 钳位[8,30]；S3 波速 60→120 同式；
// 色相 青→黄→红 按 150/600/1500 三锚点分段线性；时长 220+round(t×210) ms。
#if STRIKE_CONTINUOUS
uint32_t impactColorContinuous(uint16_t sw) {
  if (sw <= IMPACT_LEVEL1) {                      // 青→黄（150→600）
    const float u = (float)(sw - PIEZO_TRIGGER) /
                    (float)(IMPACT_LEVEL1 - PIEZO_TRIGGER);
    return pixels.Color((uint8_t)lroundf(255 * u), 255,
                        (uint8_t)lroundf(255 * (1 - u)));
  }
  const float u = (float)(sw - IMPACT_LEVEL1) /   // 黄→红（600→1500）
                  (float)(IMPACT_LEVEL2 - IMPACT_LEVEL1);
  return pixels.Color(255, (uint8_t)lroundf(255 * (1 - u)), 0);
}

uint8_t spanFromSwing(uint16_t sw) {
  const float t = (float)(sw - PIEZO_TRIGGER) /
                  (float)(IMPACT_LEVEL2 - PIEZO_TRIGGER);
  return (uint8_t)constrain(8 + (int)lroundf(t * 22), 8, 30);
}

uint16_t speedFromSwing(uint16_t sw) {
  const float t = (float)(sw - PIEZO_TRIGGER) /
                  (float)(IMPACT_LEVEL2 - PIEZO_TRIGGER);
  return (uint16_t)(60 + lroundf(t * 60));
}

uint32_t durationFromSwing(uint16_t sw) {
  const float t = (float)(sw - PIEZO_TRIGGER) /
                  (float)(IMPACT_LEVEL2 - PIEZO_TRIGGER);
  return IMPACT_BASE_MS + (uint32_t)lroundf(t * 210);
}
#endif


// ---- 确认闪 / 提示闪 FSM（非阻塞；确认闪优先，撞击渲染延迟至闪毕——规范 §3 细节 A）----
bool     flashActive = false;
uint32_t flashStart = 0;
uint32_t flashColor = 0;
uint8_t  flashTimes = 2;                      // 闪几下（模式确认 2 下，风格提示 1 下）
constexpr uint32_t FLASH_PERIOD_MS = FLASH_ON_MS + FLASH_OFF_MS;

// ---- 待机渐灭 FSM（不清缓冲区，靠降全局亮度实现"从当前亮度线性降 0"）----
bool     fadeActive = false;
uint32_t fadeStart = 0;
uint8_t  fadeFrom = LED_BRIGHTNESS_MAX;

// ---- 按键驱动（消抖 40ms + 短按释放判定 + 2s 长按）----
enum class PressEvent : uint8_t { None, Short, Long };

struct Button {
  uint8_t pin;
  bool rawPressed;
  uint32_t lastEdgeMs;
  uint32_t pressStartMs;
  bool longFired;
  explicit Button(uint8_t p)
      : pin(p), rawPressed(false), lastEdgeMs(0), pressStartMs(0), longFired(false) {}
};

Button btnEffect{BTN_EFFECT};   // btn1
Button btnMode{BTN_MODE};       // btn2

// ---- 预设效果内部状态（切档/切模式时经 resetPresetState() 复位）----
uint8_t  rainbowOffset = 0;                 // 效果0 彩虹流
int8_t   scanPos = 0;                       // 效果3 追光扫描
int8_t   scanDir = 1;
uint8_t  scanHalf = 0;                      // 2 帧/灯步 => 60 帧/向（规范 §4）
bool     striking = false;                  // 效果4 击打闪光
uint32_t strikeStart = 0;
uint32_t strikeNextAt = 0;

uint32_t wheel(uint8_t position) {
  position = 255 - position;
  if (position < 85) {
    return pixels.Color(255 - position * 3, 0, position * 3);
  }
  if (position < 170) {
    position -= 85;
    return pixels.Color(0, position * 3, 255 - position * 3);
  }
  position -= 170;
  return pixels.Color(position * 3, 255 - position * 3, 0);
}

uint32_t dimColor(uint32_t color, uint8_t brightness) {
  uint8_t red = (color >> 16) & 0xFF;
  uint8_t green = (color >> 8) & 0xFF;
  uint8_t blue = color & 0xFF;
  return pixels.Color((red * brightness) / 255,
                      (green * brightness) / 255,
                      (blue * brightness) / 255);
}

void clearPixels() {
  pixels.clear();
  pixels.show();
}

// ---- 台球标准色板（交互规范 §4 效果1；1..8 号球本色）----
uint32_t ballColor(uint8_t ball) {
  switch (ball) {
    case 1: return pixels.Color(255, 210, 0);    // 黄
    case 2: return pixels.Color(0, 80, 255);     // 蓝
    case 3: return pixels.Color(255, 0, 0);      // 红
    case 4: return pixels.Color(140, 0, 255);    // 紫
    case 5: return pixels.Color(255, 100, 0);    // 橙
    case 6: return pixels.Color(0, 170, 60);     // 绿
    case 7: return pixels.Color(125, 50, 10);    // 栗
    default: return pixels.Color(40, 40, 40);    // 8 号=炭灰微光（规范指定）
  }
}

void resetPresetState() {
  rainbowOffset = 0;
  scanPos = 0;
  scanDir = 1;
  scanHalf = 0;
  striking = false;
  strikeStart = 0;
  strikeNextAt = 0;
}

// ---- NVS 断电记忆（V2 §5.5）----
// 单 key 三字段打包（effect/style/mode）。mode 为保留字段：写侧恒写 0、读侧
// 忽略（C 语义开机恒进预设，mode 不参与恢复；字段保留为未来语义回摆预埋，
// 回摆零迁移成本）。写点仅 handleButtons 内档位变更处（P1-2 修复后
// handleButtons 只在窗口外被调，满足"写点窗口门外"冻结纪律）。
struct NvState {
  uint8_t effect;   // presetIndex（预设域档位）
  uint8_t style;    // styleIndex（传感器域档位）
  uint8_t mode;     // 保留字段，恒写 0，读侧不使用
};

void saveImpactState() {
  if (!NVS_ENABLED) return;
  const NvState s{presetIndex, styleIndex, 0};   // mode 恒写 0（V2 §5.5）
  Preferences prefs;
  prefs.begin(NVS_NS, false);
  prefs.putBytes(NVS_KEY, &s, sizeof(s));
  prefs.end();
}

void loadImpactState() {
  presetIndex = 0;
  styleIndex = 0;
  if (!NVS_ENABLED) return;
  Preferences prefs;
  prefs.begin(NVS_NS, true);                     // 只读打开；NVS 不存在时读出空
  NvState s{0, 0, 0};
  if (prefs.getBytesLength(NVS_KEY) == sizeof(s)) {
    prefs.getBytes(NVS_KEY, &s, sizeof(s));
  }
  prefs.end();
  // 越界回退默认档（冻结纪律）
  presetIndex = (s.effect < PRESET_COUNT) ? s.effect : 0;
  styleIndex = (s.style < STYLE_COUNT) ? s.style : 0;
}

// ---- 确认闪 / 提示闪 ----
void startFlash(uint32_t color, uint8_t times) {
  flashColor = color;
  flashStart = millis();
  flashTimes = times;
  flashActive = true;
}

// 返回 true=仍在闪。调用方保证窗口期不进来（细节 B：闪遇窗口暂停刷帧）。
bool flashRenderFrame() {
  const uint32_t elapsed = millis() - flashStart;
  const uint32_t total = FLASH_PERIOD_MS * flashTimes;
  if (elapsed >= total) {
    flashActive = false;
    pixels.setBrightness(LED_BRIGHTNESS_MAX);   // 渐灭可能动过全局亮度，恢复
    return false;
  }
  if (elapsed % FLASH_PERIOD_MS < FLASH_ON_MS) {
    pixels.fill(flashColor);                    // 亮度受全局帽约束
  } else {
    pixels.clear();
  }
  pixels.show();
  return true;
}

// ---- 待机渐灭 ----
void startStandbyFade() {
  fadeStart = millis();
  fadeFrom = LED_BRIGHTNESS_MAX;
  fadeActive = true;
}

void fadeRenderFrame() {
  const uint32_t elapsed = millis() - fadeStart;
  if (elapsed >= STANDBY_FADE_MS) {
    fadeActive = false;
    pixels.setBrightness(0);
    pixels.show();
    return;
  }
  pixels.setBrightness(fadeFrom - (uint8_t)((fadeFrom * elapsed) / STANDBY_FADE_MS));
  pixels.show();                                // 缓冲区保留原画面，随全局亮度线性降 0
}

// ---- 模式交接（规范 §3：清灯 → 确认闪 ∥ 静默 200ms + 静息重播种）----
void reseedRest() {
  uint32_t sum = 0;
  for (int i = 0; i < 32; ++i) {
    sum += analogRead(PIEZO_ADC_PIN);
    delayMicroseconds(100);
  }
  piezoRest = sum / 32;
}

// 64 次静息平均（比 reseedRest 更抗噪）。V2 §5.6.4：M2 上电不再触达
// （开机恒进预设模式），代码路径保留供 M3 标定模式使用。
[[maybe_unused]] void seedRestFull() {
  uint32_t restSum = 0;
  for (int i = 0; i < 64; ++i) {
    restSum += analogRead(PIEZO_ADC_PIN);
    delay(2);
  }
  piezoRest = restSum / 64;
}

void enterSensorMode() {
  fadeActive = false;
  hintStyle = 0xFF;                               // 跨模式残留卫生
  mode = RuntimeMode::Sensor;
  pixels.setBrightness(LED_BRIGHTNESS_MAX);
  clearPixels();                                             // 交接：清灯
  reseedRest();                                              // 32 次静息均值重播种
  piezoRefractUntil = millis() + HANDOVER_SILENCE_MS;        // 交接静默，防头一秒误触发
  startFlash(pixels.Color(0, 200, 80), 2);                   // 绿闪 2 下（与静默并行）
}

// withFlash=false 用于开机初始化（V2 §5.6：开机无确认闪，灯效即反馈）；
// 入模动作（清灯/复位/亮度恢复）与按键路径完全一致，仅反馈不同。
void enterPresetMode(bool withFlash = true) {
  fadeActive = false;
  hintStyle = 0xFF;                               // 跨模式残留卫生
  mode = RuntimeMode::Preset;
  pixels.setBrightness(LED_BRIGHTNESS_MAX);
  clearPixels();
  resetPresetState();
  if (withFlash) {
    startFlash(pixels.Color(40, 120, 255), 2);               // 蓝闪 2 下
  }
}

void enterStandby() {
  mode = RuntimeMode::Standby;
  flashActive = false;                                       // 中止可能在播的确认闪
  hintStyle = 0xFF;                                          // 中止可能在播的提示
  startStandbyFade();                                        // 从当前亮度 300ms 渐灭
}

// ---- 压电引擎（仅传感器模式被调度；窗口期全速采样）----
void handlePiezoSensor() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // ---- 窗口期：全速采样，记录最小/最大 ----
  if (piezoInWindow) {
    const int raw = analogRead(PIEZO_ADC_PIN);
    if (raw < piezoMin) piezoMin = raw;
    if (raw > piezoMax) piezoMax = raw;

    if (nowUs - piezoWindowStartUs >= PIEZO_WINDOW_US) {
      piezoInWindow = false;
      piezoRefractUntil = nowMs + PIEZO_REFRACT_MS;

      const int swing = piezoMax - piezoMin;
      if (swing >= PIEZO_TRIGGER) {
        effectStyle = styleIndex;                 // 快照：渲染中切档不跳变
        effectStart = nowMs;                      // 照常记录（细节 A：渲染延至闪毕）
#if STRIKE_CONTINUOUS
        // §5.3 连续映射：触发处快照 swing 原值（封顶 1500），
        // N/色相/波速/时长由渲染侧 §5.3 公式现算（activeColor/activeSpan/activeSpeed）
        effectSwing = (uint16_t)min(swing, IMPACT_LEVEL2);
        effectUntil = nowMs + durationFromSwing(effectSwing);
#else
        const uint8_t intensity =
            (swing >= IMPACT_LEVEL2) ? 2 : (swing >= IMPACT_LEVEL1) ? 1 : 0;
        if (intensity == 0) {
          effectColor = pixels.Color(0, 255, 255);
        } else if (intensity == 1) {
          effectColor = pixels.Color(255, 255, 0);
        } else {
          effectColor = pixels.Color(255, 0, 0);
        }
        effectSpan = STYLE_S2_BAR[intensity];
        effectSpeed = STYLE_S3_SPEED_MPS[intensity];
        effectUntil = nowMs + IMPACT_BASE_MS + intensity * IMPACT_LEVEL_MS;
#endif
        // §5.2：S1 判定白闪段不入效果计时——白闪后效果段仍完整 220+70×level
        if (effectStyle == 1) {
          effectUntil += STYLE_S1_FLASH_MS;
        }
        if (DEBUG_IMPACT) {
          Serial.printf("[impact] swing=%d style=%u span=%u speed=%u\n", swing,
                        effectStyle, effectSpan, effectSpeed);
        }
      }
    }
    return;
  }

  // ---- 静默期：不触发、不更新静息 ----
  if (nowMs < piezoRefractUntil) {
    return;
  }

  if (nowUs - piezoLastPollUs < PIEZO_POLL_US) {
    return;
  }
  piezoLastPollUs = nowUs;

  const int raw = analogRead(PIEZO_ADC_PIN);

  // 慢速跟踪静息电平：输出贴死在某条轨上时 rest 会跟过去，
  // 保证"从上轨向下掉"和"从下轨向上跳"都还能触发
  if (nowUs - piezoLastRestUs >= PIEZO_REST_UPDATE_US) {
    piezoLastRestUs = nowUs;
    piezoRest += (raw - piezoRest) / (1 << PIEZO_REST_SHIFT);
  }

  // 双向触发
  const int d = raw - piezoRest;
  if (d > PIEZO_TRIGGER || d < -PIEZO_TRIGGER) {
    piezoInWindow = true;
    piezoWindowStartUs = nowUs;
    piezoMin = raw;
    piezoMax = raw;
  }
}

// ---- 渲染取参（统一入口；来源随 STRIKE_CONTINUOUS 切换）----
// 离散：触发时快照的档位参数；连续：effectSwing 快照按 §5.3 公式现算。
uint32_t activeColor() {
#if STRIKE_CONTINUOUS
  return impactColorContinuous(effectSwing);
#else
  return effectColor;
#endif
}

uint8_t activeSpan() {
#if STRIKE_CONTINUOUS
  return spanFromSwing(effectSwing);
#else
  return effectSpan;
#endif
}

uint16_t activeSpeed() {
#if STRIKE_CONTINUOUS
  return speedFromSwing(effectSwing);
#else
  return effectSpeed;
#endif
}

// ---- S2/S3 绘制（撞击渲染与 §5.4 提示共用；参数已快照，全量重画无需清屏）----
void drawS2Bar(uint32_t color, uint8_t pulse, uint8_t span) {
  const int center = LED_COUNT / 2;
  const int from = center - span / 2;             // 8 灯=11..18（同 M1 击打口径）
  const int to = from + span - 1;                 // 16=7..22 / 30=0..29
  for (uint8_t index = 0; index < LED_COUNT; ++index) {
    if (index >= from && index <= to) {
      pixels.setPixelColor(index, dimColor(color, pulse));
    } else {
      pixels.setPixelColor(index, 0);
    }
  }
}

void drawS3Wave(uint32_t color, uint8_t pulse, uint32_t elapsed, uint16_t speed) {
  const int center = LED_COUNT / 2;
  // 波前距中心灯数 = elapsed × 波速；波前 2 灯全亮（§5.2），过处每灯 -40 渐灭尾迹
  const uint32_t waveFront = (uint32_t)((uint64_t)elapsed * speed / 1000);
  for (uint8_t index = 0; index < LED_COUNT; ++index) {
    const int distance = abs(index - center);
    if (distance <= (int)waveFront) {
      const uint32_t ledOffset = waveFront - (uint32_t)distance;
      const uint8_t falloff =
          (uint8_t)min((ledOffset < 2 ? 0ul : (ledOffset - 1) * 40), 255ul);
      const uint8_t brightness = (uint8_t)((pulse * (255 - falloff)) / 255);
      pixels.setPixelColor(index, dimColor(color, brightness));
    } else {
      pixels.setPixelColor(index, 0);
    }
  }
}

// ---- 撞击响应渲染（M2 多风格分派；公共=三档色 + 220+70×level ms）----
// 返回 true=撞击效果仍在活跃（§5.4 提示在其空闲时才渲染）。
bool runPiezoEffect() {
  uint32_t now = millis();
  if (now >= effectUntil) {
    clearPixels();
    return false;
  }

  const uint32_t duration = effectUntil - effectStart;
  uint32_t elapsed = now - effectStart;

  // ---- S1 判定风格：全条白闪 60ms（不入效果计时）→ 中心 12 灯力度色脉冲 ----
  if (effectStyle == 1) {
    if (elapsed < STYLE_S1_FLASH_MS) {
      pixels.fill(pixels.Color(255, 255, 255));
      pixels.show();
      return true;
    }
    elapsed -= STYLE_S1_FLASH_MS;                 // 脉冲段从 255 起跳、全时长衰减
  }
  const uint32_t dur = (effectStyle == 1) ? duration - STYLE_S1_FLASH_MS : duration;
  const uint8_t pulse = 255 - (uint8_t)min((elapsed * 255) / dur, 255ul);

  switch (effectStyle) {
    case 1: {   // S1 白闪后：中心 12 灯力度色脉冲（|index-center|<6，同 M1 现状）
      const int center = LED_COUNT / 2;
      pixels.clear();
      for (uint8_t index = 0; index < LED_COUNT; ++index) {
        const int distance = abs(index - center);
        if (distance < STYLE_S1_PULSE_SPAN) {
          const uint8_t brightness = constrain(pulse - distance * 30, 0, 255);
          pixels.setPixelColor(index, dimColor(effectColor, brightness));
        }
      }
      pixels.show();
      break;
    }
    case 2: {   // S2 力度→长度条（取参：离散档快照 / 连续 §5.3 现算）
      drawS2Bar(activeColor(), pulse, activeSpan());
      pixels.show();
      break;
    }
    case 3: {   // S3 波纹扩散（取参同上）
      drawS3Wave(activeColor(), pulse, elapsed, activeSpeed());
      pixels.show();
      break;
    }
    default: {  // 风格 0 = M1 现状：中心 12 灯脉冲（逐灯渐变）
      const int center = LED_COUNT / 2;
      pixels.clear();
      for (uint8_t index = 0; index < LED_COUNT; ++index) {
        const int distance = abs(index - center);
        if (distance < 6) {
          const uint8_t brightness = constrain(pulse - distance * 30, 0, 255);
          pixels.setPixelColor(index, dimColor(effectColor, brightness));
        }
      }
      pixels.show();
      break;
    }
  }
  return true;
}

// ---- §5.4 风格切换提示（微缩演示版）：S2=中心迷你条快衰减 ≤200ms；
// S3=单圈快速波纹 ≤250ms（120 灯/s × 250ms = 30 灯整一圈）。
// 复用撞击绘制函数合成触发，不占 effectUntil（与撞击状态机完全隔离）。
// 仲裁位置：flash/fade 之下、撞击空闲时才播；再次短按即打断进下一态。
void startStyleHint(uint8_t style) {
  hintStyle = style;
  hintStart = millis();
}

void hintRenderFrame() {
  if (hintStyle == 0xFF) {
    return;
  }
  const uint32_t total =
      (hintStyle == 3) ? STYLE_HINT_S3_MS : STYLE_HINT_S2_MS;
  const uint32_t elapsed = millis() - hintStart;
  if (elapsed >= total) {
    hintStyle = 0xFF;
    clearPixels();
    return;
  }
  const uint8_t pulse = 255 - (uint8_t)((elapsed * 255) / total);
  if (hintStyle == 2) {
    // 合成参数：L0 档形态——中心 8 灯 + 青（展示 S2"条"的特征）
    drawS2Bar(pixels.Color(0, 255, 255), pulse, 8);
  } else {
    // 合成参数：L2 档形态——120 灯/s 单圈 + 红（展示 S3"波"的特征）
    drawS3Wave(pixels.Color(255, 0, 0), pulse, elapsed, 120);
  }
  pixels.show();
}

// ---- 预设五效果（交互规范 §4，台球主题）----
void runPresetEffect() {
  uint32_t now = millis();
  if (now - lastPresetUpdate < 25) {
    return;
  }
  lastPresetUpdate = now;

  switch (presetIndex) {
    case 0: {   // 彩虹流（基线保留）
      for (uint8_t index = 0; index < LED_COUNT; ++index) {
        pixels.setPixelColor(index, wheel((index * 256 / LED_COUNT + rainbowOffset) & 0xFF));
      }
      rainbowOffset += 4;
      pixels.show();
      break;
    }
    case 1: {   // 号球色板：30 灯 = 15 球 × 2 灯，静态摆球架（灯0=1号杆尾，灯29=15号杆头）
      for (uint8_t index = 0; index < LED_COUNT; ++index) {
        const uint8_t ball = index / 2 + 1;
        if (ball <= 8) {
          pixels.setPixelColor(index, ballColor(ball));
        } else if (index % 2 == 0) {
          pixels.setPixelColor(index, pixels.Color(220, 220, 220));  // 条纹球：白
        } else {
          pixels.setPixelColor(index, ballColor(ball - 8));          // 条纹球：本色
        }
      }
      pixels.show();
      break;
    }
    case 2: {   // 呼吸：台呢绿，沿用现波形（周期 5.1s）
      uint8_t breathe = (now / 20) % 255;
      if (breathe > 127) {
        breathe = 255 - breathe;
      }
      pixels.fill(dimColor(pixels.Color(0, 150, 70), 70 + breathe));
      pixels.show();
      break;
    }
    case 3: {   // 追光扫描：白色母球单点往返，软尾迹 5 灯（60 帧/向）
      pixels.clear();
      pixels.setPixelColor(scanPos, pixels.Color(255, 255, 255));
      for (uint8_t t = 1; t <= 5; ++t) {
        const int8_t trail = scanPos - scanDir * (int8_t)t;
        if (trail >= 0 && trail < LED_COUNT) {
          pixels.setPixelColor(trail, dimColor(pixels.Color(255, 255, 255), 255 - t * 40));
        }
      }
      pixels.show();

      if (++scanHalf >= 2) {          // 2 帧/灯步 => 29 步 ≈ 60 帧/向 @25ms
        scanHalf = 0;
        const int8_t next = scanPos + scanDir;
        if (next < 0 || next >= (int8_t)LED_COUNT) {
          scanDir = -scanDir;
        } else {
          scanPos = next;
        }
      }
      break;
    }
    case 4: {   // 击打闪光：中心 2 灯白核膨胀至 8 灯，360ms 衰减，随机 2.5-4s 重演
      if (!striking) {
        if (strikeNextAt == 0) {
          strikeNextAt = now + 1500;  // 进档后 1.5s 首演
        }
        if (now >= strikeNextAt) {
          striking = true;
          strikeStart = now;
        } else {
          clearPixels();
        }
        break;
      }
      const uint32_t e = now - strikeStart;
      if (e >= 360) {
        striking = false;
        strikeNextAt = now + random(2500, 4000);
        clearPixels();
        break;
      }
      const uint8_t half = min(e / 25, 3ul);        // 0..3：2 灯 -> 8 灯（11..18，中心对 14/15）
      const uint8_t decay = 255 - (uint8_t)((e * 255) / 360);
      pixels.clear();
      const int left = LED_COUNT / 2 - 1 - half;
      const int right = LED_COUNT / 2 + half;
      for (int idx = left; idx <= right; ++idx) {
        pixels.setPixelColor(idx, dimColor(pixels.Color(255, 255, 255), decay));
      }
      pixels.show();
      break;
    }
    default:
      presetIndex = 0;                              // 兜底，不产生黑档
      break;
  }
}

// ---- 按键轮询：消抖 + 短按（释放沿）+ 长按（按住 2s 触发一次）----
PressEvent pollButton(Button &b) {
  PressEvent ev = PressEvent::None;
  const uint32_t now = millis();
  const bool physical = digitalRead(b.pin) == (BUTTON_ACTIVE_LOW ? LOW : HIGH);

  if (physical != b.rawPressed && now - b.lastEdgeMs >= BUTTON_DEBOUNCE_MS) {
    b.lastEdgeMs = now;
    b.rawPressed = physical;
    if (physical) {
      b.pressStartMs = now;                         // 按下沿
      b.longFired = false;
    } else if (!b.longFired &&
               now - b.pressStartMs < BUTTON_LONGPRESS_MS) {
      ev = PressEvent::Short;                       // 释放沿且未达长按 => 短按
    }
  }

  if (b.rawPressed && !b.longFired &&
      now - b.pressStartMs >= BUTTON_LONGPRESS_MS) {
    b.longFired = true;
    ev = PressEvent::Long;
  }
  return ev;
}

void handleButtons() {
  // 确认闪/渐灭期间按键扫描照常（规范 §2）
  const PressEvent e1 = pollButton(btnEffect);
  const PressEvent e2 = pollButton(btnMode);

  if (e2 == PressEvent::Long) {                     // btn2 长按：灭灯待机（待机中无效）
    if (mode != RuntimeMode::Standby) {
      enterStandby();
    }
    return;
  }

  if (e2 == PressEvent::Short) {                    // btn2 短按：互切 / 唤醒
    if (mode == RuntimeMode::Standby) {
      enterSensorMode();                            // 唤醒复用绿闪
    } else if (mode == RuntimeMode::Sensor) {
      enterPresetMode();
    } else {
      enterSensorMode();
    }
    return;
  }

  if (e1 == PressEvent::Short) {
    if (mode == RuntimeMode::Standby) {
      enterSensorMode();                            // 待机中任意短按唤醒
    } else if (mode == RuntimeMode::Preset) {
      presetIndex = (presetIndex + 1) % PRESET_COUNT;
      resetPresetState();
      lastPresetUpdate = 0;                         // 新效果立即渲染
      saveImpactState();                            // V2 §5.5 写点：档位变更即落 NVS
    } else if (mode == RuntimeMode::Sensor) {
      styleIndex = (styleIndex + 1) % STYLE_COUNT;  // 四态循环：M1 脉冲→S1→S2→S3
      // §5.4 四态互异提示：0=绿闪 60ms / S1=白闪 60ms（复用确认闪 FSM，单闪）；
      // S2/S3=微缩演示（复用渲染分支合成触发，见 hintRenderFrame）
      if (styleIndex == 0) {
        startFlash(pixels.Color(0, 150, 70), 1);    // 台呢绿
      } else if (styleIndex == 1) {
        startFlash(pixels.Color(255, 255, 255), 1); // 白
      } else {
        startStyleHint(styleIndex);
      }
      saveImpactState();                            // V2 §5.5 写点：档位变更即落 NVS
    }
  }
  // btn1 长按：键位语义预留（M3 现场重标定入口），暂不实现
}

}  // namespace

void cueControllerSetup() {
  Serial.begin(115200);
  analogReadResolution(12);                         // 与 piezo_test 同口径（遗留件）
  pinMode(PIEZO_ADC_PIN, INPUT);
  analogSetPinAttenuation(PIEZO_ADC_PIN, ADC_11db);

  if (BUTTON_ACTIVE_LOW) {
    pinMode(BTN_MODE, INPUT_PULLUP);
    pinMode(BTN_EFFECT, INPUT_PULLUP);
  } else {
    pinMode(BTN_MODE, INPUT_PULLDOWN);
    pinMode(BTN_EFFECT, INPUT_PULLDOWN);
  }

  pixels.begin();
  pixels.setBrightness(LED_BRIGHTNESS_MAX);         // 亮度帽（§2.6，USB 供电安全值）
  clearPixels();

  // V2 §5.6：M1 的上电 64 次静息初始化已移除——开机恒进预设模式，引擎不触达；
  // 传感器基线一律走 btn2 交接链（enterSensorMode → reseedRest，32 次均值）。
  // V2 §5.5：开机恢复 effect/style 档位索引（越界回退默认档）。
  loadImpactState();

  Serial.println();
  Serial.println(F("[ecue] runtime firmware ready (M2)"));
  Serial.println(F("[ecue] boot -> preset mode (C semantics) | NVS restore effect/style"));
  Serial.printf("[ecue] effect=%u style=%u | LED_BRIGHTNESS_MAX = %u\n",
                presetIndex, styleIndex, LED_BRIGHTNESS_MAX);

  // C 语义（V2 §5.6）：开机恒进预设灯效模式——enterPresetMode 本体（含全部
  // 入模动作）作为唯一非按键调用点，开机初始化恰一次；无确认闪，灯效即反馈。
  enterPresetMode(false);
}

void cueControllerLoop() {
  // P1-2 竞态修复（20260930 交叉复核发现）：窗口期只采样，无按键无渲染。
  // 若窗口内放行按键，btn2 释放沿会触发 enterPresetMode/enterStandby：
  // 其 clearPixels() 直接 show() 违反"窗口期禁止一切 show()"（细节B），
  // 且 mode≠Sensor 后引擎门控停转、piezoInWindow 无人关闭，
  // 下方渲染仲裁被永久跳过（预设蓝闪/五效果/渐灭全部卡死）。
  // 窗口最长 30ms，按键与渲染延后一个窗口，人手不可感（消抖 40ms 更不受影响）。
  if (piezoInWindow) {
    if (mode == RuntimeMode::Sensor) {
      handlePiezoSensor();                          // 引擎照常全速采样/关窗/记录撞击
    }
    return;
  }

  handleButtons();                                  // 闪/渐灭期间扫描照常

  if (mode == RuntimeMode::Sensor) {
    handlePiezoSensor();                            // 引擎门控：仅传感器模式运行
  }

  // ---- 渲染仲裁 ----
  if (fadeActive) {
    fadeRenderFrame();
    return;
  }
  if (flashActive) {
    flashRenderFrame();                             // 确认闪优先（细节A：撞击渲染延至闪毕）
    return;
  }
  if (mode == RuntimeMode::Sensor) {
    const bool impactActive = runPiezoEffect();
    if (!impactActive) {
      hintRenderFrame();                            // §5.4 提示：撞击空闲时播，撞击活跃即让位
    }
  } else if (mode == RuntimeMode::Preset) {
    runPresetEffect();
  }
  // 待机：渐灭结束后保持全灭，无渲染
}
