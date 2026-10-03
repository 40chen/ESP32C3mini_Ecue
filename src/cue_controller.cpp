#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

#include "cue_controller.h"
#include "piezo_config.h"

// =====================================================================
// ECUE 运行时双模式固件（M3）
//
// 模式：传感器（压电引擎+撞击响应）/ 预设灯效（十一效果循环）/ 亮度调节 / 待机（全灭）
// 按键：btn2(IO10) 短按互切模式、长按 2s 待机；
//       btn1(IO0)  传感器模式内循环撞击风格（M2）/ 预设模式内循环效果（M1）；
//                  长按 2s 进亮度模式（M4，原重标定预留让位）；
//       双键同按 ≥500ms = 现场重标定（M4，组合期屏蔽两键单触/长按）。
// 待机中任意短按/长按唤醒回传感器模式（待机互斥：不进亮度模式、不做组合判定）。
// 开机：恒进预设灯效模式（C 语义，V2 §5.6），NVS 恢复 effect/style 档位
//       索引（§5.5），开机无确认闪——灯效即反馈。
// M3（2026-10-02）：预设 5→11 档（5-10=开球爆散/母球游走/黑八环转/彩球进袋/
//       星尘/火焰，六案全上管理员实测淘汰制）；灯珠 30→32（0-29 逐位等价，
//       30/31=母球位）；S2 满条档、连续映射钳位随灯数自适应。
// M4（2026-10-03）：亮度四档 50/100/150/200（出厂默认 100，NVS 记忆，渐灭/恢复
//       用当前档）；现场重标定=双键同按（呼吸→进度条 32 采样→白双闪）；
//       反向循环不加（08:10 拍板）。交互口径：《亮度与重标定-交互口径稿-v2》。
// 参数见 piezo_config.h；交互口径见《ECUE_交互规范_设计匠人_20260930》V2。
// =====================================================================

namespace {

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---- 运行时模式 ----
enum class RuntimeMode : uint8_t {
  Sensor,       // 传感器模式：压电引擎 + 撞击响应
  Preset,       // 预设灯效模式：十一效果循环
  BrightAdjust, // 亮度调节模式（M4）：btn1 长按 2s 进入，刻度尺调档，2s 无操作保存退出
  Standby       // 待机：全灭
};

RuntimeMode mode = RuntimeMode::Sensor;       // 真实开机模式由 setup 内 enterPresetMode
                                              // 设定（V2 §5.6，唯一非按键调用点）

constexpr bool DEBUG_IMPACT = true;           // 触发时打印 swing，标定完可改 false

uint8_t presetIndex = 0;                      // 预设域档位记忆（来回切模式不重置）
constexpr uint8_t PRESET_COUNT = 11;          // M3：0-4 基线 + 5-10 新增（A-F 六案全上），无黑档；Off 语义移交 btn2 长按
uint32_t lastPresetUpdate = 0;                // 预设效果 25ms 渲染节流
uint8_t styleIndex = 0;                       // 传感器域撞击风格档位（M2 四态循环）
uint8_t brightnessIndex = BRIGHT_DEFAULT_INDEX;  // 亮度档（M4 四档；NVS 持久化，出厂默认 100）
RuntimeMode brightReturnMode = RuntimeMode::Preset;  // 亮度模式退出后轻量返回的模式
uint32_t brightLastActionMs = 0;              // 亮度模式内最后交互时刻（2s 无操作退出）
uint32_t brightNextStepMs = 0;                // 连发下一拍时刻
int8_t   brightHoldDir = 0;                   // 连发方向：+1=btn1 升 / -1=btn2 降 / 0=无

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
// N=8+round((swing-150)/1350×(LED_COUNT-8)) 钳位[8,LED_COUNT]（M3：32 灯）；S3 波速 60→120 同式；
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
  return (uint8_t)constrain(8 + (int)lroundf(t * (LED_COUNT - 8)), 8, (int)LED_COUNT);
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
uint8_t  fadeFrom = BRIGHT_LEVELS[BRIGHT_DEFAULT_INDEX];   // 渐灭起点初值（实际由 startStandbyFade 按当前档刷新）

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

// ---- M4 双键同按重标定 FSM + 按下沿跟踪（待机/亮度模式内不启用组合判定）----
enum class ComboState : uint8_t { Idle, Pending, Cancel, Flow };
ComboState comboState = ComboState::Idle;
uint32_t comboStartMs = 0;                    // 双键均按住时刻（500ms 保持计时起点）
bool prevHeld1 = false;                       // 上一轮按住态（按下沿检测用）
bool prevHeld2 = false;
bool reseedFlowActive = false;                // 重标定流程渲染接管标志
uint8_t  reseedPhase = 0;                     // 0=慢呼吸 1=进度条采样 2=满条收尾
uint32_t reseedPhaseStart = 0;
uint32_t reseedNextSampleMs = 0;
uint8_t  reseedSampleCount = 0;
uint32_t reseedSum = 0;

// ---- 预设效果内部状态（切档/切模式时经 resetPresetState() 复位）----
uint8_t  rainbowOffset = 0;                 // 效果0 彩虹流
int8_t   scanPos = 0;                       // 效果3 追光扫描
int8_t   scanDir = 1;
uint8_t  scanHalf = 0;                      // 2 帧/灯步 => 60 帧/向（规范 §4）
bool     striking = false;                  // 效果4 击打闪光
uint32_t strikeStart = 0;
uint32_t strikeNextAt = 0;

// ---- M3 效果5-10 状态（切档/切模式经 resetPresetState() 复位）----
uint8_t  breakPhase = 0;                    // 效果5 开球爆散：0=击杆 1=白闪 2=爆散反弹 3=渐灭
uint32_t breakPhaseStart = 0;
int16_t  breakPos[15];                      // 粒子位置（1/8 灯定标）
int16_t  breakVel[15];                      // 粒子速度（1/8 灯/帧，含符号）
uint16_t breakFlags = 0;                    // bit i = 粒子 i 已反弹（仅反弹一次）
int8_t   wanderPos = 0;                     // 效果6 母球游走（0..LED_COUNT-2）
int8_t   wanderDir = 1;
int8_t   wanderVel = 12;                    // 1/8 灯/帧，钳位 [4,16] = 0.5~2 灯/帧
int8_t   wanderAcc = 0;                     // 亚像素累加器（满 8/8 进一灯）
uint8_t  pocketPhase = 0;                   // 效果8 进袋：0=待机 1=滚动 2=端闪
uint32_t pocketNextAt = 0;
uint32_t pocketStart = 0;
uint32_t pocketColor = 0;                   // 开滚时取的球色快照
uint8_t  pocketBall = 1;                    // 球号 1..15
int8_t   pocketDir = 1;                     // 滚动方向 +1/-1
uint8_t  dustId[LED_COUNT];                 // 效果9 星尘：0=灭 1=暖白 2..9=球色1..8
uint8_t  dustLife[LED_COUNT];               // 星尘剩余寿命（255→0，约 400ms）
int8_t   flameHeight = 9;                   // 效果10 焰高（8-10 灯，缓变）
uint32_t flameNextShift = 0;                // 下次焰高突变时刻

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

// 动态效果用球色（8 号炭灰提亮防隐形；条纹球取本色代表）
uint32_t ballVisible(uint8_t ball) {
  if (ball == 8) {
    return pixels.Color(70, 70, 70);
  }
  return ballColor(ball <= 8 ? ball : ball - 8);
}

// 开球粒子配色：i=0..14 → 1-7 全色 / 8 号 / 9-15 条纹本色
uint32_t burstColor(uint8_t i) {
  if (i == 7) {
    return ballVisible(8);
  }
  return (i < 7) ? ballColor(i + 1) : ballColor(i - 7);
}

// 星尘配色：1=暖白，2..9 = 球色 1..8
uint32_t dustColorOf(uint8_t id) {
  if (id == 1) {
    return pixels.Color(255, 220, 170);
  }
  return ballVisible(id - 1);
}

void resetPresetState() {
  rainbowOffset = 0;
  scanPos = 0;
  scanDir = 1;
  scanHalf = 0;
  striking = false;
  strikeStart = 0;
  strikeNextAt = 0;
  // ---- M3 效果5-10 ----
  breakPhase = 0;
  breakPhaseStart = 0;
  breakFlags = 0;
  wanderPos = (int8_t)(LED_COUNT / 2);
  wanderDir = 1;
  wanderVel = 12;
  wanderAcc = 0;
  pocketPhase = 0;
  pocketNextAt = 0;
  pocketColor = 0;
  pocketDir = 1;
  flameHeight = 9;
  flameNextShift = 0;
  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    dustId[i] = 0;
    dustLife[i] = 0;
  }
}

// ---- NVS 断电记忆（V2 §5.5 + M4 亮度档）----
// 单 key 四字段打包（effect/style/mode/bright）。mode 为保留字段：写侧恒写 0、
// 读侧忽略（C 语义开机恒进预设，mode 不参与恢复；字段保留为未来语义回摆预埋，
// 回摆零迁移成本）。写点=handleButtons 内档位变更处 + 亮度模式退出瞬间（稿 §2.3；
// P1-2 修复后 handleButtons 只在窗口外被调，满足"写点窗口门外"冻结纪律）。
struct NvState {
  uint8_t effect;   // presetIndex（预设域档位）
  uint8_t style;    // styleIndex（传感器域档位）
  uint8_t mode;     // 保留字段，恒写 0，读侧不使用
  uint8_t bright;   // brightnessIndex（M4 亮度档；旧 3 字节包无此字段 → 默认档）
};

void saveImpactState() {
  if (!NVS_ENABLED) return;
  const NvState s{presetIndex, styleIndex, 0, brightnessIndex};   // mode 恒写 0（V2 §5.5）
  Preferences prefs;
  prefs.begin(NVS_NS, false);
  prefs.putBytes(NVS_KEY, &s, sizeof(s));
  prefs.end();
}

void loadImpactState() {
  presetIndex = 0;
  styleIndex = 0;
  brightnessIndex = BRIGHT_DEFAULT_INDEX;
  if (!NVS_ENABLED) return;
  Preferences prefs;
  prefs.begin(NVS_NS, true);                     // 只读打开；NVS 不存在时读出空
  NvState s{0, 0, 0, BRIGHT_DEFAULT_INDEX};
  const size_t storedLen = prefs.getBytesLength(NVS_KEY);
  if (storedLen == sizeof(NvState)) {            // M4 四字段包
    prefs.getBytes(NVS_KEY, &s, sizeof(s));
  } else if (storedLen == 3) {                   // M2/M3 旧三字段包：亮度落默认档，效果/花色照常恢复
    uint8_t legacy[3] = {0, 0, 0};
    prefs.getBytes(NVS_KEY, legacy, sizeof(legacy));
    s.effect = legacy[0];
    s.style = legacy[1];
  }
  prefs.end();
  // 越界回退默认档（冻结纪律）
  presetIndex = (s.effect < PRESET_COUNT) ? s.effect : 0;
  styleIndex = (s.style < STYLE_COUNT) ? s.style : 0;
  brightnessIndex = (s.bright < BRIGHT_LEVEL_COUNT) ? s.bright : BRIGHT_DEFAULT_INDEX;
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
    pixels.setBrightness(BRIGHT_LEVELS[brightnessIndex]);   // 渐灭可能动过全局亮度，恢复当前档
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
  fadeFrom = BRIGHT_LEVELS[brightnessIndex];   // 渐灭起点=当前设定档（稿 §5 渐灭适配）
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
  pixels.setBrightness(BRIGHT_LEVELS[brightnessIndex]);
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
  pixels.setBrightness(BRIGHT_LEVELS[brightnessIndex]);
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
  startStandbyFade();                                        // 从当前档 300ms 渐灭（M4：起点=当前设定值）
}

// ---- M4 亮度调节模式（口径稿 v2.1 §2；刻度尺 8/16/24/32 灯 = 50/100/150/200）----
void drawBrightScale() {
  pixels.clear();
  const uint8_t lit = (uint8_t)((brightnessIndex + 1) * LED_COUNT / BRIGHT_LEVEL_COUNT);
  for (uint8_t i = 0; i < lit; ++i) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 255));
  }
  pixels.show();                                 // 全局亮度=当前档，所见即所得
}

// 顶格/底格再按：当前刻度 200ms 呼吸提示（稿 §2.2；模式内无引擎无窗口，短暂阻塞安全）
void brightEdgeHint() {
  const uint8_t lit = (uint8_t)((brightnessIndex + 1) * LED_COUNT / BRIGHT_LEVEL_COUNT);
  for (uint8_t f = 0; f < 4; ++f) {
    pixels.clear();
    for (uint8_t i = 0; i < lit; ++i) {
      pixels.setPixelColor(i, dimColor(pixels.Color(255, 255, 255), (uint8_t)(90 + f * 40)));
    }
    pixels.show();
    delay(50);
  }
  drawBrightScale();
}

void enterBrightAdjust() {
  brightReturnMode = mode;                       // 退出后轻量返回（不清灯/不重播种）
  mode = RuntimeMode::BrightAdjust;
  flashActive = false;                           // 中止可能在播的确认闪
  hintStyle = 0xFF;                              // 跨模式残留卫生
  pixels.setBrightness(BRIGHT_LEVELS[brightnessIndex]);
  drawBrightScale();                             // 进模式即显示当前档刻度
  const uint32_t now = millis();
  brightLastActionMs = now;
  brightNextStepMs = now + BRIGHT_REPEAT_MS;     // 手指仍按住：500ms 后开始连发上行
  brightHoldDir = 1;                             // 入口按住的是 btn1 → 上行连发
}

// 步进一档（+1 升 / -1 降）；端点停住不回绕（稿 §2.2 定稿点）。返回是否实际换档。
bool brightStep(int8_t dir) {
  const int next = (int)brightnessIndex + dir;
  const uint32_t now = millis();
  brightLastActionMs = now;
  brightNextStepMs = now + BRIGHT_REPEAT_MS;
  if (next < 0 || next >= BRIGHT_LEVEL_COUNT) {
    return false;
  }
  brightnessIndex = (uint8_t)next;
  pixels.setBrightness(BRIGHT_LEVELS[brightnessIndex]);
  drawBrightScale();
  return true;
}

void exitBrightAdjust() {
  saveImpactState();                             // §2.3：退出瞬间 NVS 写入一次（快闪=已保存）
  startFlash(pixels.Color(255, 255, 255), 1);    // 整条白快闪 1 次 = 已保存
  mode = brightReturnMode;                       // 亮度调节未触碰两域状态，轻量返回
  if (mode == RuntimeMode::Preset) {
    lastPresetUpdate = 0;                        // 效果画面闪毕立即恢复
  }
  brightHoldDir = 0;
}

// ---- M4 现场重标定流程（双键同按 500ms 触发；呼吸→进度条→白双闪；非阻塞）----
// 重采深度（技术裁决，稿 §3.3"已裁决"）：32 次均值按 32 灯进度条节奏展开
// （40ms/采样 ≈1.28s，比瞬时 3.2ms 重采更抗噪）；不做连窗口校验；
// M3 无可判失败条件，不设失败分支（稿 §3.2 失败态已同步删除），错误码机制预留。
void startReseedFlow() {
  reseedFlowActive = true;
  reseedPhase = 0;
  reseedPhaseStart = millis();
  comboState = ComboState::Flow;                 // 流程中屏蔽一切按键事件（稿 §3.1）
  Serial.println(F("[ecue] reseed: combo triggered"));
}

void reseedFlowTick() {
  const uint32_t now = millis();
  if (reseedPhase == 0) {                        // 慢呼吸（台呢绿，区别于保存快闪）
    if (now - reseedPhaseStart >= RESEED_BREATH_MS) {
      reseedPhase = 1;
      reseedPhaseStart = now;
      reseedNextSampleMs = now;
      reseedSampleCount = 0;
      reseedSum = 0;
      pixels.clear();
      pixels.show();
    } else {
      const uint32_t e = (now - reseedPhaseStart) % 400;   // 400ms 三角波
      const uint8_t pulse = (uint8_t)(e < 200 ? 40 + e : 40 + (400 - e));
      pixels.fill(dimColor(pixels.Color(0, 150, 70), pulse));
      pixels.show();
    }
    return;
  }
  if (reseedPhase == 1) {                        // 进度条：1 灯/采样，32 次均值（语义呼应稿 §3.2）
    if (now < reseedNextSampleMs) {
      return;
    }
    reseedNextSampleMs = now + RESEED_STEP_MS;
    if (piezoInWindow) {
      return;                                    // 撞击窗口内不采样（防污染），顺延一拍
    }
    reseedSum += analogRead(PIEZO_ADC_PIN);
    pixels.setPixelColor(reseedSampleCount, pixels.Color(0, 200, 80));
    ++reseedSampleCount;
    pixels.show();
    if (reseedSampleCount >= LED_COUNT) {
      piezoRest = (int)(reseedSum / LED_COUNT);  // 与 reseedRest 同口径：32 次均值
      reseedPhase = 2;
      reseedPhaseStart = now;
    }
    return;
  }
  // 阶段 2：满条保持 150ms → 白双闪收尾，回常规显示
  if (now - reseedPhaseStart >= 150) {
    reseedFlowActive = false;
    comboState = ComboState::Cancel;             // 等双键释放回 Idle（吞残余释放事件）
    startFlash(pixels.Color(255, 255, 255), 2);
    Serial.printf("[ecue] reseed done: rest=%d\n", piezoRest);
  }
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
  const int from = center - span / 2;             // 8 灯=12..19（32 灯口径；M1 30 灯时为 11..18）
  const int to = from + span - 1;                 // 16=8..23 / 32=0..31（满条）
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
// S3=单圈快速波纹 ≤267ms（120 灯/s × 267ms = 32 灯整一圈，M3 灯数适配）。
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

// ---- 预设十一效果（0-4 基线保留 + 5-10 M3 新增；25ms 节流，全 millis() 非阻塞）----
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
    case 1: {   // 号球色板：32 灯 = 15 球 × 2 灯（0-29 与 30 灯版逐位等价）+ 杆端母球纯白 ×2
      for (uint8_t index = 0; index < LED_COUNT - 2; ++index) {
        const uint8_t ball = index / 2 + 1;
        if (ball <= 8) {
          pixels.setPixelColor(index, ballColor(ball));
        } else if (index % 2 == 0) {
          pixels.setPixelColor(index, pixels.Color(220, 220, 220));  // 条纹球：白
        } else {
          pixels.setPixelColor(index, ballColor(ball - 8));          // 条纹球：本色
        }
      }
      pixels.setPixelColor(LED_COUNT - 2, pixels.Color(255, 255, 255));  // 母球（纯白）
      pixels.setPixelColor(LED_COUNT - 1, pixels.Color(255, 255, 255));
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
      const uint8_t half = min(e / 25, 3ul);        // 0..3：2 灯 -> 8 灯（12..19，中心对 15/16，32 灯口径）
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
    case 5: {   // M3·A 开球爆散：白块加速击杆→全条白闪80ms→15球色粒子喷散+端点反弹→渐灭，周期约3.5s
      const uint32_t e = now - breakPhaseStart;
      if (breakPhase == 0) {                        // 击杆：4 灯白块自杆端(0)加速冲向中心
        if (e >= 600) {
          breakPhase = 1;
          breakPhaseStart = now;
          break;
        }
        const int32_t p = (int32_t)((e * e) * (LED_COUNT / 2) / (600 * 600));  // 二次缓入=加速
        pixels.clear();
        for (int idx = p - 3; idx <= p && idx < LED_COUNT; ++idx) {
          if (idx >= 0) {
            pixels.setPixelColor(idx, pixels.Color(255, 255, 255));
          }
        }
        pixels.show();
        break;
      }
      if (breakPhase == 1) {                        // 全条白闪 80ms → 粒子初始化
        if (e >= 80) {
          for (uint8_t i = 0; i < 15; ++i) {
            breakPos[i] = (LED_COUNT / 2) * 8;      // 自中心喷出（1/8 灯定标）
            const int vel = (int)random(16, 27);    // 2~3.4 灯/帧 起速
            breakVel[i] = (i < 8) ? (int16_t)vel : (int16_t)-vel;
          }
          breakFlags = 0;
          breakPhase = 2;
          breakPhaseStart = now;
          break;
        }
        pixels.fill(pixels.Color(255, 255, 255));
        pixels.show();
        break;
      }
      if (breakPhase == 2) {                        // 爆散：喷散+衰减，端点反弹一次（速度×0.7）
        pixels.clear();
        bool allRest = true;
        for (uint8_t i = 0; i < 15; ++i) {
          if (breakVel[i] != 0) {
            allRest = false;
            breakPos[i] += breakVel[i];
            breakVel[i] = breakVel[i] * 95 / 100;   // 线性速度衰减
            const int p = breakPos[i] >> 3;
            const int sign = (breakVel[i] >= 0) ? 1 : -1;
            if (p <= 0 || p >= LED_COUNT - 1) {     // 触端：首次反弹，二触停壁
              if ((breakFlags & (1 << i)) == 0) {
                breakFlags |= (uint16_t)(1 << i);
                breakVel[i] = (int16_t)(-breakVel[i] * 7 / 10);
                breakPos[i] = (int16_t)constrain(breakPos[i], 0, (LED_COUNT - 1) * 8);
              } else {
                breakVel[i] = 0;
              }
            }
            if (breakVel[i] != 0) {
              const int q = constrain((int)(breakPos[i] >> 3), 0, LED_COUNT - 1);
              const uint32_t col = burstColor(i);
              pixels.setPixelColor(q, col);
              const int trail = q - sign;           // 30% 衰减拖尾
              if (trail >= 0 && trail < LED_COUNT) {
                pixels.setPixelColor(trail, dimColor(col, 77));
              }
            }
          }
        }
        if (allRest || e >= 2200) {
          breakPhase = 3;
          breakPhaseStart = now;
        }
        pixels.show();
        break;
      }
      // 阶段3：渐灭 600ms → 回到击杆（全程约 3.5s 循环）
      if (e >= 600) {
        breakPhase = 0;
        breakPhaseStart = now;
        break;
      }
      const uint8_t fadeK = 255 - (uint8_t)((e * 255) / 600);
      pixels.clear();
      for (uint8_t i = 0; i < 15; ++i) {
        const int q = constrain((int)(breakPos[i] >> 3), 0, LED_COUNT - 1);
        pixels.setPixelColor(q, dimColor(burstColor(i), fadeK));
      }
      pixels.show();
      break;
    }
    case 6: {   // M3·B 母球游走：2 灯白点 + 3 灯软尾迹(40%/20%/8%)，惯性随机漫步 0.5~2 灯/帧，端点反弹
      wanderVel += (int8_t)random(-1, 2);           // 惯性漫步：速度每帧漂移 ±1/8 灯
      if (wanderVel < 4) {
        wanderVel = 4;
      }
      if (wanderVel > 16) {
        wanderVel = 16;
      }
      wanderAcc += wanderVel;
      while (wanderAcc >= 8) {                      // 累满 1/8×8 → 进一灯
        wanderAcc -= 8;
        const int8_t next = wanderPos + wanderDir;
        if (next < 0 || next > LED_COUNT - 2) {     // 预留 2 灯头位，端点反弹
          wanderDir = -wanderDir;
        } else {
          wanderPos = next;
        }
      }
      pixels.clear();
      pixels.setPixelColor(wanderPos, pixels.Color(255, 255, 255));
      const int8_t lead = wanderPos + wanderDir;
      if (lead >= 0 && lead < LED_COUNT) {
        pixels.setPixelColor(lead, dimColor(pixels.Color(255, 255, 255), 210));
      }
      for (uint8_t t = 1; t <= 3; ++t) {
        const int8_t tail = wanderPos - wanderDir * (int8_t)t;
        if (tail >= 0 && tail < LED_COUNT) {
          const uint8_t tb = (t == 1) ? 102 : (t == 2) ? 51 : 20;   // 40%/20%/8%
          pixels.setPixelColor(tail, dimColor(pixels.Color(255, 255, 255), tb));
        }
      }
      pixels.show();
      break;
    }
    case 7: {   // M3·C 黑八环转：中心 4 灯炭灰慢呼吸(2s)，两侧 14 灯球色块镜像外旋（约 1s/格）
      const uint32_t cyc = now % 2000;              // 2s 呼吸周期（三角波）
      const uint32_t tri = (cyc < 1000) ? cyc : 2000 - cyc;
      pixels.clear();
      for (uint8_t k = 0; k < LED_COUNT / 2 - 2; ++k) {   // 每侧 14 灯，2 灯/格 × 7 格循环
        const uint8_t blockIdx = (uint8_t)((k / 2 + now / 1000) % 7);   // 向两端外旋
        const uint32_t col = ballColor(blockIdx + 1);
        pixels.setPixelColor((uint8_t)(LED_COUNT / 2 - 2 - k), col);    // 左侧向杆端
        pixels.setPixelColor((uint8_t)(LED_COUNT / 2 + 1 + k), col);    // 右侧镜像向杆头
      }
      const uint8_t breath = (uint8_t)(60 + tri * 140 / 1000);
      for (uint8_t idx = LED_COUNT / 2 - 2; idx <= LED_COUNT / 2 + 1; ++idx) {
        pixels.setPixelColor(idx, dimColor(pixels.Color(40, 40, 40), breath));  // 炭灰慢呼吸
      }
      pixels.show();
      break;
    }
    case 8: {   // M3·D 彩球进袋：每 4~6s 随机球色单球滚过全条(约1.2s)→对端 3 灯连闪 2 下(间隔150ms)
      if (pocketPhase == 0) {                       // 待机
        if (pocketNextAt == 0) {
          pocketNextAt = now + 1200;                // 进档后 1.2s 首演
        }
        if (now >= pocketNextAt) {
          pocketBall = (uint8_t)random(1, 16);
          pocketDir = (random(0, 2) == 0) ? 1 : -1;
          pocketColor = ballVisible(pocketBall);
          pocketStart = now;
          pocketPhase = 1;
        } else {
          clearPixels();
        }
        break;
      }
      if (pocketPhase == 1) {                       // 滚动：0 ↔ LED_COUNT-1，约 1.2s
        const uint32_t e = now - pocketStart;
        if (e >= 1200) {
          pocketPhase = 2;
          pocketStart = now;
          break;
        }
        const int32_t progress = (int32_t)(e * (LED_COUNT - 1) / 1200);
        const int p = (pocketDir == 1) ? progress : (LED_COUNT - 1 - progress);
        pixels.clear();
        pixels.setPixelColor(p, pocketColor);
        if (p + 1 < LED_COUNT) {
          pixels.setPixelColor(p + 1, dimColor(pocketColor, 140));   // 球身 2 灯
        }
        if (p >= 1) {
          pixels.setPixelColor(p - 1, dimColor(pocketColor, 60));    // 尾迹
        }
        pixels.show();
        break;
      }
      // 端闪：入袋端 3 灯连闪 2 下（亮 150/灭 150 ×2，共 600ms）
      const uint32_t e = now - pocketStart;
      if (e >= 600) {
        pocketPhase = 0;
        pocketNextAt = now + random(2200, 4200);    // 整周期约 4~6s
        break;
      }
      pixels.clear();
      if (e % 300 < 150) {
        const int base = (pocketDir == 1) ? LED_COUNT - 3 : 0;
        for (int idx = base; idx < base + 3; ++idx) {
          pixels.setPixelColor(idx, pocketColor);
        }
      }
      pixels.show();
      break;
    }
    case 9: {   // M3·E 星尘：每帧 5% 概率随机点亮 1 灯（暖白/球色），400ms 渐灭，最省电
      if (random(0, 100) < 5) {
        const uint8_t idx = (uint8_t)random(0, LED_COUNT);
        if (dustLife[idx] == 0) {
          dustId[idx] = (uint8_t)random(1, 10);     // 1=暖白 2..9=球色 1..8
          dustLife[idx] = 255;
        }
      }
      pixels.clear();
      for (uint8_t i = 0; i < LED_COUNT; ++i) {
        if (dustLife[i] > 0) {
          pixels.setPixelColor(i, dimColor(dustColorOf(dustId[i]), dustLife[i]));
          dustLife[i] = (dustLife[i] > 16) ? (uint8_t)(dustLife[i] - 16) : 0;   // 400ms @25ms 帧
        }
      }
      pixels.show();
      break;
    }
    case 10: {  // M3·F 火焰：杆端 8-10 灯深红→橙→黄尖梯度，低频摇曳叠加每帧随机抖动，焰高缓变
      if (now >= flameNextShift) {                  // 焰高缓变：约每 0.7-1.3s ±1 灯
        flameHeight = (int8_t)constrain(flameHeight + (int)random(-1, 2), 8, 10);
        flameNextShift = now + 700 + random(0, 600);
      }
      pixels.clear();
      for (uint8_t i = 0; i < 10; ++i) {
        if (i >= flameHeight) {
          break;
        }
        const uint8_t g = (uint8_t)((i * 255) / (flameHeight - 1));   // 0=底部 → 255=焰尖
        uint8_t r, gc, b;
        if (g < 140) {                              // 深红(150,15,0) → 橙(255,90,0)
          const uint8_t u = (uint8_t)(g * 255 / 140);
          r = (uint8_t)(150 + 105 * u / 255);
          gc = (uint8_t)(15 + 75 * u / 255);
          b = 0;
        } else {                                    // 橙(255,90,0) → 黄尖(255,210,60)
          const uint8_t u = (uint8_t)((g - 140) * 255 / 115);
          r = 255;
          gc = (uint8_t)(90 + 120 * u / 255);
          b = (uint8_t)(60 * u / 255);
        }
        const uint8_t ph = (uint8_t)((now / 6 + i * 29) & 0xFF);      // 低频摇曳（相位随灯错开）
        const uint8_t sway = (ph < 128) ? (uint8_t)(ph * 2) : (uint8_t)((255 - ph) * 2);
        int bright = constrain(150 + sway * 40 / 255 + (int)random(-25, 26), 40, 255);
        pixels.setPixelColor(i, dimColor(pixels.Color(r, gc, b), (uint8_t)bright));
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
  const bool held1 = btnEffect.rawPressed;
  const bool held2 = btnMode.rawPressed;
  const bool press1Edge = held1 && !prevHeld1;   // 按下沿（消抖后）
  const bool press2Edge = held2 && !prevHeld2;
  prevHeld1 = held1;
  prevHeld2 = held2;
  const uint32_t nowMs = millis();

  // ---- M4 双键同按重标定 FSM（待机/亮度模式内不启用，让位单键语义；稿 §3）----
  if (mode != RuntimeMode::Standby && mode != RuntimeMode::BrightAdjust) {
    switch (comboState) {
      case ComboState::Idle:
        if (press1Edge && held2 &&
            nowMs - btnMode.pressStartMs <= BUTTON_COMBO_WINDOW_MS) {
          comboState = ComboState::Pending;      // 后落键 btn1，btn2 于 150ms 窗内已在按
          comboStartMs = nowMs;
        } else if (press2Edge && held1 &&
                   nowMs - btnEffect.pressStartMs <= BUTTON_COMBO_WINDOW_MS) {
          comboState = ComboState::Pending;      // 后落键 btn2，btn1 于 150ms 窗内已在按
          comboStartMs = nowMs;
        }
        break;
      case ComboState::Pending:
        if (!held1 || !held2) {
          comboState = ComboState::Cancel;       // 保持失败：作废，不补偿任何单触（稿 §3.1）
        } else if (nowMs - comboStartMs >= BUTTON_COMBO_HOLD_MS) {
          startReseedFlow();                     // 同按保持 500ms → 现场重标定
        }
        break;
      case ComboState::Cancel:
        if (!held1 && !held2) {
          comboState = ComboState::Idle;         // 双键释放，吞掉的事件到此为止
        }
        break;
      case ComboState::Flow:
        break;                                   // 流程结束转 Cancel（见 reseedFlowTick）
    }
    if (comboState != ComboState::Idle) {
      return;                                    // 组合期/流程中：屏蔽两键一切单触/长按
    }
  }

  // ---- M4 亮度模式内交互（稿 §2.2）：短按按下沿立即步进 + 按住连发；长按/互切/组合全屏蔽 ----
  if (mode == RuntimeMode::BrightAdjust) {
    if (press1Edge) {
      brightHoldDir = 1;
      if (!brightStep(1)) {
        brightEdgeHint();                        // 顶格再按：200ms 呼吸提示
      }
    }
    if (press2Edge) {
      brightHoldDir = -1;
      if (!brightStep(-1)) {
        brightEdgeHint();                        // 底格再按：同上（对称设计）
      }
    }
    if (brightHoldDir == 1 && held1 && nowMs >= brightNextStepMs) {
      if (brightnessIndex < BRIGHT_LEVEL_COUNT - 1) {
        brightStep(1);                           // 连发上行（进模式延续按住即连发）
      } else {
        brightNextStepMs = nowMs + BRIGHT_REPEAT_MS;   // 顶格驻留：停住不回绕
      }
    } else if (brightHoldDir == -1 && held2 && nowMs >= brightNextStepMs) {
      if (brightnessIndex > 0) {
        brightStep(-1);
      } else {
        brightNextStepMs = nowMs + BRIGHT_REPEAT_MS;
      }
    }
    if (!held1 && !held2 &&
        nowMs - brightLastActionMs >= BRIGHT_IDLE_EXIT_MS) {
      exitBrightAdjust();                        // 2s 无操作：NVS 保存 + 白快闪 + 轻量返回
    }
    return;                                      // 模式内 e1/e2 短按/长按事件全部丢弃
  }

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
    return;
  }

  if (e1 == PressEvent::Long) {                     // M4：btn1 长按 2s = 亮度模式入口
    if (mode == RuntimeMode::Standby) {
      enterSensorMode();                            // 稿 §2.1 待机互斥：只唤醒，不进亮度
    } else {
      enterBrightAdjust();                          // 预设/传感器模式 → 亮度调节
    }
    return;
  }
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
  pixels.setBrightness(BRIGHT_LEVELS[BRIGHT_DEFAULT_INDEX]);   // 先按出厂默认档上电（恢复档经 loadImpactState→enterPresetMode 套用）
  clearPixels();

  // V2 §5.6：M1 的上电 64 次静息初始化已移除——开机恒进预设模式，引擎不触达；
  // 传感器基线一律走 btn2 交接链（enterSensorMode → reseedRest，32 次均值）。
  // V2 §5.5：开机恢复 effect/style 档位索引（越界回退默认档）。
  loadImpactState();

  Serial.println();
  Serial.println(F("[ecue] runtime firmware ready (M3)"));
  Serial.println(F("[ecue] boot -> preset mode (C semantics) | NVS restore effect/style"));
  Serial.printf("[ecue] effect=%u style=%u bright=%u (idx %u/%u)\n",
                presetIndex, styleIndex, BRIGHT_LEVELS[brightnessIndex],
                brightnessIndex, BRIGHT_LEVEL_COUNT);

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
  if (reseedFlowActive) {
    reseedFlowTick();                               // M4：重标定流程接管（按键已屏蔽；窗口期不进来）
    return;
  }
  if (flashActive) {
    flashRenderFrame();                             // 确认闪优先（细节A：撞击渲染延至闪毕）
    return;
  }
  if (mode == RuntimeMode::BrightAdjust) {
    return;                                         // M4：刻度画面事件驱动（进入/步进时绘制），静置无刷帧
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
