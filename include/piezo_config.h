#pragma once

// =====================================================================
// ECUE 双模式固件 —— 全局可调参数（单一来源）
//
// 口径依据（三方冻结，2026-09-30）：
//   - 采样窗口 30ms（与标定 CSV 同口径，阈值 150/600/1500 免重标）
//   - 消抖 40ms（沿用实测现码）
//   - 亮度帽 40%（32 灯 WS2812 全白 @255 约 1.9A，USB 供电必压降；M3 灯数 30→32 已重算）
// 修改本文件后需重新编译烧录（pio run）。
// =====================================================================

// ---- 灯条 ----
constexpr uint8_t LED_PIN = 7;
constexpr uint8_t LED_COUNT = 32;               // M3：30→32 颗（20261002 拍板）；0-29 与旧版逐位等价，30/31=母球位
// 亮度帽：255=全亮（32 灯约 1.9A，仅外接 5V>=2A 时可用）；100 约 0.75A（USB 直供安全值，32 灯口径）。
constexpr uint8_t LED_BRIGHTNESS_MAX = 100;

// ---- 按钮 ----
// 接线口径：按钮一端 GPIO、另一端 GND，固件内部上拉，按下=低，无需外部电阻。
// 若实物另一端接 3.3V：把 BUTTON_ACTIVE_LOW 改为 false 即可（自动改用下拉），语义不变。
constexpr uint8_t BTN_EFFECT = 0;               // btn1：预设灯效模式内循环效果
constexpr uint8_t BTN_MODE = 10;                // btn2：传感器/预设互切；长按 2s 待机
constexpr bool BUTTON_ACTIVE_LOW = true;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;
constexpr uint32_t BUTTON_LONGPRESS_MS = 2000;

// ---- M4 亮度四档 + 组合键重标定（2026-10-03 拍板链 + 《亮度与重标定-交互口径稿-v2》）----
// 档位 50/100/150/200（08:11 拍板 4 档；值域 50-200 出自 08:00 拍板，255 标度）。
// 供电口径（08:00 拍板）：>100 档仅外接 5V≥2A 或电池安全（200 档 32 灯全白 ≈1.5A，
// USB 500mA 直供必压降）；固件不做供电感知硬钳，USB 直供调试期请停留在 100。
constexpr uint8_t BRIGHT_LEVEL_COUNT = 4;
constexpr uint8_t BRIGHT_LEVELS[BRIGHT_LEVEL_COUNT] = {50, 100, 150, 200};
constexpr uint8_t BRIGHT_DEFAULT_INDEX = 1;      // 出厂默认 100（恰为第 2 档；NVS 无记录/越界回退值）
constexpr uint32_t BUTTON_COMBO_WINDOW_MS = 150; // 双键同按成立窗：第二键距首键按下 ≤150ms
constexpr uint32_t BUTTON_COMBO_HOLD_MS = 500;   // 同按保持 ≥500ms 触发现场重标定
constexpr uint32_t BRIGHT_REPEAT_MS = 500;       // 亮度模式内连发节拍（按住每 500ms 进一档）
constexpr uint32_t BRIGHT_IDLE_EXIT_MS = 2000;   // 亮度模式内无操作自动退出（退出瞬间 NVS 保存+白快闪）
constexpr uint32_t RESEED_BREATH_MS = 600;       // 重标定流程：慢呼吸时长
constexpr uint32_t RESEED_STEP_MS = 40;          // 重采节奏：32 采样 × 40ms ≈ 1.28s（进度条 1 灯/采样）

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

// ---- M2 撞击三风格（交互规范 V2 §5；风格 0 = M1 现状中心脉冲）----
// 四态循环（冻结口径 §五#1）：0=M1 脉冲默认 → S1 判定 → S2 长度条 → S3 波纹。
constexpr uint8_t STYLE_COUNT = 4;
constexpr uint32_t STYLE_S1_FLASH_MS = 60;      // S1 判定：全条白闪 60ms → 中心 12 灯力度色脉冲
constexpr uint8_t  STYLE_S1_PULSE_SPAN = 6;     // S1 中心 12 灯（|index-center|<6，同 M1 现状）
constexpr uint8_t  STYLE_S2_BAR[3] = {8, 16, 32};     // S2 力度→长度条（三档灯数；满条档 M3 随 32 灯适配）
constexpr uint16_t STYLE_S3_SPEED_MPS[3] = {60, 90, 120};  // S3 波纹扩散波速（灯/秒，三档）
constexpr uint32_t IMPACT_BASE_MS = 220;        // 撞击响应时长 = 220 + 70 × level（M1 现状，公共）
constexpr uint32_t IMPACT_LEVEL_MS = 70;

// ---- M2 风格切换提示（V2 §5.4，微缩演示版；降级/砍掉由复核方裁决，裁决前按此落码）----
// 状态 0=绿闪 RGB(0,150,70) 60ms；S1=白闪 60ms（复用确认闪 FSM，单闪）；
// S2/S3=复用各自渲染分支的合成触发，时长如下：
constexpr uint32_t STYLE_HINT_S2_MS = 200;      // S2 迷你条快速衰减上限（规范 ≤200ms）
constexpr uint32_t STYLE_HINT_S3_MS = 267;      // S3 单圈波纹（120 灯/s × 267ms = 32 灯整一圈，M3 灯数适配）

// ---- M2 力度连续映射（V2 §5.3，实现但默认关）----
// 0=离散三档（C12 饱和削顶下离散更可信，默认）；1=启用 §5.3 连续映射：
//   N=8+round((swing-150)/1350×(LED_COUNT-8)) 钳位[8,LED_COUNT]（M3 随灯数自适应）；S3 波速 60→120 灯/s 同式；
//   色相 青→黄→红 按 150/600/1500 三锚点分段线性；时长 220+round(t×210) ms；
//   swing>1500 按 1500 封顶。标定数据到位后一行启用。
#define STRIKE_CONTINUOUS 0

// ---- NVS 断电记忆（V2 §5.5：开机恒进预设模式，仅恢复 effect/style 档位索引）----
constexpr bool NVS_ENABLED = true;              // false = 纯内存行为（调参期临时关闭用）
constexpr char NVS_NS[] = "ecue";               // 命名空间
constexpr char NVS_KEY[] = "state";             // 单 key 四字段打包（effect/style/mode/bright；M2/M3 旧三字节包兼容读取）
