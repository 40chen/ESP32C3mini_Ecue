#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "cue_controller.h"

namespace {
constexpr uint8_t AO_PIN = 4;
constexpr uint8_t LED_PIN = 7;
constexpr uint8_t LED_COUNT = 30;
constexpr uint8_t BUTTON_PIEZO = 6;
constexpr uint8_t BUTTON_PRESET = 5;

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum class LedMode : uint8_t {
  Off,
  Piezo,
  Preset
};

LedMode currentMode = LedMode::Off;
uint8_t presetIndex = 0;
uint32_t lastPresetUpdate = 0;

// ---- 压电撞击检测（改进版：双向触发 + 饱和免疫，调参说明见 README）----
constexpr uint32_t PIEZO_POLL_US = 500;      // 平时轮询间隔，约2kHz
constexpr uint32_t PIEZO_REST_US = 2000;     // 静息电平更新节奏，时间常数约128ms
constexpr int      PIEZO_REST_SHIFT = 6;     // 静息EMA步长1/64
constexpr int      PIEZO_TRIGGER = 150;      // 触发阈值（码），约92mV@2.5V满量程
constexpr uint32_t PIEZO_WINDOW_US = 20000;  // 触发后高速采样窗口
constexpr uint32_t PIEZO_REFRACT_MS = 200;   // 触发后静默期，防余振连发
constexpr int      IMPACT_LEVEL1 = 600;      // swing >= 此值 -> 黄
constexpr int      IMPACT_LEVEL2 = 1500;     // swing >= 此值 -> 红
constexpr bool     DEBUG_IMPACT = true;      // 触发时打印swing，标定完阈值可改false

uint32_t piezoLastPollUs = 0;
uint32_t piezoLastRestUs = 0;
int      piezoRest = 0;
bool     piezoInWindow = false;
uint32_t piezoWindowStartUs = 0;
int      piezoMin = 0;
int      piezoMax = 0;
uint32_t piezoRefractUntil = 0;

uint32_t effectStart = 0;
uint32_t effectUntil = 0;
uint32_t effectColor = 0;

bool piezoButtonPressed = false;
bool presetButtonPressed = false;
uint32_t piezoButtonDebounce = 0;
uint32_t presetButtonDebounce = 0;

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

bool readButtonPress(uint8_t pin, uint32_t &debounceTime, bool &pressed) {
  bool currentPressed = digitalRead(pin) == LOW;
  if (millis() - debounceTime < 40) {
    return false;
  }

  if (currentPressed && !pressed) {
    pressed = true;
    debounceTime = millis();
    return true;
  }

  if (!currentPressed && pressed) {
    pressed = false;
    debounceTime = millis();
  }
  return false;
}

void runPresetEffect() {
  uint32_t now = millis();
  if (now - lastPresetUpdate < 25) {
    return;
  }
  lastPresetUpdate = now;

  switch (presetIndex) {
    case 0: {
      static uint8_t rainbowOffset = 0;
      for (uint8_t index = 0; index < LED_COUNT; ++index) {
        pixels.setPixelColor(index, wheel((index * 256 / LED_COUNT + rainbowOffset) & 0xFF));
      }
      rainbowOffset += 4;
      pixels.show();
      break;
    }
    case 1: {
      static uint8_t wavePosition = 0;
      pixels.clear();
      for (uint8_t index = 0; index < LED_COUNT; ++index) {
        int distance = abs(index - wavePosition);
        if (distance < 6) {
          pixels.setPixelColor(index, dimColor(pixels.Color(0, 180, 255), 255 - distance * 40));
        }
      }
      wavePosition = (wavePosition + 1) % LED_COUNT;
      pixels.show();
      break;
    }
    case 2: {
      static uint8_t sweepPosition = 0;
      pixels.clear();
      for (uint8_t index = 0; index < LED_COUNT; ++index) {
        int distance = abs(index - sweepPosition);
        if (distance < 5) {
          pixels.setPixelColor(index, dimColor(pixels.Color(255, 120, 0), 255 - distance * 55));
        }
      }
      sweepPosition = (sweepPosition + 1) % LED_COUNT;
      pixels.show();
      break;
    }
    case 3: {
      uint8_t breathe = (now / 20) % 255;
      if (breathe > 127) {
        breathe = 255 - breathe;
      }
      pixels.fill(dimColor(pixels.Color(0, 180, 255), 70 + breathe));
      pixels.show();
      break;
    }
    case 4: {
      static uint8_t ringPosition = 0;
      pixels.clear();
      for (uint8_t index = 0; index < LED_COUNT; ++index) {
        int distance = min(abs(index - ringPosition),
                          abs(index - (ringPosition + LED_COUNT / 2) % LED_COUNT));
        if (distance < 3) {
          pixels.setPixelColor(index, dimColor(pixels.Color(255, 0, 203), 255 - distance * 70));
        }
      }
      ringPosition = (ringPosition + 1) % LED_COUNT;
      pixels.show();
      break;
    }
    default:
      clearPixels();
      break;
  }
}

void runPiezoEffect() {
  uint32_t now = millis();
  if (now >= effectUntil) {
    clearPixels();
    return;
  }

  // 修复：从真实触发时刻(effectStart)起算衰减，原式反推起点导致 pulse 恒为 0
  const uint32_t duration = effectUntil - effectStart;
  const uint32_t elapsed = now - effectStart;
  const uint8_t pulse =
      255 - (uint8_t)min((elapsed * 255) / duration, 255ul);

  pixels.clear();
  int center = LED_COUNT / 2;
  for (uint8_t index = 0; index < LED_COUNT; ++index) {
    int distance = abs(index - center);
    if (distance < 6) {
      uint8_t brightness = constrain(pulse - distance * 30, 0, 255);
      pixels.setPixelColor(index, dimColor(effectColor, brightness));
    }
  }
  pixels.show();
}

void handlePiezoSensor() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // ---- 窗口期：全速采样，记录最小/最大 ----
  if (piezoInWindow) {
    const int raw = analogRead(AO_PIN);
    if (raw < piezoMin) piezoMin = raw;
    if (raw > piezoMax) piezoMax = raw;

    if (nowUs - piezoWindowStartUs >= PIEZO_WINDOW_US) {
      piezoInWindow = false;
      piezoRefractUntil = nowMs + PIEZO_REFRACT_MS;

      const int swing = piezoMax - piezoMin;
      if (swing >= PIEZO_TRIGGER) {
        const uint8_t intensity =
            (swing >= IMPACT_LEVEL2) ? 2 : (swing >= IMPACT_LEVEL1) ? 1 : 0;
        if (intensity == 0) {
          effectColor = pixels.Color(0, 255, 255);
        } else if (intensity == 1) {
          effectColor = pixels.Color(255, 255, 0);
        } else {
          effectColor = pixels.Color(255, 0, 0);
        }
        effectStart = nowMs;
        effectUntil = nowMs + 220 + intensity * 70;
        if (DEBUG_IMPACT) {
          Serial.printf("[impact] swing=%d level=%u\n", swing, intensity);
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

  const int raw = analogRead(AO_PIN);

  // 慢速跟踪静息电平：输出贴死在某条轨上时 rest 会跟过去，
  // 保证"从上轨向下掉"和"从下轨向上跳"都还能触发
  if (nowUs - piezoLastRestUs >= PIEZO_REST_US) {
    piezoLastRestUs = nowUs;
    piezoRest += (raw - piezoRest) / (1 << PIEZO_REST_SHIFT);
  }

  // 双向触发：修复原版只认正向跳变、而实测撞击是负向跳变的问题
  const int d = raw - piezoRest;
  if (d > PIEZO_TRIGGER || d < -PIEZO_TRIGGER) {
    piezoInWindow = true;
    piezoWindowStartUs = nowUs;
    piezoMin = raw;
    piezoMax = raw;
  }
}
}

void handleButtons() {
  if (readButtonPress(BUTTON_PIEZO, piezoButtonDebounce, piezoButtonPressed)) {
    if (currentMode == LedMode::Piezo) {
      currentMode = LedMode::Off;
      clearPixels();
    } else {
      currentMode = LedMode::Piezo;
      presetIndex = 0;
    }
  }

  if (readButtonPress(BUTTON_PRESET, presetButtonDebounce, presetButtonPressed)) {
    if (currentMode == LedMode::Preset) {
      presetIndex = (presetIndex + 1) % 6;
      if (presetIndex == 0) {
        currentMode = LedMode::Off;
        clearPixels();
      }
    } else {
      currentMode = LedMode::Preset;
      presetIndex = 0;
    }
  }
}

void cueControllerSetup() {
  Serial.begin(115200);
  pinMode(AO_PIN, INPUT);
  analogSetPinAttenuation(AO_PIN, ADC_11db);
  pinMode(BUTTON_PIEZO, INPUT_PULLUP);
  pinMode(BUTTON_PRESET, INPUT_PULLUP);

  pixels.begin();
  pixels.setBrightness(255);
  clearPixels();

  // 静息电平初始化：取 64 次平均，比单次读数抗噪
  long restSum = 0;
  for (int i = 0; i < 64; ++i) {
    restSum += analogRead(AO_PIN);
    delay(2);
  }
  piezoRest = restSum / 64;
}

void cueControllerLoop() {
  handleButtons();
  handlePiezoSensor();

  if (currentMode == LedMode::Piezo) {
    runPiezoEffect();
  } else if (currentMode == LedMode::Preset) {
    runPresetEffect();
  }
}
