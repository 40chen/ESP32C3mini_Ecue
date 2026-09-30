#include <Arduino.h>

#include "cue_controller.h"
#include "piezo_test.h"

// =====================================================================
// ECUE 运行时固件入口
//
// 两套可选程序：
//   - Runtime   ：双模式运行时（传感器 + 预设灯效 + 待机），M1 交付主体
//   - PiezoTest ：压电标定观察程序（串口 CSV 口径，标定/排查电路用）
// 切换：默认 Runtime；需标定时给 platformio.ini 的 build_flags 临时加
//       -DAPP_MODE_PIEZO_TEST 重编译，标定完移除即可。
// =====================================================================

enum class AppMode : uint8_t {
  PiezoTest,   // 压电标定观察程序
  Runtime      // 双模式运行时（传感器 + 预设灯效 + 待机）
};

#if defined(APP_MODE_PIEZO_TEST)
constexpr AppMode ACTIVE_APP = AppMode::PiezoTest;
#else
constexpr AppMode ACTIVE_APP = AppMode::Runtime;
#endif

void setup() {
  if (ACTIVE_APP == AppMode::PiezoTest) {
    piezoTestSetup();
  } else {
    cueControllerSetup();
  }
}

void loop() {
  if (ACTIVE_APP == AppMode::PiezoTest) {
    piezoTestLoop();
  } else {
    cueControllerLoop();
  }
}
