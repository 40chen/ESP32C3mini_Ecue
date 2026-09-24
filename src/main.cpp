#include <Arduino.h>

#include "cue_controller.h"
#include "piezo_test.h"

enum class AppMode : uint8_t {
  PiezoTest,        //压电测试程序
//   CueController      //灯光控制程序
};

constexpr AppMode ACTIVE_APP = AppMode::PiezoTest;

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
