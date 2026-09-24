# ESP32C3mini_Ecue 集成版 —— 改动说明与调参指南

> 基于您上传的工程（espressif32@6.5.0 / Arduino core 2.0.14 / USB CDC）集成。
> `main.cpp`、`platformio.ini`、`include/*.h` 均未改动，直接用 PlatformIO 打开编译即可。

---

## 一、工程核对结果

```
ESP32C3mini_Ecue
├── platformio.ini          espressif32@6.5.0, arduino, USB CDC, NeoPixel 1.15.1
├── src/main.cpp            模式开关：PiezoTest / CueController（当前注释掉了后者）
├── src/piezo_test.cpp      原为 2kHz 原始值监视器（raw,min,max）→ 已替换为撞击观察器
├── src/cue_controller.cpp  LED 应用：30 灯 NeoPixel + 2 按键 + 压电触发 → 已修复 2 个 bug
└── include/                头文件未动（接口本来就一致）
```

## 二、发现并修复的两个 bug（cue_controller.cpp 原版）

1. **压电触发永远不亮**：原 `handlePiezoSensor` 只认正向跳变（`difference < 80` 直接 return）。
   而实测电路输出长期贴轨、撞击信号是**负向**跳变（"碰传感器数据会减小"）。
   → 已改为**双向触发**（|raw − rest| > 阈值），且静息电平会跟随贴轨状态。
2. **即使触发，LED 也是全黑**：原 `effectStart = effectUntil - 220 - effectBrightness*2`
   反推出的起点比触发时刻早 360ms 以上，`pulse` 恒为 0，亮度恒 0。
   → 已改为触发时记录真实 `effectStart`，衰减从 0ms 起算。

## 三、各文件改动清单

| 文件 | 改动 |
|---|---|
| `src/piezo_test.cpp` | 整体替换为撞击观察器：双向触发、20ms 全速采样窗口、输出 `swing,min,max,rest,hi,lo,satH,satL`、贴轨 5 秒状态提醒。用于**标定阈值**和观察电路状态 |
| `src/cue_controller.cpp` | ① 修复上述 2 个 bug；② 压电检测改为与观察器相同的双向触发 + 20ms 窗口 + 200ms 静默期；③ 强度分档改用窗口峰峰幅值 `swing`（阈值 `IMPACT_LEVEL1/2`）；④ 触发时串口打印 `[impact] swing=… level=…`（`DEBUG_IMPACT` 可关）；⑤ setup 增加显式 11dB 衰减与 64 次均值初始化静息电平。按键/预设灯效逻辑未动 |
| `include/piezo_test.h` | 增加注释，接口不变 |

## 四、使用流程

1. **先跑 PiezoTest 模式标定**（main.cpp 当前就是此模式）：烧录后敲击/捏传感器，
   记录 CSV 中 `swing` 列的典型值——轻敲、中击、重击各记几个，即可确定分档阈值。
2. 把得到的值填进 `cue_controller.cpp` 的 `IMPACT_LEVEL1`（黄）和 `IMPACT_LEVEL2`（红），
   触发灵敏度由 `PIEZO_TRIGGER`（默认 150 码 ≈ 92mV）控制。
3. `main.cpp` 注释切到 `CueController` 模式烧录：按键 GPIO6 切压电灯效、GPIO5 切预设灯效。
4. 标定完成后把 `DEBUG_IMPACT` 改为 `false` 关闭串口打印。

## 五、预期现象与注意

- **硬件未整改前**（C12 仍为 10pF、输出贴轨）：任何触碰 `swing` 都会顶满数千码，
  灯效大多显示红色级别——这是预期行为，不是软件问题。
- **换 C12=1nF 后**信号幅度缩小约 100 倍，务必按第四节重新标定阈值。
- 触发有约 20ms 窗口延迟（用于确认真实撞击并测峰峰值），肉眼不可感知。
- `pixels.show()` 刷新 30 颗灯约阻塞 0.9ms，窗口采样会有小空洞，对灯效触发无影响。
- 硬件整改清单（C12→1nF、AO_IO4 串 4.7k~10k、3.3V 供电、去耦、清洗）见
  《ESP32C3_Piezo_改进方案》中的 README，软件无法替代。
