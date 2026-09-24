#pragma once

// 压电撞击观察模块（ESP32-C3 mini, GPIO4 = ADC1_CH4）
// 对外接口与原工程保持一致：main.cpp 无需改动
void piezoTestSetup();
void piezoTestLoop();
