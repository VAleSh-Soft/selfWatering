#pragma once
#include <Arduino.h>

// ==== пины =========================================
#define LIGHT_SENSOR_PIN A6      // пин датчика света
#define BUZZER_PIN 3             // пин пищалки
#define WATER_LEVEL_SENSOR_PIN 4 // пин датчика уровня воды в емкости
#define HPOWER_1_SENSOR_PIN 5    // пин питания первого датчика влажности
#define HUMIDITY_1_SENSOR_PIN A0 // пин данных первого датчика влажности
#define PUMP_1_PIN 6             // пин управления первой помпой
#define HPOWER_2_SENSOR_PIN 7    // пин питания второго датчика влажности
#define HUMIDITY_2_SENSOR_PIN A1 // пин данных второго датчика влажности
#define PUMP_2_PIN 8             // пин управления второй помпой
#define HPOWER_3_SENSOR_PIN 9    // пин питания третьего датчика влажности
#define HUMIDITY_3_SENSOR_PIN A2 // пин данных третьего датчика влажности
#define PUMP_3_PIN 10            // пин управления третьей помпой
#define BTN_PIN 11               // пин подключения кнопки
#define LEDS_PIN 12              // пин подключения адресных светодиодов

// ==== разное =======================================

// старт полива
//  pin - номер канала от 0 до 2
//  force - принудительный старт полива без учета текущих показаний датчика влажности
void runPump(byte pin, bool force = false);

// стоп полива
//  pin - номер канала от 0 до 2
void stopPump(byte pin);

// ==== задачи =======================================
void mainTimer();
void runPump1();
void runPump2();
void runPump3();
void setLeds();
void buzzerOn();