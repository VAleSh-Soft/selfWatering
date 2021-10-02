#pragma once
#include <Arduino.h>

// ==== пины =========================================
#define LEDS_PIN 12              // пин подключения адресных светодиодов
#define LIGHT_SENSOR_PIN A6      // пин датчика света
#define BTN_PIN 2                // пин подключения кнопки
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

// ==== канал датчик/помпа ===========================

#define CNL_DONE 0  // состояние покоя, работа сделана/поливать не нужно
#define CNL_WORK 1  // состояние работы, идет полив
#define CNL_ERROR 3 // ошибка - после полива датчик показывает сухую землю

#define SNS_NONE 0     // состояние покоя
#define SNS_METERING 1 // состояние измерения, датчик включен
#define SNS_WATERING 2 // состояние полива, датчик отключен, работает помпа

struct ChannelState
{
  byte pump_pin;      // пин, на который подключена помпа
  byte p_sensor_pin;  // пин подключения питания датчика
  byte d_sensor_pin;  // пин данных датчика
  byte channel_state; // состояние канала: CNL_DONE, CNL_WORK, CNL_ERROR
  byte metering_flag; // состояние датчика: SNS_NONE, SNS_METERING, SNS_WATERING
  byte p_timer;       // счетчик итераций в процессе полива
  byte m_count;       // счетчик измерений влажности
  byte min_max_timer; // количество пустых циклов, для определения, когда еще рано запускать полив или нужно запускать в любом случае без замера влажности
};

// ==== работа =======================================

// работа канала - замер влажности и полив
//  index - номер канала от 0 до 2
//  force - принудительный старт полива без учета текущих показаний датчика влажности
void runChanel(byte index, bool force = false);

// канал в режиме измерения влажности
//  index - номер канала от 0 до 2
void cnlMetering(byte index);

// канал в режиме полива, работает помпа
//  index - номер канала от 0 до 2
void cnlWatering(byte index);

// ==== задачи =======================================

void mainTimer();
void runPump();
void setLeds();
void runBuzzer();