#pragma once
#include <Arduino.h>

#define LOG_ON // ведение отладочного лога

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

// флаги состояния канала
#define CNL_DONE 0   // состояние покоя, работа сделана/поливать не нужно
#define CNL_WORK 1   // состояние работы, идет полив
#define CNL_CHECK 2  // состояние проверки после полива
#define CNL_RESCAN 3 // состояние перепроверки после полива, если датчик показал сухую землю
#define CNL_ERROR 4  // ошибка - после полива и перепроверки датчик показывает сухую землю

// флаги состояния датчика
#define SNS_NONE 0     // состояние покоя
#define SNS_METERING 1 // состояние измерения, датчик включен
#define SNS_WATERING 2 // состояние полива, датчик отключен, работает помпа
#define SNS_TESTING 3  // только замер влажности без полива вне зависимости от результата

// структура с данными канала
struct ChannelState
{
  byte pump_pin;      // пин, на который подключена помпа
  byte p_sensor_pin;  // пин подключения питания датчика
  byte d_sensor_pin;  // пин данных датчика
  byte channel_state; // состояние канала: CNL_DONE, CNL_WORK, CNL_CHECK, CNL_RESCAN, CNL_ERROR
  byte metering_flag; // состояние датчика: SNS_NONE, SNS_METERING, SNS_WATERING, SNS_TESTING
  byte p_count;       // счетчик итераций в процессе полива
  byte m_count;       // счетчик измерений влажности
  byte min_max_count; // количество пустых циклов, для определения, когда еще рано запускать полив или нужно запускать в любом случае без замера влажности
  word m_data;        // данные последнего замера влажности
};

// ==== работа =======================================

// работа канала - замер влажности и полив
void runChanel();

// канал в режиме измерения влажности
//  channel - индекс канала
void cnlMetering(byte channel);

// канал в режиме полива, работает помпа
//  channel - индекс канала
void cnlWatering(byte channel);

// получение данных о текущих ошибках; выдает битовую маску: первый бит - канал датчика уровня воды, второй-четвертый биты - каналы полива
byte getErrors();

// ручной старт задачи
//  flag - флаг, с которым запускается задача: SNS_NONE, SNS_METERING, SNS_WATERING
//  run - собственно, флаг запуска - если false, то запуска не будет
void manualStart(byte flag, bool run = true);

// ==== задачи =======================================

void mainTimer();
void setLeds();
void runBuzzer();
void rescanStart();

// ==== разное =======================================

// опрос кнопки
void checkButton();

// ==== отладка ======================================
// если отладка не нужна, закомментировать строку 4

#ifdef LOG_ON

// вывод данных последнего замера влажности по каналу
//  cnl - номер канала
void printLastMeteringData(byte cnl);

// вывод данных по статусу канала
//  cnl - номер канала
void printChannelStatus(byte cnl);

// работа с сериалом и обработка полученных команд
void checkSerial();

#endif