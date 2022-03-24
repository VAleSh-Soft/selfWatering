#pragma once
#include <Arduino.h>

#define LOG_ON // ведение отладочного лога

#define CHANNEL_COUNT 3 // количество каналов полива (от 1 до 5)

// ==== пины =========================================
#define LEDS_PIN 12              // пин подключения адресных светодиодов (количество каналов + 1 для датчика уровня)
#define LIGHT_SENSOR_PIN A6      // пин датчика света
#define BTN_PIN 2                // пин подключения кнопки
#define BUZZER_PIN 3             // пин пищалки
#define WATER_LEVEL_SENSOR_PIN 4 // пин датчика уровня воды в емкости

#if (CHANNEL_COUNT > 0)
#define HPOWER_1_SENSOR_PIN 5    // пин питания первого датчика влажности
#define HUMIDITY_1_SENSOR_PIN A0 // пин данных первого датчика влажности
#define PUMP_1_PIN 6             // пин управления первой помпой
#endif

#if (CHANNEL_COUNT > 1)
#define HPOWER_2_SENSOR_PIN 7    // пин питания второго датчика влажности
#define HUMIDITY_2_SENSOR_PIN A1 // пин данных второго датчика влажности
#define PUMP_2_PIN 8             // пин управления второй помпой
#endif

#if (CHANNEL_COUNT > 2)
#define HPOWER_3_SENSOR_PIN 9    // пин питания третьего датчика влажности
#define HUMIDITY_3_SENSOR_PIN A2 // пин данных третьего датчика влажности
#define PUMP_3_PIN 10            // пин управления третьей помпой
#endif

#if (CHANNEL_COUNT > 3)
#define HPOWER_4_SENSOR_PIN 11   // пин питания четвертого датчика влажности
#define HUMIDITY_4_SENSOR_PIN A3 // пин данных четвертого датчика влажности
#define PUMP_4_PIN 12            // пин управления четвертой помпой
#endif

#if (CHANNEL_COUNT == 5)
#define HPOWER_5_SENSOR_PIN A4   // пин питания пятого датчика влажности
#define HUMIDITY_5_SENSOR_PIN A7 // пин данных пятого датчика влажности
#define PUMP_5_PIN A5            // пин управления пятой помпой
#endif
/* Не нужно занимать пины 0 и 1, т.к. через них идет передача данных Serial */

// ==== класс кнопки с дополнительными фишками =======
class swButton : public shButton
{
private:
  byte clickBtnCount = 0; // счетчик одиночных кликов кнопки, нужен для входа в режим настроек
  /*
3 клика – настройка времени работы помпы
4 клика – настройка порога влажности (четыре уровня, 400 – 500 – 600 – 700)
5 кликов – настройка минимального интервала (дней)
6 кликов – настройка максимального интервала (дней)
7 кликов – включение/отключение датчика влажности для канала
8 кликов – включение/отключение канала целиком
2 коротких + 1 длинный - включение/отключение датчика света и пищалки об ошибках
*/

public:
  swButton(byte button_pin) : shButton(button_pin)
  {
    shButton::setLongClickMode(LCM_ONLYONCE);
    shButton::setVirtualClickOn(true);
    shButton::setTimeout(1000);
  }

  void setClickBtnCount(byte _count)
  {
    clickBtnCount = _count;
  }

  byte getClickBtnCount()
  {
    return (clickBtnCount);
  }
};

// ==== режим работы =================================

#define MODE_DEFAULT 0 // основной режим работы
#define MODE_SETTING 1 // режим настроек каналов

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

// флаги состояния настройки канала
#define FL_NONE 0       // нормальное состояние
#define FL_RUN_DATA 1   // флаг изменения данных/включения помпы - кнопка удерживается более секунды
#define FL_STOP_DATA 2  // флаг окончания изменения данных/остановки помпы - кнопка отпущена
#define FL_SAVE_DATA 3  // флаг необходимости сохранения настройки и перехода на другой канал
#define FL_NEXT 4       // флаг перехода на другой канал без сохранения изменений
#define FL_EXIT 5       // флаг выхода из настроек по таймауту
#define FL_CHECK_DATA 6 // флаг проверки сохраненного значения; используется для проверки настройки помпы

// структура с данными канала
struct ChannelData
{
  byte pump_pin;      // пин, на который подключена помпа
  byte p_sensor_pin;  // пин подключения питания датчика
  byte d_sensor_pin;  // пин данных датчика
  byte channel_state; // состояние канала: CNL_DONE, CNL_WORK, CNL_CHECK, CNL_RESCAN, CNL_ERROR
  byte metering_flag; // состояние датчика: SNS_NONE, SNS_METERING, SNS_WATERING, SNS_TESTING
  uint32_t p_timer;   // таймер работы помпы
  byte m_count;       // счетчик циклов измерений влажности
  byte min_max_count; // количество пустых циклов, для определения, когда еще рано запускать полив или нужно запускать в любом случае без замера влажности
  word m_data;        // данные последнего замера влажности
  byte flag;          // поле для передачи данных в задачи
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

// получение данных о текущих ошибках; выдает битовую маску: первый бит - канал датчика уровня воды, второй и далее биты - каналы полива
byte getErrors();

// ручной старт задачи
//  flag - флаг, с которым запускается задача: SNS_NONE, SNS_METERING, SNS_WATERING
//  run - собственно, флаг запуска - если false, то запуска не будет
void manualStart(byte flag, bool run = true);

// ==== другие задачи ================================

void mainTimer();
void setLeds();
void runErrorBuzzer();
void rescanStart();
void runSetBuzzer();
void runSetChannels();
void returnToDefMode();

// ==== разное =======================================

// опрос кнопки
void checkButton();

// ==== отладка ======================================
// если отладка не нужна, закомментировать строку #define LOG_ON // ведение отладочного лога

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