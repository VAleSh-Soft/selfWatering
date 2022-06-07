#pragma once
#include <Arduino.h>
// #include <avr/eeprom.h>
#include <EEPROM.h>

#define LOG_ON // ведение отладочного лога

#define CHANNEL_COUNT 3 // количество каналов полива (от 1 до 5)

// ==== настройки ====================================
#define FIRMWARE_VERSION "5.3.0"       // версия прошивки
#define MAX_DAY_COUNT_DEF 14           // максимальное количество суток, по истечении которого полив будет включен безусловно, значение по умолчанию
#define MIN_DAY_COUNT_DEF 7            // минимальное количество суток, до истечения которого полив не будет включен, значение по умолчанию
#define METERING_COUNT 8               // количество замеров влажности для усреднения результата; желательно задавать значение, равное степени числа 2 (2, 4, 8, 16 и т.д.)
#define BUZZER_TIMEOUT 300             // интервал срабатывания пищалки в режиме "ошибка" в секундах
#define LIGHT_SENSOR_THRESHOLD 150     // минимальные показания датчика света (0-1023)
#define MAX_PUMP_TIMER 60000ul         // максимальное время работы помпы, мс
#define DEFAULT_PUMP_TIMER 10000ul     // значение времени работы помпы по умолчанию
#define DEFAULT_HUMIDITY_THRESHOLD 600 // порог датчика влажности по умолчанию
// ===================================================

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
Манипуляции с кнопкой для управления модулем:
  - режимы настроек
3 коротких клика – настройка времени работы помпы
4 коротких клика – настройка порога влажности (четыре уровня, 400 – 500 – 600 – 700)
5 коротких кликов – настройка минимального интервала (дней)
6 коротких кликов – настройка максимального интервала (дней)
7 коротких кликов – включение/отключение датчика влажности для канала
8 коротких кликов – включение/отключение канала целиком
3 коротких + 1 длинный - включение/отключение датчика света и пищалки об ошибках
  - режимы неавтоматического полива
1 короткий + 1 длинный - выборочный полив каналов с автоматическим дозированием воды
2 коротких + 1 длинный - полностью ручной полив без автоматического дозирования воды
*/

public:
  swButton(byte button_pin) : shButton(button_pin)
  {
    shButton::setLongClickMode(LCM_ONLYONCE);
    shButton::setVirtualClickOn(true);
    shButton::setTimeoutOfLongClick(1000);
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

enum SysMode : uint8_t
{
  MODE_DEFAULT,         // основной режим работы
  MODE_CUSTOM_RUN,      // режим выборочного запуска полива
  MODE_MANUAL_WATERING, // режим ручного полива
  MODE_SETTING          // режим настроек каналов
};

// ==== канал датчик/помпа ===========================

// флаги состояния канала
enum ChannelState : uint8_t
{
  CNL_DONE,   // состояние покоя, работа сделана/поливать не нужно
  CNL_WORK,   // состояние работы, идет полив
  CNL_CHECK,  // состояние проверки после полива
  CNL_RESCAN, // состояние перепроверки после полива, если сразу после полива датчик показал сухую землю
  CNL_ERROR   // ошибка - после полива и перепроверки датчик показывает сухую землю
};

// флаги состояния датчика
enum SensorState : uint8_t
{
  SNS_NONE,     // состояние покоя
  SNS_METERING, // состояние измерения, датчик включен
  SNS_WATERING, // состояние полива, датчик отключен, работает помпа
  SNS_TESTING,  // только замер влажности без полива вне зависимости от результата
  SNS_RESCAN    // состояние повторного измерения, датчик включен
};

// флаги состояния настройки канала
enum SettingData : uint8_t
{
  FL_NONE,      // нормальное состояние
  FL_RUN_DATA,  // флаг изменения данных/включения помпы - кнопка удерживается более секунды
  FL_STOP_DATA, // флаг окончания изменения данных/остановки помпы - кнопка отпущена
  FL_SAVE_DATA, // флаг необходимости сохранения настройки и перехода на другой канал
  FL_NEXT,      // флаг перехода на другой канал без сохранения изменений
  FL_EXIT,      // флаг выхода из настроек по таймауту
  FL_CHECK_DATA // флаг проверки сохраненного значения; используется для проверки настройки помпы
};

// ==== класс канала полива ==========================

enum IndexOffset : uint8_t // смещение от стартового индекса в EEPROM для хранения настроек
/* общий размер настроек - 11 байт */
{
  IO_PUMP_DATA = 0,  // стартовый индекс, настройка времени работы помпы, uint32_t
  IO_H_TRESHOLD = 4, // порог влажности, uint16_t
  IO_CHANNEL_ON = 6, // включение канала, uint8_t
  IO_SENSOR_ON = 7,  // включение датчика влажности, uint8_t
  IO_MIN_DAY = 8,    // минимальное количество дней, uint8_t
  IO_MAX_DAY = 9,    // максимальное количество дней, uint8_t
  IO_TOTAL_DAY = 10  // общее количество пройденных шестичасовых циклов, uint8_t
};

class WateringChannel
{
private:
  byte pump_pin;                           // пин, на который подключена помпа
  byte p_sensor_pin;                       // пин подключения питания датчика влажности
  byte d_sensor_pin;                       // пин данных датчика влажности
  int16_t eeprom_start;                    // начальный индекс EEPROM для записи настроек
  ChannelState channel_state = CNL_DONE;   // состояние канала
  SensorState metering_flag = SNS_TESTING; // состояние датчика
  uint32_t p_timer = 0;                    // таймер работы помпы
  byte m_count = 0;                        // счетчик циклов измерений влажности
  uint16_t m_data = 0;                     // данные последнего замера влажности
  SettingData flag = FL_NONE;              // поле для передачи данных в задачи

  uint8_t read_eeprom_8(IndexOffset _index)
  {
    return (EEPROM.read(eeprom_start + _index));
  }
  uint16_t read_eeprom_16(IndexOffset _index)
  {
    uint16_t _data;
    EEPROM.get(eeprom_start + _index, _data);
    return (_data);
  }
  uint32_t read_eeprom_32(IndexOffset _index)
  {
    uint32_t _data;
    EEPROM.get(eeprom_start + _index, _data);
    return (_data);
  }
  void write_eeprom_8(IndexOffset _index, uint8_t _data)
  {
    EEPROM.update(eeprom_start + _index, _data);
  }
  void write_eeprom_16(IndexOffset _index, uint16_t _data)
  {
    EEPROM.put(eeprom_start + _index, _data);
  }
  void write_eeprom_32(IndexOffset _index, uint32_t _data)
  {
    EEPROM.put(eeprom_start + _index, _data);
  }

public:
  WateringChannel(byte pump, byte pwr_sensor, byte data_sensor, uint16_t ee_index)
  {
    pump_pin = pump;
    pinMode(pump_pin, OUTPUT);
    p_sensor_pin = pwr_sensor;
    pinMode(pwr_sensor, OUTPUT);
    d_sensor_pin = data_sensor;
    eeprom_start = ee_index;
    verifyEEPROMData();
  }

  // получение сохраненного значения настройки помпы
  uint32_t getPumpData() { return (read_eeprom_32(IO_PUMP_DATA)); }
  // сохранение настройки помпы
  void setPumpData(uint32_t _data) { write_eeprom_32(IO_PUMP_DATA, _data); }

  // получение сохраненного значения порога влажности почвы
  uint16_t getHumadityTreshold() { return (read_eeprom_16(IO_H_TRESHOLD)); }
  // сохранение настройки порога влажности почвы
  void setHumadityTreshold(uint16_t _data) { write_eeprom_16(IO_H_TRESHOLD, _data); }

  // получение данных по включению/отключению канала
  bool getChannelOnOffState() { return (bool(read_eeprom_8(IO_CHANNEL_ON))); }
  // сохранение данных по включению/отключению канала
  void setChannelOnOffState(bool _data) { write_eeprom_8(IO_CHANNEL_ON, _data); }

  // получение данных по включению/отключению датчика влажности
  bool getSensorOnOffState() { return (bool(read_eeprom_8(IO_SENSOR_ON))); }
  // сохранение данных по включению/отключению датчика влажности
  void setSensorOnOffState(bool _data) { write_eeprom_8(IO_SENSOR_ON, _data); }

  // получение данных по минимальному количеству дней
  uint8_t getMinDay() { return (read_eeprom_8(IO_MIN_DAY)); }
  // сохранение данных по минимальному количеству дние
  void setMinDay(uint8_t _data) { write_eeprom_8(IO_MIN_DAY, _data); }
  // проверка, прошло ли минимальное количество дней
  bool checkMinDay() { return (getSixHourCycles() >= getMinDay() * 4); }

  // получение данных по максимальному количеству дней
  uint8_t getMaxDay() { return (read_eeprom_8(IO_MAX_DAY)); }
  // сохранение данных по максимальному количеству дней
  void setMaxDay(uint8_t _data) { write_eeprom_8(IO_MAX_DAY, _data); }
  // проверка, прошло ли макчимальное количество дней
  bool checkMaxDay() { return (getSixHourCycles() >= getMaxDay() * 4); }

  // получение количества пройденных шестичасовых циклов
  uint8_t getSixHourCycles() { return (read_eeprom_8(IO_TOTAL_DAY)); }
  // увеличение количества пройденных шестичасовых циклов на единицу
  void incSixHourCycles() { write_eeprom_8(IO_TOTAL_DAY, (read_eeprom_8(IO_TOTAL_DAY) + 1)); }
  // обнуление количества пройденных шестичасовых циклов
  void clearSixHourCycles() { write_eeprom_8(IO_TOTAL_DAY, 0); }

  // верификация данных в EEPROM
  void verifyEEPROMData(bool toReset = false)
  {
    if (toReset)
    { // настройка использования канала и датчика влажности
      setChannelOnOffState(true);
      setSensorOnOffState(true);
    }
    if (toReset || (getHumadityTreshold() > 700) || (getHumadityTreshold() < 400))
    { // настройка уровня влажности
      setHumadityTreshold(DEFAULT_HUMIDITY_THRESHOLD);
    }
    if (toReset || (getPumpData() > MAX_PUMP_TIMER) || (getPumpData() <= 1000))
    { // настройка помпы
      setPumpData(DEFAULT_PUMP_TIMER);
    }
    if (toReset || (getMinDay() > 14) || (getMinDay() == 0))
    { // настройка минимального количества дней
      setMinDay(MIN_DAY_COUNT_DEF);
    }
    if (toReset || (getMaxDay() > 28) || (getMaxDay() == 0))
    { // настройка максимального количества дней
      setMaxDay(MAX_DAY_COUNT_DEF);
    }
    if (toReset || (getSixHourCycles() > 112))
    { // сохраненное количество пройденных шестичасовых циклов
      clearSixHourCycles();
    }
  }

  // получение состояни помпы
  bool getPumpState() { return (digitalRead(pump_pin)); }
  // включение/выключение помпы
  void setPumpState(bool _state) { digitalWrite(pump_pin, _state); }

  // получение состояния датчика влажности (включен/выключен)
  bool getSensorState() { return (bool(digitalRead(p_sensor_pin))); }
  // включение/выключение датчика влажности
  void setSensorState(bool _state) { digitalWrite(p_sensor_pin, _state); }
  // получение показаний датчика влажности.
  uint16_t readSensorData() { return (analogRead(d_sensor_pin)); }

  // получение состояния канала
  ChannelState getChannelState() { return (channel_state); }
  // проверкв состояния канала
  bool checkChannelState(ChannelState _state) { return (channel_state == _state); }
  // установка состояния канала
  void setChannelState(ChannelState _state) { channel_state = _state; }

  // получение рабочего состояния датчика влажности
  SensorState getMeteringState() { return (metering_flag); }
  // проверка рабочего состояния датчика влажности
  bool checkMeteringState(SensorState _state) { return (metering_flag == _state); }
  // установка рабочего состояния датчика влажности
  void setMeteringState(SensorState _state) { metering_flag = _state; }

  // получение флага данных канала
  SettingData getSettingData() { return (flag); }
  // установка флага данных канала
  void setSettingData(SettingData _data) { flag = _data; }
  // проверка флага данных
  bool checkSettingData(SettingData _data) { return (flag == _data); }

  // получение данных таймера помпы
  uint32_t getPumpTimer() { return (p_timer); }
  // установка данных таймера
  void setPumpTimer(uint32_t _tmr) { p_timer = _tmr; }
  // установка данных таймера помпы согласно заданных настроек
  void readPumpTimerData() { p_timer = getPumpData() / 100; }
  // уменьшение данных таймера помпы на единицу
  void decPumpTimer() { p_timer--; }

  // получение значения счетчика измерений
  uint8_t getMeasurementCyclesData() { return (m_count); }
  // установка значения счетчика измерений
  void setMeasurementCyclesData(uint8_t _data) { m_count = _data; }

  // получение последнего измрения влажности почвы
  uint16_t getLastMeasurementData() { return (m_data); }
  // установка последнего измерения влажности почвы
  void setLastMeasurementData(uint16_t _data) { m_data = _data; }
};

// ==== работа =======================================

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
void manualStart(SensorState flag, bool run = true);

// ==== другие задачи ================================

void mainTimer();
void runChanel();
void setLeds();
void runErrorBuzzer();
void runSetBuzzer();
void runWateringBuzzer(bool toUp = true);
void rescanStart();
void runSetChannels();
void returnToDefMode();
void manualWateringRun();

// ==== разное =======================================

// опрос кнопки
void checkButton();
void btnOneClick();
void btnDblClick(byte n);
void btnLongClick(byte &n);

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