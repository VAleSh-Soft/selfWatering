#include <FastLED.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <shTaskManager.h>
#include <shButton.h>
#include "selfWatering.h"

// ==== настройки ====================================
#define MAX_TIMER 14                  // максимальное количество суток, по истечении которого полив будет включен безусловно
#define MIN_TIMER 7                   // минимальное количество суток, до истечения которого полив не будет включен
#define METERING_COUNT 8              // количество замеров влажности для усреднения результата; желательно задавать значение, равное степени числа 2 (2, 4, 8, 16 и т.д.)
#define BUZZER_TIMEOUT 300            // интервал срабатывания пищалки в режиме "ошибка" в секундах
#define LIGHT_SENSOR_THRESHOLD 150    // минимальные показания датчика света (0-1023)
#define MAX_PUMP_TIMER 60000ul        // максимальное время работы помпы, мс
#define DEFAULT_PUMP_TIMER 10000ul    // значение времени работы помпы по умолчанию
#define DEFAULT_HUMIDITY_THRESHLD 600 // порог датчика влажности по умолчанию
// ===================================================

// адреса ячеек памяти для хранения данных по включенным датчика влажности
byte EEMEM hs_channel_2;
byte EEMEM hs_channel_1;
byte EEMEM hs_channel_0;
// адреса ячеек памяти для хранения данных по включенным каналам
byte EEMEM on_channel_2;
byte EEMEM on_channel_1;
byte EEMEM on_channel_0;
// адреса ячеек памяти для хранения настроек времени работы помпы по каналам
uint32_t EEMEM t_pump_2;
uint32_t EEMEM t_pump_1;
uint32_t EEMEM t_pump_0;
// адреса ячеек памяти для хранения настроек влажности по каналам
uint16_t EEMEM h_channel_2;
uint16_t EEMEM h_channel_1;
uint16_t EEMEM h_channel_0;

shTaskManager tasks(8); // создаем список задач

shHandle main_timer;         // главный таймер
shHandle run_channel;        // таймер работы каналов полива
shHandle leds_guard;         // таймер индикаторов
shHandle error_buzzer_on;    // таймер сигнала ошибки
shHandle set_buzzer_on;      // таймер сигнала настройки помп
shHandle rescan_start;       // таймер перепроверки влажности после полива
shHandle run_set_channels;   // таймер режима настройки каналов
shHandle return_to_def_mode; // таймер возврата в основной режим из режима настроек

byte curChannel = 0;         // текущий канал полива
byte curMode = MODE_DEFAULT; // текущий режим работы
byte curBtnCount = 0;        // счетчик одиночных кликов кнопки, нужен для входа в режим настроек
/*
3 клика – настройка времени работы помпы
4 клика – настройка порога влажности (четыре уровня, 400 – 500 – 600 – 700)
5 кликов – настройка минимального интервала (дней)
6 кликов – настройка максимального интервала (дней)
7 кликов – включение/отключение датчика влажности для канала
8 кликов – включение/отключение канала целиком
*/

shButton btn(BTN_PIN);

// массив каналов полива
ChannelState channels[3] = {
    (ChannelState){PUMP_1_PIN, HPOWER_1_SENSOR_PIN, HUMIDITY_1_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0},
    (ChannelState){PUMP_2_PIN, HPOWER_2_SENSOR_PIN, HUMIDITY_2_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0},
    (ChannelState){PUMP_3_PIN, HPOWER_3_SENSOR_PIN, HUMIDITY_3_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0}};
// массив адресов ячеек памяти для сохранения данных по включенным датчика влажности
byte hs_eemems[3] = {hs_channel_0, hs_channel_1, hs_channel_2};
// массив адресов ячеек памяти для сохранения данных по включенным каналам
byte c_eemems[3] = {on_channel_0, on_channel_1, on_channel_2};
// массив адресов ячеек памяти для сохранения уровней влажности по каналам
uint16_t h_eemems[3] = {h_channel_0, h_channel_1, h_channel_2};
// массив адресов ячеек памяти для сохранения настроек помпы по каналам
uint32_t p_eemems[3] = {t_pump_0, t_pump_1, t_pump_2};
// массив адресных светодиодов-индикаторов
CRGB leds[4];

// ===================================================

void setWateringMode(byte channel)
{
  channels[channel].metering_flag = SNS_WATERING;
  channels[channel].p_timer = millis() + 100; // +100 мс, т.к. полив начнется со следующей итерации таймера run_channel
}

void runChanel()
{
  // если таймер полива каналов еще не запущен, запустить его
  if (!tasks.getTaskState(run_channel))
  {
    tasks.startTask(run_channel);
  }
  // запускать полив только в основном режиме, иначе ждать выхода из режима настроек
  if (curMode == MODE_DEFAULT)
  { // если канал используется и еще не в рабочем режиме, перевести его в рабочий режим
    if ((channels[curChannel].channel_state == CNL_DONE) && eeprom_read_byte(&c_eemems[curChannel]))
    {
      // и попутно увеличить счетчик срабатывания, если это не ручной запуск; при первом запуске счетчик увеличить в любом случае
      if (channels[curChannel].metering_flag == SNS_NONE || channels[curChannel].min_max_count == 0)
      {
        channels[curChannel].min_max_count++;
      }
      channels[curChannel].channel_state = CNL_WORK;
      // если датчик еще в состоянии покоя, включить его и настроить канал на работу
      if (channels[curChannel].metering_flag == SNS_NONE)
      { // продолжить только если прошло минимальное количество суток и в светлое время
        if (channels[curChannel].min_max_count >= MIN_TIMER * 4 && analogRead(LIGHT_SENSOR_PIN) > LIGHT_SENSOR_THRESHOLD)
        {
          // если использование датчика влажности для канала отключено
          if (!eeprom_read_byte(&hs_eemems[curChannel]))
          {
            setWateringMode(curChannel);
          }
          // или если прошло максимальное количество дней, включить полив без замера влажности
          else if (channels[curChannel].min_max_count >= MAX_TIMER * 4)
          {
            setWateringMode(curChannel);
            // и записать новое значение порога влажности по значению последнего замера
            word t = channels[curChannel].m_data - 30;
            if (t < 400)
            {
              t = 400;
            }
            eeprom_update_word(&h_eemems[curChannel], t);
          }
          else
          { // иначе включить режим измерения влажности
            channels[curChannel].metering_flag = SNS_METERING;
            digitalWrite(channels[curChannel].p_sensor_pin, HIGH);
            channels[curChannel].m_count = 0;
          }
        }
      }
      else if ((channels[curChannel].metering_flag == SNS_WATERING) || !eeprom_read_byte(&hs_eemems[curChannel]))
      {
        setWateringMode(curChannel);
      }
    }
    else
    {
      if (!eeprom_read_byte(&c_eemems[curChannel]))
      {
        channels[curChannel].metering_flag = SNS_NONE;
      }
      // если канал включен и уже запущен, то действовать по флагу состояния датчика: замер влажности, полив или остановка таймера, если обработаны все каналы
      switch (channels[curChannel].metering_flag)
      {
      case SNS_METERING:
      case SNS_TESTING:
        cnlMetering(curChannel);
        break;
      case SNS_WATERING:
        cnlWatering(curChannel);
        break;
      default:
        if ((channels[curChannel].channel_state != CNL_ERROR) && (channels[curChannel].channel_state != CNL_RESCAN))
        {
          channels[curChannel].channel_state = CNL_DONE;
        }
        if (++curChannel >= 3)
        {
          tasks.stopTask(run_channel);
          curChannel = 0;
        }
        break;
      }
    }
  }
}

void cnlMetering(byte channel)
{
  if (!digitalRead(channels[channel].p_sensor_pin))
  { // на всякий случай
    digitalWrite(channels[channel].p_sensor_pin, HIGH);
  }
  else
  {
    // сделать восемь замеров - METERING_COUNT == 8
    static word p = 0;
    p += analogRead(channels[channel].d_sensor_pin);
    if (++channels[channel].m_count >= METERING_COUNT)
    { // потом отключить питание датчика и вычислить среднее значение
      digitalWrite(channels[channel].p_sensor_pin, LOW);
      channels[channel].m_count = 0;
      p /= METERING_COUNT;
      channels[channel].m_data = p;
#ifdef LOG_ON
      printLastMeteringData(channel);
#endif
      // определиться с дальнейшими действиями
      if (channels[channel].metering_flag != SNS_TESTING)
      {
        channels[channel].metering_flag = SNS_NONE;
        switch (channels[channel].channel_state)
        {
        // если после простоя сухо, включить режим полива
        case CNL_WORK:
          if (p >= eeprom_read_word(&h_eemems[channel]))
          {
            setWateringMode(channel);
          }
          break;
        // если после полива влажность недостаточна, включить перепроверку через минуту, чтобы дать возможность воде разойтись в почве и дойти до датчика; если недостаточна и после перепроверки, включить ошибку и сигнализатор; иначе считать задачу выполненной и обнулить счетчик пустых циклов
        case CNL_CHECK:
        case CNL_RESCAN:
          if (p > 300)
          {
            if (channels[channel].channel_state == CNL_RESCAN)
            {
              channels[channel].channel_state = CNL_ERROR;
              if (!tasks.getTaskState(error_buzzer_on))
              {
                runErrorBuzzer();
              }
            }
            else
            {
              channels[channel].channel_state = CNL_RESCAN;
              tasks.startTask(rescan_start);
            }
          }
          else
          {
            channels[channel].min_max_count = 0;
            if (channels[channel].channel_state == CNL_RESCAN)
            {
              channels[channel].channel_state = CNL_CHECK;
            }
          }
          break;
        }
      }
      else
        channels[channel].metering_flag = SNS_NONE;
      p = 0;
    }
  }
}

void cnlWatering(byte channel)
{
  if (digitalRead(WATER_LEVEL_SENSOR_PIN))
  { // если вода есть, включить помпу и поливать, пока не истечет заданное время
    digitalWrite(channels[channel].pump_pin, HIGH);
    if (millis() - channels[channel].p_timer >= eeprom_read_dword(&p_eemems[channel]))
    { // если время истекло, остановить помпу
      digitalWrite(channels[channel].pump_pin, LOW);
      // режим измерения включить только если датчик влажности для этого канала используется
      if (eeprom_read_byte(&hs_eemems[curChannel]))
      {
        channels[channel].metering_flag = SNS_METERING;
        channels[channel].channel_state = CNL_CHECK;
      }
      else
        channels[channel].metering_flag = SNS_NONE;
    }
  }
  else
  { // если воды нет, остановить помпу
    digitalWrite(channels[channel].pump_pin, LOW);
    if (!tasks.getTaskState(error_buzzer_on))
    {
      runErrorBuzzer();
    }
  }
}

byte getErrors()
{
  byte result = 1;
  for (byte i = 0; i < 4; i++)
  {
    bool f = (i == 0) ? !digitalRead(WATER_LEVEL_SENSOR_PIN) : channels[i - 1].channel_state == CNL_ERROR;
    (f) ? (result) |= (1UL << (i)) : (result) &= ~(1UL << (i));
  }
  return (result);
}

void manualStart(byte flag, bool run)
{
  tasks.stopTask(error_buzzer_on);
  tasks.startTask(leds_guard);
  for (byte i = 0; i < 3; i++)
  {
    if (channels[i].channel_state != CNL_RESCAN)
    {
      channels[i].channel_state = CNL_DONE;
    }
    channels[i].metering_flag = flag;
    if (!run)
    { // на случай если в момент остановки идет полив
      tasks.stopTask(run_channel);
      digitalWrite(channels[i].pump_pin, LOW);
      digitalWrite(channels[i].p_sensor_pin, LOW);
    }
  }
  if (run)
  {
    curChannel = 0;
    runChanel();
    // tasks.startTask(run_channel);
  }
}

// ===================================================
void mainTimer()
{
  manualStart(SNS_NONE);
}

void setLedsDefault(byte i)
{
  switch (channels[i].channel_state)
  {
  case CNL_ERROR:
    leds[i + 1] = CRGB::Red;
    break;
  case CNL_RESCAN:
    leds[i + 1] = digitalRead(channels[i].p_sensor_pin) ? CRGB::Orange : CRGB::Red;
    break;
  case CNL_WORK:
  case CNL_CHECK:
    leds[i + 1] = digitalRead(channels[i].pump_pin) ? CRGB::Green : CRGB::Orange;
    break;
  default:
    leds[i + 1] = CRGB::Black;
    break;
  }
}

// подсветка индикаторов каналов при настройке времени работы помп
void setLeds_3(byte i)
{
  if (i == curChannel)
  {
    // текущий канал подсвечивать синим или зеленым (если помпа включена); остальные каналы не подсвечивать
    leds[i + 1] = digitalRead(channels[i].pump_pin) ? CRGB::Green : CRGB::Blue;
  }
  else
  {
    leds[i + 1] = CRGB::Black;
  }
}

// подсветка индикаторов каналов при настройке включения/отключения датчиков влажности и каналов в целом
void setLeds_7(byte i)
{
  static byte n = 8;
  static byte f = curChannel;
  // если канал изменился, его индикатор начинает с "зажмуренного" состояния ))
  if (f != curChannel)
  {
    n = 8;
    f = curChannel;
  }

  // каналы подсвечивать синим (настройка датчиков), зеленым (настройка каналов) или красным (если датчик/канал отключен);
  if (i == curChannel)
  {
    if (channels[i].m_count)
    {
      leds[i + 1] = (curBtnCount == 8) ? CRGB::Green : CRGB::Blue;
    }
    else
    {
      leds[i + 1] = CRGB::Red;
    }
    // текущий канал подмигивает с частотой 1 сек
    if (n >= 9)
    {
      leds[i + 1] = CRGB::Black;
    }
  }
  else
  {
    switch (curBtnCount)
    {
    case 7:
      leds[i + 1] = (eeprom_read_byte(&hs_eemems[i])) ? CRGB::Blue : CRGB::Red;
      break;
    case 8:
      leds[i + 1] = (eeprom_read_byte(&c_eemems[i])) ? CRGB::Green : CRGB::Red;
      break;
    }
  }
  if (++n > 9)
  {
    n = 0;
  }
}

void setLeds()
{
  static byte n = 0;
  // индикатор датчика уровня воды подмигивает каждые две секунды зеленым, если вода есть и красным, если воды нет
  leds[0] = (digitalRead(WATER_LEVEL_SENSOR_PIN)) ? CRGB::Green : CRGB::Red;
  if (n >= 19)
  {
    leds[0] = CRGB::Black;
  }
  if (++n > 19)
  {
    n = 0;
  }

  // индикаторы каналов
  for (byte i = 0; i < 3; i++)
  {
    switch (curMode)
    {
    case MODE_DEFAULT:
      setLedsDefault(i);
      break;
    case MODE_SETTING:
      switch (curBtnCount)
      {
      case 3:
        setLeds_3(i);
        break;
      case 7:
      case 8:
        setLeds_7(i);
        break;
      }
      break;
    }
  }
  FastLED.show();
}

void runErrorBuzzer()
{
  static byte n = 0;
  // "мелодия" пищалки: первая строка - частота, вторая строка - длительность
  static const PROGMEM uint32_t pick[2][12] = {
      {2000, 0, 2000, 0, 2000, 0, 2000, 0, 2000, 0, 2000, 0},
      {50, 100, 50, 500, 50, 100, 50, 500, 50, 100, 50, BUZZER_TIMEOUT * 1000ul}};
  static CRGB _leds[4];

  // если таймер пищалки еще не запущен, запустить его, иначе проверить на наличие ошибок и, если ошибок уже нет, остановить таймер
  if (!tasks.getTaskState(error_buzzer_on))
  {
    n = 0;
    tasks.startTask(error_buzzer_on);
  }
  else if (!getErrors())
  {
    tasks.stopTask(error_buzzer_on);
    n = 0;
    // возвращаем управление индикаторами
    tasks.startTask(leds_guard);
  }
  // перехватываем управление индикаторами, чтобы в момент писка нужные индикаторы проблескивали белым цветом
  tasks.stopTask(leds_guard);

  if (pgm_read_dword(&pick[0][n]) > 0)
  {
    tone(BUZZER_PIN, pgm_read_dword(&pick[0][n]), pgm_read_dword(&pick[1][n]));
    // в момент включения звука индикаторы каналов с ошибками включить белым
    for (byte i = 0; i < 4; i++)
    {
      _leds[i] = leds[i];
      if (i == 0)
      { // индикатор уровня воды
        if (!digitalRead(WATER_LEVEL_SENSOR_PIN))
        {
          leds[i] = CRGB::White;
        }
      }
      else
      { // индикаторы каналов
        if (channels[i - 1].channel_state == CNL_ERROR)
        {
          leds[i] = CRGB::White;
        }
      }
    }
  }
  else
  { // иначе возвращать цвета индикаторов к цветам по умолчанию
    for (byte i = 0; i < 4; i++)
    {
      leds[i] = _leds[i];
    }
  }
  FastLED.show();
  tasks.setTaskInterval(error_buzzer_on, pgm_read_dword(&pick[1][n]), true);

  if (++n >= 12)
  {
    n = 0;
    // возвращаем управление индикаторами
    tasks.startTask(leds_guard);
  }
}

void runSetBuzzer()
{
  static byte n = 0;
  // "мелодия" пищалки: первая строка - частота, вторая строка - длительность
  static const PROGMEM uint32_t pick[2][16] = {
      {2000, 0, 2000, 0, 2000, 0, 2000, 0, 2000, 0, 2000, 0, 2000, 0, 2000, 0},
      {50, 100, 50, 100, 50, 100, 50, 100, 50, 100, 50, 100, 50, 100, 50, 100}};
  // если таймер пищалки еще не запущен, запустить его
  if (!tasks.getTaskState(set_buzzer_on))
  {
    n = 0;
    tasks.startTask(set_buzzer_on);
    tone(BUZZER_PIN, 2000, 800UL);
  }
  else
  { // если только что включен режим настройки, то дать серию коротких пиков в зависимости от режима
    if ((curBtnCount >= 3) && (curBtnCount <= 8))
    {
      tone(BUZZER_PIN, pgm_read_dword(&pick[0][n]), pgm_read_dword(&pick[1][n]));
      tasks.setTaskInterval(set_buzzer_on, pgm_read_dword(&pick[1][n]), true);

      if (++n >= curBtnCount * 2)
      {
        n = 0;
        tasks.stopTask(set_buzzer_on);
        tasks.setTaskInterval(set_buzzer_on, 1000, false);
      }
    }
    else
    // иначе завершить работу таймера
    {
      {
        tasks.stopTask(set_buzzer_on);
        tasks.setTaskInterval(set_buzzer_on, 1000, false);
      }
    }
  }
}

void rescanStart()
{
  if (!tasks.getTaskState(run_channel))
  {
    manualStart(SNS_METERING);
    tasks.stopTask(rescan_start);
  }
}

void isBtnClosed_3(uint32_t _tmr, uint32_t &_timer, uint32_t &_result)
{
  if (!digitalRead(channels[curChannel].pump_pin))
  {
    tone(BUZZER_PIN, 2500, 100);
    digitalWrite(channels[curChannel].pump_pin, HIGH);
    _timer = _tmr;
  }
  else
  {
    if (_tmr - _timer >= MAX_PUMP_TIMER)
    {
      digitalWrite(channels[curChannel].pump_pin, LOW);
      _result = MAX_PUMP_TIMER;
      channels[curChannel].flag = FL_STOP_DATA;
    }
  }
}

void runSetChannels()
{
  static uint32_t pumpTimer = 0;
  static uint32_t result = 0;
  uint32_t tmr = millis();

  if (!tasks.getTaskState(run_set_channels))
  {
    tasks.startTask(run_set_channels);
    tasks.startTask(return_to_def_mode);
    curChannel = 0;
    channels[curChannel].flag = FL_NONE;
    // поле m_count используется только при измерении влажности, поэтому его можно с чистой совестью использовать в процессе настроек
    switch (curBtnCount)
    {
    case 7:
      channels[curChannel].m_count = eeprom_read_byte(&hs_eemems[curChannel]);
      break;
    case 8:
      channels[curChannel].m_count = eeprom_read_byte(&c_eemems[curChannel]);
      break;
    }
    curMode = MODE_SETTING;
    runSetBuzzer();
  }
  // ждать, пока отработает пищалка
  if (tasks.getTaskState(set_buzzer_on))
  {
    return;
  }
  // управление данными
  if (channels[curChannel].flag == FL_RUN_DATA)
  {
    if (btn.isButtonClosed())
    {
      tasks.restartTask(return_to_def_mode);
      switch (curBtnCount)
      {
      case 3:
        isBtnClosed_3(tmr, pumpTimer, result);
        break;
      case 7:
      case 8:
        tone(BUZZER_PIN, 2500, 100);
        channels[curChannel].m_count = !channels[curChannel].m_count;
        channels[curChannel].flag = FL_STOP_DATA;
        break;
      }
    }
    else
    {
      switch (curBtnCount)
      {
      case 3:
        result = tmr - pumpTimer;
        digitalWrite(channels[curChannel].pump_pin, LOW);
        channels[curChannel].flag = FL_STOP_DATA;
        break;
      }
    }
  }
  // управление сохранением изменений, переходом на другой канал или выходом из настроек
  if ((channels[curChannel].flag == FL_SAVE_DATA) || (channels[curChannel].flag == FL_NEXT) || (channels[curChannel].flag == FL_EXIT))
  {
    if ((channels[curChannel].flag == FL_SAVE_DATA))
    {
      switch (curBtnCount)
      {
      case 3:
        eeprom_update_dword(&p_eemems[curChannel], result);
        break;
      case 7:
        eeprom_update_byte(&hs_eemems[curChannel], channels[curChannel].m_count);
        break;
      case 8:
        eeprom_update_byte(&c_eemems[curChannel], channels[curChannel].m_count);
        break;
      }
    }
    if (channels[curChannel].flag == FL_EXIT)
    {
      curChannel = 3;
    }
    else
    {
      curChannel++;
    }
    if (curChannel < 3)
    {
      channels[curChannel].flag = FL_NONE;
      switch (curBtnCount)
      {
      case 7:
        channels[curChannel].m_count = eeprom_read_byte(&hs_eemems[curChannel]);
        break;
      case 8:
        channels[curChannel].m_count = eeprom_read_byte(&c_eemems[curChannel]);
        break;
      }
      tone(BUZZER_PIN, 2000, 100);
    }
  }
  // выход из настроек
  if (curChannel >= 3)
  {
    tasks.stopTask(run_set_channels);
    tasks.stopTask(return_to_def_mode);
    curMode = MODE_DEFAULT;
    curChannel = 0;
    curBtnCount = 0;
    runSetBuzzer();
  }
}

void returnToDefMode()
{
  if (tasks.getTaskState(run_set_channels))
  {
    channels[curChannel].flag = FL_EXIT;
  }
  tasks.stopTask(return_to_def_mode);
}
// ===================================================

void checkButton()
{
  static uint32_t btn_timer = 0;
  static byte n = 0;
  switch (btn.getButtonState())
  {
  case BTN_DOWN:
    if (curMode == MODE_DEFAULT && !tasks.getTaskState(run_channel))
    {
      n++; // счетчик кликов
      btn_timer = millis();
      if (n > 1)
      {
        tone(BUZZER_PIN, 2000, 100);
      }
    }
    break;
    // при одиночном клике
  case BTN_ONECLICK:
  {
    // в основном режиме остановить пищалку и сбросить ошибки по каналам
    switch (curMode)
    {
    case MODE_DEFAULT:
      if ((tasks.getTaskState(error_buzzer_on) && !tasks.getTaskState(run_channel)) || tasks.getTaskState(rescan_start))
      {
        manualStart(SNS_NONE, false);
        // если запущен таймер рескана датчиков влажности, остановить его
        if (tasks.getTaskState(rescan_start))
        {
          tasks.stopTask(rescan_start);
          for (byte i = 0; i < 3; i++)
          {
            if (channels[i].channel_state == CNL_RESCAN)
            {
              channels[i].channel_state = CNL_DONE;
            }
          }
        }
      }
      break;
    // в режиме настройки дать команду на переключение канала с возможным сохранением данных
    case MODE_SETTING:
      if (!tasks.getTaskState(set_buzzer_on))
      {
        channels[curChannel].flag = (channels[curChannel].flag == FL_STOP_DATA) ? FL_SAVE_DATA : FL_NEXT;
      }
      tasks.restartTask(return_to_def_mode);
      break;
    }
    break;
  }
  // при двойном клике
  case BTN_DBLCLICK:
    switch (curMode)
    {
      // в основном режиме запустить полив с замером влажности; если случайно сделан двойной клик при входе в настройки - ничего не делать
    case MODE_DEFAULT:
      if (!tasks.getTaskState(run_channel) && n <= 1)
      {
        manualStart(SNS_METERING);
      }
      break;
    // в режиме настройки просто перезапустить таймер автовыхода
    case MODE_SETTING:
      tasks.restartTask(return_to_def_mode);
      break;
    }
    break;
  // при длинном клике
  case BTN_LONGCLICK:
    switch (curMode)
    {
    // в основном режиме запустить полив без замера влажности или остановить полив, если он в текущий момент идет.
    case MODE_DEFAULT:
      if (tasks.getTaskState(run_channel))
      {
        manualStart(SNS_NONE, false);
      }
      else
      {
        manualStart(SNS_WATERING);
      }
      break;
    // установить флаг изменения данных
    case MODE_SETTING:
      channels[curChannel].flag = FL_RUN_DATA;
      tasks.restartTask(return_to_def_mode);
      break;
    }
    break;
  }
  // проверить, сколько одиночных кликов кнопки сделано
  if (millis() - btn_timer > 1000)
  {
    if (((n >= 3) && (n <= 8)) && (curBtnCount == 0) && !tasks.getTaskState(run_channel))
    {
      curBtnCount = n;
    }
    n = 0;
  }
}

// ===================================================

void setup()
{
#ifdef LOG_ON
  Serial.begin(9600);
#endif

  FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, 4);
  FastLED.setBrightness(5);

  // ==== настройка кнопки ===========================
  btn.setLongClickMode(LCM_ONLYONCE);
  btn.setVirtualClickOn(true);
  btn.setTimeout(1000);

  // ==== настройка пинов ============================
  pinMode(WATER_LEVEL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(HPOWER_1_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_1_PIN, OUTPUT);
  pinMode(HPOWER_2_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_2_PIN, OUTPUT);
  pinMode(HPOWER_3_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_3_PIN, OUTPUT);

  // ==== настройка задач ============================
  main_timer = tasks.addTask(21600000, mainTimer);                   // главный таймер - интервал 6 часов
  run_channel = tasks.addTask(100, runChanel);                       // таймер работы с каналами
  leds_guard = tasks.addTask(100, setLeds);                          // управление индикаторами
  error_buzzer_on = tasks.addTask(300000, runErrorBuzzer, false);    // таймер сигнала ошибки
  rescan_start = tasks.addTask(60000, rescanStart, false);           // таймер перепроверки влажности
  set_buzzer_on = tasks.addTask(1000, runSetBuzzer, false);          // таймер пищалки режима настройи
  run_set_channels = tasks.addTask(1000, runSetChannels, false);     // таймер режима настройки 
  return_to_def_mode = tasks.addTask(60000, returnToDefMode, false); // таймер автовыхода из настроек

  // ==== проверка каналов на использование датчика влажности; если для какого-то канала датчик отключен, отключить замер влажности при старте программы; иначе сразу будет включен полив;
  for (byte i = 0; i < 3; i++)
  {
    if (!eeprom_read_byte(&hs_eemems[i]))
    {
      channels[i].metering_flag = SNS_NONE;
    }
  }

  // ==== верификация настроек по каналам ============
  for (byte i = 0; i < 3; i++)
  {
    if ((eeprom_read_word(&h_eemems[i]) > 700) || (eeprom_read_word(&h_eemems[i]) < 400))
    {
      eeprom_update_word(&h_eemems[i], DEFAULT_HUMIDITY_THRESHLD);
    }
    if (eeprom_read_dword(&p_eemems[i]) > MAX_PUMP_TIMER || eeprom_read_dword(&p_eemems[i]) <= 1000)
    {
      eeprom_update_dword(&p_eemems[i], DEFAULT_PUMP_TIMER);
    }
  }
}

void loop()
{
  tasks.tick();
  checkButton();
  if ((curBtnCount >= 3) && (curBtnCount <= 8))
  {
    runSetChannels();
  }
#ifdef LOG_ON
  checkSerial();
#endif
}

// ===================================================

#ifdef LOG_ON

void checkSerial()
{
  if (Serial.available())
  {
    int inByte = Serial.read();
    Serial.println(inByte);
    Serial.println();
    switch (inByte)
    {
    case 49: // '1' - вывод состояния таймеров, датчиков и каналов
      // состояние таймеров
      Serial.println("=== Timers state ===");
      Serial.println();
      Serial.print("main_timer: ");
      Serial.println(tasks.getTaskState(main_timer));
      Serial.print("run_channel: ");
      Serial.println(tasks.getTaskState(run_channel));
      Serial.print("leds_guard: ");
      Serial.println(tasks.getTaskState(leds_guard));
      Serial.print("error_buzzer_on: ");
      Serial.println(tasks.getTaskState(error_buzzer_on));
      Serial.print("rescan_start: ");
      Serial.println(tasks.getTaskState(rescan_start));
      Serial.print("set_buzzer_on: ");
      Serial.println(tasks.getTaskState(set_buzzer_on));
      Serial.print("run_set_channels: ");
      Serial.println(tasks.getTaskState(run_set_channels));
      Serial.print("return_to_def_mode: ");
      Serial.println(tasks.getTaskState(return_to_def_mode));
      Serial.println();
      Serial.println("=== Sensors state ===");
      Serial.println();
      // показания датчика света
      Serial.print("Light sensor data: ");
      Serial.println(analogRead(LIGHT_SENSOR_PIN));
      // наличие воды по датчику уровня
      Serial.print("Water sensor data: ");
      digitalRead(WATER_LEVEL_SENSOR_PIN) ? Serial.println("yes") : Serial.println("no");
      Serial.println();
      Serial.println("=== Channels state ===");
      Serial.println();
      for (byte i = 0; i < 3; i++)
      {
        printChannelStatus(i);
      }
      break;
    case 50: // '2' - получение текущей влажности по каналам
      if (!tasks.getTaskState(run_channel))
      {
        manualStart(SNS_TESTING);
      }
      else
      {
        Serial.println("Denied, watering or metering is in progress");
      }
      Serial.println();
      break;
    default: // вывод данных по последнему замеру влажности по каналам
      for (byte i = 0; i < 3; i++)
      {
        printLastMeteringData(i);
      }
      break;
    }
  }
}

void printLastMeteringData(byte cnl)
{
  Serial.print("Metering data, channel ");
  Serial.print(cnl);
  Serial.print(": ");
  Serial.println(channels[cnl].m_data);
}

void printChannelStatus(byte cnl)
{
  Serial.print("Channel "); // текущий статус канала
  Serial.print(cnl + 1);
  if (!eeprom_read_byte(&c_eemems[cnl]))
  {
    Serial.println(" not used");
  }
  else
  {
    Serial.println(" used");
    Serial.print("Channel state: ");
    switch (channels[cnl].channel_state)
    {
    case CNL_DONE:
      Serial.println("CNL_DONE");
      break;
    case CNL_WORK:
      Serial.println("CNL_WORK");
      break;
    case CNL_CHECK:
      Serial.println("CNL_CHECK");
      break;
    case CNL_RESCAN:
      Serial.println("CNL_RESCAN");
      break;
    case CNL_ERROR:
      Serial.println("CNL_ERROR");
      break;
    default:
      Serial.println("unknown");
      break;
    }
    Serial.print("Pump: "); // текущий статус помпы
    digitalRead(channels[cnl].pump_pin) ? Serial.println("power ON") : Serial.println("power OFF");
    Serial.print("Pump timeout, msec: "); // время работы помпы для канала
    Serial.println(eeprom_read_dword(&p_eemems[cnl]));
    Serial.print("Humidity sensor:"); // текущий статус датчика влажности
    if (!eeprom_read_byte(&hs_eemems[cnl]))
    {
      Serial.println(" not used");
    }
    else
    {
      Serial.println(" used");
      Serial.print("Sensor state: "); // текущий статус датчика влажности
      digitalRead(channels[cnl].p_sensor_pin) ? Serial.println("power ON") : Serial.println("power OFF");
      Serial.print("Metering flag: ");
      switch (channels[cnl].metering_flag)
      {
      case SNS_NONE:
        Serial.println("SNS_NONE");
        break;
      case SNS_METERING:
        Serial.println("SNS_METERING");
        break;
      case SNS_TESTING:
        Serial.println("SNS_TESTING");
        break;
      case SNS_WATERING:
        Serial.println("SNS_WATERING");
        break;
      default:
        Serial.println("unknown");
        break;
      }
      Serial.print("Humidity threshold: "); // порог влажности для канала
      Serial.println(eeprom_read_word(&h_eemems[cnl]));
      Serial.print("Humidity last data: "); // последний замер влажности для канала
      Serial.println(channels[cnl].m_data);
    }
    Serial.print("Cycles count: "); // количество прошедших шестичасовых циклов
    Serial.println(channels[cnl].min_max_count);
    Serial.print("Next point in "); // осталось времени до следующего цикла, час/мин
    uint32_t x = tasks.getNextTaskPoint(main_timer);
    Serial.print(x / 3600000);
    Serial.print(" hour, ");
    Serial.print(x % 3600000 / 60000);
    Serial.println(" min");
  }
  Serial.println();
}

#endif