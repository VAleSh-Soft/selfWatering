#include <FastLED.h>
#include <avr/eeprom.h>
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

// адреса ячеек памяти для хранения настроек времени работы помпы по каналам
uint32_t EEMEM e_pump_2;
uint32_t EEMEM e_pump_1;
uint32_t EEMEM e_pump_0;
// адреса ячеек памяти для хранения настроек влажности по каналам
uint16_t EEMEM e_chanel_2;
uint16_t EEMEM e_chanel_1;
uint16_t EEMEM e_chanel_0;

shTaskManager tasks(8); // создаем список задач

shHandle main_timer;         // главный таймер
shHandle run_channel;        // таймер работы каналов полива
shHandle leds_guard;         // таймер индикаторов
shHandle error_buzzer_on;    // таймер сигнала ошибки
shHandle set_buzzer_on;      // таймер сигнала настройки помп
shHandle rescan_start;       // таймер перепроверки влажности после полива
shHandle run_set_pumps;      // таймер режима настройки помп
shHandle return_to_def_mode; // таймер возврата в основной режим из режима настроек

byte curChannel = 0;         // текущий канал полива
byte curMode = MODE_DEFAULT; // текущий режим работы
byte curBtnCount = 0;        // счетчик одиночных кликов кнопки, нужен для входа в режим настроек

shButton btn(BTN_PIN);

// // каналы полива
// ChannelState channel_1 = (ChannelState){PUMP_1_PIN, HPOWER_1_SENSOR_PIN, HUMIDITY_1_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0};
// ChannelState channel_2 = (ChannelState){PUMP_2_PIN, HPOWER_2_SENSOR_PIN, HUMIDITY_2_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0};
// ChannelState channel_3 = (ChannelState){PUMP_3_PIN, HPOWER_3_SENSOR_PIN, HUMIDITY_3_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0};

// массив каналов полива
// ChannelState channels[3] = {channel_1, channel_2, channel_3};
ChannelState channels[3] = {
    (ChannelState){PUMP_1_PIN, HPOWER_1_SENSOR_PIN, HUMIDITY_1_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0},
    (ChannelState){PUMP_2_PIN, HPOWER_2_SENSOR_PIN, HUMIDITY_2_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0},
    (ChannelState){PUMP_3_PIN, HPOWER_3_SENSOR_PIN, HUMIDITY_3_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0, 0, 0}};
// массив адресов ячеек памяти для сохранения уровней влажности по каналам
uint16_t h_emems[3] = {e_chanel_0, e_chanel_1, e_chanel_2};
// массив адресов ячеек памяти для сохранения настроек помпы по каналам
uint32_t p_emems[3] = {e_pump_0, e_pump_1, e_pump_2};
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
  { // если канал еще не в рабочем режиме, перевести его в рабочий режим
    if (channels[curChannel].channel_state == CNL_DONE)
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
          // если прошло максимальное количество дней, включить полив без замера влажности
          if (channels[curChannel].min_max_count >= MAX_TIMER * 4)
          {
            setWateringMode(curChannel);
            // и записать новое значение порога влажности по значению последнего замера
            word t = channels[curChannel].m_data - 30;
            if (t < 400)
            {
              t = 400;
            }
            eeprom_update_word(&h_emems[curChannel], t);
          }
          else
          { // иначе включить режим измерения влажности
            channels[curChannel].metering_flag = SNS_METERING;
            digitalWrite(channels[curChannel].p_sensor_pin, HIGH);
            channels[curChannel].m_count = 0;
          }
        }
      }
      else if (channels[curChannel].metering_flag == SNS_WATERING)
      {
        setWateringMode(curChannel);
      }
    }
    else
    { // если канал уже запущен, то действовать по флагу состояния датчика: замер влажности, полив или остановка таймера, если обработаны все каналы
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
        if (channels[curChannel].channel_state != CNL_ERROR && channels[curChannel].channel_state != CNL_RESCAN)
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
          if (p >= eeprom_read_word(&h_emems[channel]))
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
    if (millis() - channels[channel].p_timer >= eeprom_read_dword(&p_emems[channel]))
    { // если время истекло, остановить помпу и включить режим измерения
      digitalWrite(channels[channel].pump_pin, LOW);
      channels[channel].metering_flag = SNS_METERING;
      channels[channel].channel_state = CNL_CHECK;
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
      break;
    case MODE_SET_PUMPS:
      if (i == curChannel)
      {
        // при настройке времени работы помпы текущий канал подсвечивать синим или зеленым, если помпа включена; остальные каналы не подсвечивать
        leds[i + 1] = digitalRead(channels[i].pump_pin) ? CRGB::Green : CRGB::Blue;
      }
      else
      {
        leds[i + 1] = CRGB::Black;
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
  uint32_t pick[2][12] = {
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

  if (pick[0][n] > 0)
  {
    tone(BUZZER_PIN, pick[0][n], pick[1][n]);
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
  tasks.setTaskInterval(error_buzzer_on, pick[1][n], true);

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
  uint32_t pick[2][6] = {
      {2000, 0, 2000, 0, 2000, 0},
      {50, 100, 50, 100, 50, 100}};
  // если таймер пищалки еще не запущен, запустить его
  if (!tasks.getTaskState(set_buzzer_on))
  {
    n = 0;
    tasks.startTask(set_buzzer_on);
    tone(BUZZER_PIN, 2000, 800UL);
  }
  else
  { // если только что включен режим настройки помпы, то дать три коротких пика
    if (curBtnCount == 3)
    {
      tone(BUZZER_PIN, pick[0][n], pick[1][n]);
      tasks.setTaskInterval(set_buzzer_on, pick[1][n], true);

      if (++n >= 6)
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

void runSetPumps()
{
  static uint32_t pumpTimer = 0;
  static uint32_t result = 0;
  uint32_t tmr = millis();

  if (!tasks.getTaskState(run_set_pumps))
  {
    tasks.startTask(run_set_pumps);
    tasks.startTask(return_to_def_mode);
    curChannel = 0;
    channels[curChannel].flag = FL_NONE;
    curMode = MODE_SET_PUMPS;
    runSetBuzzer();
  }
  // ждать, пока отработает пищалка
  if (tasks.getTaskState(set_buzzer_on))
  {
    return;
  }
  // управление помпой
  if (channels[curChannel].flag == FL_RUN_PUMP)
  {
    if (btn.isButtonClosed())
    {
      if (!digitalRead(channels[curChannel].pump_pin))
      {
        tone(BUZZER_PIN, 2500, 100);
        digitalWrite(channels[curChannel].pump_pin, HIGH);
        pumpTimer = tmr;
      }
      else
      {
        tasks.restartTask(return_to_def_mode);
        if (tmr - pumpTimer >= MAX_PUMP_TIMER)
        {
          digitalWrite(channels[curChannel].pump_pin, LOW);
          result = MAX_PUMP_TIMER;
          channels[curChannel].flag = FL_STOP_PUMP;
        }
      }
    }
    else
    {
      result = tmr - pumpTimer;
      digitalWrite(channels[curChannel].pump_pin, LOW);
      channels[curChannel].flag = FL_STOP_PUMP;
    }
  }
  // управление переходом на другой канал или выходом из настроек
  if ((channels[curChannel].flag == FL_SAVE_DATA) || (channels[curChannel].flag == FL_NEXT) || (channels[curChannel].flag == FL_EXIT))
  {
    if ((channels[curChannel].flag == FL_SAVE_DATA))
    {
      eeprom_update_dword(&p_emems[curChannel], result);
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
      tone(BUZZER_PIN, 2000, 100);
    }
  }
  // выход из настроек
  if (curChannel >= 3)
  {
    tasks.stopTask(run_set_pumps);
    tasks.stopTask(return_to_def_mode);
    curMode = MODE_DEFAULT;
    curChannel = 0;
    curBtnCount = 0;
    runSetBuzzer();
  }
}

void returnToDefMode()
{
  if (tasks.getTaskState(run_set_pumps))
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
    if (curMode == MODE_DEFAULT)
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
    // в режиме настройки помпы дать команду на переключение канала с возможным сохранением данных
    case MODE_SET_PUMPS:
      channels[curChannel].flag = (channels[curChannel].flag == FL_STOP_PUMP) ? FL_SAVE_DATA : FL_NEXT;
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
    // в режиме настройки помпы просто перезапустить таймер автовыхода
    case MODE_SET_PUMPS:
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
    // установить флаг включения помпы
    case MODE_SET_PUMPS:
      channels[curChannel].flag = FL_RUN_PUMP;
      tasks.restartTask(return_to_def_mode);
      break;
    }
    break;
  }
  // проверить, сколько одиночных кликов кнопки сделано
  if (millis() - btn_timer > 1000)
  {
    if ((n == 3) && (curBtnCount == 0))
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
  set_buzzer_on = tasks.addTask(1000, runSetBuzzer, false);          // таймер пищалки настройи помп
  run_set_pumps = tasks.addTask(1000, runSetPumps, false);           // таймер настройи помп
  return_to_def_mode = tasks.addTask(60000, returnToDefMode, false); // таймер автовыхода из настроек

  // ==== верификация настроек по каналам ============
  for (byte i = 0; i < 3; i++)
  {
    if ((eeprom_read_word(&h_emems[i]) > 1023) || (eeprom_read_word(&h_emems[i]) <= 150))
    {
      eeprom_update_word(&h_emems[i], DEFAULT_HUMIDITY_THRESHLD);
    }
    if (eeprom_read_dword(&p_emems[i]) > MAX_PUMP_TIMER || eeprom_read_dword(&p_emems[i]) <= 1000)
    {
      eeprom_update_dword(&p_emems[i], DEFAULT_PUMP_TIMER);
    }
  }
}

void loop()
{
  tasks.tick();
  checkButton();
  if (curBtnCount == 3)
  {
    runSetPumps();
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
      Serial.print("run_set_pumps: ");
      Serial.println(tasks.getTaskState(run_set_pumps));
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
  Serial.print(cnl);
  Serial.print(" state: ");
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
  Serial.println(eeprom_read_dword(&p_emems[cnl]));
  Serial.print("Humidity sensor: "); // текущий статус датчика влажности
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
  Serial.println(eeprom_read_word(&h_emems[cnl]));
  Serial.print("Humidity last data: "); // последний замер влажности для канала
  Serial.println(channels[cnl].m_data);
  Serial.print("Cycles count: "); // количество прошедших шестичасовых циклов
  Serial.println(channels[cnl].min_max_count);
  Serial.print("Next point in "); // осталось времени до следующего цикла, час/мин
  uint32_t x = tasks.getNextTaskPoint(main_timer);
  Serial.print(x / 3600000);
  Serial.print(" hour, ");
  Serial.print(x % 3600000 / 60000);
  Serial.println(" min");
  Serial.println();
}

#endif