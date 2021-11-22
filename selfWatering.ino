#include <FastLED.h>
#include <avr/eeprom.h>
#include <shTaskManager.h>
#include <shButton.h>
#include "selfWatering.h"

// ==== настройки ====================================
#define MAX_TIMER 14               // максимальное количество суток, по истечении которого полив будет включен безусловно
#define MIN_TIMER 7                // минимальное количество суток, до истечения которого полив не будет включен
#define PUMP_TIMER 10              // время работы помпы в секундах во время полива
#define METERING_COUNT 8           // количество замеров влажности для усреднения результата; желательно задавать значение, равное степени числа 2 (2, 4, 8, 16 и т.д.)
#define BUZZER_TIMEOUT 300         // интервал срабатывания пищалки в режиме "ошибка" в секундах
#define LIGHT_SENSOR_THRESHOLD 150 // минимальные показания датчика света (0-1023)
// ===================================================

// адреса ячеек памяти для хранения настроек влажности по каналам
uint16_t EEMEM e_chanel_2;
uint16_t EEMEM e_chanel_1;
uint16_t EEMEM e_chanel_0;

shTaskManager tasks(4); // создаем список задач

shHandle main_timer;  // главный таймер
shHandle run_channel; // таймер работы каналов полива
shHandle leds_guard;  // таймер индикаторов
shHandle buzzer_on;   // таймер пищалки

byte curChannel = 0; // текущий канал полива

shButton btn(BTN_PIN);

// каналы полива
ChannelState channel_1 = (ChannelState){PUMP_1_PIN, HPOWER_1_SENSOR_PIN, HUMIDITY_1_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0};
ChannelState channel_2 = (ChannelState){PUMP_2_PIN, HPOWER_2_SENSOR_PIN, HUMIDITY_2_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0};
ChannelState channel_3 = (ChannelState){PUMP_3_PIN, HPOWER_3_SENSOR_PIN, HUMIDITY_3_SENSOR_PIN, CNL_DONE, SNS_METERING, 0, 0, 0};

// массив каналов полива
ChannelState channels[3] = {channel_1, channel_2, channel_3};
// массив адресов ячеек памяти для сохранения уровней влажности по каналам
uint16_t emems[3] = {e_chanel_0, e_chanel_1, e_chanel_2};
// массив адресных светодиодов-индикаторов
CRGB leds[4];

// ===================================================
void runChanel()
{
  // если таймер полива каналов еще не запущен, запустить его
  if (!tasks.getTaskState(run_channel))
  {
    tasks.startTask(run_channel);
  }
  // если канал еще не в рабочем режиме, перевести его в рабочий режим
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
        channels[curChannel].p_count = 0;
        // если прошло максимальное количество дней, включить полив без замера влажности
        if (channels[curChannel].min_max_count >= MAX_TIMER * 4)
        {
          channels[curChannel].metering_flag = SNS_WATERING;
          // и записать новое значение порога влажности по значению последнего замера
          word t = channels[curChannel].m_data - 30;
          if (t < 400)
          {
            t = 400;
          }
          eeprom_update_word(&emems[curChannel], t);
        }
        else
        { // иначе включить режим измерения влажности
          channels[curChannel].metering_flag = SNS_METERING;
          digitalWrite(channels[curChannel].p_sensor_pin, HIGH);
          channels[curChannel].m_count = 0;
        }
      }
      else
      {
        channels[curChannel].metering_flag = SNS_NONE;
      }
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
      if (channels[curChannel].channel_state != CNL_ERROR)
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
#ifdef LOG_ON
      channels[channel].m_data = p;
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
          if (p >= eeprom_read_word(&emems[channel]))
          {
            channels[channel].metering_flag = SNS_WATERING;
          }
          break;
        // если после полива влажность недостаточна, включить ошибку и сигнализатор, иначе считать задачу выполненной и обнулить счетчик пустых циклов
        case CNL_CHECK:
          if (p > 300)
          {
            channels[channel].channel_state = CNL_ERROR;
            // перезапустить пищалку, чтобы пропищало после ошибки полива в каждом канале
            tasks.stopTask(buzzer_on);
            runBuzzer();
          }
          else
          {
            channels[channel].min_max_count = 0;
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
    if (++channels[channel].p_count >= PUMP_TIMER * 10)
    { // если время истекло, остановить помпу и включить режим измерения
      digitalWrite(channels[channel].pump_pin, LOW);
      channels[channel].metering_flag = SNS_METERING;
      channels[channel].channel_state = CNL_CHECK;
      channels[channel].p_count = 0;
    }
  }
  else
  { // если воды нет, остановить помпу
    digitalWrite(channels[channel].pump_pin, LOW);
    // перезапустить пищалку, чтобы пропищало сразу
    tasks.stopTask(buzzer_on);
    runBuzzer();
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
  for (byte i = 0; i < 3; i++)
  {
    channels[i].channel_state = CNL_DONE;
    channels[i].metering_flag = flag;
    if (!run)
    { // на случай если в момент остановки идет полив
      tasks.stopTask(run_channel);
      digitalWrite(channels[i].pump_pin, LOW);
      digitalWrite(channels[i].p_sensor_pin, LOW);
    }
  }
  tasks.stopTask(buzzer_on);
  tasks.startTask(leds_guard);
  if (run)
  {
    curChannel = 0;
    tasks.startTask(run_channel);
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
    switch (channels[i].channel_state)
    {
    case CNL_ERROR:
      leds[i + 1] = CRGB::Red;
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
  FastLED.show();
}

void runBuzzer()
{
  static byte n = 0;
  // "мелодия" пищалки: первая строка - частота, вторая строка - длительность
  static uint32_t pick[2][12] = {
      {2000, 0, 2000, 0, 2000, 0, 2000, 0, 2000, 0, 2000, 0},
      {50, 100, 50, 500, 50, 100, 50, 500, 50, 100, 50, BUZZER_TIMEOUT * 1000ul}};
  static CRGB _leds[4];

  // если таймер пищалки еще не запущен, запустить его, иначе проверить на наличие ошибок и, если ошибок уже нет, остановить таймер
  if (!tasks.getTaskState(buzzer_on))
  {
    n = 0;
    tasks.startTask(buzzer_on);
  }
  else if (!getErrors())
  {
    tasks.stopTask(buzzer_on);
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
  tasks.setTaskInterval(buzzer_on, pick[1][n], true);

  if (++n >= 12)
  {
    n = 0;
    // возвращаем управление индикаторами
    tasks.startTask(leds_guard);
  }
}

// ===================================================

void checkButton()
{
  switch (btn.getButtonState())
  {
  // при коротком клике остановить пищалку и сбросить ошибки по каналам
  case BTN_ONECLICK:
    if (tasks.getTaskState(buzzer_on) && !tasks.getTaskState(run_channel))
    {
      manualStart(SNS_NONE, false);
    }
    break;
  // при двойном клике запустить полив с замером влажности
  case BTN_DBLCLICK:
    if (!tasks.getTaskState(run_channel))
    {
      manualStart(SNS_METERING);
    }
    break;
  // при длинном клике запустить полив без замера влажности или остановить полив, если он в текущий момент идет.
  case BTN_LONGCLICK:
    if (tasks.getTaskState(run_channel))
    {
      manualStart(SNS_NONE, false);
    }
    else
    {
      manualStart(SNS_WATERING);
    }
    break;
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
  main_timer = tasks.addTask(21600000, mainTimer);     // главный таймер - интервал 6 часов
  run_channel = tasks.addTask(100, runChanel);         // таймер работы с каналами
  leds_guard = tasks.addTask(100, setLeds);            // управление индикаторами
  buzzer_on = tasks.addTask(300000, runBuzzer, false); // таймер пищалки

  // ==== пороги влажности по каналам ==============
  for (byte i = 0; i < 3; i++)
  {
    if (eeprom_read_word(&emems[i]) > 1023)
    {
      eeprom_update_word(&emems[i], 600);
    }
  }
}

void loop()
{
  tasks.tick();
  checkButton();
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
    case 49: // '1'
      // показания датчика света
      Serial.print("Light sensor data: ");
      Serial.println(analogRead(LIGHT_SENSOR_PIN));
      Serial.println();
      for (byte i = 0; i < 3; i++)
      {
        printChannelStatus(i);
      }
      break;
    case 50: // '2'
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
    default:
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
  Serial.print("Status data, channel ");
  Serial.println(cnl);
  Serial.print("Channel state: "); // текущий статус канала
  switch (channels[cnl].channel_state)
  {
  case CNL_DONE:
    Serial.println("CNL_DONE");
    break;
  case CNL_WORK:
    Serial.println("CNL_WORK");
    break;
  case CNL_ERROR:
    Serial.println("CNL_ERROR");
    break;
  case CNL_CHECK:
    Serial.println("CNL_CHECK");
    break;
  default:
    Serial.println("unknown");
    break;
  }
  Serial.print("Humidity threshold: "); // порог влажности для канала
  Serial.println(eeprom_read_word(&emems[cnl]));
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