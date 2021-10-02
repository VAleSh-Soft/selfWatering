#include <FastLED.h>
#include <shTaskManager.h>
#include <shButton.h>
#include "selfWatering.h"

// ==== настройки ====================================
#define MAX_TIMER 14       // максимальное количество суток, по истечении которого полив будет включен безусловно
#define MIN_TIMER 7        // минимальное количество суток, до истечения которого полив не будет включен
#define PUMP_TIMER 10      // время работы помпы в секундах во время полива
#define METERING_COUNT 8   // количество замеров влажности для усреднения результата; желательно задавать значение, равное степени числа 2 (2, 4, 8, 16 и т.д.)
#define BUZZER_TIMEOUT 300 // интервал срабатывания пищалки в режиме "ошибка" в секундах
// ===================================================

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
// массив адресных светодиодов-индикаторов
CRGB leds[4];

// ===================================================
void runChanel(byte index, bool force)
{
  static bool _force = force;
  if (!tasks.getTaskState(run_channel))
  { // если таймер каналов полива еще не запущен, запустить его
    tasks.startTask(run_channel);
    channels[index].min_max_timer++;

    if (channels[index].metering_flag == SNS_NONE)
    {
      if (_force || channels[index].min_max_timer >= MIN_TIMER * 4)
      // продолжить только если прошло минимальное количество суток или задан принудительный старт
      {
        channels[index].p_timer = 0;
        if (_force || channels[index].min_max_timer >= MAX_TIMER * 4)
        { // если задан принудительный старт или прошло максимальное количество суток, запустить полив без замера влажности
          channels[index].metering_flag = SNS_WATERING;
        }
        else
        {
          if (analogRead(LIGHT_SENSOR_PIN) > 300) // иначе включить датчик и режим измерения; но только в светлое время суток, чтобы не создавать шума ночью
          {
            channels[index].metering_flag = SNS_METERING;
            digitalWrite(channels[index].p_sensor_pin, HIGH);
            channels[index].m_count = 0;
          }
          else
          {
            channels[index].metering_flag = SNS_NONE;
          }
        }
      }
    }
  }
  else
  { // если таймер уже запущен, то действовать по флагу состояния датчика: замер влажности, полив или остановка таймера
    switch (channels[index].metering_flag)
    {
    case SNS_METERING:
      cnlMetering(index);
      break;
    case SNS_WATERING:
      cnlWatering(index);
      break;
    default:
      if (++index >= 3)
      {
        index = 0;
        tasks.stopTask(run_channel);
      }
      curChannel = index;
      break;
    }
  }
}

void cnlMetering(byte index)
{
  if (!digitalRead(channels[index].p_sensor_pin))
  { // на всякий случай
    digitalWrite(channels[index].p_sensor_pin, HIGH);
    channels[index].m_count = 0;
  }
  else
  {
    // сделать восемь замеров
    static word p = 0;
    p += analogRead(channels[index].d_sensor_pin);
    if (++channels[index].m_count >= METERING_COUNT)
    { // потом отключить питание датчика и вычислить среднее значение
      digitalWrite(channels[index].p_sensor_pin, LOW);
      channels[index].metering_flag = SNS_NONE;
      p /= METERING_COUNT;
      // определиться с дальнейшими действиями
      switch (channels[index].channel_state)
      {
      // если после простоя сухо, включить режим полива
      case CNL_DONE:
        if (p >= 1000)
        {
          channels[index].channel_state = CNL_WORK;
          channels[index].metering_flag = SNS_WATERING;
        }
        break;
      // если после полива влажность недостаточна, включить ошибку и сигнализатор, иначе считать задачу выполненной и остановить таймер канала
      case CNL_WORK:
      case CNL_ERROR:
        if (p > 300)
        {
          channels[index].channel_state = CNL_ERROR;
          if (!tasks.getTaskState(buzzer_on))
          {
            runBuzzer();
          }
        }
        else
        {
          channels[index].channel_state = CNL_DONE;
          channels[index].metering_flag = SNS_NONE;
          channels[index].min_max_timer = 0;
        }
        break;
      }
      p = 0;
    }
  }
}

void cnlWatering(byte index)
{
  if (digitalRead(WATER_LEVEL_SENSOR_PIN))
  { // если вода есть, включить помпу и поливать, пока не истечет заданное время
    digitalWrite(channels[index].pump_pin, HIGH);
    if (++channels[index].p_timer >= PUMP_TIMER * 10)
    { // если время истекло, остановить помпу и включить режим измерения
      digitalWrite(channels[index].pump_pin, LOW);
      channels[index].metering_flag = SNS_METERING;
    }
  }
  else
  { // если воды нет, остановить помпу и включить сигнализатор, если он еще не запущен
    digitalWrite(channels[index].pump_pin, LOW);
    if (!tasks.getTaskState(buzzer_on))
    {
      runBuzzer();
    }
  }
}

// ===================================================
void mainTimer()
{
  curChannel = 0;
  runChanel(curChannel);
}

void runPump()
{
  runChanel(curChannel);
}

void setLeds()
{
  static byte n = 0;
  // индикатор датчика уровня воды
  if (digitalRead(WATER_LEVEL_SENSOR_PIN))
  {
    leds[0] = CRGB::Green;
    n = 0;
  }
  else
  { // при срабатывании датчика индикатор светит красным и подмигивает каждые две секунды
    leds[0] = (n < 20) ? CRGB::Red : CRGB::Black;
    if (++n > 20)
    {
      n = 0;
    }
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

  if (!tasks.getTaskState(buzzer_on))
  {
    n = 0;
    tasks.startTask(buzzer_on);
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

void setup()
{
  FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, 4);
  FastLED.setBrightness(50);

  // ==== настройка кнопки =============================
  btn.setLongClickMode(LCM_ONLYONCE);
  btn.setVirtualClickOn(true);

  // ==== настройка пинов ==============================
  pinMode(WATER_LEVEL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(HPOWER_1_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_1_PIN, OUTPUT);
  pinMode(HPOWER_2_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_2_PIN, OUTPUT);
  pinMode(HPOWER_3_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_3_PIN, OUTPUT);

  // ==== настройка задач ==============================
  main_timer = tasks.addTask(21600000, mainTimer);     // главный таймер - интервал 6 часов
  run_channel = tasks.addTask(100, runPump);           // таймер первого канала
  leds_guard = tasks.addTask(100, setLeds);            // управление индикаторами
  buzzer_on = tasks.addTask(300000, runBuzzer, false); // таймер пищалки
}

void loop()
{
  tasks.tick();

  switch (btn.getButtonState())
  {
  case BTN_LONGCLICK:
    for (byte i = 0; i < 3; i++)
    {
      // if (!tasks.getTaskState(channel_timers[i]))
      // {
      //   runChanel(i, true);
      // }
    }
    break;
  case BTN_ONECLICK:
    tasks.stopTask(buzzer_on);
    tasks.startTask(leds_guard);
    break;
  }
}