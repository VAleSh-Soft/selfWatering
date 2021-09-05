#include <FastLED.h>
#include <shTaskManager.h>
#include <shButton.h>
#include "selfWatering.h"

shTaskManager tasks(6); // создаем список задач

shHandle main_timer; // главный таймер
shHandle run_pump1;  // таймер работы первой помпы
shHandle run_pump2;  // таймер работы второй помпы
shHandle run_pump3;  // таймер работы третьей помпы
shHandle leds_guard; // таймер индикаторов
shHandle buzzer_on;  // таймер пищалки

shButton btn(BTN_PIN);

// массив адресных светодиодов-индикаторов
CRGB leds[4];
// массив пинов питания датчиков влажности
byte hsensors[3]{HPOWER_1_SENSOR_PIN, HPOWER_2_SENSOR_PIN, HPOWER_3_SENSOR_PIN};
// массив пинов данных датчиков влажности
byte sensors[3]{HUMIDITY_1_SENSOR_PIN, HUMIDITY_2_SENSOR_PIN, HUMIDITY_3_SENSOR_PIN};
// массив пинов помп
byte pumps[3]{PUMP_1_PIN, PUMP_2_PIN, PUMP_3_PIN};
// массив таймеров полива
shHandle pump_timers[3]{run_pump1, run_pump2, run_pump3};

// ===================================================
void runPump(byte pin, bool force)
{
  static byte _count = 0; // счетчик проходов, по истечении 10 секунд останавливать полив в любом случае
  if (!tasks.getTaskState(pump_timers[pin]))
  { // если задача еще не запущена, включить питание датчика и запустить задачу
    digitalWrite(hsensors[pin], HIGH);
    tasks.startTask(pump_timers[pin]);
    _count = 0;
  }
  else
  { // иначе заниматься собственно поливом, если есть вода
    if (digitalRead(WATER_LEVEL_SENSOR_PIN))
    {
      _count++;
      if (analogRead(sensors[pin]) == 0)
      {
        digitalWrite(pumps[pin], HIGH);
      }

      if (analogRead(sensors[pin]) >= 1000)
      {
        stopPump(pin);
      }
    }
  }
}

void stopPump(byte pin)
{
  digitalWrite(pumps[pin], LOW);
  digitalWrite(hsensors[pin], LOW);
  tasks.stopTask(pump_timers[pin]);
}

// ===================================================
void mainTimer()
{
  // поливать только в светлое время суток, чтобы не создавать шума ночью
  if (analogRead(LIGHT_SENSOR_PIN) > 300)
  {
    runPump(0);
    runPump(1);
    runPump(3);
  }
}

void runPump1()
{
  runPump(0);
}

void runPump2()
{
  runPump(1);
}

void runPump3()
{
  runPump(3);
}

void setLeds()
{
  leds[3] = digitalRead(WATER_LEVEL_SENSOR_PIN) ? CRGB::Green : CRGB::Red;
  for (byte i = 0; i < 3; i++)
  {
    /* code */
  }
  FastLED.show();
}

void buzzerOn()
{
  static byte n = 0;
  static word pick[2][4]{
      {2000, 0, 2000, 0},
      {20, 50, 20, 300000}};
  if (!tasks.getTaskState(buzzer_on))
  {
    n = 0;
    tasks.startTask(buzzer_on);
  }
  if (pick[0][n] > 0)
  {
    tone(BUZZER_PIN, pick[0][n], pick[1][n]);
  }
  tasks.setTaskInterval(buzzer_on, pick[1][n + 1], true);
  if (++n > 3)
  {
    n = 0;
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
  main_timer = tasks.addTask(21600000, mainTimer); // главный таймер - интервал 6 часов
  run_pump1 = tasks.addTask(100, runPump1);
  run_pump2 = tasks.addTask(100, runPump2);
  run_pump3 = tasks.addTask(100, runPump3);
  leds_guard = tasks.addTask(100, setLeds);
  buzzer_on = tasks.addTask(300000, buzzerOn, false);
}

void loop()
{
  tasks.tick();

  switch (btn.getButtonState())
  {
  case BTN_LONGCLICK:
    runPump(0, true);
    runPump(1, true);
    runPump(3, true);
    break;
  }
}