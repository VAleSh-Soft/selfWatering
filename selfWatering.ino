#include <FastLED.h>
#include <avr/pgmspace.h>
#include <shTaskManager.h>
#include <shButton.h>
#include "selfWatering.h"

shTaskManager tasks(9); // создаем список задач

shHandle main_timer;          // главный таймер
shHandle run_channel;         // режим работы каналов полива
shHandle leds_guard;          // управление индикаторами
shHandle error_buzzer_on;     // сигнал ошибки полива
shHandle set_buzzer_on;       // сигнал в режиме настройки
shHandle rescan_start;        // перепроверка влажности после полива
shHandle run_set_channels;    // режим настройки каналов
shHandle return_to_def_mode;  // таймер возврата в основной режим из любоого другого режима
shHandle manual_watering_run; // ручной запуск помпы

byte curChannel = 0;            // текущий канал полива
SysMode curMode = MODE_DEFAULT; // текущий режим работы

swButton btn(BTN_PIN);

// массив каналов полива
WateringChannel channels[] = {
#if (CHANNEL_COUNT > 0)
    (WateringChannel){PUMP_1_PIN, HPOWER_1_SENSOR_PIN, HUMIDITY_1_SENSOR_PIN, 100}
#endif
#if (CHANNEL_COUNT > 1)
    ,
    (WateringChannel){PUMP_2_PIN, HPOWER_2_SENSOR_PIN, HUMIDITY_2_SENSOR_PIN, 111}
#endif
#if (CHANNEL_COUNT > 2)
    ,
    (WateringChannel){PUMP_3_PIN, HPOWER_3_SENSOR_PIN, HUMIDITY_3_SENSOR_PIN, 122}
#endif
#if (CHANNEL_COUNT > 3)
    ,
    (WateringChannel){PUMP_4_PIN, HPOWER_4_SENSOR_PIN, HUMIDITY_4_SENSOR_PIN, 133}
#endif
#if (CHANNEL_COUNT == 5)
    ,
    (WateringChannel){PUMP_5_PIN, HPOWER_5_SENSOR_PIN, HUMIDITY_5_SENSOR_PIN, 144}
#endif
};

// адреса ячеек памяти для сохранения настроек датчика света и пищалки
uint16_t ss_eemems_1 = 99; // пищалка (uint8_t)
uint16_t ss_eemems_0 = 98; // датчик света (uint8_t)

// массив адресных светодиодов-индикаторов, первый - индикатор уровня воды, остальные - индикаторы каналов
CRGB leds[CHANNEL_COUNT + 1];

// ===================================================

void setWateringMode(byte channel)
{
  channels[channel].setMeteringState(SNS_WATERING);
  // время будет отсчитываться по циклам, 10 раз в секунду
  channels[channel].readPumpTimerData();
}

void runChanel()
{
  // если таймер полива каналов еще не запущен, запустить его
  if (!tasks.getTaskState(run_channel))
  {
    tasks.startTask(run_channel);
  }
  // запускать полив только в основном или выборочном режиме, иначе ждать выхода из режима настроек
  if (curMode == MODE_DEFAULT || curMode == MODE_CUSTOM_RUN)
  { // если канал используется и еще не в рабочем режиме, перевести его в рабочий режим
    if ((channels[curChannel].checkChannelState(CNL_DONE)) &&
        channels[curChannel].getChannelOnOffState())
    {
      // и попутно увеличить счетчик срабатывания, если это не ручной запуск;
      if (channels[curChannel].checkMeteringState(SNS_NONE))
      {
        channels[curChannel].incSixHourCycles();
      }
      channels[curChannel].setChannelState(CNL_WORK);
      // если датчик еще в состоянии покоя, включить его и настроить канал на работу
      if (channels[curChannel].checkMeteringState(SNS_NONE))
      { // продолжить только если прошло минимальное количество суток и, если в настройках разрешено использование датчика света - в светлое время
        if (channels[curChannel].checkMinDay() &&
            // (!eeprom_read_byte(ss_eemems_0) ||
            (!EEPROM.read(ss_eemems_0) ||
             analogRead(LIGHT_SENSOR_PIN) > LIGHT_SENSOR_THRESHOLD))
        {
          // если использование датчика влажности для канала отключено
          if (!channels[curChannel].getSensorOnOffState())
          {
            setWateringMode(curChannel);
          }
          // или если прошло максимальное количество дней, включить полив без замера влажности
          else
          {
            if (channels[curChannel].checkMaxDay())
            {
              setWateringMode(curChannel);
              if (channels[curChannel].getMaxDay() > channels[curChannel].getMinDay())
              { // и уменьшить порог срабатывания на ступень при условии, что максимальное количество дней задано больше минимального, т.е. запуск реально по таймауту, а порог так не был достигнут
                word t = channels[curChannel].getHumadityTreshold();
                if (t > 400)
                {
                  t -= 100;
                }
                channels[curChannel].setHumadityTreshold(t);
              }
            }
            else
            { // иначе включить режим измерения влажности
              channels[curChannel].setMeteringState(SNS_METERING);
              channels[curChannel].setPumpState(true);
              channels[curChannel].setMeasurementCyclesData(0);
            }
          }
        }
      }
      else if ((channels[curChannel].checkMeteringState(SNS_WATERING)) ||
               !channels[curChannel].getSensorOnOffState())
      {
        setWateringMode(curChannel);
      }
    }
    else
    {
      if (!channels[curChannel].getChannelOnOffState())
      {
        channels[curChannel].setMeteringState(SNS_NONE);
      }
      // если канал включен и уже запущен, то действовать по флагу состояния датчика: замер влажности, полив или остановка таймера, если обработаны все каналы
      switch (channels[curChannel].getMeteringState())
      {
      case SNS_METERING:
      case SNS_TESTING:
        cnlMetering(curChannel);
        break;
      case SNS_RESCAN: // повторную проверку делать только для каналов, которым такое назначено
        if (channels[curChannel].checkChannelState(CNL_RESCAN))
        {
          cnlMetering(curChannel);
        }
        else
        {
          channels[curChannel].setMeteringState(SNS_NONE);
        }
        break;
      case SNS_WATERING:
        cnlWatering(curChannel);
        break;
      default:
        if ((!channels[curChannel].checkChannelState(CNL_ERROR)) &&
            (!channels[curChannel].checkChannelState(CNL_RESCAN)))
        {
          channels[curChannel].setChannelState(CNL_DONE);
        }
        if (curMode == MODE_CUSTOM_RUN)
        {
          tasks.stopTask(run_channel);
        }
        else
        {
          if (++curChannel >= CHANNEL_COUNT)
          {
            tasks.stopTask(run_channel);
            curChannel = 0;
          }
        }
        break;
      }
    }
  }
}

void cnlMetering(byte channel)
{
  if (!channels[channel].getSensorState())
  { // на всякий случай
    channels[channel].setSensorState(HIGH);
  }
  else
  {
    // сделать восемь замеров - METERING_COUNT == 8
    static word p = 0;
    p += channels[channel].readSensorData();
    channels[curChannel].setMeasurementCyclesData(channels[curChannel].getMeasurementCyclesData() + 1);
    if (channels[channel].getMeasurementCyclesData() >= METERING_COUNT)
    { // потом отключить питание датчика и вычислить среднее значение
      channels[channel].setSensorState(LOW);
      channels[channel].setMeasurementCyclesData(0);
      p /= METERING_COUNT;
      channels[channel].setLastMeasurementData(p);
#ifdef LOG_ON
      printLastMeteringData(channel);
#endif
      // определиться с дальнейшими действиями
      if (channels[channel].checkMeteringState(SNS_TESTING))
      {
        channels[channel].setMeteringState(SNS_NONE);
      }
      else
      {
        channels[channel].setMeteringState(SNS_NONE);
        switch (channels[channel].getChannelState())
        {
        // если после простоя сухо, включить режим полива
        case CNL_WORK:
          if (p >= channels[channel].getHumadityTreshold())
          {
            setWateringMode(channel);
          }
          break;
        // если после полива влажность недостаточна, включить перепроверку через минуту, чтобы дать возможность воде разойтись в почве и дойти до датчика; если недостаточна и после перепроверки, включить ошибку и сигнализатор; иначе считать задачу выполненной и обнулить счетчик пустых циклов
        case CNL_CHECK:
        case CNL_RESCAN:
          uint16_t t;
          t = channels[channel].getHumadityTreshold();
          // полив считать совершившимся, если датчик влажности выдает на 200 ниже порогового значения (ниже 100 для порога 400)
          t -= (t >= 500) ? 200 : 100;
          if (p > t)
          {
            if (channels[channel].checkChannelState(CNL_RESCAN))
            {
              channels[channel].setChannelState(CNL_ERROR);
              if (!tasks.getTaskState(error_buzzer_on))
              {
                runErrorBuzzer();
              }
            }
            else if (curMode != MODE_CUSTOM_RUN)
            {
              channels[channel].setChannelState(CNL_RESCAN);
              tasks.startTask(rescan_start);
            }
          }
          else
          {
            channels[channel].clearSixHourCycles();
            if (channels[channel].checkChannelState(CNL_RESCAN))
            {
              channels[channel].setChannelState(CNL_CHECK);
            }
          }
          break;
        default:
          break;
        }
      }
      p = 0;
    }
  }
}

void cnlWatering(byte channel)
{
  if (digitalRead(WATER_LEVEL_SENSOR_PIN))
  {
    if (tasks.getTaskState(return_to_def_mode))
    {
      tasks.restartTask(return_to_def_mode);
    }
    // если вода есть, включить помпу и поливать, пока не истечет заданное время
    channels[channel].setPumpState(channels[channel].getPumpTimer() > 0);
    if (channels[channel].getPumpTimer() == 0)
    { // если время истекло
      // режим измерения включить только в автоматическом режиме и если датчик влажности для этого канала используется
      if (channels[curChannel].getSensorOnOffState() && curMode == MODE_DEFAULT)
      {
        channels[channel].setMeteringState(SNS_METERING);
        channels[channel].setChannelState(CNL_CHECK);
      }
      else
      {
        channels[channel].setMeteringState(SNS_NONE);
      }
    }
    else
    {
      channels[channel].decPumpTimer();
    }
  }
  else
  { // если воды нет, остановить помпу
    channels[channel].setPumpState(false);
    if (!tasks.getTaskState(error_buzzer_on))
    {
      runErrorBuzzer();
    }
  }
}

byte getErrors()
{
  byte result = 0;
  for (byte i = 0; i < CHANNEL_COUNT + 1; i++)
  {
    bool f = (i == 0) ? !digitalRead(WATER_LEVEL_SENSOR_PIN) : channels[i - 1].checkChannelState(CNL_ERROR);
    (f) ? (result) |= (1UL << (i)) : (result) &= ~(1UL << (i));
  }
  return (result);
}

void manualStart(SensorState flag, bool run)
{
  tasks.stopTask(error_buzzer_on);
  tasks.startTask(leds_guard);
  for (byte i = 0; i < CHANNEL_COUNT; i++)
  {
    if (!channels[i].checkChannelState(CNL_RESCAN))
    {
      channels[i].setChannelState(CNL_DONE);
    }
    channels[i].setMeteringState(flag);

    if (!run)
    { // на случай если в момент остановки идет полив
      tasks.stopTask(run_channel);
      channels[i].setPumpState(false);
      channels[i].setSensorState(false);
    }
  }
  if (run)
  {
    if (curMode == MODE_DEFAULT)
    {
      curChannel = 0;
    }
    runChanel();
  }
}

// ===================================================
void mainTimer()
{
  manualStart(SNS_NONE);
}

// подсветка индикаторов каналов в основном режиме
void setLedsDefault(byte i)
{
  leds[i + 1] = CRGB::Black;
  static byte n = 0;
  switch (channels[i].getChannelState())
  {
  case CNL_ERROR:
    leds[i + 1] = CRGB::Red;
    break;
  case CNL_RESCAN:
    leds[i + 1] = (channels[i].getSensorState()) ? CRGB::Orange : CRGB::DarkCyan;
    break;
  case CNL_WORK:
  case CNL_CHECK:
    leds[i + 1] = (channels[i].getPumpState()) ? CRGB::Green : CRGB::Orange;
    break;
  default:
    if (curMode != MODE_MANUAL_WATERING)
    {
      n = 0;
    }

    if (i == curChannel)
    {
      switch (curMode)
      {
      case MODE_CUSTOM_RUN:
        leds[i + 1] = CRGB::Cyan;
        break;
      case MODE_MANUAL_WATERING:
        if (n < 9)
        {
          leds[i + 1] = CRGB::Cyan;
        }
        if (++n > 9)
        {
          n = 0;
        }
        break;
      default:
        break;
      }
    }
    break;
  }
}

// подсветка индикаторов каналов при настройке использования пищалки и датчика света
void setLeds_2(byte i)
{
  CRGB x = CRGB::Black;
#if (CHANNEL_COUNT <= 2)
  if ((i == 0) || (i == (CHANNEL_COUNT)))
#else
  if ((i == 0) || (i == (CHANNEL_COUNT - 1)))
#endif
  {
    if (i == curChannel)
    {
      static byte n = 0;
      static byte f = curChannel;

      // если канал изменился, его индикатор начинает с "зажмуренного" состояния ))
      if (f != curChannel)
      {
        n = 9;
        f = curChannel;
      }

      byte h = (i == 0) ? 0 : 1;
      if (((channels[0].getMeasurementCyclesData()) >> (h)) & 0x01)
      {
        x = (i == 0) ? CRGB::Orange : CRGB::DarkCyan;
      }
      else
      {
        x = CRGB::Red;
      }
      // текущий канал подмигивает с частотой 1 сек
      if (n >= 9)
      { // если отключено, то подмаргивать цветом канала (на случай, если отключены все, чтобы было понятно, что за настройка делается), иначе подмаргивать просто отключением
        if (((channels[0].getMeasurementCyclesData()) >> (h)) & 0x01)
        {
          x = CRGB::Black;
        }
        else
        {
          x = (i == 0) ? CRGB::Orange : CRGB::DarkCyan;
        }
      }
      if (++n > 9)
      {
        n = 0;
      }
    }
    else
    {
      if (i == 0)
      {
        // x = (eeprom_read_byte(ss_eemems_0)) ? CRGB::Orange : CRGB::Red;
        x = (EEPROM.read(ss_eemems_0)) ? CRGB::Orange : CRGB::Red;
      }
      else
      {
        // x = (eeprom_read_byte(ss_eemems_1)) ? CRGB::DarkCyan : CRGB::Red;
        x = (EEPROM.read(ss_eemems_1)) ? CRGB::DarkCyan : CRGB::Red;
      }
    }
  }
#if (CHANNEL_COUNT <= 2)
  // если используются только один или два канала, то задействовать и индикатор уровня воды
  leds[i] = x;
#else
  leds[i + 1] = x;
#endif
}

// подсветка индикаторов каналов при настройке времени работы помп
void setLeds_3(byte i)
{
  if (i == curChannel)
  {
    static byte n = 0;
    // текущий канал подсвечивать синим или зеленым (если помпа включена); в зеленом режиме индикатор подмигивает с частотой полсекунды и проблескивает белым с частотой 1 секунда
    if (channels[i].getPumpState())
    {
      switch (n)
      {
      case 4:
        leds[i + 1] = CRGB::Black;
        break;
      case 9:
        leds[i + 1] = CRGB::White;
        break;
      default:
        leds[i + 1] = CRGB::Green;
        break;
      }
      if (++n > 9)
      {
        n = 0;
      }
    }
    else
    {
      leds[i + 1] = CRGB::Blue;
      n = 0;
    }
  }
  else
  // остальные каналы не подсвечивать
  {
    leds[i + 1] = CRGB::Black;
  }
}

// подсветка индикаторов каналов при настройке порога влажности и количества дней
void setLeds_4(byte i, bool reset = false)
{
  static byte n = 0;
  static byte k = 0;
  static byte b = 0;
  static byte f = curChannel;

  // если канал изменился, сразу обновить данные
  if (f != curChannel)
  {
    f = curChannel;
    n = 0;
    k = 0;
    b = 0;
  }

  if (reset)
  {
    n = 0;
    k = 0;
    b = 0;
    return;
  }

  if (i == curChannel)
  {
    // если кнопка нажата, ничего не предпринимать - в этот момент идет настройка данных
    if (btn.isButtonClosed())
    {
      n = 0;
      k = 0;
      b = 0;
    }
    else
    {
      if (!k)
      {
        if (++n > 5)
        {
          n = 0;
          if (++b >= channels[curChannel].getMeasurementCyclesData())
          {
            k++;
          }
        }
      }
      else
      {
        n = 0;
        if (++k > 12)
        {
          n = 0;
          k = 0;
          b = 0;
        }
      }
    }
    if ((n >= 5) && (!k))
    {
      leds[i + 1] = CRGB::White;
    }
    else
    {
      switch (btn.getClickBtnCount())
      {
      case 4:
        leds[i + 1] = CRGB::Blue;
        break;
      case 5:
        leds[i + 1] = CRGB::Orange;
        break;
      case 6:
        leds[i + 1] = CRGB::Purple;
        break;
      }
    }
  }
  else
    leds[i + 1] = CRGB::Black;
}

// подсветка индикаторов каналов при настройке включения/отключения датчиков влажности и каналов в целом
void setLeds_7(byte i)
{
  // каналы подсвечивать синим (настройка датчиков), зеленым (настройка каналов) или красным (если датчик/канал отключен);
  if (i == curChannel)
  {
    static byte n = 0;
    static byte f = curChannel;

    // если канал изменился, его индикатор начинает с "зажмуренного" состояния ))
    if (f != curChannel)
    {
      n = 9;
      f = curChannel;
    }

    if (channels[i].getMeasurementCyclesData())
    {
      leds[i + 1] = (btn.getClickBtnCount() == 8) ? CRGB::Green : CRGB::Blue;
    }
    else
    {
      leds[i + 1] = CRGB::Red;
    }
    // текущий канал подмигивает с частотой 1 сек
    if (n >= 9)
    { // если отключено, то подмаргивать цветом канала (на случай, если отключены все, чтобы было понятно, что за настройка делается), иначе подмаргивать просто отключением
      if (channels[i].getMeasurementCyclesData())
      {
        leds[i + 1] = CRGB::Black;
      }
      else
      {
        leds[i + 1] = (btn.getClickBtnCount() == 8) ? CRGB::Green : CRGB::Blue;
      }
    }
    if (++n > 9)
    {
      n = 0;
    }
  }
  else
  {
    switch (btn.getClickBtnCount())
    {
    case 7:
      leds[i + 1] = (channels[i].getSensorOnOffState()) ? CRGB::Blue : CRGB::Red;
      break;
    case 8:
      leds[i + 1] = (channels[i].getChannelOnOffState()) ? CRGB::Green : CRGB::Red;
      break;
    }
  }
}

void setLeds()
{
  static byte n = 0;
  // индикатор датчика уровня воды подмигивает каждые две секунды зеленым, если вода есть и красным, если воды нет; в режиме настройки индикатор отключен
  if (curMode != MODE_SETTING)
  {
    leds[0] = (digitalRead(WATER_LEVEL_SENSOR_PIN)) ? CRGB::Green : CRGB::Red;
    if (n >= 19)
    {
      leds[0] = CRGB::Black;
    }
    if (++n > 19)
    {
      n = 0;
    }
  }
  else
  {
    leds[0] = CRGB::Black;
  }

  // индикаторы каналов
#if (CHANNEL_COUNT <= 2)
  byte z = CHANNEL_COUNT;
  if (btn.getClickBtnCount() == 2)
  {
    z = (CHANNEL_COUNT < 2) ? 2 : 3;
  }
  for (byte i = 0; i < z; i++)
#else
  for (byte i = 0; i < CHANNEL_COUNT; i++)
#endif
  {
    switch (curMode)
    {
    case MODE_DEFAULT:
    case MODE_CUSTOM_RUN:
    case MODE_MANUAL_WATERING:
      if (tasks.getTaskState(manual_watering_run))
      {
        setLeds_3(i);
      }
      else
      {
        setLedsDefault(i);
      }
      break;
    case MODE_SETTING:
      switch (btn.getClickBtnCount())
      {
      case 2:
        setLeds_2(i);
        break;
      case 3:
        setLeds_3(i);
        break;
      case 4:
      case 5:
      case 6:
        setLeds_4(i);
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
  static CRGB _leds[CHANNEL_COUNT + 1];

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
    // if (eeprom_read_byte(ss_eemems_1))
    if (EEPROM.read(ss_eemems_1))
    { // пищалка срабатывает если только это разрешено в настройках; проблескивание белым будет в любом случае
      tone(BUZZER_PIN, pgm_read_dword(&pick[0][n]), pgm_read_dword(&pick[1][n]));
    }
    // в момент включения звука индикаторы каналов с ошибками включить белым
    for (byte i = 0; i < CHANNEL_COUNT + 1; i++)
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
        if (channels[i - 1].checkChannelState(CNL_ERROR))
        {
          leds[i] = CRGB::White;
        }
      }
    }
  }
  else
  { // иначе возвращать цвета индикаторов к цветам по умолчанию
    for (byte i = 0; i < CHANNEL_COUNT + 1; i++)
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
    if ((btn.getClickBtnCount() >= 2) && (btn.getClickBtnCount() <= 8))
    {
      tone(BUZZER_PIN, pgm_read_dword(&pick[0][n]), pgm_read_dword(&pick[1][n]));
      tasks.setTaskInterval(set_buzzer_on, pgm_read_dword(&pick[1][n]), true);

      if (++n >= btn.getClickBtnCount() * 2)
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
    manualStart(SNS_RESCAN);
    tasks.stopTask(rescan_start);
  }
}

void isBtnClosed_3(uint32_t _tmr, uint32_t &_result)
{
  if (!channels[curChannel].getPumpState())
  {
    tone(BUZZER_PIN, 2500, 100ul);
    leds[curChannel + 1] = CRGB::White;
    FastLED.show();
    channels[curChannel].setPumpState(true);
    channels[curChannel].setPumpTimer(_tmr);
  }
  else if (channels[curChannel].checkSettingData(FL_RUN_DATA))
  { // если истекло максимальное разрешенное время работы помпы, остановить ее
    if (_tmr - channels[curChannel].getPumpTimer() >= MAX_PUMP_TIMER)
    {
      channels[curChannel].setPumpState(false);
      tone(BUZZER_PIN, 2500, 300ul);
      _result = MAX_PUMP_TIMER;
      channels[curChannel].setSettingData(FL_STOP_DATA);
    }
  }
  else
  { // если идет проверочный слив воды, и заданное время вышло, остановить помпу
    if (_tmr - channels[curChannel].getPumpTimer() >= channels[curChannel].getPumpData())
    {
      channels[curChannel].setPumpState(false);
      tone(BUZZER_PIN, 2500, 300ul);
      channels[curChannel].setSettingData(FL_NONE);
    }
  }
}

void isBtnClosed_4()
{
  static uint32_t _timer = 0;
  byte max_data;
  switch (btn.getClickBtnCount())
  {
  case 4:
    max_data = 4;
    break;
  case 5:
    max_data = MIN_DAY_COUNT_DEF * 2;
    break;
  case 6:
    max_data = MAX_DAY_COUNT_DEF * 2;
    break;
  }
  // увеличивать значение каждые полсекунды, пока нажата кнопка
  if (millis() - _timer >= 500)
  {
    _timer = millis();
    channels[curChannel].setMeasurementCyclesData(channels[curChannel].getMeasurementCyclesData() + 1);
    if (channels[curChannel].getMeasurementCyclesData() == max_data)
    {
      // останавливать прирост данных при достижении максимального значения
      channels[curChannel].setSettingData(FL_STOP_DATA);
    }
    if (channels[curChannel].getMeasurementCyclesData() > max_data)
    {
      channels[curChannel].setMeasurementCyclesData(1);
    }
    tone(BUZZER_PIN, 2500, 100ul);
    leds[curChannel + 1] = CRGB::White;
    FastLED.show();
  }
}

void getCurrentData()
{
  // поле m_count используется только при измерении влажности, поэтому его можно с чистой совестью использовать в процессе настроек
  switch (btn.getClickBtnCount())
  {
  case 2:
    byte x;
    x = 0;
    // (eeprom_read_byte(ss_eemems_0)) ? x |= (1 << (0)) : x &= ~(1 << (0));
    // (eeprom_read_byte(ss_eemems_1)) ? x |= (1 << (1)) : x &= ~(1 << (1));
    (EEPROM.read(ss_eemems_0)) ? x |= (1 << (0)) : x &= ~(1 << (0));
    (EEPROM.read(ss_eemems_1)) ? x |= (1 << (1)) : x &= ~(1 << (1));
    channels[curChannel].setMeasurementCyclesData(x);
    break;
  case 4:
    channels[curChannel].setMeasurementCyclesData(channels[curChannel].getHumadityTreshold() / 100 - 3);
    break;
  case 5:
    channels[curChannel].setMeasurementCyclesData(channels[curChannel].getMinDay());
    break;
  case 6:
    channels[curChannel].setMeasurementCyclesData(channels[curChannel].getMaxDay());
    break;
  case 7:
    channels[curChannel].setMeasurementCyclesData(channels[curChannel].getSensorOnOffState());
    break;
  case 8:
    channels[curChannel].setMeasurementCyclesData(channels[curChannel].getChannelOnOffState());
    break;
  }
}

void runSetChannels()
{
  static uint32_t result = 0;
  uint32_t tmr = millis();
  byte max_channel = CHANNEL_COUNT;
#if (CHANNEL_COUNT <= 2)
  if (btn.getClickBtnCount() == 2)
  {
    max_channel = (CHANNEL_COUNT < 2) ? 2 : 3;
  }
#endif
  if (!tasks.getTaskState(run_set_channels))
  {
    curChannel = 0;
    channels[curChannel].setMeasurementCyclesData(0);
    channels[curChannel].setSettingData(FL_NONE);
    getCurrentData();
    curMode = MODE_SETTING;
    tasks.startTask(run_set_channels);
    tasks.startTask(return_to_def_mode);
    runSetBuzzer();
  }
  // ждать, пока отработает пищалка
  if (tasks.getTaskState(set_buzzer_on))
  {
    return;
  }
  // проверка настройки помпы
  if ((btn.getClickBtnCount() == 3) && (channels[curChannel].checkSettingData(FL_CHECK_DATA)))
  {
    isBtnClosed_3(tmr, result);
  }

  // управление данными
  if (((btn.getClickBtnCount() == 2) && (channels[0].checkSettingData(FL_RUN_DATA))) ||
      (channels[curChannel].checkSettingData(FL_RUN_DATA)))
  {
    if (btn.isButtonClosed())
    {
      tasks.restartTask(return_to_def_mode);
      switch (btn.getClickBtnCount())
      {
      case 2:
        tone(BUZZER_PIN, 2500, 100ul);
        byte z;
        z = (curChannel == 0) ? curChannel : 1;
        byte x;
        x = channels[0].getMeasurementCyclesData();
        x ^= (1 << z);
        channels[0].setMeasurementCyclesData(x);
        channels[0].setSettingData(FL_STOP_DATA);
        break;
      case 3:
        isBtnClosed_3(tmr, result);
        break;
      case 4:
      case 5:
      case 6:
        isBtnClosed_4();
        break;
      case 7:
      case 8:
        tone(BUZZER_PIN, 2500, 100ul);
        channels[curChannel].setMeasurementCyclesData(!channels[curChannel].getMeasurementCyclesData());
        channels[curChannel].setSettingData(FL_STOP_DATA);
        break;
      }
    }
    else
    {
      byte y = (btn.getClickBtnCount() == 2) ? 0 : curChannel;
      channels[y].setSettingData(FL_STOP_DATA);
      switch (btn.getClickBtnCount())
      {
      case 3:
        channels[curChannel].setPumpState(false);
        result = tmr - channels[curChannel].getPumpTimer();
        if (result < 1000)
        {
          channels[curChannel].setSettingData(FL_NONE);
        }
        break;
      }
    }
  }
  // управление сохранением изменений, переходом на другой канал или выходом из настроек
  byte k = (btn.getClickBtnCount() == 2) ? 0 : curChannel;
  if ((channels[k].checkSettingData(FL_SAVE_DATA)) ||
      (channels[k].checkSettingData(FL_NEXT)) ||
      (channels[k].checkSettingData(FL_EXIT)))
  {
    if (channels[k].checkSettingData(FL_SAVE_DATA))
    {
      switch (btn.getClickBtnCount())
      {
      case 2:
        if (curChannel == 0)
        {
          // eeprom_update_byte(ss_eemems_0, (((channels[0].getMeasurementCyclesData()) >> (0)) & 0x01));
          EEPROM.update(ss_eemems_0, (((channels[0].getMeasurementCyclesData()) >> (0)) & 0x01));
        }
        else
        {
          // eeprom_update_byte(ss_eemems_1, (((channels[0].getMeasurementCyclesData()) >> (1)) & 0x01));
          EEPROM.update(ss_eemems_1, (((channels[0].getMeasurementCyclesData()) >> (1)) & 0x01));
        }
        break;
      case 3:
        channels[curChannel].setPumpData(result);
        break;
      case 4:
        channels[curChannel].setHumadityTreshold((channels[curChannel].getMeasurementCyclesData() + 3) * 100);
        break;
      case 5:
        channels[curChannel].setMinDay(channels[curChannel].getMeasurementCyclesData());
        break;
      case 6:
        channels[curChannel].setMaxDay(channels[curChannel].getMeasurementCyclesData());
        break;
      case 7:
        channels[curChannel].setSensorOnOffState(channels[curChannel].getMeasurementCyclesData());
        break;
      case 8:
        channels[curChannel].setChannelOnOffState(channels[curChannel].getMeasurementCyclesData());
        break;
      }
    }
    if (channels[k].checkSettingData(FL_EXIT))
    {
      curChannel = max_channel;
    }
    else
    {
      curChannel++;
      if ((btn.getClickBtnCount() == 2) && (curChannel == 1))
      {
        curChannel = max_channel - 1; // перескочить с первого канала сразу на последний
      }
    }
    if (curChannel < max_channel)
    {
      byte y = (btn.getClickBtnCount() == 2) ? 0 : curChannel;
      channels[y].setMeasurementCyclesData(0);
      channels[y].setSettingData(FL_NONE);
      getCurrentData();
      tone(BUZZER_PIN, 2000, 100ul);
    }
  }
  // выход из настроек
  if (curChannel >= max_channel)
  {
    tasks.stopTask(run_set_channels);
    tasks.stopTask(return_to_def_mode);
    curMode = MODE_DEFAULT;
    curChannel = 0;
    // в одноканальной системе или в многоканальной при выходе из настройки
    // первого канала по таймауту могут не обнуляться счетчики в setLeds_4(),
    // и при следующем входе в настройки 4-6 первый цикл моргания может не
    // соответствовать реальным настройкам, поэтому принудительно обнуляем
    // счетчики
    switch (btn.getClickBtnCount())
    {
    case 4:
    case 5:
    case 6:
      setLeds_4(0, true);
      break;
    }
    btn.setClickBtnCount(0);
    runSetBuzzer();
  }
}

void returnToDefMode()
{
  if (tasks.getTaskState(run_set_channels))
  {
    channels[curChannel].setSettingData(FL_EXIT);
  }
  else
  {
    curMode = MODE_DEFAULT;
  }
  tasks.stopTask(manual_watering_run);
  tasks.stopTask(return_to_def_mode);
}

void manualWateringRun()
{
  if (!tasks.getTaskState(manual_watering_run))
  {
    curMode = MODE_MANUAL_WATERING;
    tasks.startTask(manual_watering_run);
    tasks.startTask(return_to_def_mode);
  }
  channels[curChannel].setPumpState(btn.isButtonClosed() && digitalRead(WATER_LEVEL_SENSOR_PIN));
  if (!channels[curChannel].getPumpState())
  {
    tasks.stopTask(manual_watering_run);
  }
  else
  {
    tasks.restartTask(return_to_def_mode);
  }
}
// ===================================================

void btnOneClick()
{
  switch (curMode)
  {
  // в основном режиме остановить пищалку и сбросить ошибки по каналам
  case MODE_DEFAULT:
    if ((tasks.getTaskState(error_buzzer_on) && !tasks.getTaskState(run_channel)) ||
        tasks.getTaskState(rescan_start))
    {
      manualStart(SNS_NONE, false);
      // если запущен таймер рескана датчиков влажности, остановить его
      if (tasks.getTaskState(rescan_start))
      {
        tasks.stopTask(rescan_start);
        for (byte i = 0; i < CHANNEL_COUNT; i++)
        {
          if (channels[i].checkChannelState(CNL_RESCAN))
          {
            channels[i].setChannelState(CNL_DONE);
          }
        }
      }
    }
    break;
  // в выборочном и ручном режимах переключить канал (если не идет полив)
  case MODE_CUSTOM_RUN:
  case MODE_MANUAL_WATERING:
    if (!tasks.getTaskState(run_channel))
    {
      if (++curChannel >= CHANNEL_COUNT)
      {
        tone(BUZZER_PIN, 2000, 1000ul);
        returnToDefMode();
      }
      else
      {
        tone(BUZZER_PIN, 2000, 100ul);
      }
    }
    break;
  // в режиме настройки дать команду на переключение канала с возможным сохранением данных
  case MODE_SETTING:
    if (!tasks.getTaskState(set_buzzer_on))
    {
      byte z = (btn.getClickBtnCount() == 2) ? 0 : curChannel;
      channels[z].setSettingData((channels[z].checkSettingData(FL_STOP_DATA)) ? FL_SAVE_DATA : FL_NEXT);
    }
    tasks.restartTask(return_to_def_mode);
    break;
  }
}

void btnDblClick(byte n)
{
  switch (curMode)
  {
    // в основном и выборочном режимах запустить полив с замером влажности; если случайно сделан двойной клик при входе в настройки - ничего не делать
  case MODE_DEFAULT:
  case MODE_CUSTOM_RUN:
    if (!tasks.getTaskState(run_channel) && n <= 1)
    {
      manualStart(SNS_METERING);
    }
    break;
  // в режиме настройки просто перезапустить таймер автовыхода; в случае настройки помпы поднять флаг тестового запуска помпы
  case MODE_SETTING:
    if ((btn.getClickBtnCount() == 3) && (!channels[curChannel].checkSettingData(FL_RUN_DATA)))
    {
      channels[curChannel].setSettingData(FL_CHECK_DATA);
    }
    tasks.restartTask(return_to_def_mode);
    break;
  default:
    break;
  }
}

void btnLongClick(byte &n)
{
  switch (curMode)
  {
  // в основном и выборочном режимах запустить полив без замера влажности или остановить полив, если он в текущий момент идет.
  case MODE_CUSTOM_RUN:
  case MODE_DEFAULT:
  case MODE_MANUAL_WATERING:
    if (tasks.getTaskState(run_channel))
    {
      manualStart(SNS_NONE, false);
    }
    else
    {
      if (n <= 1)
      {
        tone(BUZZER_PIN, 2500, 100ul);
        switch (curMode)
        {
        case MODE_CUSTOM_RUN:
        case MODE_DEFAULT:
          manualStart(SNS_WATERING);
          break;
        // в ручном режиме включить помпу
        case MODE_MANUAL_WATERING:
          manualWateringRun();
          break;
        default:
          break;
        }
      }
      else
      {
        switch (n)
        {
        case 2:
        case 3:
          curMode = (n == 2) ? MODE_CUSTOM_RUN : MODE_MANUAL_WATERING;
          tasks.startTask(return_to_def_mode);
          curChannel = 0;
          tone(BUZZER_PIN, 2000, 1000ul);
          n = 0;
          break;
        case 4:
          n = 200;
          break;
        default:
          n = 0;
          break;
        }
      }
    }
    break;
  // в режиме настроек установить флаг изменения данных
  case MODE_SETTING:
    byte k = (btn.getClickBtnCount() == 2) ? 0 : curChannel;
    channels[k].setSettingData(FL_RUN_DATA);
    tasks.restartTask(return_to_def_mode);
    break;
  }
}

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
        tone(BUZZER_PIN, 2000, 100ul);
      }
    }
    if (tasks.getTaskState(return_to_def_mode))
    {
      tasks.restartTask(return_to_def_mode);
    }
    break;
    // при одиночном клике
  case BTN_ONECLICK:
    btnOneClick();
    break;
  // при двойном клике
  case BTN_DBLCLICK:
    btnDblClick(n);
    break;
  // при длинном клике
  case BTN_LONGCLICK:
    btnLongClick(n);
    break;
  }

  // проверить, сколько одиночных кликов кнопки сделано
  if ((btn.getClickBtnCount() == 0) && !tasks.getTaskState(run_channel))
  {
    if (n == 200)
    {
      btn.setClickBtnCount(2);
      n = 0;
    }
    else if (millis() - btn_timer > 1000)
    {
      if (((n >= 3) && (n <= 8)))
      {
        btn.setClickBtnCount(n);
      }
      n = 0;
    }
  }
}

// ===================================================

void verifyEEPROM()
{
  // если при включении питания зажата кнопка, сбросить к настройкам по умолчанию
  bool to_reset = !digitalRead(BTN_PIN);

  if (to_reset)
  { // настройки использования датчика света и пищалки при ошибках
    // обозначить звуковым сигналом сброс настроек
    tone(BUZZER_PIN, 2000, 1000ul);
    // eeprom_update_byte(ss_eemems_0, 1);
    // eeprom_update_byte(ss_eemems_1, 1);
    EEPROM.update(ss_eemems_0, 1);
    EEPROM.update(ss_eemems_1, 1);
    for (byte i = 0; i < CHANNEL_COUNT; i++)
    {
      channels[i].verifyEEPROMData(to_reset);
    }
  }
}

void setup()
{
#ifdef LOG_ON
  Serial.begin(9600);
#endif

  FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, CHANNEL_COUNT + 1);
  FastLED.setBrightness(5);

  // ==== настройка пинов ============================
  pinMode(WATER_LEVEL_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  // ==== настройка задач ============================
  main_timer = tasks.addTask(21600000, mainTimer);
  run_channel = tasks.addTask(100, runChanel);
  leds_guard = tasks.addTask(100, setLeds);
  error_buzzer_on = tasks.addTask(300000, runErrorBuzzer, false);
  rescan_start = tasks.addTask(60000, rescanStart, false);
  set_buzzer_on = tasks.addTask(1000, runSetBuzzer, false);
  run_set_channels = tasks.addTask(100, runSetChannels, false);
  return_to_def_mode = tasks.addTask(60000, returnToDefMode, false);
  manual_watering_run = tasks.addTask(100, manualWateringRun, false);

  // ==== верификация настроек =======================
  verifyEEPROM();
}

void loop()
{
  tasks.tick();
  checkButton();
  if ((btn.getClickBtnCount() >= 2) &&
      (btn.getClickBtnCount() <= 8) &&
      !tasks.getTaskState(run_set_channels))
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
    Serial.println();
    int inByte = Serial.read();
    Serial.println(inByte);
    Serial.println();
    switch (inByte)
    {
    case 49: // '1' - вывод информации о системе
      Serial.print(F("Firmware version: "));
      Serial.println(F(FIRMWARE_VERSION));
      Serial.println();
      // состояние таймеров
      Serial.println(F("=== Timers state ==="));
      Serial.println();
      Serial.print(F("main_timer: "));
      Serial.println(tasks.getTaskState(main_timer));
      Serial.print(F("run_channel: "));
      Serial.println(tasks.getTaskState(run_channel));
      Serial.print(F("leds_guard: "));
      Serial.println(tasks.getTaskState(leds_guard));
      Serial.print(F("error_buzzer_on: "));
      Serial.println(tasks.getTaskState(error_buzzer_on));
      Serial.print(F("rescan_start: "));
      Serial.println(tasks.getTaskState(rescan_start));
      Serial.print(F("set_buzzer_on: "));
      Serial.println(tasks.getTaskState(set_buzzer_on));
      Serial.print(F("run_set_channels: "));
      Serial.println(tasks.getTaskState(run_set_channels));
      Serial.print(F("return_to_def_mode: "));
      Serial.println(tasks.getTaskState(return_to_def_mode));
      Serial.print(F("manual_watering_run: "));
      Serial.println(tasks.getTaskState(manual_watering_run));
      Serial.println();

      Serial.println(F("=== Sensors state ==="));
      Serial.println();
      // показания датчика света
      Serial.print(F("Light sensor "));
      // if (!eeprom_read_byte(ss_eemems_0))
      if (!EEPROM.read(ss_eemems_0))
      {
        Serial.println(F(" not used"));
      }
      else
      {
        Serial.println(F(" used"));
        Serial.print(F("Light sensor data: "));
        Serial.println(analogRead(LIGHT_SENSOR_PIN));
      }
      // наличие воды по датчику уровня
      Serial.print(F("Water sensor data: "));
      digitalRead(WATER_LEVEL_SENSOR_PIN) ? Serial.println(F("yes")) : Serial.println(F("no"));
      // использование пищалки в сообщениях об ошибках
      Serial.print(F("Error buzzer "));
      // if (!eeprom_read_byte(ss_eemems_1))
      if (!EEPROM.read(ss_eemems_1))
      {
        Serial.println(F(" not used"));
      }
      else
      {
        Serial.println(F(" used"));
      }
      Serial.println();

      Serial.println(F("=== Channels state ==="));
      Serial.println();
      Serial.print(F("Total channel count: "));
      Serial.println(CHANNEL_COUNT);
      Serial.println();
      for (byte i = 0; i < CHANNEL_COUNT; i++)
      {
        printChannelStatus(i);
      }
      break;
    case 50: // '2' - получение текущей влажности по каналам
      if (curMode != MODE_SETTING)
      {
        if (!tasks.getTaskState(run_channel))
        {
          manualStart(SNS_TESTING);
        }
        else
        {
          Serial.println(F("Denied, watering or metering is in progress"));
        }
      }
      else
      {
        Serial.println(F("Denied, system in settings mode"));
      }
      break;
    default: // вывод данных по последнему замеру влажности по каналам
      for (byte i = 0; i < CHANNEL_COUNT; i++)
      {
        printLastMeteringData(i);
      }
      break;
    }
  }
}

void printLastMeteringData(byte cnl)
{
  Serial.print(F("Metering data, channel "));
  Serial.print(cnl + 1);
  Serial.print(F(": "));
  Serial.println(channels[cnl].getLastMeasurementData());
}

void printChannelStatus(byte cnl)
{
  Serial.print(F("Channel ")); // текущий статус канала
  Serial.print(cnl + 1);
  if (!channels[cnl].getChannelOnOffState())
  {
    Serial.println(F(" not used"));
  }
  else
  {
    Serial.println(F(" used"));
    Serial.print(F("Channel state: "));
    switch (channels[cnl].getChannelState())
    {
    case CNL_DONE:
      Serial.println(F("CNL_DONE"));
      break;
    case CNL_WORK:
      Serial.println(F("CNL_WORK"));
      break;
    case CNL_CHECK:
      Serial.println(F("CNL_CHECK"));
      break;
    case CNL_RESCAN:
      Serial.println(F("CNL_RESCAN"));
      break;
    case CNL_ERROR:
      Serial.println(F("CNL_ERROR"));
      break;
    default:
      Serial.println(F("unknown"));
      break;
    }
    Serial.print(F("Pump state: ")); // текущий статус помпы
    (channels[cnl].getPumpState()) ? Serial.println(F("power ON")) : Serial.println(F("power OFF"));
    Serial.print(F("Pump timeout, sec: ")); // время работы помпы для канала
    Serial.print(channels[cnl].getPumpData() / 1000);
    Serial.print(F("."));
    Serial.println(channels[cnl].getPumpData() % 1000);
    Serial.print(F("Humidity sensor:")); // текущий статус датчика влажности
    if (!channels[cnl].getSensorOnOffState())
    {
      Serial.println(F(" not used"));
    }
    else
    {
      Serial.println(F(" used"));
      Serial.print(F("Sensor state: ")); // текущий статус датчика влажности
      digitalRead(channels[cnl].getSensorState()) ? Serial.println(F("power ON")) : Serial.println(F("power OFF"));
      Serial.print(F("Metering flag: "));
      switch (channels[cnl].getMeteringState())
      {
      case SNS_NONE:
        Serial.println(F("SNS_NONE"));
        break;
      case SNS_METERING:
        Serial.println(F("SNS_METERING"));
        break;
      case SNS_TESTING:
        Serial.println(F("SNS_TESTING"));
        break;
      case SNS_WATERING:
        Serial.println(F("SNS_WATERING"));
        break;
      case SNS_RESCAN:
        Serial.println(F("SNS_RESCAN"));
        break;
      default:
        Serial.println(F("unknown"));
        break;
      }
      Serial.print(F("Humidity threshold: ")); // порог влажности для канала
      Serial.println(channels[cnl].getHumadityTreshold());
      Serial.print(F("Humidity last data: ")); // последний замер влажности для канала
      Serial.println(channels[cnl].getLastMeasurementData());
    }
    Serial.print(F("Set interval of days: ")); // настройки минимального и максимального количества дней
    Serial.print(channels[cnl].getMinDay());
    Serial.print(F(" - ")); // настройки минимального и максимального количества дней
    Serial.println(channels[cnl].getMaxDay());
    Serial.print(F("Six-hour cycles passed: ")); // количество прошедших шестичасовых циклов
    Serial.println(channels[cnl].getSixHourCycles());
    Serial.print(F("Next point in ")); // осталось времени до следующего цикла, час/мин
    uint32_t x = tasks.getNextTaskPoint(main_timer);
    Serial.print(x / 3600000);
    Serial.print(F(" hour, "));
    Serial.print(x % 3600000 / 60000);
    Serial.println(F(" min"));
  }
  Serial.println();
}

#endif