#include <FastLED.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <shTaskManager.h>
#include <shButton.h>
#include "selfWatering.h"

// ==== настройки ====================================
#define FIRMWARE_VERSION "3.7.2"       // версия прошивки
#define MAX_DAY_COUNT_DEF 14           // максимальное количество суток, по истечении которого полив будет включен безусловно, значение по умолчанию
#define MIN_DAY_COUNT_DEF 7            // минимальное количество суток, до истечения которого полив не будет включен, значение по умолчанию
#define METERING_COUNT 8               // количество замеров влажности для усреднения результата; желательно задавать значение, равное степени числа 2 (2, 4, 8, 16 и т.д.)
#define BUZZER_TIMEOUT 300             // интервал срабатывания пищалки в режиме "ошибка" в секундах
#define LIGHT_SENSOR_THRESHOLD 150     // минимальные показания датчика света (0-1023)
#define MAX_PUMP_TIMER 60000ul         // максимальное время работы помпы, мс
#define DEFAULT_PUMP_TIMER 10000ul     // значение времени работы помпы по умолчанию
#define DEFAULT_HUMIDITY_THRESHOLD 600 // порог датчика влажности по умолчанию
// ===================================================

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

swButton btn(BTN_PIN);

// массив каналов полива
ChannelState channels[] = {
#if (CHANNEL_COUNT > 0)
    (ChannelState){PUMP_1_PIN, HPOWER_1_SENSOR_PIN, HUMIDITY_1_SENSOR_PIN, CNL_DONE, SNS_TESTING, 0, 0, 0, 0, 0}
#endif
#if (CHANNEL_COUNT > 1)
    ,
    (ChannelState){PUMP_2_PIN, HPOWER_2_SENSOR_PIN, HUMIDITY_2_SENSOR_PIN, CNL_DONE, SNS_TESTING, 0, 0, 0, 0, 0}
#endif
#if (CHANNEL_COUNT > 2)
    ,
    (ChannelState){PUMP_3_PIN, HPOWER_3_SENSOR_PIN, HUMIDITY_3_SENSOR_PIN, CNL_DONE, SNS_TESTING, 0, 0, 0, 0, 0}
#endif
#if (CHANNEL_COUNT > 3)
    ,
    (ChannelState){PUMP_4_PIN, HPOWER_4_SENSOR_PIN, HUMIDITY_4_SENSOR_PIN, CNL_DONE, SNS_TESTING, 0, 0, 0, 0, 0}
#endif
#if (CHANNEL_COUNT == 5)
    ,
    (ChannelState){PUMP_5_PIN, HPOWER_5_SENSOR_PIN, HUMIDITY_5_SENSOR_PIN, CNL_DONE, SNS_TESTING, 0, 0, 0, 0, 0}
#endif
};

// адреса ячеек памяти для сохранения настроек датчика света и пищалки
uint16_t ss_eemems_1 = 156; // пищалка (uint8_t)
uint16_t ss_eemems_0 = 155; // датчик света (uint8_t)

// массив адресов ячеек памяти для сохранения данных по пройденным циклам min_max_count (uint8_t)
uint16_t mm_eemems[] = {
#if (CHANNEL_COUNT > 0)
    150
#endif
#if (CHANNEL_COUNT > 1)
    ,
    151
#endif
#if (CHANNEL_COUNT > 2)
    ,
    152
#endif
#if (CHANNEL_COUNT > 3)
    ,
    153
#endif
#if (CHANNEL_COUNT == 5)
    ,
    154
#endif
};

// массив адресов ячеек памяти для сохранения максимального количества дней (uint8_t)
uint16_t md_eemems[] = {
#if (CHANNEL_COUNT > 0)
    145
#endif
#if (CHANNEL_COUNT > 1)
    ,
    146
#endif
#if (CHANNEL_COUNT > 2)
    ,
    147
#endif
#if (CHANNEL_COUNT > 3)
    ,
    148
#endif
#if (CHANNEL_COUNT == 5)
    ,
    149
#endif
};

// массив адресов ячеек памяти для сохранения минимального количества дней (uint8_t)
uint16_t d_eemems[] = {
#if (CHANNEL_COUNT > 0)
    140
#endif
#if (CHANNEL_COUNT > 1)
    ,
    141
#endif
#if (CHANNEL_COUNT > 2)
    ,
    142
#endif
#if (CHANNEL_COUNT > 3)
    ,
    143
#endif
#if (CHANNEL_COUNT == 5)
    ,
    144
#endif
};

// массив адресов ячеек памяти для сохранения данных по включенным датчикам влажности (uint8_t)
uint16_t hs_eemems[] = {
#if (CHANNEL_COUNT > 0)
    135
#endif
#if (CHANNEL_COUNT > 1)
    ,
    136
#endif
#if (CHANNEL_COUNT > 2)
    ,
    137
#endif
#if (CHANNEL_COUNT > 3)
    ,
    138
#endif
#if (CHANNEL_COUNT == 5)
    ,
    139
#endif
};

// массив адресов ячеек памяти для сохранения данных по включенным каналам (uint8_t)
uint16_t c_eemems[] = {
#if (CHANNEL_COUNT > 0)
    130
#endif
#if (CHANNEL_COUNT > 1)
    ,
    131
#endif
#if (CHANNEL_COUNT > 2)
    ,
    132
#endif
#if (CHANNEL_COUNT > 3)
    ,
    133
#endif
#if (CHANNEL_COUNT == 5)
    ,
    134
#endif
};

// массив адресов ячеек памяти для сохранения уровней влажности по каналам (uint16_t)
uint16_t h_eemems[] = {
#if (CHANNEL_COUNT > 0)
    120
#endif
#if (CHANNEL_COUNT > 1)
    ,
    122
#endif
#if (CHANNEL_COUNT > 2)
    ,
    124
#endif
#if (CHANNEL_COUNT > 3)
    ,
    126
#endif
#if (CHANNEL_COUNT == 5)
    ,
    128
#endif
};

// массив адресов ячеек памяти для сохранения настроек помпы по каналам (uint16_t)
uint16_t p_eemems[] = {
#if (CHANNEL_COUNT > 0)
    100
#endif
#if (CHANNEL_COUNT > 1)
    ,
    104
#endif
#if (CHANNEL_COUNT > 2)
    ,
    108
#endif
#if (CHANNEL_COUNT > 3)
    ,
    112
#endif
#if (CHANNEL_COUNT == 5)
    ,
    116
#endif
};

// массив адресных светодиодов-индикаторов, первый - индикатор уровня воды, остальные - индикаторы каналов
CRGB leds[CHANNEL_COUNT + 1];

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
    if ((channels[curChannel].channel_state == CNL_DONE) && eeprom_read_byte(c_eemems[curChannel]))
    {
      // и попутно увеличить счетчик срабатывания, если это не ручной запуск;
      if (channels[curChannel].metering_flag == SNS_NONE)
      {
        channels[curChannel].min_max_count++;
        eeprom_update_byte(mm_eemems[curChannel], channels[curChannel].min_max_count);
      }
      channels[curChannel].channel_state = CNL_WORK;
      // если датчик еще в состоянии покоя, включить его и настроить канал на работу
      if (channels[curChannel].metering_flag == SNS_NONE)
      { // продолжить только если прошло минимальное количество суток и, если в настройках разрешено использование датчика света - в светлое время
        if (channels[curChannel].min_max_count >= eeprom_read_byte(d_eemems[curChannel]) * 4 && (!eeprom_read_byte(ss_eemems_0) || analogRead(LIGHT_SENSOR_PIN) > LIGHT_SENSOR_THRESHOLD))
        {
          // если использование датчика влажности для канала отключено
          if (!eeprom_read_byte(hs_eemems[curChannel]))
          {
            setWateringMode(curChannel);
          }
          // или если прошло максимальное количество дней, включить полив без замера влажности
          else
          {
            byte x = eeprom_read_byte(md_eemems[curChannel]) * 4;
            if (channels[curChannel].min_max_count >= x)
            {
              setWateringMode(curChannel);
              if (x > eeprom_read_byte(d_eemems[curChannel]) * 4)
              { // и уменьшить порог срабатывания на ступень при условии, что максимальное количество дней задано больше минимального, т.е. запуск реально по таймауту, а порог так не был достигнут
                word t = eeprom_read_word(h_eemems[curChannel]);
                if (t > 400)
                {
                  t -= 100;
                }
                eeprom_update_word(h_eemems[curChannel], t);
              }
            }
            else
            { // иначе включить режим измерения влажности
              channels[curChannel].metering_flag = SNS_METERING;
              digitalWrite(channels[curChannel].p_sensor_pin, HIGH);
              channels[curChannel].m_count = 0;
            }
          }
        }
      }
      else if ((channels[curChannel].metering_flag == SNS_WATERING) || !eeprom_read_byte(hs_eemems[curChannel]))
      {
        setWateringMode(curChannel);
      }
    }
    else
    {
      if (!eeprom_read_byte(c_eemems[curChannel]))
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
        if (++curChannel >= CHANNEL_COUNT)
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
          if (p >= eeprom_read_word(h_eemems[channel]))
          {
            setWateringMode(channel);
          }
          break;
        // если после полива влажность недостаточна, включить перепроверку через минуту, чтобы дать возможность воде разойтись в почве и дойти до датчика; если недостаточна и после перепроверки, включить ошибку и сигнализатор; иначе считать задачу выполненной и обнулить счетчик пустых циклов
        case CNL_CHECK:
        case CNL_RESCAN:
          uint16_t t = eeprom_read_word(h_eemems[channel]);
          // полив считать совершившимся, если датчик влажности выдает на 200 ниже порогового значения (ниже 100 для порога 400)
          t -= (t >= 500) ? 200 : 100;
          if (p > t)
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
            eeprom_update_byte(mm_eemems[curChannel], channels[curChannel].min_max_count);
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
    if (millis() - channels[channel].p_timer >= eeprom_read_dword(p_eemems[channel]))
    { // если время истекло, остановить помпу
      digitalWrite(channels[channel].pump_pin, LOW);
      // режим измерения включить только если датчик влажности для этого канала используется
      if (eeprom_read_byte(hs_eemems[curChannel]))
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
  byte result = 0;
  for (byte i = 0; i < CHANNEL_COUNT + 1; i++)
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
  for (byte i = 0; i < CHANNEL_COUNT; i++)
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

// подсветка индикаторов каналов в основном режиме
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
      if (((channels[0].m_count) >> (h)) & 0x01)
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
        if (((channels[0].m_count) >> (h)) & 0x01)
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
        x = (eeprom_read_byte(ss_eemems_0)) ? CRGB::Orange : CRGB::Red;
      }
      else
      {
        x = (eeprom_read_byte(ss_eemems_1)) ? CRGB::DarkCyan : CRGB::Red;
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
    if (digitalRead(channels[i].pump_pin))
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
          if (++b >= channels[curChannel].m_count)
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

    if (channels[i].m_count)
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
      if (channels[i].m_count)
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
      leds[i + 1] = (eeprom_read_byte(hs_eemems[i])) ? CRGB::Blue : CRGB::Red;
      break;
    case 8:
      leds[i + 1] = (eeprom_read_byte(c_eemems[i])) ? CRGB::Green : CRGB::Red;
      break;
    }
  }
}

void setLeds()
{
  static byte n = 0;
  // индикатор датчика уровня воды подмигивает каждые две секунды зеленым, если вода есть и красным, если воды нет; в режиме настройки индикатор отключен
  if (curMode == MODE_DEFAULT)
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
      setLedsDefault(i);
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
    if (eeprom_read_byte(ss_eemems_1))
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
        if (channels[i - 1].channel_state == CNL_ERROR)
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
    manualStart(SNS_METERING);
    tasks.stopTask(rescan_start);
  }
}

void isBtnClosed_3(uint32_t _tmr, uint32_t &_result)
{
  if (!digitalRead(channels[curChannel].pump_pin))
  {
    tone(BUZZER_PIN, 2500, 100);
    leds[curChannel + 1] = CRGB::White;
    FastLED.show();
    digitalWrite(channels[curChannel].pump_pin, HIGH);
    channels[curChannel].p_timer = _tmr;
  }
  else if (channels[curChannel].flag == FL_RUN_DATA)
  { // если истекло максимальное разрешенное время работы помпы, остановить ее
    if (_tmr - channels[curChannel].p_timer >= MAX_PUMP_TIMER)
    {
      digitalWrite(channels[curChannel].pump_pin, LOW);
      tone(BUZZER_PIN, 2500, 300);
      _result = MAX_PUMP_TIMER;
      channels[curChannel].flag = FL_STOP_DATA;
    }
  }
  else
  { // если идет проверочный слив воды, и заданное время вышло, остановить помпу
    if (_tmr - channels[curChannel].p_timer >= eeprom_read_dword(p_eemems[curChannel]))
    {
      digitalWrite(channels[curChannel].pump_pin, LOW);
      tone(BUZZER_PIN, 2500, 300);
      channels[curChannel].flag = FL_NONE;
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
    channels[curChannel].m_count++;
    if (channels[curChannel].m_count == max_data)
    {
      // останавливать прирост данных при достижении максимального значения
      channels[curChannel].flag = FL_STOP_DATA;
    }
    if (channels[curChannel].m_count > max_data)
    {
      channels[curChannel].m_count = 1;
    }
    tone(BUZZER_PIN, 2500, 100);
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
    channels[0].m_count = 0;
    (eeprom_read_byte(ss_eemems_0)) ? (channels[0].m_count) |= (1 << (0)) : (channels[0].m_count) &= ~(1 << (0));
    (eeprom_read_byte(ss_eemems_1)) ? (channels[0].m_count) |= (1 << (1)) : (channels[0].m_count) &= ~(1 << (1));
    break;
  case 4:
    channels[curChannel].m_count = eeprom_read_word(h_eemems[curChannel]) / 100 - 3;
    break;
  case 5:
    channels[curChannel].m_count = eeprom_read_byte(d_eemems[curChannel]);
    break;
  case 6:
    channels[curChannel].m_count = eeprom_read_byte(md_eemems[curChannel]);
    break;
  case 7:
    channels[curChannel].m_count = eeprom_read_byte(hs_eemems[curChannel]);
    break;
  case 8:
    channels[curChannel].m_count = eeprom_read_byte(c_eemems[curChannel]);
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
    channels[curChannel].m_count = 0;
    channels[curChannel].flag = FL_NONE;
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
  if ((btn.getClickBtnCount() == 3) && (channels[curChannel].flag == FL_CHECK_DATA))
  {
    isBtnClosed_3(tmr, result);
  }

  // управление данными
  if (((btn.getClickBtnCount() == 2) && (channels[0].flag == FL_RUN_DATA)) || (channels[curChannel].flag == FL_RUN_DATA))
  {
    if (btn.isButtonClosed())
    {
      tasks.restartTask(return_to_def_mode);
      switch (btn.getClickBtnCount())
      {
      case 2:
        tone(BUZZER_PIN, 2500, 100);
        byte z;
        z = (curChannel == 0) ? curChannel : 1;
        (channels[0].m_count) ^= (1 << z);
        channels[0].flag = FL_STOP_DATA;
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
        tone(BUZZER_PIN, 2500, 100);
        channels[curChannel].m_count = !channels[curChannel].m_count;
        channels[curChannel].flag = FL_STOP_DATA;
        break;
      }
    }
    else
    {
      byte y = (btn.getClickBtnCount() == 2) ? 0 : curChannel;
      channels[y].flag = FL_STOP_DATA;
      switch (btn.getClickBtnCount())
      {
      case 3:
        digitalWrite(channels[curChannel].pump_pin, LOW);
        result = tmr - channels[curChannel].p_timer;
        if (result < 1000)
        {
          channels[curChannel].flag = FL_NONE;
        }
        break;
      }
    }
  }
  // управление сохранением изменений, переходом на другой канал или выходом из настроек
  byte k = (btn.getClickBtnCount() == 2) ? 0 : curChannel;
  if ((channels[k].flag == FL_SAVE_DATA) || (channels[k].flag == FL_NEXT) || (channels[k].flag == FL_EXIT))
  {
    if (channels[k].flag == FL_SAVE_DATA)
    {
      switch (btn.getClickBtnCount())
      {
      case 2:
        if (curChannel == 0)
        {
          eeprom_update_byte(ss_eemems_0, (((channels[0].m_count) >> (0)) & 0x01));
        }
        else
        {
          eeprom_update_byte(ss_eemems_1, (((channels[0].m_count) >> (1)) & 0x01));
        }
        break;
      case 3:
        eeprom_update_dword(p_eemems[curChannel], result);
        break;
      case 4:
        eeprom_update_word(h_eemems[curChannel], (channels[curChannel].m_count + 3) * 100);
        break;
      case 5:
        eeprom_update_byte(d_eemems[curChannel], channels[curChannel].m_count);
        break;
      case 6:
        eeprom_update_byte(md_eemems[curChannel], channels[curChannel].m_count);
        break;
      case 7:
        eeprom_update_byte(hs_eemems[curChannel], channels[curChannel].m_count);
        break;
      case 8:
        eeprom_update_byte(c_eemems[curChannel], channels[curChannel].m_count);
        break;
      }
    }
    if (channels[k].flag == FL_EXIT)
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
      channels[y].m_count = 0;
      channels[y].flag = FL_NONE;
      getCurrentData();
      tone(BUZZER_PIN, 2000, 100);
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
          for (byte i = 0; i < CHANNEL_COUNT; i++)
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
        byte z = (btn.getClickBtnCount() == 2) ? 0 : curChannel;
        channels[z].flag = (channels[z].flag == FL_STOP_DATA) ? FL_SAVE_DATA : FL_NEXT;
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
    // в режиме настройки просто перезапустить таймер автовыхода; в случае настройки помпы поднять флаг тестового запуска помпы
    case MODE_SETTING:
      if ((btn.getClickBtnCount() == 3) && (channels[curChannel].flag != FL_RUN_DATA))
      {
        channels[curChannel].flag = FL_CHECK_DATA;
      }
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
      else if (n <= 1)
      {
        manualStart(SNS_WATERING);
      }
      else if (n == 3)
      {
        n = 200;
      }

      break;
    // установить флаг изменения данных
    case MODE_SETTING:
      (btn.getClickBtnCount() == 2) ? channels[0].flag = FL_RUN_DATA : channels[curChannel].flag = FL_RUN_DATA;
      tasks.restartTask(return_to_def_mode);
      break;
    }
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
    eeprom_update_byte(ss_eemems_0, 1);
    eeprom_update_byte(ss_eemems_1, 1);
    // обозначить звуковым сигналом сброс настроек
    tone(BUZZER_PIN, 2000, 1000);
  }
  for (byte i = 0; i < CHANNEL_COUNT; i++)
  {
    if (to_reset)
    { // настройки использования каналов и датчиков влажности
      eeprom_update_byte(hs_eemems[i], 1);
      eeprom_update_byte(c_eemems[i], 1);
    }
    if (to_reset || (eeprom_read_word(h_eemems[i]) > 700) || (eeprom_read_word(h_eemems[i]) < 400))
    { // настройки уровней влажности
      eeprom_update_word(h_eemems[i], DEFAULT_HUMIDITY_THRESHOLD);
    }
    if (to_reset || (eeprom_read_dword(p_eemems[i]) > MAX_PUMP_TIMER) || (eeprom_read_dword(p_eemems[i]) <= 1000))
    { // настройки помп
      eeprom_update_dword(p_eemems[i], DEFAULT_PUMP_TIMER);
    }
    if (to_reset || (eeprom_read_byte(d_eemems[i]) > 14) || (eeprom_read_byte(d_eemems[i]) == 0))
    { // настройки минимального количества дней
      eeprom_update_byte(d_eemems[i], MIN_DAY_COUNT_DEF);
    }
    if (to_reset || (eeprom_read_byte(md_eemems[i]) > 28) || (eeprom_read_byte(md_eemems[i]) == 0))
    { // настройки максимального количества дней
      eeprom_update_byte(md_eemems[i], MAX_DAY_COUNT_DEF);
    }
    if (to_reset || (eeprom_read_byte(mm_eemems[i]) > 112))
    { // сохраненное количество пройденных шестичасовых циклов
      eeprom_update_byte(mm_eemems[i], 0);
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
#if (CHANNEL_COUNT > 0)
  pinMode(HPOWER_1_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_1_PIN, OUTPUT);
#endif
#if (CHANNEL_COUNT > 1)
  pinMode(HPOWER_2_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_2_PIN, OUTPUT);
#endif
#if (CHANNEL_COUNT > 2)
  pinMode(HPOWER_3_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_3_PIN, OUTPUT);
#endif
#if (CHANNEL_COUNT > 3)
  pinMode(HPOWER_4_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_4_PIN, OUTPUT);
#endif
#if (CHANNEL_COUNT == 5)
  pinMode(HPOWER_5_SENSOR_PIN, OUTPUT);
  pinMode(PUMP_5_PIN, OUTPUT);
#endif

  // ==== настройка задач ============================
  main_timer = tasks.addTask(21600000, mainTimer);                   // главный таймер - интервал 6 часов
  run_channel = tasks.addTask(100, runChanel);                       // таймер работы с каналами
  leds_guard = tasks.addTask(100, setLeds);                          // управление индикаторами
  error_buzzer_on = tasks.addTask(300000, runErrorBuzzer, false);    // таймер сигнала ошибки
  rescan_start = tasks.addTask(60000, rescanStart, false);           // таймер перепроверки влажности
  set_buzzer_on = tasks.addTask(1000, runSetBuzzer, false);          // таймер пищалки режима настройи
  run_set_channels = tasks.addTask(100, runSetChannels, false);      // таймер режима настройки
  return_to_def_mode = tasks.addTask(60000, returnToDefMode, false); // таймер автовыхода из настроек

  // ==== верификация настроек =======================
  verifyEEPROM();
  for (byte i = 0; i < CHANNEL_COUNT; i++)
  {
    channels[i].min_max_count = eeprom_read_byte(mm_eemems[i]);
  }
}

void loop()
{
  tasks.tick();
  checkButton();
  if ((btn.getClickBtnCount() >= 2) && (btn.getClickBtnCount() <= 8) && !tasks.getTaskState(run_set_channels))
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
      Serial.println();

      Serial.println(F("=== Sensors state ==="));
      Serial.println();
      // показания датчика света
      Serial.print(F("Light sensor "));
      if (!eeprom_read_byte(ss_eemems_0))
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
      if (!eeprom_read_byte(ss_eemems_1))
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
      if (curMode == MODE_DEFAULT)
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

      Serial.println();
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
  Serial.println(channels[cnl].m_data);
}

void printChannelStatus(byte cnl)
{
  Serial.print(F("Channel ")); // текущий статус канала
  Serial.print(cnl + 1);
  if (!eeprom_read_byte(c_eemems[cnl]))
  {
    Serial.println(F(" not used"));
  }
  else
  {
    Serial.println(F(" used"));
    Serial.print(F("Channel state: "));
    switch (channels[cnl].channel_state)
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
    digitalRead(channels[cnl].pump_pin) ? Serial.println(F("power ON")) : Serial.println(F("power OFF"));
    Serial.print(F("Pump timeout, sec: ")); // время работы помпы для канала
    Serial.print(eeprom_read_dword(p_eemems[cnl]) / 1000);
    Serial.print(F("."));
    Serial.println(eeprom_read_dword(p_eemems[cnl]) % 1000);
    Serial.print(F("Humidity sensor:")); // текущий статус датчика влажности
    if (!eeprom_read_byte(hs_eemems[cnl]))
    {
      Serial.println(F(" not used"));
    }
    else
    {
      Serial.println(F(" used"));
      Serial.print(F("Sensor state: ")); // текущий статус датчика влажности
      digitalRead(channels[cnl].p_sensor_pin) ? Serial.println(F("power ON")) : Serial.println(F("power OFF"));
      Serial.print(F("Metering flag: "));
      switch (channels[cnl].metering_flag)
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
      default:
        Serial.println(F("unknown"));
        break;
      }
      Serial.print(F("Humidity threshold: ")); // порог влажности для канала
      Serial.println(eeprom_read_word(h_eemems[cnl]));
      Serial.print(F("Humidity last data: ")); // последний замер влажности для канала
      Serial.println(channels[cnl].m_data);
    }
    Serial.print(F("Set interval of days: ")); // настройки минимального и максимального количества дней
    Serial.print(eeprom_read_byte(d_eemems[cnl]));
    Serial.print(F(" - ")); // настройки минимального и максимального количества дней
    Serial.println(eeprom_read_byte(md_eemems[cnl]));
    Serial.print(F("Six-hour cycles passed: ")); // количество прошедших шестичасовых циклов
    Serial.println(channels[cnl].min_max_count);
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