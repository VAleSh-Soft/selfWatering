# SelfWatering v 2.7.5

Модуль для автоматического полива комнатных растений. Может иметь от одного до пяти каналов полива, может поливать растения как с контролем влажности почвы, так и по расписанию.

Количество каналов задается при прошивке в строке `#define CHANNEL_COUNT 3 // количество каналов полива (от 1 до 5)` в файле **selfWatering.h**

Модуль позволяет гибко настраивать параметры каналов полива с сохранением настроек в EEPROM.

Датчики влажности почвы включаются буквально на несколько секунд в неделю, что позволяет свести практически на нет электрохимическую коррозию датчиков.

Процесс полива контролируется по изменению влажности почвы после отключения помпы.

Состояние модуля и каналов полива можно контролировать через разъем **UART** с помощью **USB_TTL** переходника и любого монитора COM-порта. Команда "**1**" выводит полную информацию о состоянии модуля и всех каналов полива, команда "**2**" запускает внеочередной замер влажности почвы по всем каналам и вывод его результатов. Любая другая команда выводит в монитор данные по последнему замеру влажности, когда бы он ни проводился. Если возможность мониторинга не требуется, можно закомментировать строку `#define LOG_ON // ведение отладочного лога` в файле **selfWatering.h**, что ко всему прочему позволит сократить размер кода.

Для индикации состояния модуля и каналов полива используются адресные светодиоды WS2812B.

Более подробную информацию по работе с модулем см. в файле **docs/manual.pdf**

Принципиальная схема модуля приведена в файле **docs/Schematic_Self_Watering.png**

Если возникнут вопросы, пишите на valesh-soft@yandex.ru 