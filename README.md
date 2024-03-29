# ModRob Framework
## Основные настройки
Проект базируется на следующих основных сущностях:
  * Фреймворк [PlarformIO](https://platformio.org/) для удобства работы с широким спектром микроконтроллеров различных производителей (мы преимущественно используем контроллры семейства STM32).
  * Протокол UAVCAN/OpenCyphal (его адаптация для шины CAN - [libcanard](https://github.com/OpenCyphal/libcanard)) позволяющий создавать распределённые вычислительные сети из субмодулей (робота), работающие в реальном времени.
  * Библиотеки LL и HAL для создания драйверов для контроллеров STM32.

Все дальнейшие настройки описываются исходя из предположения, что пользователь работает на ОС Linux. Хотя связка VSCode-PlatformIO вполне работоспособна и на ОС Windows. 
## Установка среды разработки (IDE) и первоначальное конфигурирование
Рекомендуется использовать IDE [VSCode](https://code.visualstudio.com/). Это обусловлено тем, что фреймворк PlatformIO хорошо адаптирован для работы с этой средой.
___
Если вы хотите использовать другую среду, вам нужно установить PlatformIO Core (CLI) отдельно (желательно непосредственно в систему). Затем в конкретной среде разработки можно подключить отдельные функции PlatformIO. Если фреймворк прописан в пути, его всегда можно использовать в терминале. 
___

В VScode фреймворк PlatformIO устанавливается через расширения **Extensions**. 
Для начала работы следует скачать этот репозиторий и распаковать его в отдельную директорию. Далее в VSCode нужно открыть только что распакованную папку: File -> Open Folder. 

## Структура проекта

Все ключевые настройки проекта прописаны в файле `platformio.ini`: используемый контроллер, библиотеки, флаги сборки проекта, идентификаторы модулей и т.д.
Назначение директорий проекта:

`docs` - для хранения различных файлов, относящихся к документации, например, рисунков и схем.

`lib` - содержит библиотеки проекта: `bxcan`, `cQueue`, `libcanard`, `lwrb` и т.д. Все необходимые библиотеки (свои и сторонние) должны располагаться в этой директории, каждая в своей папке в соответствии с инструкцией в `README`, которая находится в этой же папке. Также здесь находится папка со сгенерироваными заголовочниками типов данных DSDL - `nunavut_out`. В этой директории есть две папки: `uavcan` с базовыми типами данных от разработчика протокола, а также `modrob` с типами данных, которые используются в нашем проекте ModRob (они строятся на базе типов `uavcan`).   

`src` - в этой директории находятся все папки с прошивками: TEST_MODULE, SENSOR_MODULE и т.д. Каждая папка включает в себя свой `main.cpp` (при необходимости `main.hpp`) и набор файлов-драйверов для конкретного вида микроконтроллера, для которого пишется **текущая прошивка**. Эти драйвера могут быть написаны просто как заголовочники, например, `gpio.h` или в паре `driver_xx.h` и `driver_xx.c` (`driver_xx.cpp`). 

`test` - в этой папке располагаются тесты.
## Работа с прошивками модулей
Файлы прошивок находятся в папке `src`. Чтобы создать новую прошивку необходимо выполнить следующие шаги.

Например, необходимо создать прошивку для модуля, управляющего двигателем постоянного тока (мощность до 30 Вт).
1. В директории `src` создаём новую директорию, например `DC30W_MODULE`. Переходим в эту директорию и создаём внутри неё файл `main.cpp`. На этом этапе можно написать минимальный код в этом файле.
2. В файле `platformio.ini` создаём новое окружение для нашей прошивки:
```ini
...
[env:DC30W_MODULE]
platform = ststm32
board = bluepill_f103c8 # прописываем используемую плату
framework = stm32cube   # выбираем фреймворк для работы с контроллером
monitor_speed = 115200  # опционально для выбора скорости Serial
build_flags =
  ${env.build_flags}  # включаем общие настройки для всех модулей
  -D DC_30W      # эквивалентно #define DC_30W
  -D MODULE_ID=1 # уникальный идентификатор модуля (значение до 127!)
src_filter = ${env.src_filter} +<DC30W_MODULE> # собираем только эту директорию
```
3. После этого можно перейти непосредственно к работе с кодом прошивки. Для проверки правильности всех настроек следует попробовать собрать проект. При этом можно сначала собрать уже какую-нибудь готовую прошивку, например `TEST_MODULE`. При это среда заново проиндексирует все зависимости, подключит новое окружение. Затем можно уже собирать новую прошивку. 
4. Для работы с конкретным окружением следует использовать переключение окружений - **Switch PlatformIO Project Environment** (кнопка находится в нижней панели среды VSCode). Следует в раскрывающемся списке выбрать нужное окружение (прошивку). Там же в нижней панели можно выбирать сборку проекта (Build) и загрузку прошивки (Upload).

## Ограничения и замечания

По неизвестной пока причине PlatformIO не индексирует зависимости для файлов других прошивок (окружений), кроме той, что выбрана в данный момент. Поэтому в остальных файлах других окружений везде появляются красные подчёркивания (red squigles), и среда жалуется, что не может найти нужные зависимости.

Большая часть субмодулей (по крайней мере в настоящий момент) строится на базе микроконтроллера STM32F103C8T6 (плата Bluepill). 

**Аппаратные ограничения для Bluepill (Hardware restrictions for Bluepill)**:

Для работы с шиной CAN зарезервированы следующие порты (**их нельзя использовать для других целей!**)
 * CANRX - PB8 (45)
 * CANTX - PB9 (46)

Для отладки контроллеров с помощью интерфейса UART (Serial) на скорости 115200 бод/с зарезервированы эти порты:
 * RX2 - PA3 (13)
 * TX2 - PA2 (12)

**Программные ограничения (Software restrictions)**

 * Канал 7 DMA занят для отправки данных из памяти в буфер передачи (TX) UART.
 * Таймер 2 (timer2) используется как главный счётчик времени контроллера, его нельзя использовать для других целей.

