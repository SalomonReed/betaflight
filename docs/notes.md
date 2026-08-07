## Сборка Online прошивки из приложения
make SPEEDYBEEF405V5 EXTRA_FLAGS="-D'BUILD_KEY=c3499492f578f2c16cbca2ae0907d8bf' -D'RELEASE_NAME=2025.12.5' -DCLOUD_BUILD -DUSE_ACRO_TRAINER -DUSE_DSHOT -DUSE_GPS -DUSE_GPS_PLUS_CODES -DUSE_LED_STRIP -DUSE_MAG -DUSE_OPTICALFLOW -DUSE_OSD_HD -DUSE_PINIO -DUSE_RANGEFINDER -DUSE_SERIALRX -DUSE_SERIALRX_CRSF -DUSE_SERVOS -DUSE_TELEMETRY -DUSE_TELEMETRY_CRSF -DUSE_VTX"

## Наша сборка
make SPEEDYBEEF405V5 EXTRA_FLAGS="-D'BUILD_KEY=altynaltunya' -D'RELEASE_NAME=INROEL' -DCLOUD_BUILD -DUSE_ACRO_TRAINER -DUSE_DSHOT -DUSE_GPS -DUSE_GPS_PLUS_CODES -DUSE_LED_STRIP -DUSE_MAG -DUSE_OPTICALFLOW -DUSE_OSD_HD -DUSE_PINIO -DUSE_RANGEFINDER -DUSE_SERIALRX -DUSE_SERIALRX_CRSF -DUSE_SERVOS -DUSE_TELEMETRY -DUSE_TELEMETRY_CRSF -DUSE_VTX"

## Подменить версию:
/src/main/build/version.h

## Барометр
Бетка по умолчанию смешивает показания GPS и барометра в полетнике. GPS даёт калл. Для определения высоты только по барометру ставим:
set altitude_source = BARO_ONLY
save 

При отключенном GPS показывает Давление 1000
Воткнул на горячюю GPS стало 803. Взлетели на 1833 метра
Перезагрузился с GPS: показывает 911
Выдернул на горячую GPS получил 872. Взлетели на 367 метра.

Реальная проблема -- аппаратная. Рекомендации:
1. Ферритовое кольцо на кабель GPS
2. Отдельный BEC для GPS с хорошим фильтром
3. Проверка земли -- убедитесь, что GND GPS и FC соединены надёжно
4. Экранирование -- оберните кабель GPS фольгой

Сканирует шину и втыкает первый попавшийся барометр!!!
# get baro_i2c_address
baro_i2c_address = 0
Allowed range: 0 - 119

# get baro_i2c_device
baro_i2c_device = 1
Allowed range: 0 - 5

## SITL
Документация говно. В разделе SITL из документации написана устаревшая информация 9 летней давности. Вот здесь гайд:
https://betaflight.com/docs/development/autopilot/SITL_Autopilot_Testing_Gazebo
На прошивке 2025.12.5 не получалось сделать арм, нужно было починить barometer.c:
https://github.com/betaflight/betaflight/discussions/13683


Настройка Betaloop
Создайте config.txt файл в указанной betaloopдиректории:

[Betaloop]
AeroloopGazeboHome=/home/orion/work/aeroloop_gazebo
World=betaloop_iris_betaflight_demo_harmonic.sdf
BetaflightElf=/home/orion/work/betaflight/obj/main/betaflight_SITL.elf
DisableWebsockify=True


Порядок запуска сима в разных терминалах:

1) cd ~/work/betaloop/
python3 start.py --gazebo

2) websockify 127.0.0.1:6761 127.0.0.1:5761
3) cd ~/work/betaflight/obj/main
./betaflight_SITL.elf
4) cd ~/work/betaflight/scripts/
./sitl_udp_controller.py


Рекомендации по использованию
Скрипт для удобного запуска
#!/bin/bash
# sitl.sh
pkill -9 -f betaflight_SITL 2>/dev/null
sleep 1
./obj/main/betaflight_SITL.elf "$@" &
sleep 1
echo "SITL started, UART1 on port 5761"
Если порт всё ещё занят
# Проверить кто занимает порт
lsof -i :5761

# Убить процесс занимающий порт
fuser -k 5761/tcp

# Или убить все процессы betaflight
pkill -9 -f betaflight_SITL
Проверка работы
После запуска SITL попробуйте подключиться несколько раз подряд - теперь порт должен освобождаться корректно между подключениями.


В sdf нужно было уменьшить max_step_size до 0.0025 и меньше.
<!-- Physics settings -->
<physics name="realtime" type="ignored">
    <real_time_factor>1</real_time_factor>
    <max_step_size>0.001</max_step_size>
</physics>

┌──────────────────────────────────────────────────────────────────────┐
│                        Ubuntu 24.04 Host                            │
│                                                                      │
│  ┌────────────────────┐       UDP        ┌────────────────────────┐  │
│  │  Betaflight SITL   │◄────────────────►│   Gazebo Harmonic      │  │
│  │  (betaflight_SITL  │  9002: PWM out   │                        │  │
│  │       .elf)        │  9003: FDM in    │  ┌──────────────────┐  │  │
│  │                    │  9004: RC in     │  │  Quadcopter      │  │  │
│  │ ┌────────────────┐ │                  │  │  Model + Plugin  │  │  │
│  │ │ Autopilot      │ │                  │  └──────────────────┘  │  │
│  │ │ - Waypoints    │ │                  │                        │  │
│  │ │ - Pos Control  │ │                  │  ┌──────────────────┐  │  │
│  │ │ - L1 Guidance  │ │                  │  │  World Physics   │  │  │
│  │ │ - Landing      │ │                  │  │  (ODE/Bullet)    │  │  │
│  │ └────────────────┘ │                  │  └──────────────────┘  │  │
│  └──────┬─────────────┘                  └────────────────────────┘  │
│         │ TCP :5761                                                   │
│  ┌──────▼─────────────┐                                              │
│  │  Betaflight App    │                                              │
│  │  (Configurator)    │                                              │
│  └────────────────────┘                                              │
└──────────────────────────────────────────────────────────────────────┘


# TAKEOFF
Для успешной работы Takeoff на реальном дроне нужно настроить следующие параметры:
Основные параметры Takeoff
# Целевая высота взлёта (в метрах)
set takeoff_altitude_m = 2

# Скорость набора высоты (50 = 5 м/с)
# Для первого полёта рекомендую медленнее
set takeoff_climb_rate = 20
Рекомендация: Начните с высоты 2 метра и скорости 2 м/с (значение 20) для безопасного тестирования.
Параметры автопилота высоты
# Hover throttle - базовый газ для удержания (обычно 1500-1600)
# Это значение должно быть примерно равно газу висения
set autopilot_hover_throttle = 1550

# Минимальный и максимальный throttle для altitudeControl
set autopilot_throttle_min = 1100
set autopilot_throttle_max = 1900

# PID коэффициенты для altitudeControl
set autopilot_altitude_p = 15
set autopilot_altitude_i = 10
set autopilot_altitude_d = 8
set autopilot_altitude_f = 10
Настройки Angle mode
Так как Takeoff автоматически включает Angle mode:
# Максимальный угол наклона в Angle mode
set angle_limit = 30

# PID для Angle mode (если ещё не настроены)
set angle_p = 5
set angle_i = 0
set angle_d = 0
Калибровка датчиков
Перед полётом обязательно выполните:
# Калибровка акселерометра (на ровной поверхности)
calibrate acc

# Калибровка барометра (убедитесь что нет сквозняков)
# Барометр калибруется автоматически при включении
Проверка перед полётом
# Проверьте что все датчики работают
status

# Должно показать:
# BARO: работает
# ACC: работает
# GPS: работает (если используется)
Рекомендуемый порядок настройки
1. Настройте hoverThrottle - подберите значение, при котором дрон висит на месте
2. Установите малую высоту - takeoff_altitude_m = 1 для первого теста
3. Установите медленную скорость - takeoff_climb_rate = 15 (1.5 м/с)
4. Протестируйте на земле - убедитесь что моторы реагируют правильно
5. Первый полёт - на высоте 1-2 метра, готовьтесь к аварийной disarm
6. Постепенно увеличивайте - высоту и скорость после успешных тестов
Важные предупреждения
- Всегда имейте готовность к disarm - держите палец на переключателе
- Тестируйте в безветренную погоду - ветер влияет на барометр
- Проверьте работу Angle mode отдельно перед тестом Takeoff
- Убедитесь что барометр не закрыт и защищён от прямого потока воздуха от пропеллеров
Отладка
Включите debug режим для мониторинга:
set debug_mode = TAKEOFF
save
Это покажет в реальном времени:
- Текущую скорость набора высоты
- Процент throttle
- Целевую и текущую высоту
Начните с этих параметров и корректируйте по результатам тестов.

// Отладочный вывод - всегда обновляется независимо от состояния
- debug[0] - текущая вертикальная скорость (см/с)
- debug[1] - целевая скорость (см/с)
- debug[2] - процент throttle (0-100)
- debug[3] - целевая высота (м)
- debug[4] - текущая целевая высота (см)
- debug[5] - состояние takeoff (0=IDLE, 1=ARMED, 2=CLIMBING, 3=HOLDING)
- debug[6] - флаг throttleRaised (0/1) - поднят ли газ выше 50%
- debug[7] - флаг FLIGHT_MODE(TAKEOFF_MODE) (0/1)