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