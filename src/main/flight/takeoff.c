/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_TAKEOFF

#include "math.h"

#include "build/debug.h"
#include "common/maths.h"
#include "config/config.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/imu.h"

#include "rx/rx.h"

#include "pg/takeoff.h"

#include "takeoff.h"

// Takeoff angle offset in centidegrees (used in pid.c)
float takeoffAngle[RP_AXIS_COUNT] = { 0 };

typedef enum {
    TAKEOFF_STATE_IDLE = 0,
    TAKEOFF_STATE_ARMED,
    TAKEOFF_STATE_CLIMBING,
    TAKEOFF_STATE_ROTATING,
    TAKEOFF_STATE_HOLDING
} takeoffState_e;

typedef struct {
    takeoffState_e state;
    float baseAltitudeCm;        // Начальная высота (от которой начинаем подъём)
    float targetAltitudeCm;      // Финальная целевая высота (например, 20м)
    bool throttleRaised;         // Флаг: газ поднят выше 50%
    float currentPitchAngleDeg;  // Текущий угол тангажа (градусы)
    float startHeadingDeg;       // Начальный курс при начале поворота
    float targetHeadingDeg;      // Целевой курс для поворота
    float desiredYawRate;        // Желаемая скорость вращения по yaw (градусы/сек)
    int8_t preferredDirection;   // Предпочтительное направление поворота: 1 = по часовой, -1 = против, 0 = не определено
    bool rotationCompleted;      // Флаг: поворот на целевой курс завершён
} takeoff_t;

static takeoff_t takeoffState;

void takeoffInit(void)
{
    takeoffState.state = TAKEOFF_STATE_IDLE;
    takeoffState.baseAltitudeCm = 0.0f;
    takeoffState.targetAltitudeCm = 0.0f;
    takeoffState.throttleRaised = false;
    takeoffState.currentPitchAngleDeg = 0.0f;
    takeoffState.startHeadingDeg = 0.0f;
    takeoffState.targetHeadingDeg = 0.0f;
    takeoffState.desiredYawRate = 0.0f;
    takeoffState.preferredDirection = 0;
    takeoffState.rotationCompleted = false;
    takeoffAngle[FD_ROLL] = 0;
    takeoffAngle[FD_PITCH] = 0;
}

static void takeoffReset(void)
{
    takeoffState.baseAltitudeCm = getAltitudeCm();
    takeoffState.targetAltitudeCm = getAltitudeCm();
    takeoffState.currentPitchAngleDeg = 0.0f;
    takeoffState.startHeadingDeg = attitude.values.yaw / 10.0f; // Convert from decidegrees to degrees
    takeoffState.targetHeadingDeg = takeoffConfig()->targetHeadingDeg;
    takeoffState.desiredYawRate = 0.0f;
    takeoffState.preferredDirection = 0;
    takeoffState.rotationCompleted = false;
    takeoffAngle[FD_ROLL] = 0;
    takeoffAngle[FD_PITCH] = 0;
}

static void takeoffProcessTransitions(void)
{
    // Проверяем активен ли режим TAKEOFF_MODE
    if (FLIGHT_MODE(TAKEOFF_MODE)) {
        // Проверяем арминг
        if (ARMING_FLAG(ARMED)) {
            if (takeoffState.state == TAKEOFF_STATE_IDLE) {
                takeoffState.state = TAKEOFF_STATE_ARMED;
                takeoffReset();
            }
            
            // Проверяем поднят ли газ выше 50%
            const float rcThrottle = rcCommand[THROTTLE];
            if (rcThrottle > (PWM_RANGE_MIN + PWM_RANGE_MAX) / 2) {
                takeoffState.throttleRaised = true;
            }
            
            // Начинаем подъём если газ поднят и дрон заармален
            if (takeoffState.throttleRaised && takeoffState.state == TAKEOFF_STATE_ARMED) {
                takeoffState.state = TAKEOFF_STATE_CLIMBING;
                // Устанавливаем целевую высоту взлёта (от текущей высоты)
                float targetAlt = takeoffConfig()->takeoffAltitudeM * 100.0f;
                takeoffState.targetAltitudeCm = takeoffState.baseAltitudeCm + targetAlt;
            }
            
            // Проверяем достижение минимальной высоты для начала поворота
            if (takeoffState.state == TAKEOFF_STATE_CLIMBING) {
                const float currentAlt = getAltitudeCm();
                const float minHeightCm = takeoffConfig()->minHeightM * 100.0f;
                const float altitudeAboveBase = currentAlt - takeoffState.baseAltitudeCm;
                
                // Если достигли минимальной высоты, поворот ещё не выполнен и есть целевой курс для поворота
                if (altitudeAboveBase >= minHeightCm && !takeoffState.rotationCompleted && takeoffConfig()->targetHeadingDeg >= 0) {
                    takeoffState.state = TAKEOFF_STATE_ROTATING;
                    takeoffState.startHeadingDeg = attitude.values.yaw / 10.0f;
                    takeoffState.targetHeadingDeg = takeoffConfig()->targetHeadingDeg;
                }
            }
            
            // Проверяем завершение поворота
            if (takeoffState.state == TAKEOFF_STATE_ROTATING) {
                const float currentHeadingDeg = attitude.values.yaw / 10.0f;
                float headingError = takeoffState.targetHeadingDeg - currentHeadingDeg;
                
                // Нормализуем ошибку курса в диапазон [-180, 180]
                if (headingError > 180.0f) {
                    headingError -= 360.0f;
                } else if (headingError < -180.0f) {
                    headingError += 360.0f;
                }
                
                // Если курс достигнут (в пределах 5 градусов)
                if (fabsf(headingError) < 5.0f) {
                    takeoffState.state = TAKEOFF_STATE_CLIMBING;
                    takeoffState.rotationCompleted = true; // Помечаем, что поворот завершён
                }
            }
            
            // Проверяем достижение целевой высоты (только в режиме CLIMBING после поворота)
            if (takeoffState.state == TAKEOFF_STATE_CLIMBING) {
                const float currentAlt = getAltitudeCm();
                if (fabsf(currentAlt - takeoffState.targetAltitudeCm) < 50.0f) { // В пределах 50 см
                    takeoffState.state = TAKEOFF_STATE_HOLDING;
                }
            }
        } else {
            // Дизарм - сбрасываем состояние
            takeoffState.state = TAKEOFF_STATE_IDLE;
            takeoffState.throttleRaised = false;
        }
    } else {
        // TAKEOFF_MODE не активен - сбрасываем состояние
        takeoffState.state = TAKEOFF_STATE_IDLE;
        takeoffState.throttleRaised = false;
    }
}

static void takeoffUpdate(void)
{
    if (takeoffState.state == TAKEOFF_STATE_CLIMBING) {
        // Вычисляем желаемый угол тангажа пропорционально высоте
        float altitudeRange = takeoffState.targetAltitudeCm - takeoffState.baseAltitudeCm;
        float currentAltitude = getAltitudeCm() - takeoffState.baseAltitudeCm;
        float progress = 0.0f;
        
        if (altitudeRange > 0.0f) {
            progress = currentAltitude / altitudeRange;
            progress = constrainf(progress, 0.0f, 1.0f);
        }
        
        // Угол пропорционален прогрессу подъёма
        takeoffState.currentPitchAngleDeg = takeoffConfig()->pitchAngleDeg * progress;
        
        // Устанавливаем угол в сантиградусах для pid.c
        takeoffAngle[FD_PITCH] = takeoffState.currentPitchAngleDeg * 100.0f;
        
        // Сбрасываем yaw rate после завершения поворота
        takeoffState.desiredYawRate = 0.0f;
        
    } else if (takeoffState.state == TAKEOFF_STATE_ROTATING) {
        // Во время поворота pitch остаётся нулевым
        takeoffState.currentPitchAngleDeg = 0.0f;
        takeoffAngle[FD_PITCH] = 0;
        
        // Вычисляем ошибку курса
        const float currentHeadingDeg = attitude.values.yaw / 10.0f;
        float headingError = takeoffState.targetHeadingDeg - currentHeadingDeg;
        
        // Нормализуем ошибку курса в диапазон [-180, 180]
        if (headingError > 180.0f) {
            headingError -= 360.0f;
        } else if (headingError < -180.0f) {
            headingError += 360.0f;
        }
        
        // Отладочная информация
        // DEBUG_SET for heading/yawRate removed — slots 3/4/5 now used by mixer.c/pid.c
        
        // Вычисляем желаемую скорость вращения
        const float yawRate = takeoffConfig()->yawRate;
        
        // В Betaflight:
        // - Положительный yaw rate = поворот ПО часовой стрелке = УМЕНЬШЕНИЕ курса
        // - Отрицательный yaw rate = поворот ПРОТИВ часовой стрелки = УВЕЛИЧЕНИЕ курса
        //
        // Если headingError > 0, нужно УВЕЛИЧИТЬ курс → отрицательный yaw rate
        // Если headingError < 0, нужно УМЕНЬШИТЬ курс → положительный yaw rate
        
        if (headingError > 5.0f) {
            // Нужно увеличить курс (поворот ПРОТИВ часовой стрелки)
            takeoffState.desiredYawRate = -yawRate;
        } else if (headingError < -5.0f) {
            // Нужно уменьшить курс (поворот ПО часовой стрелке)
            takeoffState.desiredYawRate = yawRate;
        } else {
            // Курс достигнут
            takeoffState.desiredYawRate = 0.0f;
        }
        
    } else if (takeoffState.state == TAKEOFF_STATE_HOLDING) {
        // Поддерживаем последний угол тангажа
        takeoffAngle[FD_PITCH] = takeoffState.currentPitchAngleDeg * 100.0f;
    } else {
        // В других состояниях сбрасываем угол и yaw rate
        takeoffState.currentPitchAngleDeg = 0.0f;
        takeoffAngle[FD_PITCH] = 0;
        takeoffState.desiredYawRate = 0.0f;
    }
}

void updateTakeoff(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    
    takeoffProcessTransitions();
    // Отладочный вывод - всегда обновляется независимо от состояния
    // debug[0]: состояние takeoff (0=IDLE, 1=ARMED, 2=CLIMBING, 3=ROTATING, 4=HOLDING)
    // debug[1]: final throttle после mixer adjustment (×1000) — из mixer.c
    // debug[2]: bitmap (bit0=airmode, bit1=takeoffActive, bit2=throttleHigh) — из mixer.c
    // debug[3]: throttle после takeoff override (×100) — из mixer.c
    // debug[4]: pidStabilisationEnabled (1=ON, 0=OFF) — из pid.c
    // debug[5]: zeroThrottleItermReset (1=reset, 0=no) — из pid.c
    // debug[6]: motorMixMax (PID output max, ×1000) — из mixer.c
    // debug[7]: upper limit (1.0 - normalizedMotorMixMax, ×1000) — из mixer.c


    DEBUG_SET(DEBUG_TAKEOFF, 0, takeoffState.state);
    
    if (takeoffState.state == TAKEOFF_STATE_CLIMBING || 
        takeoffState.state == TAKEOFF_STATE_ROTATING || 
        takeoffState.state == TAKEOFF_STATE_HOLDING) {
        takeoffUpdate();
    }
}
bool isTakeoffActive(void)
{
    return takeoffState.state == TAKEOFF_STATE_CLIMBING || 
           takeoffState.state == TAKEOFF_STATE_ROTATING || 
           takeoffState.state == TAKEOFF_STATE_HOLDING;
}

float getTakeoffThrottle(void)
{
    // Возвращаем фиксированный throttle из конфига в диапазоне 0.0-1.0
    return (float)(takeoffConfig()->takeoffThrottle - PWM_RANGE_MIN) / (PWM_RANGE_MAX - PWM_RANGE_MIN);
}

float takeoffGetYawRate(void)
{
    // Возвращаем желаемую скорость вращения по yaw (градусы/сек)
    return takeoffState.desiredYawRate;
}

#endif // USE_TAKEOFF
