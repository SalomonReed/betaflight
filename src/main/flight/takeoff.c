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

#include "rx/rx.h"

#include "pg/takeoff.h"

#include "takeoff.h"

// Takeoff angle offset in centidegrees (used in pid.c)
float takeoffAngle[RP_AXIS_COUNT] = { 0 };

typedef enum {
    TAKEOFF_STATE_IDLE = 0,
    TAKEOFF_STATE_ARMED,
    TAKEOFF_STATE_CLIMBING,
    TAKEOFF_STATE_HOLDING
} takeoffState_e;

typedef struct {
    takeoffState_e state;
    float baseAltitudeCm;        // Начальная высота (от которой начинаем подъём)
    float targetAltitudeCm;      // Финальная целевая высота (например, 20м)
    bool throttleRaised;         // Флаг: газ поднят выше 50%
    float currentPitchAngleDeg;  // Текущий угол тангажа (градусы)
} takeoff_t;

static takeoff_t takeoffState;

void takeoffInit(void)
{
    takeoffState.state = TAKEOFF_STATE_IDLE;
    takeoffState.baseAltitudeCm = 0.0f;
    takeoffState.targetAltitudeCm = 0.0f;
    takeoffState.throttleRaised = false;
    takeoffState.currentPitchAngleDeg = 0.0f;
    takeoffAngle[FD_ROLL] = 0;
    takeoffAngle[FD_PITCH] = 0;
}

static void takeoffReset(void)
{
    takeoffState.baseAltitudeCm = getAltitudeCm();
    takeoffState.targetAltitudeCm = getAltitudeCm();
    takeoffState.currentPitchAngleDeg = 0.0f;
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
            
            // Проверяем достижение целевой высоты
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
        
    } else if (takeoffState.state == TAKEOFF_STATE_HOLDING) {
        // Возврат тангажа к нулю
        takeoffState.currentPitchAngleDeg = 0.0f;
        takeoffAngle[FD_PITCH] = 0;
    } else {
        // В других состояниях сбрасываем угол
        takeoffState.currentPitchAngleDeg = 0.0f;
        takeoffAngle[FD_PITCH] = 0;
    }
}

void updateTakeoff(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    
    takeoffProcessTransitions();

    // Отладочный вывод - всегда обновляется независимо от состояния
    // debug[0]: текущая высота (см)
    // debug[1]: целевая высота (см)
    // debug[2]: состояние takeoff (0=IDLE, 1=ARMED, 2=CLIMBING, 3=HOLDING)
    // debug[3]: угол тангажа (градусы * 10)
    // debug[4]: takeoff_throttle из конфига

    DEBUG_SET(DEBUG_TAKEOFF, 0, lrintf(getAltitudeCm()));
    DEBUG_SET(DEBUG_TAKEOFF, 1, lrintf(takeoffState.targetAltitudeCm));
    DEBUG_SET(DEBUG_TAKEOFF, 2, takeoffState.state);
    DEBUG_SET(DEBUG_TAKEOFF, 3, lrintf(takeoffState.currentPitchAngleDeg * 10.0f));
    DEBUG_SET(DEBUG_TAKEOFF, 4, takeoffConfig()->takeoffThrottle);
    
    if (takeoffState.state == TAKEOFF_STATE_CLIMBING || takeoffState.state == TAKEOFF_STATE_HOLDING) {
        takeoffUpdate();
    }
}
bool isTakeoffActive(void)
{
    return takeoffState.state == TAKEOFF_STATE_CLIMBING || takeoffState.state == TAKEOFF_STATE_HOLDING;
}

float getTakeoffThrottle(void)
{
    // Возвращаем фиксированный throttle из конфига в диапазоне 0.0-1.0
    return (float)(takeoffConfig()->takeoffThrottle - PWM_RANGE_MIN) / (PWM_RANGE_MAX - PWM_RANGE_MIN);
}

#endif // USE_TAKEOFF
