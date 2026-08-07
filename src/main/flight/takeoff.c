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

#include "flight/autopilot.h"
#include "flight/position.h"

#include "rx/rx.h"
#include "pg/takeoff.h"

#include "takeoff.h"

static const float taskIntervalSeconds = HZ_TO_INTERVAL(TAKEOFF_TASK_RATE_HZ);

typedef enum {
    TAKEOFF_STATE_IDLE = 0,
    TAKEOFF_STATE_ARMED,
    TAKEOFF_STATE_CLIMBING,
    TAKEOFF_STATE_HOLDING
} takeoffState_e;

typedef struct {
    takeoffState_e state;
    float targetAltitudeCm;      // Финальная целевая высота (например, 20м)
    float currentTargetAltitudeCm; // Текущая целевая высота (меняется постепенно)
    float maxVelocity;           // Максимальная скорость подъёма (см/с)
    float climbRate;             // Скорость набора высоты из конфигурации
    bool throttleRaised;         // Флаг: газ поднят выше 50%
    float throttleOutput;        // Текущее значение throttle
} takeoff_t;

static takeoff_t takeoffState;

void takeoffInit(void)
{
    takeoffState.state = TAKEOFF_STATE_IDLE;
    takeoffState.targetAltitudeCm = 0.0f;
    takeoffState.currentTargetAltitudeCm = 0.0f;
    takeoffState.maxVelocity = takeoffConfig()->climbRateCmS * 10.0f; // 50 означает 500 см/с
    takeoffState.climbRate = takeoffConfig()->climbRateCmS;
    takeoffState.throttleRaised = false;
}

static void takeoffReset(void)
{
    resetAltitudeControl();
    takeoffState.targetAltitudeCm = getAltitudeCm();
    takeoffState.currentTargetAltitudeCm = getAltitudeCm();
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
                float currentAlt = getAltitudeCm();
                float targetAlt = takeoffConfig()->takeoffAltitudeM * 100.0f;
                takeoffState.targetAltitudeCm = currentAlt + targetAlt;
                // currentTargetAltitudeCm начинается с текущей высоты и будет увеличиваться постепенно
                takeoffState.currentTargetAltitudeCm = currentAlt;
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
        // Постепенно увеличиваем целевую высоту с ограничением скорости (как в alt_hold)
        float targetVelocity = takeoffState.maxVelocity; // Всегда поднимаемся с максимальной скоростью
        
        // Увеличиваем currentTargetAltitudeCm постепенно
        takeoffState.currentTargetAltitudeCm += targetVelocity * taskIntervalSeconds;
        
        // Не превышаем финальную целевую высоту
        if (takeoffState.currentTargetAltitudeCm > takeoffState.targetAltitudeCm) {
            takeoffState.currentTargetAltitudeCm = takeoffState.targetAltitudeCm;
        }
        
        // Используем currentTargetAltitudeCm для управления высотой
        altitudeControl(takeoffState.currentTargetAltitudeCm, taskIntervalSeconds, targetVelocity);
    } else if (takeoffState.state == TAKEOFF_STATE_HOLDING) {
        // Удержание на целевой высоте
        altitudeControl(takeoffState.targetAltitudeCm, taskIntervalSeconds, 0.0f);
    }
    
    // Сохраняем текущее значение throttle
    takeoffState.throttleOutput = getAutopilotThrottle();
}

void updateTakeoff(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    
    takeoffProcessTransitions();

    // Отладочный вывод - всегда обновляется независимо от состояния
    // debug[0]: текущая вертикальная скорость (см/с)
    // debug[1]: целевая скорость (см/с)
    // debug[2]: процент throttle (0-100)
    // debug[3]: целевая высота (м)
    // debug[4]: текущая целевая высота (см)
    // debug[5]: состояние takeoff (0=IDLE, 1=ARMED, 2=CLIMBING, 3=HOLDING)
    // debug[6]: throttleRaised флаг (0/1)
    // debug[7]: FLIGHT_MODE(TAKEOFF_MODE) флаг (0/1)

    DEBUG_SET(DEBUG_TAKEOFF, 0, lrintf(getAltitudeDerivative()));
    DEBUG_SET(DEBUG_TAKEOFF, 1, takeoffConfig()->climbRateCmS);
    DEBUG_SET(DEBUG_TAKEOFF, 2, lrintf(takeoffState.throttleOutput * 100.0f));
    DEBUG_SET(DEBUG_TAKEOFF, 3, takeoffConfig()->takeoffAltitudeM);
    DEBUG_SET(DEBUG_TAKEOFF, 4, lrintf(takeoffState.currentTargetAltitudeCm));
    DEBUG_SET(DEBUG_TAKEOFF, 5, takeoffState.state);
    DEBUG_SET(DEBUG_TAKEOFF, 6, takeoffState.throttleRaised ? 1 : 0);
    DEBUG_SET(DEBUG_TAKEOFF, 7, FLIGHT_MODE(TAKEOFF_MODE) ? 1 : 0);
    
    if (takeoffState.state == TAKEOFF_STATE_CLIMBING || takeoffState.state == TAKEOFF_STATE_HOLDING) {
        takeoffUpdate();
    }
}

bool isTakeoffActive(void)
{
    return takeoffState.state == TAKEOFF_STATE_CLIMBING || takeoffState.state == TAKEOFF_STATE_HOLDING;
}

#endif // USE_TAKEOFF
