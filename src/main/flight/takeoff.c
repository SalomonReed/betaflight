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
    float climbRate;
    bool throttleRaised;
    float throttleOutput;
} takeoff_t;

static takeoff_t takeoffState;

void takeoffInit(void)
{
    takeoffState.state = TAKEOFF_STATE_IDLE;
    takeoffState.targetAltitudeCm = 0.0f;
    takeoffState.currentTargetAltitudeCm = 0.0f;
    takeoffState.maxVelocity = takeoffConfig()->climbRateCmS * 10.0f; // 50 means 500cm/s
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
    // Check if TAKEOFF_MODE is active
    if (FLIGHT_MODE(TAKEOFF_MODE)) {
        // Check if armed
        if (ARMING_FLAG(ARMED)) {
            if (takeoffState.state == TAKEOFF_STATE_IDLE) {
                takeoffState.state = TAKEOFF_STATE_ARMED;
                takeoffReset();
            }
            
            // Check if throttle is raised (above 50%)
            const float rcThrottle = rcCommand[THROTTLE];
            if (rcThrottle > (PWM_RANGE_MIN + PWM_RANGE_MAX) / 2) {
                takeoffState.throttleRaised = true;
            }
            
            // Start climbing if throttle is raised and we're armed
            if (takeoffState.throttleRaised && takeoffState.state == TAKEOFF_STATE_ARMED) {
                takeoffState.state = TAKEOFF_STATE_CLIMBING;
                // Set target altitude to takeoff altitude (from current altitude)
                float currentAlt = getAltitudeCm();
                float targetAlt = takeoffConfig()->takeoffAltitudeM * 100.0f;
                takeoffState.targetAltitudeCm = currentAlt + targetAlt;
                // currentTargetAltitudeCm starts from current altitude and will increase gradually
                takeoffState.currentTargetAltitudeCm = currentAlt;
            }
            
            // Check if we reached target altitude
            if (takeoffState.state == TAKEOFF_STATE_CLIMBING) {
                const float currentAlt = getAltitudeCm();
                if (fabsf(currentAlt - takeoffState.targetAltitudeCm) < 50.0f) { // Within 50cm
                    takeoffState.state = TAKEOFF_STATE_HOLDING;
                }
            }
        } else {
            // Disarmed - reset state
            takeoffState.state = TAKEOFF_STATE_IDLE;
            takeoffState.throttleRaised = false;
        }
    } else {
        // TAKEOFF_MODE not active - reset state
        takeoffState.state = TAKEOFF_STATE_IDLE;
        takeoffState.throttleRaised = false;
    }
}

static void takeoffUpdate(void)
{
    if (takeoffState.state == TAKEOFF_STATE_CLIMBING) {
        // Gradually increase target altitude with velocity limit (like alt_hold)
        float targetVelocity = takeoffState.maxVelocity; // Always climb at max velocity
        
        // Increase currentTargetAltitudeCm gradually
        takeoffState.currentTargetAltitudeCm += targetVelocity * taskIntervalSeconds;
        
        // Don't exceed final target altitude
        if (takeoffState.currentTargetAltitudeCm > takeoffState.targetAltitudeCm) {
            takeoffState.currentTargetAltitudeCm = takeoffState.targetAltitudeCm;
        }
        
        // Use currentTargetAltitudeCm for altitude control
        altitudeControl(takeoffState.currentTargetAltitudeCm, taskIntervalSeconds, targetVelocity);
    } else if (takeoffState.state == TAKEOFF_STATE_HOLDING) {
        // Holding at target altitude
        altitudeControl(takeoffState.targetAltitudeCm, taskIntervalSeconds, 0.0f);
    }
    
    // Сохраняем текущее значение throttle
    takeoffState.throttleOutput = getAutopilotThrottle();
}

void updateTakeoff(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    
    takeoffProcessTransitions();

    // Debug output - always update regardless of state
    // debug[0]: current velocity (cm/s)
    // debug[1]: throttle percentage (0-100)
    // debug[2]: target altitude (m)
    // debug[3]: current target altitude (cm)
    // debug[4]: takeoff state (0=IDLE, 1=ARMED, 2=CLIMBING, 3=HOLDING)
    DEBUG_SET(DEBUG_TAKEOFF, 0, lrintf(getAltitudeDerivative()));
    DEBUG_SET(DEBUG_TAKEOFF, 1, lrintf(takeoffState.throttleOutput * 100.0f));
    DEBUG_SET(DEBUG_TAKEOFF, 2, takeoffConfig()->takeoffAltitudeM);
    DEBUG_SET(DEBUG_TAKEOFF, 3, lrintf(takeoffState.currentTargetAltitudeCm));
    // DEBUG_SET(DEBUG_TAKEOFF, 4, takeoffState.state);
    
    if (takeoffState.state == TAKEOFF_STATE_CLIMBING || takeoffState.state == TAKEOFF_STATE_HOLDING) {
        takeoffUpdate();
    }
}

bool isTakeoffActive(void)
{
    return takeoffState.state == TAKEOFF_STATE_CLIMBING || takeoffState.state == TAKEOFF_STATE_HOLDING;
}

#endif // USE_TAKEOFF
