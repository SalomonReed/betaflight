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

#pragma once

#include "pg/pg.h"

#ifdef USE_TAKEOFF

typedef struct takeoffConfig_s {
    uint16_t takeoffAltitudeM;    // Target altitude in meters (default: 20m)
    uint16_t climbRateCmS;        // Climb rate: 50 means 5 m/s (same format as alt_hold)
    int16_t pitchAngleDeg;        // Target pitch angle in degrees (positive = forward tilt)
    uint16_t takeoffThrottle;     // Fixed throttle for takeoff (1000-2000)
} takeoffConfig_t;

PG_DECLARE(takeoffConfig_t, takeoffConfig);

#endif // USE_TAKEOFF
