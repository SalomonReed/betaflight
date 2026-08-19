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

#include "pg/takeoff.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(takeoffConfig_t, takeoffConfig, PG_TAKEOFF_CONFIG, 0);

PG_RESET_TEMPLATE(takeoffConfig_t, takeoffConfig,
    .takeoffAltitudeM = 20,      // 20 meters target altitude
    .pitchAngleDeg = 0,          // No pitch tilt by default
    .takeoffThrottle = 1600,     // Fixed throttle for takeoff (1600 = ~60%)
    .targetHeadingDeg = 0,       // Target heading 0 degrees (North)
    .minHeightM = 5,             // Minimum height 5 meters before rotation
    .yawRate = 30,               // Yaw rotation rate 30 degrees/second
);

#endif // USE_TAKEOFF
