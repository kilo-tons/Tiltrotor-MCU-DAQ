/*
 * Tiltrotor Propulsion Analyzer DAQ-STM32
 * Copyright (C) 2021 Ho Chi Minh University of Technology, Department of Aerospace Engineering
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "airspeed.h"
#include <math.h>

static float barometer_read(void){
	// Since a barometer is not present, nor we can exactly verify the current
	// elevation of the testbench, assume P = 1 atm
	return 101325.0f; // Pa
}

/* The air density equation on https://www.gribble.org/cycling/air_density.html is
 * quite nice. But since we don't have a relative humidity sensor and a barometer at
 * the time of writing, a simpler formula to calculate density of dry air using
 * Ideal Gas Law is used.
 *
 * Hardware and software improvements in the future is greatly appreciated.
 */

// Calculate airspeed from temperature and delta_P
float airspeed_read(float temperature, float delta_p)
{
	//const float atmospheric_pressure = barometer_read();
	//const float density = atmospheric_pressure / (DRY_AIR_R_SPECIFIC * temperature);
	const float density = 1.225f;

	if (delta_p < 0) {
		delta_p = 0;
	}

	// Since M is obviously smaller than 0.3, apply Bernoulli's Equation
	const float airspeed = sqrtf(2 * fabsf(delta_p) / density);

	return airspeed;
}
