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

#ifndef AIRSPEED_H
#define AIRSPEED_H

#define DRY_AIR_R_SPECIFIC 287.058f // J/kg.K

float airspeed_read(float temperature, float delta_p);

#endif
