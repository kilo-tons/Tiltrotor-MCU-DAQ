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


#ifndef UAVCAN_H
#define UAVCAN_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

#define LOCALNODE_ID 			10
#define DAQ_NODE_NAME 			"org.hcmut.tiltrotor_daq"

typedef struct
{
	float airspeed;
	uint32_t timestamp;

} uavcan_airspeed_data_t;

extern osMutexId Airspeed_MutexHandle;
extern uavcan_airspeed_data_t uavcan_airspeed_data;

void UAVCAN_init(void);
void sendCanard(void);
void receiveCanard(void);
void spinCanard(void);


#endif
