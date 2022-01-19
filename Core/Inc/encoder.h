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

#include "main.h"
#include "cmsis_os.h"

typedef struct
{
	uint32_t timestamp;
	uint16_t tick; 		// Number of pulse triggered
	uint16_t rpm;

	// Config parameters
	uint16_t freq; 		// Pre-configured frequency for RPM calculation
	uint16_t period; 	// 1000/freq, in ms
	uint16_t lin_count; // Number of slots on RPM encoder disk

} encoder_rpm_data_t;

extern osMutexId Encoder_MutexHandle;
extern encoder_rpm_data_t encoder_rpm_data;

void encoder_init(encoder_rpm_data_t *input);
void encoder_read_rpm(encoder_rpm_data_t *input);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
