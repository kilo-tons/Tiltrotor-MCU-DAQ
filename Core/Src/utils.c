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

#include "utils.h"

/*
 * TIM4 is used as a counter generator for microsecond ticks, and is hardcoded for
 * the STM32F103 series at max clock speed (72 MHz). The tick_us is updated
 * at the end of every overflow (tick = 2^16 - 1)
 *
 * Re - configure this function if you port the codes to other STM32 MCUs
 */

// Variable to save old microsecond ticks
static uint64_t tick_us = 0;

uint64_t get_time_usec()
{
	return tick_us + (&htim4)->Instance->CNT;
}

void update_tick_us()
{
	tick_us += 0xFFFF - 1;
}
