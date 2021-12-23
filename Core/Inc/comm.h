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

#ifndef COMM_H
#define COMM_H

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdbool.h>

#define COMM_BUFFER_LENGTH 12

#define COMM_IDLE 0x00
#define COMM_RESPOND 0x01
#define HEARTBEAT_TIMEOUT 3000 // Timeout after 3s

// CommIO Receive semaphore
extern osSemaphoreId CommIO_Read_SemHandle;

typedef struct
{
	uint32_t	timestamp;
	uint16_t 	polling_period;
	uint16_t 	pwm;
	uint8_t 	func;
} DAQ_input_t;

enum function
{
	READ = 0x01, 		// Master specific, unused on MCU
	WRITE = 0x02, 		// Master specific, unused on MCU
	RESPOND = 0x03, 	// Slave specific
	STOP = 0x04,		// Master specific, unused on MCU
	HEARTBEAT = 0x05	// Master / Slave health check
};

extern DAQ_input_t DAQ_input;

void comm_IO_init(void);
void comm_IO_read_init(void);
void comm_IO_packet_respond_encode(uint8_t func, uint16_t rpm, float airspeed, uint8_t *buf);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
