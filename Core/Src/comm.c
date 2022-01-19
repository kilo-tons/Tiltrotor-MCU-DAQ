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

#include <string.h>
#include <math.h>
#include "comm.h"
#include "utils.h"

osSemaphoreId CommIO_Read_SemHandle;

DAQ_input_t DAQ_input;

static uint8_t comm_packet_buff_rx[COMM_BUFFER_LENGTH];

void comm_IO_init(void)
{
	memset(&DAQ_input, 0, sizeof(DAQ_input));
}

void comm_IO_read_init(void)
{
	memset(comm_packet_buff_rx, 0, sizeof(uint8_t) * COMM_BUFFER_LENGTH);
	HAL_UART_Receive_IT(&huart1, comm_packet_buff_rx, COMM_BUFFER_LENGTH);

	// Create semaphore
	osSemaphoreDef(CommIO_Read_Sem);
	CommIO_Read_SemHandle = osSemaphoreCreate(osSemaphore(CommIO_Read_Sem), 1);
}

// Write respond data to buffer
void comm_IO_packet_respond_encode(uint8_t func, uint16_t rpm, float airspeed, uint8_t *buf)
{
	// Only accept RESPOND function
	if (func != RESPOND){
		return;
	}

	// Constrain and convert to u16
	// RPM can be left without constraining

	// Airspeed
	airspeed = round(fabsf(airspeed) * 1000.0f); // mm/s
	const uint16_t airspeed_u16 = (uint16_t)constrain_float(0, 0xFFFF, airspeed);

	// The buffer has a fixed length of 12 bytes (2 headers, 1 function code, 6 payloads, 2 CRCs, 1 tail byte)
	uint8_t temp_buf[] = {0x42, 						//Header "B"
						0x4B,							//Header "K"
						func,							//Function code, should be 0x03 or RESPOND
						(rpm >> 8) & 0xFF,				//RPM MSB
						rpm & 0xFF,						//RPM LSB
						(airspeed_u16 >> 8) & 0xFF,		//Airspeed MSB
						airspeed_u16 & 0xFF,			//Airspeed LSB
						0x00,							//Unused
						0x00,							//Unused
						0x00,							//CRC MSB, will be added later
						0x00,							//CRC LSB, will be added later
						0x4B							//Tail byte "K"
						};

	uint16_t crc = 0xFFFF;
	crc = crc16_ccitt(temp_buf, 9, crc);

	temp_buf[9] = (crc >> 8) & 0xFF; //CRC MSB
	temp_buf[10] = crc & 0xFF; //CRC_LSB

	// Copy to main buffer
	memcpy(buf, temp_buf, sizeof(uint8_t) * COMM_BUFFER_LENGTH);
}

// Write data from master to input struct, return false if invalid data, return true if success
bool comm_IO_packet_decode(uint8_t *buf, DAQ_input_t *input)
{
	// Check for errors
	if (buf[0] != 0x42 && buf[1] != 0x4B && buf[11] != 0x4B) {
		return false;
	}

	const uint16_t crc = buf[9] << 8 | buf[10];
	if (crc != crc16_ccitt(buf, 9, 0xFFFF)) {
		return false;
	}

	const uint8_t func = buf[2];
	input->func = func;

	switch (func) {
	case READ:
		// Write polling period to input
		input->polling_period 	= buf[3] << 8 | buf[4];
		break;

	case WRITE:
		// Write PWM high time period to input
		input->pwm = buf[3] << 8 | buf[4];
		break;

	case STOP:
		input->pwm = 1000 - 1;
		input->polling_period = 0;
		break;

	case HEARTBEAT:
		input->timestamp = HAL_GetTick();
		break;
	}

	return true;
}


/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		// Enable interrupt for another packet
		HAL_UART_Receive_IT(&huart1, comm_packet_buff_rx, COMM_BUFFER_LENGTH);

		if (comm_IO_packet_decode(comm_packet_buff_rx, &DAQ_input)) {
			//Notify main thread
			osSemaphoreRelease(CommIO_Read_SemHandle);
		}
	}
}

