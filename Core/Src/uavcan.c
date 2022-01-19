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

#include <stdint.h>
#include <stdio.h>
#include "uavcan.h"
#include "can.h"
#include "stm32f1xx_hal.h"
#include "utils.h"
#include "comm.h"
#include "airspeed.h"

#include "canard.h"
#include "canard_stm32.h"
#include <uavcan/equipment/air_data/RawAirData.h>
#include <uavcan/protocol/GetNodeInfo.h>
#include <uavcan/protocol/NodeStatus.h>

osMutexId Airspeed_MutexHandle;
uavcan_airspeed_data_t uavcan_airspeed_data;

static CanardInstance 	g_canard;                     					//The library instance
static uint8_t 			g_canard_memory_pool[1024];          			//Arena for memory allocation, used by the library
static uint32_t 		spin_uptime = 0;

static void handleGetNodeInfo(CanardRxTransfer* transfer)
{
    uint8_t buf[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    memset(buf, 0, UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE);

    uavcan_protocol_GetNodeInfoResponse msg;

    //Allocate memory for node name
    uint8_t node_name_buf[] = DAQ_NODE_NAME;

    msg.name.data = node_name_buf;
    msg.name.len = strlen(DAQ_NODE_NAME);
    msg.software_version.major = 0;
    msg.software_version.minor = 1;
    msg.software_version.vcs_commit = 1;

    msg.status.uptime_sec = HAL_GetTick() / 1000;
	msg.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	msg.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	msg.status.sub_mode = 0;
	msg.status.vendor_specific_status_code = 0;

	const uint32_t len = uavcan_protocol_GetNodeInfoResponse_encode(&msg, buf);

    (void) canardRequestOrRespond(&g_canard,
								transfer->source_node_id,
								UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
								UAVCAN_PROTOCOL_GETNODEINFO_ID,
								&transfer->transfer_id,
								transfer->priority,
								CanardResponse,
								&buf[0],
								(uint16_t)len);
}

static void handleRawAirData(CanardRxTransfer* transfer)
{
	uint8_t raw_air_data_buff[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE]; //Allocate memory for raw air data buffer
	memset(raw_air_data_buff, 0, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE);

	uint8_t *raw_air_data_dyn_buf_ptr = raw_air_data_buff;

	uavcan_equipment_air_data_RawAirData raw_air_data;

	// Let's ignore any error for now
	// Decode
	(void) uavcan_equipment_air_data_RawAirData_decode(transfer,
													transfer->payload_len,
													&raw_air_data,
													&raw_air_data_dyn_buf_ptr);

	const float airspeed = airspeed_read((&raw_air_data)->static_air_temperature,
										(&raw_air_data)->differential_pressure);

	// Push to uavcan_airspeed_data
	osMutexWait(Airspeed_MutexHandle, osWaitForever);

	uavcan_airspeed_data.airspeed = airspeed;
	uavcan_airspeed_data.timestamp = HAL_GetTick();

	osMutexRelease(Airspeed_MutexHandle);
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if ((transfer->transfer_type == CanardTransferTypeRequest) && (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID)) {
    	handleGetNodeInfo(transfer);
    }

    if (transfer->data_type_id == UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID) {
    	HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    	handleRawAirData(transfer);
    }
}

/*
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if ((transfer_type == CanardTransferTypeRequest) &&(data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID)) {
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    }
    if (data_type_id == UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID) {
        *out_data_type_signature = UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE;
        return true;
    }
    return false;
}

// Initialize the CAN module with UAVCAN parameters
void UAVCAN_init(void)
{
	memset(&uavcan_airspeed_data, 0, sizeof(uavcan_airspeed_data));

	// Create thread mutex
	osMutexDef(Airspeed_Mutex);
	Airspeed_MutexHandle = osMutexCreate(osMutex(Airspeed_Mutex));

	CanardSTM32CANTimings timings;
	int16_t result = canardSTM32ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), 1000000, &timings);

	if (result) {
		Error_Handler();
	}

	result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);

	if (result) {
		Error_Handler();
	}

	canardInit(&g_canard,
			g_canard_memory_pool,
			sizeof(g_canard_memory_pool),
			onTransferReceived,
			shouldAcceptTransfer,
			NULL);

	canardSetLocalNodeID(&g_canard, LOCALNODE_ID);
}

void sendCanard(void)
{
	const CanardCANFrame* txf = canardPeekTxQueue(&g_canard);
	while (txf) {
		const int tx_res = canardSTM32Transmit(txf);
		if (tx_res < 0) {                 // Failure - drop the frame and report
			asm("nop");
		}

		if(tx_res > 0) {
			canardPopTxQueue(&g_canard);
		}

		txf = canardPeekTxQueue(&g_canard);
    }
}

void receiveCanard(void)
{
    CanardCANFrame rx_frame;
    int res = canardSTM32Receive(&rx_frame);
    if(res) {
        canardHandleRxFrame(&g_canard, &rx_frame, get_time_us64());
    }
}

void spinCanard(void)
{
	// 1Hz broadcast
	if (HAL_GetTick() - spin_uptime < UAVCAN_PROTOCOL_NODESTATUS_MAX_BROADCASTING_PERIOD_MS) {
		return;
	}

	spin_uptime = HAL_GetTick();

	uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
	memset(buf, 0, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);

	uavcan_protocol_NodeStatus msg;

	msg.uptime_sec = HAL_GetTick() / 1000;
	msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
	msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
	msg.sub_mode = 0;
	msg.vendor_specific_status_code = 0;

	/* Encode the filled struct into buf, ready to be sent */
	const uint32_t len = uavcan_protocol_NodeStatus_encode(&msg, buf);

	static uint8_t transfer_id = 0;

    canardBroadcast(&g_canard,
    				UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
					UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buf,
					(uint16_t)len);

}
