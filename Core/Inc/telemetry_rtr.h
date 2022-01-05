/* Includes ------------------------------------------------------------------*/
#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <float.h>


/* Private variables ---------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	PARSE_OK,
	PARSE_ERROR,
	PARSE_WIP,
	PARSE_COMPLETE
} ParseStatusType;

typedef struct {
	uint8_t BaleHead;
	uint8_t DataLength;
	uint8_t SoftwareVersion;
	uint8_t ActualData;
	uint16_t BaleNo;
	float RxThrottle;
	float ActualOutput;
	uint16_t rpm;
	float voltage;
	float current;
	float phase;
	uint8_t mosfet;
	uint8_t capacitance;
	uint16_t status;
	uint16_t verify;
	uint16_t check_sum;
	uint8_t bufKissESC[10];
	ParseStatusType parseStatus;
} TelemetryType;

typedef enum {
	PARSE_IDLE = 0,
	BALE_HEAD,
	DATA_LENGTH,
	SOFTWARE_VERSION,
	ACTUAL_DATA,
	BALE_NO_WIP,
	BALE_NO_COMPLETE,
	RX_THROTTLE_WIP,
	RX_THROTTLE_COMPLETE,
	ACTUAL_OUTPUT_WIP,
	ACTUAL_OUTPUT_COMPLETE,
	ELECTRIC_RPM_WIP,
	ELECTRIC_RPM_COMPLETE,
	BUSBAR_VOLTAGE_WIP,
	BUSBAR_VOLTAGE_COMPLETE,
	BUSBAR_CURRENT_WIP,
	BUSBAR_CURRENT_COMPLETE,
	PHASE_WIRE_WIP,
	PHASE_WIRE_COMPLETE,
	MOSFET_TEMP,
	CAPACITANCE_TEMP,
	STATUS_CODE_WIP,
	STATUS_CODE_COMPLETE,
	VERIFY_CODE_WIP,
	VERIFY_CODE_COMPLETE
} ReceiveStatusTyte;



/* Private functions ---------------------------------------------------------*/
ParseStatusType telemetryParse(uint8_t recByte, TelemetryType *telemetry, ReceiveStatusTyte *recStatus);
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
#endif

/************************ END OF FILE ****/

