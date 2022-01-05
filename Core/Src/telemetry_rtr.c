/* Includes ------------------------------------------------------------------*/
#include "telemetry_rtr.h"
/* Private functions ---------------------------------------------------------*/

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
	uint8_t crc_u, i;
	crc_u = crc;
	crc_u ^= crc_seed;
	for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? ( crc_u << 1 ) ^ 0x07 : ( crc_u << 1 );
	return (crc_u);
}
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
	uint8_t crc = 0, i;
	for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
	return (crc);
}

ParseStatusType telemetryParse(uint8_t recByte, TelemetryType *telemetry, ReceiveStatusTyte *recStatus) {
	uint8_t arrayKissESC[10];
	if ((recByte == 0x9b)) {
		*recStatus = BALE_HEAD;
		telemetry->check_sum = 0;
	}
	switch (*recStatus) {
		case PARSE_IDLE:
			telemetry->check_sum = 0;
			telemetry->parseStatus = PARSE_WIP;
			break;
		case BALE_HEAD:
			telemetry->check_sum = 0;
			telemetry->BaleHead = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = DATA_LENGTH;
			break;
		case DATA_LENGTH:
			if (recByte != 0x16) {
				*recStatus = PARSE_IDLE;
			} else {
				telemetry->DataLength = recByte;
				telemetry->check_sum += recByte;
				telemetry->parseStatus = PARSE_WIP;
				*recStatus = SOFTWARE_VERSION;
			}
			break;
		case SOFTWARE_VERSION:
			if (recByte != 0x01) {
				*recStatus = PARSE_IDLE;
			} else {
				telemetry->SoftwareVersion = recByte;
				telemetry->check_sum += recByte;
				telemetry->parseStatus = PARSE_WIP;
				*recStatus = ACTUAL_DATA;
			}
			break;
		case ACTUAL_DATA:
			telemetry->ActualData = recByte;
			telemetry->check_sum  += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = BALE_NO_WIP;
			break;
		case BALE_NO_WIP:
			telemetry->BaleNo = ((recByte | 0x0000) << 8);
			telemetry->check_sum  += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = BALE_NO_COMPLETE;
			break;
		case BALE_NO_COMPLETE:
			telemetry->BaleNo = ((telemetry->BaleNo) | recByte);
			telemetry->check_sum  += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = RX_THROTTLE_WIP;
			break;
		case RX_THROTTLE_WIP:
			telemetry->RxThrottle = (float)((recByte | 0x0000) << 8);
			telemetry->check_sum  += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = RX_THROTTLE_COMPLETE;
			break;
		case RX_THROTTLE_COMPLETE:
			telemetry->RxThrottle = (float)(((uint16_t)(telemetry->RxThrottle) | recByte)*100/1024.0f);
			telemetry->check_sum  += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = ACTUAL_OUTPUT_WIP;
			break;
		case ACTUAL_OUTPUT_WIP:
			telemetry->ActualOutput = (float)((recByte | 0x0000) << 8);
			telemetry->check_sum  += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = ACTUAL_OUTPUT_COMPLETE;
			break;
		case ACTUAL_OUTPUT_COMPLETE:
			telemetry->ActualOutput = (float)(((uint16_t)(telemetry->ActualOutput) | recByte)*100/1024.0f);
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = ELECTRIC_RPM_WIP;
			break;
		case ELECTRIC_RPM_WIP:
			telemetry->rpm = ((recByte | 0x0000) << 8);
			arrayKissESC[7] = recByte;
			telemetry->bufKissESC[7] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = ELECTRIC_RPM_COMPLETE;
			break;
		case ELECTRIC_RPM_COMPLETE:
			telemetry->rpm = ((telemetry->rpm) | recByte)*10/21;
			arrayKissESC[8] = recByte;
			telemetry->bufKissESC[8] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = BUSBAR_VOLTAGE_WIP;
			break;
		case BUSBAR_VOLTAGE_WIP:
			telemetry->voltage = (float)((recByte | 0x0000) << 8);
			arrayKissESC[1] = recByte;
			telemetry->bufKissESC[1] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = BUSBAR_VOLTAGE_COMPLETE;
			break;
		case BUSBAR_VOLTAGE_COMPLETE:
			telemetry->voltage = (float)(((uint16_t)(telemetry->voltage) | recByte)/10.0f);
			arrayKissESC[2] = recByte;
			telemetry->bufKissESC[2] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = BUSBAR_CURRENT_WIP;
			break;
		case BUSBAR_CURRENT_WIP:
			telemetry->current = (float)((recByte | 0x0000) << 8);
			arrayKissESC[3] = recByte;
			telemetry->bufKissESC[3] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = BUSBAR_CURRENT_COMPLETE;
			break;
		case BUSBAR_CURRENT_COMPLETE:
			telemetry->current = (float)(((int)(telemetry->current) | recByte)/64.0f);
			arrayKissESC[4] = recByte;
			telemetry->bufKissESC[4] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = PHASE_WIRE_WIP;
			break;
		case PHASE_WIRE_WIP:
			telemetry->phase = (float)((recByte | 0x0000) << 8);
			arrayKissESC[5] = recByte;
			telemetry->bufKissESC[5] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = PHASE_WIRE_COMPLETE;
			break;
		case PHASE_WIRE_COMPLETE:
			telemetry->phase = (float)(((int)(telemetry->phase) | recByte)/64.0f);
			arrayKissESC[6] = recByte;
			telemetry->bufKissESC[6] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = MOSFET_TEMP;
			break;
		case MOSFET_TEMP:
			telemetry->mosfet = recByte;
			arrayKissESC[0]=recByte;
			telemetry->bufKissESC[0] = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = CAPACITANCE_TEMP;
			break;
		case CAPACITANCE_TEMP:
			telemetry->capacitance = recByte;
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = STATUS_CODE_WIP;
			break;
		case STATUS_CODE_WIP:
			telemetry->status = ((recByte | 0x0000) << 8);
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = STATUS_CODE_COMPLETE;
			break;
		case STATUS_CODE_COMPLETE:
			telemetry->status = ((telemetry->status) | recByte);
			telemetry->check_sum += recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = VERIFY_CODE_WIP;
			break;
		case VERIFY_CODE_WIP:
			telemetry->verify = recByte;
			telemetry->parseStatus = PARSE_WIP;
			*recStatus = VERIFY_CODE_COMPLETE;
			break;
		case VERIFY_CODE_COMPLETE:
			telemetry->verify = (((telemetry->verify) | ((recByte | 0x0000) << 8)));
			telemetry->parseStatus = PARSE_COMPLETE;
			*recStatus = BALE_HEAD;
			break;
	}
	if (telemetry->parseStatus == PARSE_COMPLETE){
		if ((telemetry->check_sum) == (telemetry->verify)){
			telemetry->parseStatus = PARSE_OK;
			arrayKissESC[9] = get_crc8(arrayKissESC, 9);
			telemetry->bufKissESC[9] = arrayKissESC[9];
			return PARSE_OK;
		} else {
			telemetry->parseStatus = PARSE_ERROR;
			return PARSE_ERROR;
		}
	}
}

