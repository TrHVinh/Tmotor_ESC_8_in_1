/**
  ******************************************************************************
  * @file
  * @author
  * @version
  * @date
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t size;
    volatile uint8_t *pt;
} RINGBUF;

typedef struct {
	uint8_t *buff;
	uint8_t index;
	uint8_t len;
} COMPARE_TYPE;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int32_t RINGBUF_Init(RINGBUF *r, uint8_t* buf, uint32_t size);
int32_t RINGBUF_Put(RINGBUF *r, uint8_t c);
int32_t RINGBUF_Puts(RINGBUF *r, uint8_t* buf, uint32_t length);
int32_t RINGBUF_Get(RINGBUF *r, uint8_t* c);
int32_t RINGBUF_GetFill(RINGBUF *r);
uint8_t RINGBUF_Check(RINGBUF *buf, uint8_t *str, uint32_t t);
void RINGBUF_DelayMs(void);
void InitFindData(COMPARE_TYPE *cmpData, uint8_t *data);
uint8_t FindData(COMPARE_TYPE *cmpData,uint8_t c);


#endif

/************************ END OF FILE ****/
