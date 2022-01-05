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
#include "ringbuf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  init a RINGBUF object
  * @param  r pointer to a RINGBUF object
  * @param  buf pointer to a byte array
  * @param 	size size of buf
  * @retval 0 if successfull, otherwise failed
  */
int32_t RINGBUF_Init(RINGBUF *r, uint8_t* buf, uint32_t size)
{
	if(r == NULL || buf == NULL || size < 2) return -1;

	r->pt = buf;
	r->head = 0;
	r->tail = 0;
	r->size = size;

	return 0;
}

/**
  * @brief  put a character into ring buffer
  * @param  r pointer to a ringbuf object
  * @param  c character to be put
  * @param 	size size of buf
  * @retval 0 if successfull, otherwise failed
  */
int32_t RINGBUF_Put(RINGBUF *r, uint8_t c)
{
	uint32_t temp;
	temp = r->head;
	temp++;

	if(temp >= r->size)
	{
		temp = 0;
	}
	if(temp == r->tail)
	{
	 	// return -1;		// ring buffer is full
	}

	r->pt[r->head] = c;
	r->head = temp;

	return 0;
}

int32_t RINGBUF_Puts(RINGBUF *r, uint8_t* buf, uint32_t length)
{
	for (uint32_t i = 0; i<length; i++) {
		if (RINGBUF_Put(r, buf[i]) == -1) {
			return -1;
		}
	}

	return 0;
}

/**
  * @brief  get a character from ring buffer
  * @param  r pointer to a ringbuf object
  * @param  c read character
  * @retval 0 if successfull, otherwise failed
  */
int32_t  RINGBUF_Get(RINGBUF *r, uint8_t* c)
{
	if(r->tail == r->head)
	{
		return -1;				// ring buffer is empty, this should be atomic operation
	}

	*c = r->pt[r->tail];
	r->tail++;

	if(r->tail >= r->size)
	{
		r->tail = 0;
	}
	return 0;
}

/**
  * @brief  get number of byte in buffer
  * @param
  * @param
  * @retval
  */
int32_t RINGBUF_GetFill(RINGBUF *r)
{
  if(r->head >= r->tail)
	{
		return (r->head - r->tail);
	}
	else
	{
	   return( r->size - r->tail + r->head);
	}
}

uint8_t RINGBUF_Check(RINGBUF *buf, uint8_t *str, uint32_t t)
{
	COMPARE_TYPE cmp;
	uint8_t c;
	InitFindData(&cmp, str);
	while(t--)
	{
		// RINGBUF_DelayMs();

		if(RINGBUF_Get(buf,&c) == 0)
		{
			if(FindData(&cmp,c) == 0)
			{
				return 0;
			}
		}
	}
	return 0xff;
}

/**
  * @brief  InitFindData
  * @param  add data to compare variable
  * @param  None
  * @retval None
  */
void InitFindData(COMPARE_TYPE *cmpData,uint8_t *data)
{
	cmpData->buff = data;
	cmpData->index = 0;
	cmpData->len = strlen((char *)data);
}

uint8_t FindData(COMPARE_TYPE *cmpData,uint8_t c)
{
	if(cmpData->buff[cmpData->index] == c) cmpData->index++;
	else cmpData->index = 0;

	if(cmpData->index >= cmpData->len) return 0;

	return 0xff;
}

/************************ END OF FILE ****/
