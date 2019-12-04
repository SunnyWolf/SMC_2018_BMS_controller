#include "fifo.h"

TCHARFIFO* FIFO_Init(int size){
	TCHARFIFO* fifo;
	if (size <= 0){
		return NULL;
	}
	fifo = pvPortMalloc(sizeof(TCHARFIFO));
	if (fifo == NULL)
		return NULL;
	fifo->Buffer = pvPortMalloc(size);
	if (fifo->Buffer == NULL){
		vPortFree(fifo);
		return NULL;
	}
	fifo->size = size;
	FIFO_Free(fifo);
	vSemaphoreCreateBinary(fifo->semaphore);
	if (fifo->semaphore == NULL){
		vPortFree(fifo->Buffer);
		vPortFree(fifo);
		return NULL;
	}
	return fifo;
}

void FIFO_Free(TCHARFIFO* fifo){
	fifo->start = 0;
	fifo->end = 0;
}

int FIFO_GetLenth(TCHARFIFO* fifo)
{
	if (fifo == NULL)
		return 0;
  int val;
  val = fifo->end - fifo->start;
  if( val < 0 ) val += fifo->size;
  return val;
}

int FIFO_ReadByte(TCHARFIFO* fifo, uint8_t* out)
{
	if (fifo == NULL)
		return 0;
	if (fifo->Buffer == NULL)
		return 0;
  int ret = FIFO_GetLenth(fifo);
  if( ret > 0 )
  {
    *out = fifo->Buffer[fifo->start];
    fifo->start+=1;
    if( fifo->start >= fifo->size ) 
			fifo->start-=fifo->size;
    return 1;
  }
  return 0;
}

int FIFO_Read(TCHARFIFO* fifo, uint8_t *out, int len)
{
  int i;
  i = FIFO_GetLenth(fifo);
  if( len > i ) len = i;
  for(i=0;i<len;i++) FIFO_ReadByte(fifo, &out[i]);
  
  return len;
}

int FIFO_WriteByte(TCHARFIFO* fifo, uint8_t in)
{
	if (fifo == NULL)
		return 0;
	if (fifo->Buffer == NULL)
		return 0;
  int ret = fifo->size - 1 - FIFO_GetLenth(fifo);
  if(ret > 0)
  {
		fifo->Buffer[fifo->end++]=in;
		if( fifo->end >= fifo->size ) 
			fifo->end -= fifo->size;
  }
  return ret-1;
}

int FIFO_Write(TCHARFIFO* fifo, uint8_t *in, int len)
{
	if (fifo == NULL)
		return 0;
	if (fifo->Buffer == NULL)
		return 0;
	xSemaphoreTake(fifo->semaphore, portMAX_DELAY);
  int i;
  int ret=fifo->size;
  for(i=0; i<len && ret>0; i++){
		ret = FIFO_WriteByte(fifo, in[i]);
	}
  xSemaphoreGive(fifo->semaphore);
	return i;
}

int FIFO_GetFree(TCHARFIFO* fifo)
{
	if (fifo == NULL)
		return 0;
	if (fifo->Buffer == NULL)
		return 0;
  int val;
  val = fifo->end - fifo->start;
  if( val < 0 ) 
		val += fifo->size;
  return fifo->size - 1 - val;
}
