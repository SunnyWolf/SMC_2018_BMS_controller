#ifndef _FIFO_H_
#define _FIFO_H_

#ifdef __cplusplus
extern "C" {
#endif
	
#include "cmsis_os.h"
	
typedef struct{
	int start;
	int end;
	int size;
	uint8_t* Buffer;
	xSemaphoreHandle semaphore;
} TCHARFIFO;

TCHARFIFO* FIFO_Init(int size);
void FIFO_Free(TCHARFIFO* fifo);
int FIFO_GetLenth(TCHARFIFO* fifo);
int FIFO_ReadByte(TCHARFIFO* fifo, uint8_t* out);
int FIFO_Read(TCHARFIFO* fifo, uint8_t *out, int len);
int FIFO_WriteByte(TCHARFIFO* fifo, uint8_t in);
int FIFO_Write(TCHARFIFO* fifo, uint8_t *in, int len);

#ifdef __cplusplus
}
#endif

#endif
