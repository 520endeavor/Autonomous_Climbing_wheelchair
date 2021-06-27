#ifndef __Datatypedef_H
#define __Datatypedef_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"

	 
typedef union {
volatile 	float floatdata;
volatile	uint8_t chardata[4];
}unionData;

typedef struct {
volatile	uint16_t Add;
volatile	unionData Data;
		
}FloatData;
typedef struct {
volatile		uint16_t ID;
volatile		uint8_t *Message;
}UART1Message_Typedef;
typedef struct {
	char Data;
	uint16_t Add;
}ManualCMD;	 
	 
	 
	 
	 
	 
	 #endif


