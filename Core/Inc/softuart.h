#include "main.h"

#define 							Number_Of_SoftUart		6

#define								SoftUartTxBufferSize	32
#define								SoftUartRxBufferSize	64

typedef enum {
	SoftUart_OK,
	SoftUart_Error
}SoftUartState_E;

typedef struct{
	uint8_t					TxBuffer[SoftUartTxBufferSize];
	uint8_t					RxBuffer[SoftUartRxBufferSize];
}SoftUartBuffer_S;

typedef struct {
	__IO uint8_t 			TxNComplated;
	//__IO uint8_t 			RxNComplated;
	
	uint8_t						TxEnable;
	uint8_t						RxEnable;
	
	uint8_t 					TxBitShift,TxBitConter;
	uint8_t 					RxBitShift,RxBitConter;
	
	uint8_t						TxIndex,TxSize;
	uint8_t						RxIndex;//,RxSize;
	
	SoftUartBuffer_S	Buffer;
	
	GPIO_TypeDef  		*TxPort;
	uint16_t 					TxPin;
	
	GPIO_TypeDef  		*RxPort;
	uint16_t 					RxPin;
	
} SoftUart_S;

//Call Every 104.16666666uS or 9600Hz
//But in real 104 us work Fine & Beter of 104.1666us!
void SoftUartHandler(void);

SoftUartState_E SoftUart_Puts(SoftUart_S *SU,uint8_t *Str,uint8_t Len);
void SoftUartEnableRx(uint8_t SoftUartNumber);
void SoftUartDisableRx(uint8_t SoftUartNumber);
void SoftUartInit(uint8_t SoftUartNumber,GPIO_TypeDef *TxPort,uint16_t TxPin,GPIO_TypeDef *RxPort,uint16_t RxPin);
