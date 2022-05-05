# Multi Software Serial (UART) For STM32

The library work fine for virtualize 6 UART full duplex in baud rate 9600.
All UART work together parallelly!



### Library Dir:

* [Softuart.h](./softuart.h)
* [Softuart.c](./softuart.c)


### Handler & baud rate

The function `SoftUartHandler(void)` must call in interrupt every `0.2*(1/BR)` .

if `BR=9600` then `0.2*(1/9600)=20.8333333 uS` 

*highly recommended set maximum CPU clock for Run Handler faster as possible!*
*you also available for use [lite](https://github.com/liyanboy74/soft-uart-lite) version by limited options for slow MCUs.*

The library don't change any GPIO config!
before using must config TX pins as output and RX pins as input , any TX pin must set to 1 as IDLE and any RX pin must be Pullup.



### Example Timer Config 

*for Baud Rate 9600 :*

| Config                                                | value  |
| :---------------------------------------------------- | :----- |
| Timer Clock                                           | 72 MHz |
| Prescaler (PSC - 16 bits value)                       | 74     |
| Counter Period (AutoReload Register - 16 bits value ) | 19     |
| auto-reload preload                                   | Enable |
| Tim global interrupt                                  | Enable |

*You also can use [Timer frequency calculator](https://github.com/liyanboy74/timer-frequency-calculator).*

### Calling handler

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIMX)
	{
		SoftUartHandler();
	}
}
```



### Config Soft UART

Open `softuart.h` and edit bellow line as you want:

```c
#define 	Number_Of_SoftUarts	 6
#define		SoftUartTxBufferSize	32
#define		SoftUartRxBufferSize	64
```

If `Number_Of_SoftUarts=6` that mean you `SoftUartNumber` is `0,1,2,3,4,5` 

After config timer for config Soft UART use `SoftUartInit`:

```c
SoftUartState_E SoftUartInit(uint8_t SoftUartNumber,GPIO_TypeDef *TxPort,uint16_t TxPin,GPIO_TypeDef *RxPort,uint16_t RxPin);
```



### Using Soft UART

Transmit always is Possible but for Receiving data you must enable listening by calling:

```c
SoftUartState_E SoftUartEnableRx(uint8_t SoftUartNumber);
```

Received data stored in buffer accessible by below functions:

```c
uint8_t 	SoftUartRxAlavailable(uint8_t SoftUartNumber);
SoftUartState_E SoftUartReadRxBuffer(uint8_t SoftUartNumber,uint8_t *Buffer,uint8_t Len);
```

Transmit data:

```c
SoftUartState_E SoftUartPuts(uint8_t SoftUartNumber,uint8_t *Str,uint8_t Len);
void 		SoftUartWaitUntilTxComplate(uint8_t SoftUartNumber);
```


### Example test:
#### Transmit
```c
while(1)
{
	SoftUartPuts(0,(uint8_t *)"Hello",5);
	SoftUartPuts(1,(uint8_t *)"My",2);
	SoftUartPuts(2,(uint8_t *)"Name",4);
	SoftUartPuts(3,(uint8_t *)"Is",2);
	SoftUartPuts(4,(uint8_t *)"Esmaeill",8);
	SoftUartPuts(5,(uint8_t *)"Maarfavi",8);
}
```
![LogicAnalizer](https://user-images.githubusercontent.com/64005694/121798942-836e1380-cc3e-11eb-96bd-faa72cd72c03.jpg)

#### Receive
**Example 1:**

```c
uint8_t getchar(uint8_t SoftUartNumber)
{
    uint8_t ch;
    while(SoftUartRxAlavailable(SoftUartNumber)==0);
    SoftUartReadRxBuffer(SoftUartNumber,&ch,1);
    return ch;
}
```
**Example 2:**

```c
uint8_t Buffer[SIZE],Len;

while(1)
{
	// Read >= 10 Byte Data if Received
	if(Len=SoftUartRxAlavailable(0),Len>=10)
	{
		// Move Received Data To Another Buffer
		if(SoftUartReadRxBuffer(0,Buffer,Len)==SoftUart_OK)
		{
			// Done
		}
	}
}
```

**Full Example 3:**

```c
#include "softuart.h"

...

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIMX)
	{
		SoftUartHandler();
	}
}

uint8_t getchar(uint8_t SoftUartNumber)
{
    uint8_t ch;
    while(SoftUartRxAlavailable(SoftUartNumber)==0);
    SoftUartReadRxBuffer(SoftUartNumber,&ch,1);
    return ch;
}

int main(void)
{
    uint8_t ch;
    
    ...
    
    HAL_TIM_Base_Start_IT(&htimX);
    
    SoftUartInit(0,SU_TX_GPIO_Port,SU_TX_Pin,SU_RX_GPIO_Port,SU_RX_Pin);
    SoftUartEnableRx(0);
    
    ...
    
    while (1)
    {
        ch=getchar(0);
        SoftUartPuts(0,&ch,1);
        //SoftUartWaitUntilTxComplate(0);
        HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    }
}
```

