#ifndef _STM32_DEF_H_
#define _STM32_DEF_H_


#include "hal_inc.h"

//#ifndef F_CPU
//  #define F_CPU SystemCoreClock
//#endif
#ifdef F_CPU
#undef F_CPU
#endif
#define F_CPU SystemCoreClock

// Here define some compatibility
#ifndef CAN1
  #define CAN1 CAN
#endif

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
  #define WEAK __attribute__ ((weak))
#endif

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
void _Error_Handler(const char *, int);

void hw_init(void);
void SystemClock_Config(void);
void timer_init(void);

void USART_IDLE_IRQ_handler(uint32_t index);
void USARTx_DMA_handler(uint32_t index);
void GPIO_EXTI_IRQHandler(uint16_t pin);

#endif //_STM32_DEF_H_
