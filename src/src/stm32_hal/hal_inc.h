#ifndef _HAL_INC__
#define _HAL_INC__

#ifndef USE_HAL_DRIVER
#define USE_HAL_DRIVER
#endif

//#include <cmsis_armcc.h>

#ifdef STM32F0xx
  #include "stm32f0xx.h"
#elif defined(STM32F1xx)
  #include "stm32f1xx.h"
#elif defined(STM32F2xx)
  #include "stm32f2xx.h"
#elif defined(STM32F3xx)
  #include "stm32f3xx.h"
#elif defined(STM32F4xx)
  #include "stm32f4xx.h"
#elif defined(STM32F7xx)
  #include "stm32f7xx.h"
#elif defined(STM32G0xx)
  #include "stm32g0xx.h"
#elif defined(STM32G4xx)
  #include "stm32g4xx.h"
#elif defined(STM32H7xx)
  #include "stm32h7xx.h"
#elif defined(STM32L0xx)
  #include "stm32l0xx.h"
#elif defined(STM32L1xx)
  #include "stm32l1xx.h"
#elif defined(STM32L4xx)
  #include "stm32l4xx.h"
#elif defined(STM32MP1xx)
  #include "stm32mp1xx.h"
#elif defined(STM32WBxx)
  #include "stm32wbxx.h"
#else
  #error "STM32YYxx chip series is not defined in boards.txt."
#endif

#endif /* _HAL_INC__ */
