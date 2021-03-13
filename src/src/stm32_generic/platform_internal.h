#ifndef _PLATFORM_INTERNAL_H_
#define _PLATFORM_INTERNAL_H_

#include "irq.h"

#ifndef noinline
#define noinline __attribute__((noinline))
#endif
#ifndef __section
#define __section(S) __attribute__((section(S)))
#endif

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

#if STM32F7xx
#define FAST_CODE       __section(".ram_code")
#else
#define FAST_CODE
#endif

#define ICACHE_RAM_ATTR FAST_CODE
#define DRAM_ATTR //DRAM_FORCE_ATTR
#define DRAM_FORCE_ATTR __section(".data")
#define DMA_ATTR WORD_ALIGNED_ATTR DRAM_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(32)))

#define _DISABLE_IRQ() irq_disable()
#define _ENABLE_IRQ() irq_enable()
#define _SAVE_IRQ() irq_save()
#define _RESTORE_IRQ(_x) irq_restore(_x)

#endif /* _PLATFORM_INTERNAL_H_ */
