#ifndef _PLATFORM_INTERNAL_H_
#define _PLATFORM_INTERNAL_H_

#include "irq.h"

#ifndef noinline
#define noinline __attribute__((noinline))
#endif
#ifndef __section
#define __section(S) __attribute__((section(S)))
#endif

#define ICACHE_RAM_ATTR
#define DRAM_ATTR __section(".data")
#define DMA_ATTR WORD_ALIGNED_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(32)))

#define _DISABLE_IRQ() irq_disable()
#define _ENABLE_IRQ() irq_enable()
#define _SAVE_IRQ() irq_save()
#define _RESTORE_IRQ(_x) irq_restore(_x)

#endif /* _PLATFORM_INTERNAL_H_ */
