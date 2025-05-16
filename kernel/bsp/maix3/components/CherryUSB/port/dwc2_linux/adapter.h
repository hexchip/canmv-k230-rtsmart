#ifndef __DWC2_ADAPTER_H__
#define __DWC2_ADAPTER_H__

#include <rtthread.h>
#include <rthw.h>
#include <ipc/workqueue.h>
#include <riscv_io.h>
#include "debug.h"
#include "dwc2_dep.h"
#include "tick.h"
#include "usbh_core.h"
#include "mmu.h"

#include "cache.h"
#include "page.h"
#include "ioremap.h"

static inline uint64_t perf_get_times(void)
{
    uint64_t cnt;
    __asm__ __volatile__(
                         "rdtime %0"
                         : "=r"(cnt));
    return cnt;
}

static inline void connector_delay_us(uint64_t us)
{
    uint64_t delay = (TIMER_CLK_FREQ / 1000000) * us;
    volatile uint64_t cur_time = perf_get_times();
    while (1) {
        if ((perf_get_times() - cur_time) >= delay)
            break;
    }
}

#define GFP_KERNEL (0x0)

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef rt_size_t size_t;
typedef u64 dma_addr_t;
typedef uint32_t gfp_t;

#ifdef RT_USING_SMP
typedef struct rt_spinlock spinlock_t;
#else
typedef rt_spinlock_t spinlock_t;
#endif

#define udelay connector_delay_us
#define __packed __attribute__((packed))
#define mdelay(ms) udelay((ms) * 1000)

#define msleep rt_thread_mdelay
#define urb usbh_urb
#define usb_hcd usbh_hcd
#define usb_bus usbh_bus
#define container_of usb_container_of
#define msecs_to_jiffies rt_tick_from_millisecond
#define list_head usb_dlist_node
#define kmem_cache rt_mempool

static inline void *rt_mp_calloc(rt_mp_t mp, rt_int32_t time)
{
    void *ptr;

    ptr = rt_mp_alloc(mp, time);
    if (ptr != RT_NULL) {
        rt_memset(ptr, 0, mp->block_size);
    }

    return ptr;
}

#define dma_unmap_single rt_hw_cpu_dcache_invalidate
#define kmem_cache_free rt_mp_free_align
#define kmem_cache_alloc rt_mp_alloc_align
#define dma_sync_single_for_device rt_hw_cpu_dcache_clean
#define dma_sync_single_for_cpu rt_hw_cpu_dcache_invalidate

#define DIV_ROUND_UP __KERNEL_DIV_ROUND_UP
#define __KERNEL_DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define struct_size(p, member, count)   \
    (sizeof(*(p)) + (sizeof(*(p)->member) * count))


#define WARN_ON(condition) ({						\
                            int __ret_warn_on = !!(condition);				\
                            if (__ret_warn_on)					\
                            RT_ASSERT(0);								\
                            (__ret_warn_on);					\
                            })

#define min(x,y) ({ \
                  typeof(x) _x = (x);	\
                  typeof(y) _y = (y);	\
                  (void) (&_x == &_y);	\
                  _x < _y ? _x : _y; })

#define max(x,y) ({ \
                  typeof(x) _x = (x);	\
                  typeof(y) _y = (y);	\
                  (void) (&_x == &_y);	\
                  _x > _y ? _x : _y; })

#define min_t(type, a, b) min(((type) a), ((type) b))
#define max_t(type, a, b) max(((type) a), ((type) b))

static inline struct usb_bus *hcd_to_bus(struct usb_hcd *hcd)
{
    return container_of(hcd, struct usb_bus, hcd);
}

static inline struct usb_hcd *bus_to_hcd(struct usb_bus *bus)
{

    return &bus->hcd;
}

#define USB_RESUME_TIMEOUT (40)
/**
 * enum irqreturn - irqreturn type values
 * @IRQ_NONE:		interrupt was not from this device or was not handled
 * @IRQ_HANDLED:	interrupt was handled by this device
 * @IRQ_WAKE_THREAD:	handler requests to wake the handler thread
 */
enum irqreturn {
    IRQ_NONE		= (0 << 0),
    IRQ_HANDLED		= (1 << 0),
    IRQ_WAKE_THREAD		= (1 << 1),
};

typedef enum irqreturn irqreturn_t;

extern rt_mmu_info mmu_info;
static inline dma_addr_t dma_map_single(void *addr, int size)
{
    dma_addr_t dma = (dma_addr_t)rt_hw_mmu_v2p(&mmu_info, addr);

    rt_hw_cpu_dcache_clean(addr, size);

    return dma;
}


static inline void *dma_alloc_coherent(void *dev, size_t size,
                                       dma_addr_t *dma_handle, gfp_t gfp)
{
    void *va_page, *pa, *va;

    if (size > PAGE_SIZE) {
        dev_err(dev, "too big size\n");
        RT_ASSERT(1);
    }

    va_page = (void *)rt_pages_alloc(0);
    if (va_page) {
        pa = (void *)((char *)va_page + PV_OFFSET);
        va = (void *)rt_ioremap(pa, PAGE_SIZE);
#if 0
        dev_err(dev, "%p %p %p\n", va_page, va, pa);
#endif
        if (va) {
            *dma_handle = (dma_addr_t)pa;
            return va;
        }
    } else {
        dev_err(dev, "alloc page fail\n");
    }

    return NULL;
}

static inline void dma_free_coherent(void *dev, size_t size,
                                     void *vaddr, dma_addr_t dma_handle)
{
    void *va_page, *pa = (void*)dma_handle;

    rt_iounmap(vaddr);
    va_page = (void *)((char *)pa - PV_OFFSET);

#if 0
    dev_err(dev, "%p %p %p\n", va_page, vaddr, pa);
#endif

    rt_pages_free(va_page, 0);

}

static inline void rt_timer_mod(rt_timer_t timer, rt_ubase_t tick)
{
    rt_timer_control(timer, RT_TIMER_CTRL_SET_TIME, &tick);
    rt_timer_start(timer);
}

#endif
