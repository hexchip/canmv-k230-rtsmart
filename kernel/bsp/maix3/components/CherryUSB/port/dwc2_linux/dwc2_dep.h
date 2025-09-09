#ifndef __DWC2_DEP_H__
#define __DWC2_DEP_H__

#ifdef __cplusplus
extern "C" {
#endif

#define BITS_PER_LONG (64)

#define DECLARE_BITMAP(name, bits) \
    unsigned long name[(bits + BITS_PER_LONG - 1) / BITS_PER_LONG]

unsigned long bitmap_find_next_zero_area(unsigned long *map, unsigned long size,
                                         unsigned long start, unsigned int nr,
                                         unsigned long align_mask);
void bitmap_set(unsigned long *map, unsigned int start, int len);
void bitmap_clear(unsigned long *map, unsigned int start, int len);

unsigned long gcd(unsigned long a, unsigned long b);

#include <rthw.h>
#include <rtthread.h>

#define atomic_set(ptr, val) (*(volatile typeof(*(ptr)) *)(ptr) = val)
#define atomic_read(ptr) (*(volatile typeof(*(ptr)) *)(ptr))

#ifndef __riscv_atomic
#define atomic_inc(ptr)                                             \
({                                                                  \
    rt_base_t level;                                                \
    level = rt_hw_interrupt_disable();                              \
    (*(volatile typeof(*(ptr)) *)(ptr))++;                          \
    rt_hw_interrupt_enable(level);                                  \
 })

#define atomic_dec(ptr)                                             \
({                                                                  \
    rt_base_t level;                                                \
    level = rt_hw_interrupt_disable();                              \
    (*(volatile typeof(*(ptr)) *)(ptr))--;                          \
    rt_hw_interrupt_enable(level);                                  \
 })

#define atomic_dec_and_test(ptr)                                    \
({                                                                  \
    rt_base_t level;                                                \
    typeof(*(ptr)) old_val;                                         \
    level = rt_hw_interrupt_disable();                              \
    old_val = (*(volatile typeof(*(ptr)) *)(ptr));                  \
    (*(volatile typeof(*(ptr)) *)(ptr))--;                          \
    rt_hw_interrupt_enable(level);                                  \
    (old_val == 1);                                                 \
})

#define atomic_inc_and_test(ptr)                                    \
({                                                                  \
    rt_base_t level;                                                \
    typeof(*(ptr)) old_val;                                         \
    level = rt_hw_interrupt_disable();                              \
    old_val = (*(volatile typeof(*(ptr)) *)(ptr));                  \
    (*(volatile typeof(*(ptr)) *)(ptr))++;                          \
    rt_hw_interrupt_enable(level);                                  \
    (old_val == -1);                                                \
})
#else
#define atomic_add(ptr, inc) __sync_fetch_and_add(ptr, inc)
#define atomic_inc(ptr) atomic_add(ptr, 1)
#define atomic_dec(ptr) atomic_add(ptr, -1)
#define atomic_dec_and_test(ptr) (0 == __sync_sub_and_fetch(ptr, 1))
#define atomic_inc_and_test(ptr) (0 == __sync_add_and_fetch(ptr, 1))
#endif

rt_mp_t rt_mp_create_align(const char *name, rt_size_t block_count, rt_size_t block_size,
                           rt_size_t align);
rt_err_t rt_mp_delete_align(rt_mp_t mp);
void *rt_mp_alloc_align(rt_mp_t mp, rt_int32_t time);
void rt_mp_free_align(struct rt_mempool *mp, void *block);

#ifdef __cplusplus
}
#endif

#endif
