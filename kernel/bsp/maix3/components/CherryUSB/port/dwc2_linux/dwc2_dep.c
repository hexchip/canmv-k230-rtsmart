#include "dwc2_dep.h"

#define BITMAP_INDEX(bit) ((bit) / BITS_PER_LONG)
#define BITMAP_OFFSET(bit) ((bit) % BITS_PER_LONG)

unsigned long bitmap_find_next_zero_area(unsigned long *map,
                                         unsigned long size,
                                         unsigned long start,
                                         unsigned int nr,
                                         unsigned long align_mask) {
    unsigned long bit, index, offset;

    // 对齐起点
    start = (start + align_mask) & ~align_mask;

    while (start + nr <= size) {
        // 找到起始字（unsigned long）和偏移量
        index = BITMAP_INDEX(start);
        offset = BITMAP_OFFSET(start);

        // 检查第一个字中的零位情况
        unsigned long word = map[index] | ((1UL << offset) - 1); // 屏蔽起始前的位
        if (~word == 0) { // 如果当前字全被占用，跳到下一个字
            start = (index + 1) * BITS_PER_LONG;
            continue;
        }

        // 检查从 start 开始的 nr 位是否全为 0
        bit = start;
        while (bit < start + nr) {
            index = BITMAP_INDEX(bit);
            offset = BITMAP_OFFSET(bit);

            // 如果当前字剩余的位不足 nr，取当前字的剩余位数
            unsigned long available_bits = BITS_PER_LONG - offset;
            unsigned long needed_bits = nr - (bit - start);
            unsigned long check_bits = (available_bits < needed_bits) ? available_bits : needed_bits;

            // 创建掩码，用于检查连续零位
            unsigned long mask = (1UL << check_bits) - 1;
            if (map[index] & (mask << offset)) { // 如果当前字中存在非零位
                start = bit + 1;
                start = (start + align_mask) & ~align_mask; // 对齐到下一个合适位置
                break;
            }

            // 当前段满足，继续检查下一段
            bit += check_bits;
        }

        if (bit == start + nr) {
            return start; // 找到符合条件的区域
        }
    }

    return size; // 未找到合适区域
}

// 设置位图中的一段位为 1
void bitmap_set(unsigned long *map, unsigned int start, int len) {
    unsigned long end = start + len;
    unsigned long index, offset;

    while (start < end) {
        index = BITMAP_INDEX(start);
        offset = BITMAP_OFFSET(start);

        // 计算当前字可以设置的位数
        unsigned long bits_to_set = BITS_PER_LONG - offset;
        if (bits_to_set > (end - start)) {
            bits_to_set = end - start;
        }

        // 创建掩码并设置对应位
        unsigned long mask = ((1UL << bits_to_set) - 1) << offset;
        map[index] |= mask;

        start += bits_to_set;
    }
}

// 清除位图中的一段位（置为 0）
void bitmap_clear(unsigned long *map, unsigned int start, int len) {
    unsigned long end = start + len;
    unsigned long index, offset;

    while (start < end) {
        index = BITMAP_INDEX(start);
        offset = BITMAP_OFFSET(start);

        // 计算当前字可以清除的位数
        unsigned long bits_to_clear = BITS_PER_LONG - offset;
        if (bits_to_clear > (end - start)) {
            bits_to_clear = end - start;
        }

        // 创建掩码并清除对应位
        unsigned long mask = ((1UL << bits_to_clear) - 1) << offset;
        map[index] &= ~mask;

        start += bits_to_clear;
    }
}

/**
 * gcd - calculate and return the greatest common divisor of 2 unsigned longs
 * @a: first value
 * @b: second value
 */
unsigned long gcd(unsigned long a, unsigned long b) {

    if (!a || !b)
        return (a | b);

    // 计算 a 和 b 的共同因子 2 的次数
    unsigned long shift = 0;
    while (((a | b) & 1) == 0) { // a 和 b 都是偶数
        a >>= 1;
        b >>= 1;
        shift++;
    }

    // 处理 a 为奇数的情况
    while ((a & 1) == 0) {
        a >>= 1;
    }

    // 主循环
    while (b != 0) {
        // 去掉 b 的因子 2
        while ((b & 1) == 0) {
            b >>= 1;
        }

        // 保证 a > b，若不满足则交换
        if (a > b) {
            unsigned long temp = a;
            a = b;
            b = temp;
        }

        // 用减法代替模运算
        b -= a;
    }

    // 恢复因子 2
    return a << shift;
}

#ifdef RT_USING_MEMPOOL

#ifdef RT_USING_HEAP
/**
 * This function will create a mempool object and allocate the memory pool from
 * heap.
 *
 * @param name the name of memory pool
 * @param block_count the count of blocks in memory pool
 * @param block_size the size for each block
 *
 * @return the created mempool object
 */
rt_mp_t rt_mp_create_align(const char *name, rt_size_t block_count,
                           rt_size_t block_size, rt_size_t align)
{
    rt_uint8_t *block_ptr;
    struct rt_mempool *mp;
    register rt_size_t offset;
    rt_size_t block_size_align;

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* parameter check */
    RT_ASSERT(name != RT_NULL);
    RT_ASSERT(block_count > 0 && block_size > 0);

    /* allocate object */
    mp = (struct rt_mempool *)rt_object_allocate(RT_Object_Class_MemPool, name);
    /* allocate object failed */
    if (mp == RT_NULL)
        return RT_NULL;

    /* initialize memory pool */
    block_size = RT_ALIGN(block_size, RT_ALIGN_SIZE);
    mp->block_size = block_size;
    block_size_align = RT_ALIGN(block_size + sizeof(rt_uint8_t *), align);
    mp->size = block_size_align * block_count;

    /* allocate memory */
    mp->start_address = rt_malloc_align(block_size_align * block_count, align);
    if (mp->start_address == RT_NULL) {
        /* no memory, delete memory pool object */
        rt_object_delete(&(mp->parent));

        return RT_NULL;
    }

    mp->block_total_count = block_count;
    mp->block_free_count  = mp->block_total_count;

    /* initialize suspended thread list */
    rt_list_init(&(mp->suspend_thread));

    /* initialize free block list */
    block_ptr = (rt_uint8_t *)mp->start_address;
    for (offset = 0; offset < mp->block_total_count; offset ++) {
        *(rt_uint8_t **)(block_ptr + (offset * block_size_align) + block_size)
            = block_ptr + (offset + 1) * block_size_align + block_size;
    }

    *(rt_uint8_t **)(block_ptr + ((offset - 1) * block_size_align) + block_size)
        = RT_NULL;

    mp->block_list = block_ptr + block_size;

    return mp;
}
RTM_EXPORT(rt_mp_create_align);

/**
 * This function will delete a memory pool and release the object memory.
 *
 * @param mp the memory pool object
 *
 * @return RT_EOK
 */
rt_err_t rt_mp_delete_align(rt_mp_t mp)
{
    struct rt_thread *thread;
    register rt_ubase_t level;

    RT_DEBUG_NOT_IN_INTERRUPT;

    /* parameter check */
    RT_ASSERT(mp != RT_NULL);
    RT_ASSERT(rt_object_get_type(&mp->parent) == RT_Object_Class_MemPool);
    RT_ASSERT(rt_object_is_systemobject(&mp->parent) == RT_FALSE);

    /* wake up all suspended threads */
    while (!rt_list_isempty(&(mp->suspend_thread)))
    {
        /* disable interrupt */
        level = rt_hw_interrupt_disable();

        /* get next suspend thread */
        thread = rt_list_entry(mp->suspend_thread.next, struct rt_thread, tlist);
        /* set error code to RT_ERROR */
        thread->error = -RT_ERROR;

        /*
         * resume thread
         * In rt_thread_resume function, it will remove current thread from
         * suspend list
         */
        rt_thread_resume(thread);

        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }

    /* release allocated room */
    rt_free_align(mp->start_address);

    /* detach object */
    rt_object_delete(&(mp->parent));

    return RT_EOK;
}
RTM_EXPORT(rt_mp_delete_align);
#endif

/**
 * This function will allocate a block from memory pool
 *
 * @param mp the memory pool object
 * @param time the waiting time
 *
 * @return the allocated memory block or RT_NULL on allocated failed
 */
void *rt_mp_alloc_align(rt_mp_t mp, rt_int32_t time)
{
    rt_uint8_t *block_ptr;
    register rt_base_t level;
    struct rt_thread *thread;
    rt_uint32_t before_sleep = 0;

    /* parameter check */
    RT_ASSERT(mp != RT_NULL);

    /* get current thread */
    thread = rt_thread_self();

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    while (mp->block_free_count == 0)
    {
        /* memory block is unavailable. */
        if (time == 0)
        {
            /* enable interrupt */
            rt_hw_interrupt_enable(level);

            rt_set_errno(-RT_ETIMEOUT);

            return RT_NULL;
        }

        RT_DEBUG_NOT_IN_INTERRUPT;

        thread->error = RT_EOK;

        /* need suspend thread */
        rt_thread_suspend_with_flag(thread, RT_UNINTERRUPTIBLE);
        rt_list_insert_after(&(mp->suspend_thread), &(thread->tlist));

        if (time > 0)
        {
            /* get the start tick of timer */
            before_sleep = rt_tick_get();

            /* init thread timer and start it */
            rt_timer_control(&(thread->thread_timer),
                             RT_TIMER_CTRL_SET_TIME,
                             &time);
            rt_timer_start(&(thread->thread_timer));
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        /* do a schedule */
        rt_schedule();

        if (thread->error != RT_EOK)
            return RT_NULL;

        if (time > 0)
        {
            time -= rt_tick_get() - before_sleep;
            if (time < 0)
                time = 0;
        }
        /* disable interrupt */
        level = rt_hw_interrupt_disable();
    }

    /* memory block is available. decrease the free block counter */
    mp->block_free_count--;

    /* get block from block list */
    block_ptr = mp->block_list;
    RT_ASSERT(block_ptr != RT_NULL);

    /* Setup the next free node. */
    mp->block_list = *(rt_uint8_t **)block_ptr;

    /* point to memory pool */
    *(rt_uint8_t **)(block_ptr) = (rt_uint8_t *)mp;

    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    return (rt_uint8_t *)(block_ptr - mp->block_size);
}
RTM_EXPORT(rt_mp_alloc_align);

/**
 * This function will release a memory block
 *
 * @param block the address of memory block to be released
 */
void rt_mp_free_align(struct rt_mempool *mp, void *block)
{
    rt_uint8_t **block_ptr;
    struct rt_thread *thread;
    register rt_base_t level;

    /* parameter check */
    if (block == RT_NULL) return;

    /* get the control block of pool which the block belongs to */
    block_ptr = (rt_uint8_t **)((rt_uint8_t *)block + mp->block_size);

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    /* increase the free block count */
    mp->block_free_count ++;

    /* link the block into the block list */
    *block_ptr = mp->block_list;
    mp->block_list = (rt_uint8_t *)block_ptr;

    if (!rt_list_isempty(&(mp->suspend_thread)))
    {
        /* get the suspended thread */
        thread = rt_list_entry(mp->suspend_thread.next,
                               struct rt_thread,
                               tlist);

        /* set error */
        thread->error = RT_EOK;

        /* resume thread */
        rt_thread_resume(thread);

        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        /* do a schedule */
        rt_schedule();

        return;
    }

    /* enable interrupt */
    rt_hw_interrupt_enable(level);
}
RTM_EXPORT(rt_mp_free_align);

/**@}*/

#endif



