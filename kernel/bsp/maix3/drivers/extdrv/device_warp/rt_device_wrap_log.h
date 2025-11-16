#ifndef RT_DEVICE_WRAP_LOG_H
#define RT_DEVICE_WRAP_LOG_H


#define DBG_SECTION_NAME    "DEV_WRAP"
#define DBG_LEVEL           DBG_INFO
#include <rtdbg.h>

void rt_device_wrap_printf(const char* format, ...);

#define S_DBG_LOG_HDR(lvl_name, color_n)                    \
    rt_device_wrap_printf("[" lvl_name "/" DBG_SECTION_NAME "] ")
#define S_DBG_LOG_X_END                                     \
    rt_device_wrap_printf("\n")

#define safe_dbg_log_line(lvl, color_n, fmt, ...)                \
    do                                                      \
    {                                                       \
        S_DBG_LOG_HDR(lvl, color_n);                         \
        rt_device_wrap_printf(fmt, ##__VA_ARGS__);                     \
        S_DBG_LOG_X_END;                                     \
    }                                                       \
    while (0)

#undef LOG_D
#if (DBG_LEVEL >= DBG_LOG)
#define LOG_D(fmt, ...)      safe_dbg_log_line("D", 0, fmt, ##__VA_ARGS__)
#else
#define LOG_D(...)
#endif

#undef LOG_I
#if (DBG_LEVEL >= DBG_INFO)
#define LOG_I(fmt, ...)      safe_dbg_log_line("D", 32, fmt, ##__VA_ARGS__)
#else
#define LOG_I(...)
#endif

#undef LOG_W
#if (DBG_LEVEL >= DBG_WARNING)
#define LOG_W(fmt, ...)      safe_dbg_log_line("W", 33, fmt, ##__VA_ARGS__)
#else
#define LOG_W(...)
#endif

#undef LOG_E
#if (DBG_LEVEL >= DBG_ERROR)
#define LOG_E(fmt, ...)      safe_dbg_log_line("E", 31, fmt, ##__VA_ARGS__)
#else
#define LOG_E(...)
#endif

#endif // RT_DEVICE_WRAP_LOG_H