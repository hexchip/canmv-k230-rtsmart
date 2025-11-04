#ifndef TERMINAL_H
#define TERMINAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

struct terminal_context;

typedef struct terminal_lifecycle {
    void (*on_line_update)(size_t index, char *buffer, size_t size);
    void (*on_refresh_dirty_lines)(size_t *dirty_line_index_array, size_t size);
    void (*on_cursor_move)(size_t row, size_t col);
} terminal_lifecycle_t;

typedef struct terminal {
    struct terminal_context *context;
    char *name;
    terminal_lifecycle_t *lifecycle;
    size_t rows;
    size_t cols;
    size_t thread_stack_size;
    int thread_priority;
} terminal_t;

terminal_t * terminal_create(char *name, 
                                size_t rows, size_t cols, 
                                terminal_lifecycle_t *lifecycle, 
                                size_t thread_stack_size, int thread_priority);

int terminal_init(terminal_t *terminal);

void terminal_deinit(terminal_t *terminal);

void terminal_destroy(terminal_t *terminal);

void terminal_debug_printf(terminal_t *terminal, const char * format, ...);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif // TERMINAL_H