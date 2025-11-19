#include "rtthread.h"
#include "shell.h"
#include "shell_core.h"

static const char *s_custom_cmd_list[] = {
    "ls",           /* 列出目录 */
    "ps",           /* 查看线程 */
    "free",         /* 查看内存 */
    "version",      /* 查看版本 */
    "list_device",  /* 列出设备 */
    "list_timer",   /* 列出定时器 */
    "list_sem",     /* 列出信号量 */
    "list_mutex",   /* 列出互斥量 */
    "list_event",   /* 列出事件 */
    "list_mailbox", /* 列出邮箱 */
    RT_NULL,
};

#define CMD_COUNT (sizeof(s_custom_cmd_list) / sizeof(s_custom_cmd_list[0]) - 1)

typedef struct encoder_custom_state {
    rt_uint8_t is_active;           /* 是否处于旋钮浏览模式 */
    rt_uint16_t origin_line_curpos; /* 原始光标位置 */
    rt_uint16_t origin_line_position; /* 原始命令长度 */
    char origin_shell_line[FINSH_CMD_SIZE]; /* 原始命令 */
    rt_uint16_t current_index;      /* 当前命令索引 */
    rt_uint16_t total_commands;     /* 总命令数量 */
} encoder_custom_state_t;

static encoder_custom_state_t s_encoder_custom = {
    .is_active = 0,
    .origin_line_curpos = 0,
    .origin_line_position = 0,
    .origin_shell_line = {0},
    .current_index = 0,
    .total_commands = CMD_COUNT
};


void shell_enter_extension_mode(struct finsh_shell *shell) {
    if (!s_encoder_custom.is_active) {
        s_encoder_custom.is_active = 1;
        s_encoder_custom.origin_line_curpos = shell->line_curpos;
        s_encoder_custom.origin_line_position = shell->line_position;
        rt_strncpy(s_encoder_custom.origin_shell_line, shell->line, shell->line_position + 1);
        s_encoder_custom.current_index = 0;
    }
}


/* 显示自定义命令 */
static void encoder_show_custom_command(struct finsh_shell *shell, int direction) {
    /* 根据方向更新索引 */
    if (direction > 0) {
        /* 顺时针 - 下一个命令 */
        if (s_encoder_custom.current_index < s_encoder_custom.total_commands - 1) {
            s_encoder_custom.current_index++;
        } else {
            /* 循环到第一个命令 */
            s_encoder_custom.current_index = 0;
        }
    } else {
        /* 逆时针 - 上一个命令 */
        if (s_encoder_custom.current_index > 0) {
            s_encoder_custom.current_index--;
        } else {
            /* 循环到最后一个命令 */
            s_encoder_custom.current_index = s_encoder_custom.total_commands - 1;
        }
    }
    
    /* 更新命令行显示 */
    const char *cmd = s_custom_cmd_list[s_encoder_custom.current_index];
    rt_strncpy(shell->line, cmd, FINSH_CMD_SIZE - 1);
    shell->line[FINSH_CMD_SIZE - 1] = '\0';  /* 确保字符串终止 */
    shell->line_curpos = shell->line_position = rt_strlen(shell->line);
    
    /* 清除当前行并重新显示提示符和命令 */
    rt_kprintf("\033[2K\r");
    rt_kprintf("%s [CMD %d/%d] %s", 
        FINSH_PROMPT, 
        s_encoder_custom.current_index + 1,
        s_encoder_custom.total_commands,
        shell->line);
    
    /* 如果光标不在行尾，移动光标到正确位置 */
    // if (shell->line_curpos < shell->line_position) {
    //     int i;
    //     for (i = shell->line_curpos; i < shell->line_position; i++) {
    //         rt_kprintf("\b");
    //     }
    // }
}

static void handle_rotary_knob_press(struct finsh_shell *shell) {
    shell_exec(shell);
    shell_exit_extension_mode(shell);
}

int shell_handle_custom_func_key(struct finsh_shell *shell, char key) {
    if (key == '1') /* 顺时针旋转 - ESC [ ? 1 */
    {
        encoder_show_custom_command(shell, 1);
        return 0;
    }
    else if (key == '2') /* 逆时针旋转 - ESC [ ? 2 */
    {
        encoder_show_custom_command(shell, -1);
        return 0;
    }
    else if (key == '3') /* 旋钮按下 - ESC [ ? 3 */
    {
        handle_rotary_knob_press(shell);
        return 0;
    }

    return -1;
}

void shell_exit_extension_mode(struct finsh_shell *shell) {
    if (s_encoder_custom.is_active) {
        s_encoder_custom.is_active = 0;
        
        shell->line_curpos = s_encoder_custom.origin_line_curpos;
        shell->line_position = s_encoder_custom.origin_line_position;
        /* 恢复光标位置 */
        if (shell->line_curpos < shell->line_position) {
            int move_back = shell->line_position - shell->line_curpos;
            for (int i = 0; i < move_back; i++) {
                rt_kprintf("\b");
            }
        }
        rt_strncpy(shell->line, s_encoder_custom.origin_shell_line, shell->line_position + 1);
        rt_kprintf("\033[2K\r");  // 回到行首并清除整行
        rt_kprintf("%s%s", FINSH_PROMPT, shell->line);       /* 显示命令 */
    }
}
