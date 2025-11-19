#include "shell.h"
#include "shell_core.h"
#include "msh.h"

#include <string.h>
#include <stdio.h>

#ifdef FINSH_USING_HISTORY

void shell_push_history(struct finsh_shell *shell)
{
    if (shell->line_position != 0)
    {
        /* push history */
        if (shell->history_count >= FINSH_HISTORY_LINES)
        {
            /* if current cmd is same as last cmd, don't push */
            if (memcmp(&shell->cmd_history[FINSH_HISTORY_LINES - 1], shell->line, FINSH_CMD_SIZE))
            {
                /* move history */
                int index;
                for (index = 0; index < FINSH_HISTORY_LINES - 1; index ++)
                {
                    memcpy(&shell->cmd_history[index][0],
                           &shell->cmd_history[index + 1][0], FINSH_CMD_SIZE);
                }
                memset(&shell->cmd_history[index][0], 0, FINSH_CMD_SIZE);
                memcpy(&shell->cmd_history[index][0], shell->line, shell->line_position);

                /* it's the maximum history */
                shell->history_count = FINSH_HISTORY_LINES;
            }
        }
        else
        {
            /* if current cmd is same as last cmd, don't push */
            if (shell->history_count == 0 || memcmp(&shell->cmd_history[shell->history_count - 1], shell->line, FINSH_CMD_SIZE))
            {
                shell->current_history = shell->history_count;
                memset(&shell->cmd_history[shell->history_count][0], 0, FINSH_CMD_SIZE);
                memcpy(&shell->cmd_history[shell->history_count][0], shell->line, shell->line_position);

                /* increase count and set current history position */
                shell->history_count ++;
            }
        }
    }
    shell->current_history = shell->history_count;
}

#endif // FINSH_USING_HISTORY

void shell_exec(struct finsh_shell *shell) {
#ifdef FINSH_USING_HISTORY
    shell_push_history(shell);
#endif

    if (shell->echo_mode)
        rt_kprintf("\n");
    msh_exec(shell->line, shell->line_position);

    rt_kprintf(FINSH_PROMPT);
#if defined(CHERRY_USB_DEVICE_ENABLE_CLASS_ADB)
    extern int adb_exit(void);
    extern rt_bool_t use_adb_command;

    if (use_adb_command && (shell->line[0] != 0x0)) {
        adb_exit();
        use_adb_command = RT_FALSE;
    }
#endif
    memset(shell->line, 0, sizeof(shell->line));
    shell->line_curpos = shell->line_position = 0;
}

RT_WEAK void shell_enter_extension_mode(struct finsh_shell *shell) {
    rt_kprintf("\r");
    rt_kprintf(FINSH_PROMPT);
    // rt_kprintf("shell_enter_extension_mode");
}

RT_WEAK int shell_handle_custom_func_key(struct finsh_shell *shell, char key) {
    // rt_kprintf("\033[2K\r");
    // rt_kprintf(FINSH_PROMPT);
    rt_kprintf("handle_custom_func_key");
}

RT_WEAK void shell_exit_extension_mode(struct finsh_shell *shell) {
    rt_kprintf("\033[2K\r");
    // rt_kprintf(FINSH_PROMPT);
    // rt_kprintf("shell_exit_extension_mode");
}