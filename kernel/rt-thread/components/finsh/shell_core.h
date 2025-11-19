#ifndef SHELL_CORE_H
#define SHELL_CORE_H

void shell_exec(struct finsh_shell *shell);

// TOOD It should work in the form of a callback listener.
void shell_enter_extension_mode(struct finsh_shell *shell);

int shell_handle_custom_func_key(struct finsh_shell *shell, char key);

void shell_exit_extension_mode(struct finsh_shell *shell);

#endif // SHELL_CORE_H