#include "yb_debug.h"



void yb_debug_msg(char *fmt, char *file, const char *func, int line, ...)
{
    char buffer[128] = {0};
    va_list arg_list;
    va_start(arg_list, line);
    vsnprintf(buffer, sizeof(buffer), fmt, arg_list);
    printf("[Debug]: %s (file: %s, func: %s, line: %d)\n", buffer, file, func, line);
    va_end(arg_list);
}

