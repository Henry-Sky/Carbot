#ifndef _YB_DEBUG_H_
#define _YB_DEBUG_H_


#include <stdio.h>
#include <stdarg.h>


#define YB_DEBUG_ENABLE               (0)
#define YB_PRINTF_MESSAGE             (0)


#if YB_DEBUG_ENABLE
    #if YB_PRINTF_MESSAGE
        #define DEBUG(message, ...)  yb_debug_msg(message, __FILE__, __func__, __LINE__, ##__VA_ARGS__)
    #else
        #define DEBUG(message, ...)  printf(message, ##__VA_ARGS__)
    #endif /* YB_PRINTF_MESSAGE */

#else
    #define DEBUG(message, ...)  
#endif /* YB_DEBUG_ENABLE */



void yb_debug_msg(char *fmt, char *file, const char *func, int line, ...);


#endif /* _YB_DEBUG_H_ */
