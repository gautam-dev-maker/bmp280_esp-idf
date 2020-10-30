#include "logger.h"
#include <stdio.h>


void logger(esp_log_level_t level, const char *TAG, int line, const char *func, const char *fmt, ...)
{   
    char *log_print_buffer;
    log_print_buffer=(char*) malloc (LOG_BUFFER_SIZE);
    memset(log_print_buffer, '\0', LOG_BUFFER_SIZE);
    int count=snprintf(log_print_buffer,LOG_BUFFER_SIZE ,"%s (%s:%d) ", TAG, func, line);

    va_list args;
    va_start(args, fmt);
    int len=strlen(log_print_buffer);
    count+=vsnprintf(&log_print_buffer[len],LOG_BUFFER_SIZE-len,fmt,args);
    if(count>LOG_BUFFER_SIZE){
        logE("Buffer Overflow","Please increase the buffer size");
        log_print_buffer=realloc(log_print_buffer,count);
        vsnprintf(&log_print_buffer[len],count,fmt,args);
    }
    va_end(args);

    
    switch (level)
    {
    case ESP_LOG_ERROR:
        ESP_LOGE("", "%s", log_print_buffer);
        break;
    case ESP_LOG_WARN:
        ESP_LOGW("", "%s", log_print_buffer);
        break;
    case ESP_LOG_INFO:
        ESP_LOGI("", "%s", log_print_buffer);
        break;
    case ESP_LOG_DEBUG:
        ESP_LOGD("", "%s", log_print_buffer);
        break;
    case ESP_LOG_VERBOSE:
        ESP_LOGV("", "%s", log_print_buffer);
        break;
    default:
        break;
    }
    free(log_print_buffer);
}
