#ifndef __LOG_H__
#define __LOG_H__

/* Logs */
#include <logger.h>
#include <esp_log.h>

#ifdef UNIQ_LOGGING
extern char *log_tag;
#define DEBUG(format, ...)	ESP_LOGD(log_tag, format, ##__VA_ARGS__)
#define INFO(format, ...)	ESP_LOGI(log_tag, format, ##__VA_ARGS__)
#define WARN(format, ...)	ESP_LOGW(log_tag, format, ##__VA_ARGS__)
#define ERROR(format, ...)	ESP_LOGE(log_tag, format, ##__VA_ARGS__)
#else
#define DEBUG(format, ...)	ESP_LOGD(__func__, format, ##__VA_ARGS__)
#define INFO(format, ...)	ESP_LOGI(__func__, format, ##__VA_ARGS__)
#define WARN(format, ...)	ESP_LOGW(__func__, format, ##__VA_ARGS__)
#define ERROR(format, ...)	ESP_LOGE(__func__, format, ##__VA_ARGS__)
#endif

#endif /* __LOG_H__ */
