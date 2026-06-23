
#pragma once
#include <stdarg.h>
#ifdef __cplusplus
extern "C" {
#endif
enum {
  SCCB_LOG_NONE = 0,
  SCCB_LOG_ERROR = 1,
  SCCB_LOG_WARN = 2,
  SCCB_LOG_INFO = 3,
  SCCB_LOG_DEBUG = 4,
  SCCB_LOG_VERBOSE = 5
};
void sccb_log_set_level(int lvl);
int  sccb_log_get_level(void);
void sccb_log_write(int lvl, const char *tag, const char *fmt, ...);
#define SCCB_LOGE(tag, fmt, ...) sccb_log_write(SCCB_LOG_ERROR,   tag, fmt, ##__VA_ARGS__)
#define SCCB_LOGW(tag, fmt, ...) sccb_log_write(SCCB_LOG_WARN,    tag, fmt, ##__VA_ARGS__)
#define SCCB_LOGI(tag, fmt, ...) sccb_log_write(SCCB_LOG_INFO,    tag, fmt, ##__VA_ARGS__)
#define SCCB_LOGD(tag, fmt, ...) sccb_log_write(SCCB_LOG_DEBUG,   tag, fmt, ##__VA_ARGS__)
#define SCCB_LOGV(tag, fmt, ...) sccb_log_write(SCCB_LOG_VERBOSE, tag, fmt, ##__VA_ARGS__)
#ifdef __cplusplus
}
#endif
