
#include "log.h"
#include <stdio.h>
#ifndef SCCB_LOG_DEFAULT_LEVEL
#define SCCB_LOG_DEFAULT_LEVEL SCCB_LOG_INFO
#endif

static volatile int s_log_level = SCCB_LOG_DEFAULT_LEVEL;
void sccb_log_set_level(int lvl){ s_log_level = lvl; }
int  sccb_log_get_level(void){ return s_log_level; }
static const char* lvl_name(int l){
  switch(l){
    case SCCB_LOG_ERROR: return "E";
    case SCCB_LOG_WARN: return "W";
    case SCCB_LOG_INFO: return "I";
    case SCCB_LOG_DEBUG: return "D";
    case SCCB_LOG_VERBOSE: return "V";
    default: return "-";
  }
}
void sccb_log_write(int lvl, const char *tag, const char *fmt, ...){
  if (lvl > s_log_level) return;
  if (!tag) tag = "SCCB";
  printf("[%s][%s] ", lvl_name(lvl), tag);
  va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
  printf("\n"); fflush(stdout);
}
