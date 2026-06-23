#pragma once
#include <stdint.h>
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t time_between_images;


static inline uint32_t get_time_between_images(void){
    return time_between_images; 
}

static uint64_t get_boot_time_us(void)
{
    return esp_timer_get_time(); // µs since boot
}

#ifdef __cplusplus
}
#endif
