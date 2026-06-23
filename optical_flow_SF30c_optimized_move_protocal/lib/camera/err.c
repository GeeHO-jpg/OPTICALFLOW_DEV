#include "err.h"

#if defined(__has_include)
    #if __has_include("esp_err.h")
        #include "esp_err.h"
        #define OVCAM_HAVE_ESP_ERR 1
    #endif
#endif

const char* err_str(ovcam_err_t e){
    switch(e){
        case OVCAM_OK:          return "OK";
        case OVCAM_E_FAIL:      return "FAIL";
        case OVCAM_E_IO:        return "IO";
        case OVCAM_E_TIMED:     return "TIMEOUT";
        case OVCAM_E_NOMEM:     return "NO_MEM";
        case OVCAM_E_INVAL:     return "INVALID_ARG";
        case OVCAM_E_NOENT:     return "NOT_FOUND";
        case OVCAM_E_BUSY:      return "BUSY";
        case OVCAM_E_NOT_SUP:   return "UNSUPPORTED";
        default:                return "UNKNOWN";
    }
}


ovcam_err_t err_from_esp(esp_err_t e)
{
    switch (e) {
        case ESP_OK:                      return OVCAM_OK;
        case ESP_FAIL:                    return OVCAM_E_FAIL;
        case ESP_ERR_NO_MEM:              return OVCAM_E_NOMEM;
        case ESP_ERR_TIMEOUT:             return OVCAM_E_TIMED;
        case ESP_ERR_INVALID_ARG:         return OVCAM_E_INVAL;
        case ESP_ERR_NOT_SUPPORTED:       return OVCAM_E_NOT_SUP;
        case ESP_ERR_NOT_FOUND:           return OVCAM_E_NOENT;
        case ESP_ERR_INVALID_STATE:       return OVCAM_E_BUSY;        // “ยังไม่พร้อม/กำลังยุ่ง”
        case ESP_ERR_INVALID_RESPONSE:    return OVCAM_E_IO;
        
   
        default:
            return (e == ESP_OK) ? OVCAM_OK : OVCAM_E_FAIL;
    }
}
