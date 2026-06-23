#pragma once


#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"{
#endif

typedef enum {
    OVCAM_OK      = 0,
    OVCAM_E_FAIL   = -1,    // FAIL generic
    OVCAM_E_IO     = -2,    // I/O error
    OVCAM_E_TIMED  = -3,    // timed out
    OVCAM_E_NOMEM  = -4,    // no memory
    OVCAM_E_INVAL  = -5,    // invalid arg
    OVCAM_E_NOENT  = -6,    // not found
    OVCAM_E_BUSY   = -7,    // busy
    OVCAM_E_NOT_SUP = -8,    // unsupported
} ovcam_err_t;

const char* err_str(ovcam_err_t e);

#define OVCAM_OK_OR_RETURN(expr)                     \
    do { ovcam_err_t _e = (ovcam_err_t)(expr); if (_e != OVCAM_OK) return _e; } while (0)

#define HAL_ON_FALSE(cond, code)                   \
    do { if (!(cond)) return (code); } while (0)    

#define ERR_GUARD_PTR(p)   ERR_ON_FALSE((p)!=NULL, ERR_INVALID_ARG)
#define ERR_GUARD_LEN(n)   ERR_ON_FALSE((n)>0,     ERR_INVALID_ARG)

#define ERR_GOTO_ON(expr, label)                   \
    do { ovcam_err_t _e = (ovcam_err_t)(expr); if (_e != OVCAM_OK) { rc = _e; goto label; } } while (0)


ovcam_err_t err_from_esp(esp_err_t e);

#define ERR_MAP_ESP(e) err_from_esp((esp_err_t)(e))


#ifdef __cplusplus
}
#endif


