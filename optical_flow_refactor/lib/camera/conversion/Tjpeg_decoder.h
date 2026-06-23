#pragma once
#include <stdint.h>
#include <stddef.h>

/* ใช้ ovcam_err_t ถ้ามี err.h ในโปรเจกต์คุณอยู่แล้ว */
#if __has_include("err.h")
  #include "err.h" /* ควรมี OVCAM_OK / OVCAM_E_FAIL / ovcam_err_t */
#else
  typedef int ovcam_err_t;
  #define OVCAM_OK     0
  #define OVCAM_E_FAIL (-1)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  JPEG_IMAGE_FORMAT_RGB565 = 0,
  JPEG_IMAGE_FORMAT_RGB888 = 1,
} esp_jpeg_image_format_t;

typedef enum {
  JPEG_IMAGE_SCALE_0   = 0,  /* 1:1 */
  JPEG_IMAGE_SCALE_1_2 = 1,  /* 1/2 */
  JPEG_IMAGE_SCALE_1_4 = 2,  /* 1/4 */
  JPEG_IMAGE_SCALE_1_8 = 3,  /* 1/8 */
} esp_jpeg_image_scale_t;

typedef struct {
  const uint8_t *indata;
  size_t         indata_size;

  uint8_t       *outbuf;      /* caller จัดสรร */
  size_t         outbuf_size;

  esp_jpeg_image_format_t out_format; /* RGB565 / RGB888 */
  esp_jpeg_image_scale_t  out_scale;  /* 0/1/2/3 */

  struct {
    uint8_t swap_color_bytes : 1; /* RGB565: สลับ byte ในคำ 16-bit, RGB888: สลับ R<->B */
  } flags;

  struct {
    void  *working_buffer;    /* ไม่บังคับ ถ้าไม่ใส่จะใช้สแตกภายใน */
    size_t working_buffer_size;
  } advanced;
} esp_jpeg_image_cfg_t;

typedef struct {
  uint8_t *output;    /* = cfg->outbuf (คืนเพื่อความสะดวก) */
  size_t   output_len;
  int      width;     /* หลัง scale แล้ว */
  int      height;
} esp_jpeg_image_output_t;

ovcam_err_t esp_jpeg_get_image_info(const esp_jpeg_image_cfg_t *cfg,
                                    esp_jpeg_image_output_t *out);

ovcam_err_t esp_jpeg_decode(const esp_jpeg_image_cfg_t *cfg,
                            esp_jpeg_image_output_t *out);

#ifdef __cplusplus
} /* extern "C" */
#endif
