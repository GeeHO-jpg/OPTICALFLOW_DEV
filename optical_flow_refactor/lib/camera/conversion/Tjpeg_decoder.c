#include "Tjpeg_decoder.h"
#include "tjpgd.h"   // มาจาก .pio/libdeps/.../TJpg_Decoder/src/tjpgd.h
#include <string.h>
#include <stdint.h>
#include <stddef.h>

/* ---------- ปรับแต่งได้ ---------- */
#ifndef JD_SZBUF
#define JD_SZBUF  512
#endif
#ifndef JD_USE_SCALE
#define JD_USE_SCALE 1
#endif
#ifndef JD_TBLCLIP
#define JD_TBLCLIP 1
#endif
/* 1=ให้ tjpgd ส่ง RGB565 มา (เร็ว/กินแรมน้อย), 0=ส่ง RGB888 */
#ifndef JD_FORMAT
#define JD_FORMAT 1
#endif
/* ---------------------------------- */


typedef struct {
  const uint8_t *ptr; size_t len; size_t off;
} memsrc_t;

typedef struct {
  uint8_t *dst; size_t dst_size;
  int pitch, channels;
  int w, h;
  int swap;           /* swap byte (565) หรือ R/B (888) */
  int want_rgb888;    /* ถ้า JD_FORMAT=565 แต่ผู้ใช้ขอ 888 -> แปลง 565->888 */
} memsink_t;

static memsink_t g_sink; /* ไม่ thread-safe ถ้าจะเรียกพร้อมกันหลาย task ให้ล็อกเอง */

static size_t in_func(JDEC* jd, uint8_t* buf, size_t nbyte) {
  memsrc_t *ms = (memsrc_t*)jd->device;
  if (buf) {
    size_t remain = (ms->off < ms->len) ? (ms->len - ms->off) : 0;
    if (nbyte > remain) nbyte = remain;
    if (nbyte) { memcpy(buf, ms->ptr + ms->off, nbyte); ms->off += nbyte; }
  } else {
    /* skip */
    if (ms->off + nbyte > ms->len) nbyte = ms->len - ms->off;
    ms->off += nbyte;
  }
  return nbyte;
}

static inline void rgb565_to_rgb888(uint16_t p, uint8_t* r, uint8_t* g, uint8_t* b){
  *r = (p >> 8) & 0xF8;
  *g = (p >> 3) & 0xFC;
  *b = (p << 3) & 0xF8;
}

static int out_func(JDEC* jd, void* bitmap, JRECT* rect) {
  int x0 = rect->left,  y0 = rect->top;
  int x1 = rect->right, y1 = rect->bottom;
  int bw = x1 - x0 + 1, bh = y1 - y0 + 1;

#if JD_FORMAT == 1
  /* bitmap เป็น RGB565 */
  uint16_t *src = (uint16_t*)bitmap;
  for (int j=0; j<bh; ++j) {
    uint8_t *dst = g_sink.dst + (size_t)(y0 + j) * g_sink.pitch + x0 * g_sink.channels;
    if (g_sink.want_rgb888) {
      for (int i=0; i<bw; ++i) {
        uint8_t r,g,b; rgb565_to_rgb888(src[i], &r,&g,&b);
        if (g_sink.swap) { *dst++ = b; *dst++ = g; *dst++ = r; }
        else             { *dst++ = r; *dst++ = g; *dst++ = b; }
      }
    } else {
      for (int i=0; i<bw; ++i) {
        uint16_t px = src[i];
        if (g_sink.swap) { *dst++ = (uint8_t)(px & 0xFF); *dst++ = (uint8_t)(px >> 8); }
        else             { *dst++ = (uint8_t)(px >> 8);   *dst++ = (uint8_t)(px & 0xFF); }
      }
    }
    src += bw;
  }
#else
  /* JD_FORMAT=0 => bitmap เป็น RGB888 */
  uint8_t *src = (uint8_t*)bitmap;
  for (int j=0; j<bh; ++j) {
    uint8_t *dst = g_sink.dst + (size_t)(y0 + j) * g_sink.pitch + x0 * g_sink.channels;
    if (g_sink.channels == 3) {
      if (g_sink.swap) {
        for (int i=0;i<bw;i++){ dst[0]=src[2]; dst[1]=src[1]; dst[2]=src[0]; dst+=3; src+=3; }
      } else {
        memcpy(dst, src, (size_t)bw*3); src += bw*3;
      }
    } else {
      for (int i=0;i<bw;i++){
        uint8_t r=src[0], g=src[1], b=src[2]; src+=3;
        uint16_t px = ((r&0xF8)<<8) | ((g&0xFC)<<3) | (b>>3);
        if (g_sink.swap) { *dst++ = (uint8_t)(px & 0xFF); *dst++ = (uint8_t)(px >> 8); }
        else             { *dst++ = (uint8_t)(px >> 8);   *dst++ = (uint8_t)(px & 0xFF); }
      }
    }
  }
#endif
  return 1;
}

/* map scale enum -> divider & code ของ jd_decomp */
static void map_scale(esp_jpeg_image_scale_t s, int *div_out, unsigned int *jd_scale_out){
  int div = 1; unsigned int sc = 0;
  switch (s) {
    case JPEG_IMAGE_SCALE_1_2: div=2; sc=1; break;
    case JPEG_IMAGE_SCALE_1_4: div=4; sc=2; break;
    case JPEG_IMAGE_SCALE_1_8: div=8; sc=3; break;
    default:                   div=1; sc=0; break;
  }
  if (div_out) *div_out = div;
  if (jd_scale_out) *jd_scale_out = sc;
}

ovcam_err_t esp_jpeg_get_image_info(const esp_jpeg_image_cfg_t *cfg,
                                    esp_jpeg_image_output_t *out)
{
  if (!cfg || !cfg->indata || !out) return OVCAM_E_FAIL;

  JDEC jd;
  memsrc_t ms = { cfg->indata, cfg->indata_size, 0 };

  unsigned char tmp_ws[4096];
  void *ws = cfg->advanced.working_buffer ? cfg->advanced.working_buffer : tmp_ws;
  size_t ws_sz = cfg->advanced.working_buffer ? cfg->advanced.working_buffer_size : sizeof(tmp_ws);

  if (jd_prepare(&jd, in_func, ws, (size_t)ws_sz, &ms) != JDR_OK) return OVCAM_E_FAIL;

  int div; map_scale(cfg->out_scale, &div, NULL);
  int sw = ((int)jd.width  + (div-1)) / div;
  int sh = ((int)jd.height + (div-1)) / div;

  out->width  = sw;
  out->height = sh;
  out->output = (uint8_t*)cfg->outbuf;
  out->output_len = (cfg->out_format==JPEG_IMAGE_FORMAT_RGB565)
                    ? (size_t)sw*sh*2 : (size_t)sw*sh*3;
  return OVCAM_OK;
}

ovcam_err_t esp_jpeg_decode(const esp_jpeg_image_cfg_t *cfg,
                            esp_jpeg_image_output_t *out)
{
  if (!cfg || !cfg->indata || !cfg->outbuf || !out) return OVCAM_E_FAIL;

  esp_jpeg_image_output_t info = {0};
  if (esp_jpeg_get_image_info(cfg, &info) != OVCAM_OK) return OVCAM_E_FAIL;
  if (cfg->outbuf_size < info.output_len) return OVCAM_E_FAIL;

  JDEC jd;
  memsrc_t ms = { cfg->indata, cfg->indata_size, 0 };

  unsigned char tmp_ws[4096];
  void *ws = cfg->advanced.working_buffer ? cfg->advanced.working_buffer : tmp_ws;
  size_t ws_sz = cfg->advanced.working_buffer ? cfg->advanced.working_buffer_size : sizeof(tmp_ws);

  if (jd_prepare(&jd, in_func, ws, (size_t)ws_sz, &ms) != JDR_OK) return OVCAM_E_FAIL;

  g_sink.dst        = cfg->outbuf;
  g_sink.dst_size   = cfg->outbuf_size;
  g_sink.w          = (int)jd.width;
  g_sink.h          = (int)jd.height;
  g_sink.channels   = (cfg->out_format==JPEG_IMAGE_FORMAT_RGB565) ? 2 : 3;
  g_sink.pitch      = info.width * g_sink.channels;
  g_sink.swap       = cfg->flags.swap_color_bytes ? 1 : 0;
#if JD_FORMAT == 1
  g_sink.want_rgb888 = (cfg->out_format==JPEG_IMAGE_FORMAT_RGB888);
#else
  g_sink.want_rgb888 = 0;
#endif

  unsigned int jd_scale; map_scale(cfg->out_scale, NULL, &jd_scale);
  if (jd_decomp(&jd, out_func, jd_scale) != JDR_OK) return OVCAM_E_FAIL;

  out->output     = cfg->outbuf;
  out->output_len = info.output_len;
  out->width      = info.width;
  out->height     = info.height;
  return OVCAM_OK;
}
