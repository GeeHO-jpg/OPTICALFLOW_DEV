
// #if defined(OVCAMSCCB_USE_IDF4) || \
//    (!defined(OVCAMSCCB_USE_IDF5) && (!defined(__has_include) || !__has_include("driver/i2c_master.h")))
#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR < 5

#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#include "../camera/hal/sccb.h"
#include "log.h"
#include "err.h"
#include "../camera/camera_sensor_registry.h"



// #if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
// #include "esp32-hal-log.h"
// #else
// #include "esp_log.h"
// static const char* TAG = "sccb";
// #endif

static const char* TAG = "sccb IDF4.5";

#define LITTLETOBIG(x)          ((x<<8)|(x>>8))

#ifndef OVCAMSCCB_TIMEOUT_MS
#define OVCAMSCCB_TIMEOUT_MS 50
#endif

#ifndef OVCAMSCCB_PORT
#define OVCAMSCCB_PORT 0
#endif


#define SCCB_FREQ               CONFIG_SCCB_CLK_FREQ  /*!< I2C master frequency*/
#define WRITE_BIT               I2C_MASTER_WRITE      /*!< I2C master write */
#define READ_BIT                I2C_MASTER_READ       /*!< I2C master read */
#define ACK_CHECK_EN            0x1                   /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0                   /*!< I2C master will not check ack from slave */
#define ACK_VAL                 0x0                   /*!< I2C ack value */
#define NACK_VAL                0x1                   /*!< I2C nack value */
#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif


static int sccb_i2c_port;
static bool sccb_owns_i2c_port;

int SCCB_Init(int pin_sda, int pin_scl)
{
    // ESP_LOGI(TAG, "pin_sda %d pin_scl %d", pin_sda, pin_scl);
    SCCB_LOGI(TAG, "pin_sda %d pin_scl %d", pin_sda, pin_scl);
    i2c_config_t conf;
    esp_err_t ret;

    memset(&conf, 0, sizeof(i2c_config_t));

    sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
    sccb_owns_i2c_port = true;
    // ESP_LOGI(TAG, "sccb_i2c_port=%d", sccb_i2c_port);
    SCCB_LOGI(TAG, "sccb_i2c_port=%d", sccb_i2c_port);

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = SCCB_FREQ;

    if ((ret =  i2c_param_config(sccb_i2c_port, &conf)) != ESP_OK) {
        return ret;
    }

    return i2c_driver_install(sccb_i2c_port, conf.mode, 0, 0, 0);
}



int SCCB_Use_Port(int i2c_num) { // sccb use an already initialized I2C port
    if (sccb_owns_i2c_port) {
        SCCB_Deinit();
    }
    if (i2c_num < 0 || i2c_num > I2C_NUM_MAX) {
        // return ESP_ERR_INVALID_ARG;
        return OVCAM_E_INVAL;
    }
    sccb_i2c_port = i2c_num;
    // return ESP_OK;
    return OVCAM_OK;
}

int SCCB_Deinit(void)
{
    if (!sccb_owns_i2c_port) {
    // return ESP_OK;
      return OVCAM_OK;
    }
    sccb_owns_i2c_port = false;
    return i2c_driver_delete(sccb_i2c_port);
}



// int SCCB_Probe(uint8_t slv_addr)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, true);
//     i2c_master_stop(cmd);
    // esp_err_t ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, pdMS_TO_TICKS(200));
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// esp_err_t SCCB_Probe(uint8_t slv_addr)
// {
//     const int TMO_MS = 200;   // ลอง 200ms เผื่อบัสอืด
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     if (!cmd) return ESP_ERR_NO_MEM;

//     esp_err_t st;
//     uint8_t addr_wr = (slv_addr << 1) | I2C_MASTER_WRITE; // WRITE_BIT ต้องเป็น 0

//     st = i2c_master_start(cmd);
//     if (st != ESP_OK) { SCCB_LOGE("SCCB", "start err=%s", esp_err_to_name(st)); goto out; }

//     st = i2c_master_write_byte(cmd, addr_wr, true /*ACK_CHECK_EN*/);
//     if (st != ESP_OK) { SCCB_LOGE("SCCB", "addr wr byte err=%s", esp_err_to_name(st)); goto out; }

//     st = i2c_master_stop(cmd);
//     if (st != ESP_OK) { SCCB_LOGE("SCCB", "stop err=%s", esp_err_to_name(st)); goto out; }

//     st = i2c_master_cmd_begin(sccb_i2c_port, cmd, pdMS_TO_TICKS(TMO_MS));
//     if (st != ESP_OK) {
//         SCCB_LOGE("SCCB", "probe 0x%02X -> %s", slv_addr, esp_err_to_name(st));
//     } else {
//         SCCB_LOGD("SCCB", "probe 0x%02X -> OK", slv_addr);
//     }

// out:
//     i2c_cmd_link_delete(cmd);
//     return st;
// }

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data=0;
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) return -1;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        // ESP_LOGE(TAG, "SCCB_Read Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
        SCCB_LOGE(TAG, "SCCB_Read Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return data;
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        // ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
        SCCB_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    uint8_t data=0;
    esp_err_t ret = ESP_FAIL;
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_u8[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_u8[1], ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) return -1;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        // ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
        SCCB_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }
    return data;
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    static uint16_t i = 0;
    esp_err_t ret = ESP_FAIL;
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_u8[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_u8[1], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        // ESP_LOGE(TAG, "W [%04x]=%02x %d fail\n", reg, data, i++);
        SCCB_LOGE(TAG, "W [%04x]=%02x %d fail\n", reg, data, i++);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    uint16_t data = 0;
    uint8_t *data_u8 = (uint8_t *)&data;
    esp_err_t ret = ESP_FAIL;
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_u8[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_u8[1], ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) return -1;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_u8[1], ACK_VAL);
    i2c_master_read_byte(cmd, &data_u8[0], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        // ESP_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
        SCCB_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
    }
    return data;
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    esp_err_t ret = ESP_FAIL;
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    uint16_t data_htons = LITTLETOBIG(data);
    uint8_t *data_u8 = (uint8_t *)&data_htons;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_u8[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_u8[1], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_u8[0], ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_u8[1], ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(sccb_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        // ESP_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
        SCCB_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}

#endif /* idf4 guard */
