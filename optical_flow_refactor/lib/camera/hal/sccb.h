
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif



int SCCB_Init(int pin_sda, int pin_scl);
int SCCB_Use_Port(int sccb_i2c_port);
int SCCB_Deinit(void);
//int SCCB_Probe(uint8_t slv_addr);
// esp_err_t SCCB_Probe(uint8_t slv_addr);
uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);
uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg);
int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data);
uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg);
int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data);

#ifdef __cplusplus
}
#endif
