// udp.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool c_udp_open(const char* ip, uint16_t port);
bool c_udp_listen(uint16_t port);
int  c_udp_send(const void* data, int len);
int  c_udp_recv(uint8_t* buf, int cap, uint32_t* from_ip, uint16_t* from_port);
void c_udp_set_recv_timeout_ms(int ms);
void c_udp_close(void);

#ifdef __cplusplus
}
#endif
