// udp.c
#include "udp.h"
#include <string.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>

static int udp_sock = -1;
static struct sockaddr_in udp_peer;

bool c_udp_open(const char* ip, uint16_t port) {
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) return false;
    memset(&udp_peer, 0, sizeof(udp_peer));
    udp_peer.sin_family = AF_INET;
    udp_peer.sin_port   = htons(port);
    if (!inet_aton(ip, &udp_peer.sin_addr)) { c_udp_close(); return false; }
    return true;
}

bool c_udp_listen(uint16_t port) {
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) return false;
    int yes = 1; setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in me; memset(&me,0,sizeof(me));
    me.sin_family = AF_INET; me.sin_port = htons(port); me.sin_addr.s_addr = INADDR_ANY;
    if (bind(udp_sock, (struct sockaddr*)&me, sizeof(me)) < 0) { c_udp_close(); return false; }
    return true;
}

int c_udp_send(const void* data, int len) {
    if (udp_sock < 0 || len < 0) return -1;
    int n = (int)sendto(udp_sock, (const char*)data, len, 0,
                        (const struct sockaddr*)&udp_peer, sizeof(udp_peer));
    return (n < 0) ? -1 : n;
}

int c_udp_recv(uint8_t* buf, int cap, uint32_t* from_ip, uint16_t* from_port) {
    if (udp_sock < 0 || !buf || cap <= 0) return -1;
    struct sockaddr_in src; socklen_t sl = sizeof(src);
    int n = (int)recvfrom(udp_sock, (char*)buf, cap, 0, (struct sockaddr*)&src, &sl);
    if (n < 0) return -1;
    if (from_ip)   *from_ip   = src.sin_addr.s_addr;      // network order
    if (from_port) *from_port = ntohs(src.sin_port);      // host order
    return n;
}

void c_udp_set_recv_timeout_ms(int ms) {
    struct timeval tv;
    tv.tv_sec  = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    if (udp_sock >= 0) setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

void c_udp_close(void) {
    if (udp_sock >= 0) { close(udp_sock); udp_sock = -1; }
}
