
#include "lidar.h"

TFMPlus tfmP;

int16_t tfDist = 0;
int16_t tfFlux = 0;
int16_t tfTemp = 0;

void lidar_init() {
  printf("lidar_init start\n");

  Serial1.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(50);

  bool ok = tfmP.begin(&Serial1);
  printf(ok ? "lidar init DONE" : "lidar init FAIL\n");
}