/* Runs cube_firmware/tick_avg.c on the host for tests/test_cube_firmware.py..
 *
 * stdin: one tick per line: the number of FIFO frames n, then 3n int16 values (x, y, z per frame).
 * stdout: one line per message the tick sends, its 20 bytes in hex.
 * The state starts as the shipped image's does: all zero, the message's command byte 2.
 */
#define HOST_TEST
#include "../cube_firmware/tick_avg.c"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FRAMES 128

AccelBlock host_block;
static int16_t fifo[MAX_FRAMES][3];
static int fifo_count, fifo_next;

uint8_t spi_read8(uint8_t address) {
  return address == FIFO_STATUS ? (uint8_t)fifo_count : 0;
}

void spi_read(uint8_t address, int length, uint8_t* data) {
  if (address != FIFO_DATA || length != 6 || fifo_next >= fifo_count) {
    fprintf(stderr, "unexpected read\n");
    exit(3);
  }
  memcpy(data, fifo[fifo_next++], 6);
}

uint32_t udiv(uint32_t n, uint32_t d) {
  if (d == 0) {
    fprintf(stderr, "divide by zero\n");
    exit(3);
  }
  return n / d;
}

void host_ble_send(uint8_t length, const void* data) {
  const uint8_t* bytes = (const uint8_t*)data;
  for (int i = 0; i < length; i++) printf("%02x", bytes[i]);
  printf("\n");
}

int main(void) {
  memset(&host_block, 0, sizeof(host_block));
  host_block.message.command = 2;
  int n;
  for (long line = 0; line < 1000000 && scanf("%d", &n) == 1; line++) {
    if (n < 0 || n > MAX_FRAMES) return 2;
    for (int i = 0; i < n; i++) {
      int x, y, z;
      if (scanf("%d %d %d", &x, &y, &z) != 3) return 2;
      fifo[i][0] = (int16_t)x;
      fifo[i][1] = (int16_t)y;
      fifo[i][2] = (int16_t)z;
    }
    fifo_count = n;
    fifo_next = 0;
    hal_acc_tick();
    if (fifo_next != fifo_count) return 4;
  }
  return 0;
}
