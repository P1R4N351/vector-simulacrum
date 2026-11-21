/* The light cube app's accelerometer tick, keeping the mean of each tick's FIFO frames instead of the last one.
 *.
 *
 * This is the same tick as tick_avg.s, in C, for reading and for the host test. It is not what is flashed:
 * GCC 16.1 makes 256 bytes of it at -Os, and the shipped tick's place holds 204, so tick_avg.s is written by
 * hand. tests/test_cube_firmware.py runs both against simulacrum/cube.py's "mean" app.
 *
 * Why: the shipped tick (robot/cube_firmware/app/accel.c, hal_acc_tick) reads every frame in the chip's FIFO
 * into one slot and sends whichever came last, one sample per 10 ms tick with no filter in front. Vibration
 * between 50 and 250 Hz then lands in the samples. The mean of the tick's frames (about five, 2 ms apart) is a
 * 10 ms boxcar: it has zeros at 100 Hz and its multiples, the frequencies that alias onto a steady reading.
 *
 * The shipped app's state and routines it uses are listed in tick_avg.s. The message keeps its layout, so the
 * robot reads it unchanged, and the tap detector is the shipped one, compare for compare. The mean is of the
 * 12-bit values (the four low bits "may have any value", BMA253DS:841): each sum starts at 4096 n so it is
 * never negative, and (sum x 16 + n / 2) / n, unsigned, is 16 x the mean (halves rounded up) + 65536, whose
 * low 16 bits are the mean on the chip's left-justified scale.
 */
#include <stdint.h>

#define FIFO_STATUS 0x0E
#define FIFO_DATA 0x3F
#define FRAME_MASK 0x7F
#define FRAMES_PER_MESSAGE 3
#define TAP_DEBOUNCE 90
#define TAP_DURATION 10
#define TAP_THRESH (90 * 32)

typedef struct {
  uint8_t debounce, tap_time, pos_first, unused0;
  int16_t last, tap_neg, tap_pos, unused1;
  int32_t frame;
} TickState;

typedef struct {
  uint8_t command, tap_count;
  int16_t axis[FRAMES_PER_MESSAGE][3];
} AccelMessage;

typedef struct {
  TickState state;     /* 0x20006430 */
  AccelMessage message; /* 0x20006440 */
} AccelBlock;

_Static_assert(sizeof(TickState) == 16, "tick state must match the shipped layout");
_Static_assert(sizeof(AccelMessage) == 20, "the message must match protocol.h");

/* The shipped app's routines: build.py's linker script places these names at their Thumb addresses, and the
 * host test's harness defines them. */
void spi_read(uint8_t address, int length, uint8_t* data);
uint8_t spi_read8(uint8_t address);
uint32_t udiv(uint32_t n, uint32_t d);

#ifdef HOST_TEST
extern AccelBlock host_block;
void host_ble_send(uint8_t length, const void* data);
#define BLOCK (&host_block)
#define BLE_SEND host_ble_send
#else
#define BLOCK ((AccelBlock*)0x20006430)
#define BLE_SEND (*(void (**)(uint8_t, const void*))0x20005820)
#endif

static void tap(AccelBlock* b, int16_t z) {
  int16_t diff = (int16_t)(z - b->state.last);
  uint8_t debounce = b->state.debounce;
  b->state.last = z;
  if (debounce > 0) {
    if (debounce > TAP_DEBOUNCE - TAP_DURATION) {
      int limit = b->state.pos_first ? -TAP_THRESH : TAP_DEBOUNCE;
      if (diff < limit) {
        b->message.tap_count++;
        debounce = TAP_DEBOUNCE - TAP_DURATION;
      }
    }
    b->state.debounce = (uint8_t)(debounce - 1);
  } else if (diff > TAP_THRESH) {
    b->state.pos_first = 1;
    b->state.debounce = TAP_DEBOUNCE;
  } else if (diff < -TAP_THRESH) {
    b->state.pos_first = 0;
    b->state.debounce = TAP_DEBOUNCE;
  }
}

void hal_acc_tick(void) {
  AccelBlock* b = BLOCK;
  int frames = spi_read8(FIFO_STATUS) & FRAME_MASK;
  uint32_t n = (uint32_t)frames;
  uint32_t sum[3] = {n << 12, n << 12, n << 12};
  int16_t f[3];
  for (; frames > 0; frames--) {
    spi_read(FIFO_DATA, (int)sizeof(f), (uint8_t*)f);
    for (int i = 0; i < 3; i++) sum[i] += (uint32_t)(int32_t)(f[i] >> 4);
    tap(b, f[2]);
  }
  if (n > 0) {
    int16_t* slot = b->message.axis[b->state.frame];
    for (int i = 0; i < 3; i++) slot[i] = (int16_t)(udiv((sum[i] << 4) + (n >> 1), n) & 0xFFFFu);
  }
  if (++b->state.frame >= FRAMES_PER_MESSAGE) {
    BLE_SEND((uint8_t)sizeof(AccelMessage), &b->message);
    b->state.frame = 0;
  }
}
