# The Vector cube's pins, from its shipped firmware (2026-11-21)

## What was read

- The application the robot sends its cube:
  `/anki/data/assets/cozmo_resources/assets/cubeFirmware/cube.dfu` on unit 3.
  - 3209 bytes, version `2018-08-14T17:40`.
  - SHA-256 `f22829db489a034360e78c89c27e36eab47e4007b13eb13524485467a18b633d`.
  - The same bytes are in kercre123/victor (snowboy branch) at
    `resources/assets/cubeFirmware/cube.dfu`.
- How the file is made: `robot/cube_firmware/tools/sign.py`. It is the 16-byte version, then the
  image (with its first 16 bytes replaced by the version), XXTEA-encrypted under the 16-byte key in
  `robot/cube_firmware/tools/key.h`, then `0xFF`.
- How the cube checks it: `robot/cube_firmware/src/user_app/user_custs1_impl.c`. The bootloader, in
  one-time-programmable memory, decrypts with the same key and runs the image only if the 16 bytes at
  offset 0x24 match its nonce constant.
- Check, executed: decrypting with `key.h` and that file's own XXTEA routine gives back the version
  string and the nonce. With one key bit flipped, both checks fail. The plaintext is 3192 bytes of
  Cortex-M0 Thumb code, linked at `0x20005800`.

## Method

1. Disassembled with `arm-none-eabi-objdump -D -b binary -marm -Mforce-thumb
   --adjust-vma=0x20005800`.
2. Took the GPIO register addresses from the Dialog SDK's `datasheet.h`. Each port block has data,
   then set at +2, reset at +4, and pin modes at +6 + 2×pin:
   - P0 `0x50003000`
   - P1 `0x50003020`
   - P2 `0x50003040`
   - P3 `0x50003080` (the code maps port 3 to block 4)
3. Found the SDK's GPIO helpers in the binary:

   | Helper | Address | Calls |
   |---|---|---|
   | `GPIO_ConfigurePin` | `0x2000590c` | 31 |
   | `GPIO_ConfigurePinPower` | `0x20005930` | 31 |
   | `GPIO_SetActive` | `0x20005960` | |
   | `GPIO_SetInactive` | `0x20005978` | |
   | `GPIO_SetPinFunction` | `0x20005990` | |

4. Read the port, pin, mode and state that every call loads into r0-r3 and the stack.
5. Named the mode values from the SDK's `gpio.h`: `INPUT` 0, `INPUT_PULLUP` 0x100, `OUTPUT` 0x300.
   Named the power-rail values from the same file: 0 is 3 V, 1 is 1 V.

## The map

| Pin | Use | How the app sets it up |
|---|---|---|
| P0_0 | accelerometer chip select (`ACC_nCS`) | output, high when idle; driven low around each transfer |
| P0_2 | accelerometer power (`ACC_PWR`) | output high while the app runs; driven low when it stops |
| P0_3 | accelerometer clock (`ACC_SCK`) | output |
| P0_6 | accelerometer data (`ACC_SDA`), both directions | output, turned around for reads |
| P0_4, P0_5 | UART TX, RX | parked as inputs with pull-up |
| P0_7, P1_0, P1_1, P2_1 to P2_9 | the 12 LED lines D0 to D11 (4 RGB LEDs) | outputs, 3 V rail; high is off |
| P2_0 | LED boost converter enable (`BOOST_EN`) | output, 1 V rail; low when lights stop |
| P0_1, P1_2, P1_3 | not used | `board.h` says P1_2 and P1_3 "CORRUPTS 16M" |

The light setup configures BOOST_EN, then D0 to D11 in that exact order. That is the order of the
names in the public `robot/cube_firmware/app/board.h`. So `board.h` matches the shipped firmware.
`board.c` in the same tree uses pins `board.h` does not define (`CAPI`, `CAPO`, `NC_P12`,
`NC_P13`). None of them is touched by the shipped binary, so `board.c` is the stale file.

## The accelerometer

The app's accelerometer setup:

1. Power the chip.
2. Wait about 32 µs, then about 1.8 ms.
3. Write these registers (names from the repo's `app/BMA253.h`):

   | Register | Name | Value |
   |---|---|---|
   | 0x14 | `BGW_SOFTRESET` | 0xB6 |
   | 0x34 | `BGW_SPI3_WDT` | 1 |
   | 0x12 | `PMU_LOW_POWER` | 0x40 |
   | 0x11 | `PMU_LPW` | 0 |
   | 0x0F | `PMU_RANGE` | 5 |
   | 0x10 | `PMU_BW` | 13 |
   | 0x3E | `FIFO_CONFIG_1` | 0x40 |

4. Read register 0x00 (`BGW_CHIPID`) and report an error unless it is 0xFA.

From the Bosch BMA253 datasheet, not checked here:

- range 5 is ±4 g, which matches the source's "use 4G";
- bandwidth 13 is 250 Hz;
- `BGW_SPI3_WDT` bit 0 selects 3-wire SPI, which is why one data line serves both directions;
- 0xFA is the BMA253's chip id.

The accelerometer's interrupt pins go to no GPIO. Every configured input is accounted for above.
Taps are found in software from samples read every 10 ms (`accel.c`: "Frequency will be 10 ms *
ACCEL_FRAMES_PER_MSG", 3). And the app powers the accelerometer off (P0_2 low) when the robot
disconnects. So a stock cube cannot wake on a tap while unconnected. Firmware of ours could sample
it periodically, but only while a connection exists, because the bootloader starts the app on
connect and stops it on disconnect.

## The light driver

The light setup also does the following:

- writes 9 to `0x50000004`;
- sets up Timer 0 at `0x50003400`, with reload 0x9C40;
- points the Timer 0 interrupt (IRQ 8, `SWTIM_IRQn`) at code in the app (`0x200059ad`), and enables
  it in the NVIC.

The lights-off path reverses all of this, drives every LED line high, and drops BOOST_EN.

## Building our own

The shipped image was built with Keil ARMCC 5.06 update 2 (build 183), per
`robot/cube_firmware/cube_application.uvprojx`. The Makefile's GCC path does not reproduce it.

Built with Homebrew `arm-none-eabi-gcc` 16.1.0, the public app needs four changes:

- an empty `<stdio.h>`;
- a `<string.h>` with `memcpy`/`memset` prototypes (the app has its own);
- `__nop` defined as one NOP;
- `ACCEL_FRAMES_PER_MSG` made a `#define`.

It then stops at `board.c`'s undefined pins.

Two cautions:

- The delays are counted `__nop()` loops, so any other compiler changes them, including the bit-banged
  SPI timing. Check that before sending a rebuilt app to a cube.
- The robot reflashes its own `cube.dfu` whenever the cube's version string differs, on a read-only
  root filesystem. A custom app survives only if it reports the stock version or that file is
  replaced.
