@ The light cube app's accelerometer tick, keeping the mean of each tick's FIFO frames instead of the last one.
@ Authored-by: CPCS (cpcs-66ffc23), 2026-11-21. Cortex-M0 Thumb, hand-written to fit where the shipped tick was.
@
@ It replaces hal_acc_tick in the shipped app (2018-08-14T17:40) in place, at 0x20005db4, within the 204 bytes up
@ to 0x20005e80; the only call to it is the bl at 0x200063ba. tick_avg.c is the same tick in C, for reading and for
@ the host test; it does not fit (256 bytes from GCC 16.1 at -Os), which is why this file exists. Both are run
@ against simulacrum/cube.py's "mean" app by tests/test_cube_firmware.py, this one in a Thumb interpreter.
@
@ It uses the shipped app's own state and routines, at the addresses the shipped tick uses (see tick_avg.c):
@   r4 = 0x20006430: debounce u8 @0, last i16 @4, accel_frame i32 @12, the message @16 (command @16,
@        tap_count @17, axis[3][3] from @18). The shipped tick's posFirst @2, tapNeg @6 and tapPos @8 were its
@        own; this one keeps the window's limit (-2880 after a rise, 90 after a drop) as a word at @8 instead.
@        Nothing outside the shipped tick addresses this block (the base appears only in its literal pool and as
@        the end of the start-up table).
@   spi_read8 0x20006350, spi_read 0x200062f0, udiv 0x200058ba (r0 / r1 -> r0), BLE_Send pointer 0x20005820
@ The stack holds the frame just read at sp+0..5 and three 32-bit sums at sp+8, +12, +16.
@
@ Per frame: each axis's 12-bit value (raw >> 4) is added to its sum, which starts at 4096 n so it never goes
@ negative; the raw z goes through the shipped tap detector, compare for compare. After the frames, if there
@ were any (n > 0), each slot gets (sum * 16 + n / 2) / n, unsigned: that is 16 x (the mean of the 12-bit values,
@ halves rounded up) + 65536, and the 16-bit store drops the 65536, leaving the mean on the chip's
@ left-justified scale. With n <= 32 the dividend stays under 2^22. With no frame the slot keeps its old value,
@ as the shipped tick does. After the third tick the 20-byte message goes out, as before.

    .syntax unified
    .cpu cortex-m0
    .thumb
    .text
    .global hal_acc_tick
    .thumb_func
hal_acc_tick:
    push  {r4, r5, r6, r7, lr}
    sub   sp, #20
    ldr   r4, .Lstate
    movs  r0, #14                @ FIFO_STATUS
    bl    spi_read8
    lsls  r5, r0, #25
    lsrs  r5, r5, #25            @ r5 = frames in the FIFO (bits 6:0)
    movs  r7, r5                 @ r7 = n, the frames this tick averages
    mov   r6, sp
    lsls  r0, r5, #12            @ each sum starts at 4096 n
    str   r0, [r6, #8]
    str   r0, [r6, #12]
    str   r0, [r6, #16]
    b     .Lcheck
.Lframe:
    mov   r2, r6
    movs  r1, #6
    movs  r0, #63                @ FIFO_DATA
    bl    spi_read               @ six bytes, x y z little-endian, into sp+0
    movs  r3, #4                 @ r3 = byte offset of the axis in the frame: 4, 2, 0
.Laxis:
    ldrsh r0, [r6, r3]
    asrs  r0, r0, #4             @ the 12-bit value; the four low bits carry no data
    lsls  r1, r3, #1
    adds  r1, #8                 @ the axis's sum: sp+16, +12, +8
    ldr   r2, [r6, r1]
    adds  r2, r2, r0
    str   r2, [r6, r1]
    subs  r3, #2
    bge   .Laxis
    ldrh  r1, [r6, #4]           @ the tap detector, on the raw z
    ldrh  r0, [r4, #4]
    strh  r1, [r4, #4]           @ last = z
    subs  r0, r1, r0
    sxth  r0, r0                 @ r0 = diff, as int16
    ldrb  r1, [r4, #0]           @ r1 = debounce
    cmp   r1, #0
    beq   .Lidle
    cmp   r1, #80                @ TAP_DEBOUNCE - TAP_DURATION
    bls   .Lcount_down
    ldr   r3, [r4, #8]           @ the window's limit
    cmp   r3, r0
    ble   .Lcount_down           @ a tap only when diff < limit
    ldrb  r2, [r4, #17]
    adds  r2, #1
    strb  r2, [r4, #17]          @ tap_count++
    movs  r1, #80
.Lcount_down:
    subs  r1, #1
    strb  r1, [r4, #0]
    b     .Lcheck
.Lidle:
    ldr   r3, .Lminus_thresh     @ r3 = -2880
    movs  r2, #90                @ after a drop the shipped limit is TAP_DEBOUNCE
    cmp   r0, r3
    blt   .Lwindow               @ diff < -2880: a window, negative first
    cmn   r0, r3
    ble   .Lcheck                @ diff <= 2880: nothing
    movs  r2, r3                 @ diff > 2880: a window, positive first, limit -2880
.Lwindow:
    str   r2, [r4, #8]
    movs  r1, #90                @ TAP_DEBOUNCE
    strb  r1, [r4, #0]
.Lcheck:
    subs  r5, #1
    bge   .Lframe
    cmp   r7, #0
    beq   .Lsend                 @ no frame: the slot keeps its old value
    ldr   r0, [r4, #12]
    movs  r1, #6
    muls  r0, r1, r0
    adds  r0, #22
    adds  r5, r4, r0             @ r5 = this tick's slot, its z
    adds  r6, #8                 @ r6 + 8 = the z sum
.Lmean:
    ldr   r0, [r6, #8]
    lsls  r0, r0, #4
    lsrs  r1, r7, #1
    adds  r0, r0, r1             @ sum x 16, plus n / 2 to round
    movs  r1, r7
    bl    udiv
    strh  r0, [r5, #0]           @ the low 16 bits: 16 x mean
    subs  r5, #2
    subs  r6, #4
    cmp   r6, sp
    bhs   .Lmean                 @ z, y, x
.Lsend:
    ldr   r0, [r4, #12]
    adds  r0, #1
    cmp   r0, #3                 @ FRAMES_PER_MESSAGE
    blt   .Lstore
    ldr   r2, .Lble_send
    ldr   r2, [r2, #0]
    movs  r1, r4
    adds  r1, #16
    movs  r0, #20
    blx   r2                     @ BLE_Send(20, &message)
    movs  r0, #0
.Lstore:
    str   r0, [r4, #12]
    add   sp, #20
    pop   {r4, r5, r6, r7, pc}
    .p2align 2
.Lstate:
    .word 0x20006430
.Lminus_thresh:
    .word -2880
.Lble_send:
    .word 0x20005820
