/* Places the replacement tick where the shipped one was, and names the shipped app's routines it calls at
   their Thumb addresses (bit 0 set). Authored-by: CPCS (cpcs-66ffc23). */
spi_read8 = 0x20006351;
spi_read = 0x200062f1;
udiv = 0x200058bb;
SECTIONS
{
  . = 0x20005db4;
  .text : { *(.text) }
  /DISCARD/ : { *(.ARM.attributes) *(.comment) }
}
ASSERT(SIZEOF(.text) <= 0xcc, "the tick must fit in the 204 bytes up to 0x20005e80")
