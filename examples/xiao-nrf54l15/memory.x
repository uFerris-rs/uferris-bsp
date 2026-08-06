/* Flash layout of a Seeed Xiao nRF54L15.

   Nothing is resident on this board: it ships empty, with no bootloader and no
   SoftDevice, so the application owns the memory from the bottom up and links
   at 0.

     0x00000000 - 0x0017CFFF   application region (this image)
     0x0017D000 - 0x0017FFFF   left to the FLPR coprocessor

   The nRF54L15 stores its program in RRAM rather than flash. It is byte
   writable and needs no page erase, but it is mapped and executed exactly like
   flash, so `cortex-m-rt` needs no special treatment for it. The part carries
   1536 KB of it. The top 12 KB are left out of the application region because
   they are where an FLPR coprocessor image is conventionally placed, matching
   the split Zephyr uses between its `cpuapp` and `cpuflpr` partitions. Nothing
   here loads an FLPR image, and `embassy_nrf::init` stops the coprocessor on
   every boot, so the region simply stays empty; reserving it keeps a later
   FLPR image from having to move the application.

   RAM is not shared with anything either, so all 256 KB is available. */

MEMORY {
    FLASH : ORIGIN = 0x00000000, LENGTH = 1524K
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
