/* Flash layout of a stock Seeed Xiao nRF52840.

   The board ships with two things already resident in its 1 MB of flash, and
   an application has to fit around both of them:

     0x00000000 - 0x00026FFF   Nordic SoftDevice S140 7.3.0
     0x00027000 - 0x000ECFFF   application region (this image)
     0x000ED000 - 0x000FFFFF   Adafruit nRF52 UF2 bootloader 0.6.1 + MBR
                               settings, at the top of flash

   The bootloader lives at the top of flash and the SoftDevice at the bottom,
   which is why the application starts at 0x27000 rather than 0. That start
   address is fixed by the SoftDevice: S140 v7 occupies everything below it, and
   the bootloader's UF2 writer expects application blocks at exactly this
   address. `INFO_UF2.TXT` on the mounted bootloader volume reports which
   SoftDevice the board carries.

   Nothing in this project starts the SoftDevice, so its RAM reservation does
   not apply and the full 256 KB is available. */

MEMORY {
    FLASH : ORIGIN = 0x00027000, LENGTH = 0xED000 - 0x27000
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
