MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

/* The second stage bootloader lives in `embassy-rp`, and the `.boot2` output
   section that places it into the region above comes from `link-rp.x`, the
   linker script fragment `embassy-rp` emits for the RP2040. This file only has
   to carve out the region it is placed in; see `.cargo/config.toml` for where
   the fragment is added to the link. */
