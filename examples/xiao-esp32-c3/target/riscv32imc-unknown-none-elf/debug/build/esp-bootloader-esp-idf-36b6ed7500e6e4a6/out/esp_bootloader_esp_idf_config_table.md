
| Option | Stability | Default&nbsp;value | Allowed&nbsp;values |
|--------|:---------:|:------------------:|:-------------------:|
| <p>**ESP_BOOTLOADER_ESP_IDF_CONFIG_MMU_PAGE_SIZE**</p> <p>ESP32-C2, ESP32-C6 and ESP32-H2 support configurable page sizes. This is currently only used to populate the app descriptor.</p> | ⚠️ Unstable | 64k | One of:<br/><ul style="display: inline-block; text-align: left"><li>8k</li><li>16k</li><li>32k</li><li>64k</li></ul>
| <p>**ESP_BOOTLOADER_ESP_IDF_CONFIG_ESP_IDF_VERSION**</p> <p>ESP-IDF version used in the application descriptor. Currently it's not checked by the bootloader.</p> | ⚠️ Unstable | 0.0.0 | 
| <p>**ESP_BOOTLOADER_ESP_IDF_CONFIG_PARTITION_TABLE_OFFSET**</p> <p>The address of partition table (by default 0x8000). Allows you to move the partition table, it gives more space for the bootloader. Note that the bootloader and app will both need to be compiled with the same PARTITION_TABLE_OFFSET value.</p> | ⚠️ Unstable | 32768 | 
| <p>**ESP_BOOTLOADER_ESP_IDF_CONFIG_SECURE_VERSION**</p> <p>Secure version. Used to enforce anti-rollback protection.</p> | ⚠️ Unstable | 0 | Positive integer or 0
