#!/bin/bash
# Navigate to the build directory
cd /home/may/esp/esp-idf/examples/peripherals/usb/device/tusb_composite_msc_serialdevice/build

# Create the output directory
mkdir -p firmware_package

# Merge binaries into a single firmware file
esptool.py --chip esp32s2 merge_bin -o firmware_package/ap_esp32s2_firmware.bin \
  --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x1000 bootloader/bootloader.bin \
  0x9000 partition_table/partition-table.bin \
  0x20000 tusb_composite.bin

# Optional: Generate a custom encryption key
espsecure.py generate_flash_encryption_key firmware_package/my_flash_key.bin

# Package the files into a zip
cd firmware_package
zip -r ../firmware_package.zip .