CROSS_COMPILE=riscv64-linux-gnu- make dtbs ARCH=riscv
CROSS_COMPILE=riscv64-linux-gnu- make Image ARCH=riscv -j32
gen_esp32part.py partition-table.csv partition-table.bin
esptool -b 921600 --no-stub \
        write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB \
        0x2000 bootloader.bin \
        0x10000 arch/riscv/boot/Image \
        0x8000 partition-table.bin

