pushd linux_loader
idf.py build
popd

CROSS_COMPILE=riscv64-linux-gnu- make dtbs ARCH=riscv
CROSS_COMPILE=riscv64-linux-gnu- make Image ARCH=riscv -j32
gen_esp32part.py partition-table.csv partition-table.bin
esptool.py --chip esp32p4 --port /dev/ttyUSB0 --baud 921600 \
  write_flash \
  0x2000 linux_loader/build/bootloader/bootloader.bin \
  0x8000 linux_loader/build/partition_table/partition-table.bin \
  0x10000 linux_loader/build/linux_loader.bin \
  0x210000 arch/riscv/boot/Image
picocom -q -b 115200 /dev/ttyUSB0

