set -e
cargo build --release
gdb -nx --batch \
  -ex 'target extended-remote /dev/ttyACM0' \
  -ex 'monitor jtag_scan' \
  -ex 'attach 1' \
  -ex 'load' \
  -ex 'compare-sections' \
  -ex 'kill' \
  -ex 'monitor hard_srst' \
  target/thumbv7em-none-eabihf/release/td-deflect-firmware
