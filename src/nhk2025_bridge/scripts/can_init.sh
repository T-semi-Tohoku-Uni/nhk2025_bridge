# !/bin/sh

sudo -A ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on
sudo -A ip link set can0 up

exit 0
