#!/bin/bash

echo "remap the devices serial port(ttyAMAX) to  RPLIDAR"
echo "devices connection as /dev/PRLIDAR, check it using the command : ls -l /dev|grep -e -e ttyAMA0"
sudo cp $HOME/hana_violet/src/sllidar_ros2/scripts/rplidar.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish "
