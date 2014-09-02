#!/bin/bash

# Permissions
chgrp gpio  /sys/class/gpio/export /sys/class/gpio/unexport
chmod 775 /sys/class/gpio/export /sys/class/gpio/unexport

# setup laser pin
echo 172 > /sys/class/gpio/export
LASER=/sys/class/gpio/gpio172
chgrp -HR gpio $LASER
chmod -R  775  $LASER
echo "out" > $LASER/direction
echo "0" > $LASER/active_low
echo "0" > $LASER/value