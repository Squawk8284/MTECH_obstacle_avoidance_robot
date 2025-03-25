#!/bin/bash

echo "Remapping the device serial port (ttyUSBx) to ttyRobot, ttyIMU"
echo "Robot USB connection as /dev/ttyROBOT. Check it using the command: ls -l /dev | grep ttyUSB"
echo "Starting to copy robot.rules to /etc/udev/rules.d/"

# Get the path to control_stack package
RULES_PATH=$(rospack find control_stack)/scripts/robot.rules

if [ -f "$RULES_PATH" ]; then
    sudo cp "$RULES_PATH" /etc/udev/rules.d
    echo "Robot.rules successfully copied to /etc/udev/rules.d."
else
    echo "Error: robot.rules file not found at $RULES_PATH"
    exit 1
fi

echo " "
echo "Restarting udev"
echo " "
sudo udevadm control --reload
sudo udevadm trigger

echo "UDEV rules created successfully."
echo "Finish."
