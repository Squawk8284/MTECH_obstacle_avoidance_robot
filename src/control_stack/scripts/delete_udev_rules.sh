#! /bin/bash

echo "Deleting the remapped rules for ttyRobot and ttyIMU"

# Define the path to the rules file
RULES_FILE="/etc/udev/rules.d/robot.rules"

if [ -f "$RULES_FILE" ]; then
    echo "Found $RULES_FILE. Removing it..."
    sudo rm "$RULES_FILE"
    echo "File $RULES_FILE has been removed."
else
    echo "Error: $RULES_FILE does not exist. Nothing to remove."
fi

echo " "
echo "Restarting udev"
echo " "
sudo udevadm control --reload
sudo udevadm trigger

echo "UDEV rules deleted successfully."
echo "Finish."
