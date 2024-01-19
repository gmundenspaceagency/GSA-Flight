#!/bin/bash

# Check if the script is run with sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run with sudo."
  exit 1
fi

# Set ownership and permissions
sudo chown root.gpio /dev/gpiomem && sudo chmod g+rw /dev/gpiomem

# Check if the commands were successful
if [ $? -eq 0 ]; then
  echo "Now the error should be gone."
else
  echo "Error setting ownership and permissions on /dev/gpiomem."
fi
