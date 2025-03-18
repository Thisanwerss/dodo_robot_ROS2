# Dodo USB Command Node

This package provides a USB command interface for the Dodo robot, allowing for basic directional control (front/back/left/right).

## Features

- Reads commands from USB joystick/gamepad
- Publishes movement commands based on user input
- Configurable USB device settings

## Topics

### Published Topics
- `/usb_commands` (std_msgs/Int32): Command input for walking direction

## Parameters

- `usb_device`: USB device path (default: `/dev/input/js0`)
- `publish_rate`: Rate at which commands are published (default: 20Hz)