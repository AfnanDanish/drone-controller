# Drone Controller System

A drone controller system with OLED display interface for controlling various flying platforms (Ornithopter, Plane, Helicopter, Quadcopter).

## Features

- OLED display interface with multiple pages (Platform Selection, Tuning, Settings, Calibration)
- Dual joystick controls
- Debug mode that allows GUI navigation using the right joystick
- Rotary encoder for precise adjustments
- LoRa communication capability
- AES encryption for secure communications
- Battery monitoring with visual indicators

## Hardware Requirements

- ESP32 WROOM microcontroller
- SSD1306 OLED Display
- LoRa radio module
- Dual analog joysticks with press buttons
- Rotary encoder with push button
- Slide switches for debug and kill functions
- Status LEDs
- Battery monitoring circuit

## Pin Configuration

### Joystick Pins
- LEFT_JOYSTICK_X: GPIO 34 (Yaw)
- LEFT_JOYSTICK_Y: GPIO 35 (Throttle)
- RIGHT_JOYSTICK_X: GPIO 32 (Roll)
- RIGHT_JOYSTICK_Y: GPIO 33 (Pitch)
- LEFT_JOYSTICK_SW: GPIO 0 (Press button)
- RIGHT_JOYSTICK_SW: GPIO 5 (Press button)

### Button/Switch Pins
- DEBUG_SWITCH_PIN: GPIO 13
- KILL_SWITCH_PIN: GPIO 12
- ROTARY_ENABLE_PIN: GPIO 27
- ROTARY_CLK_PIN: GPIO 25
- ROTARY_DT_PIN: GPIO 15
- ROTARY_SW_PIN: GPIO 4

### LED Pins
- STATUS_LED_GREEN: GPIO 2
- STATUS_LED_RED: GPIO 16
- BATTERY_LED_1: GPIO 17
- BATTERY_LED_2: GPIO 21
- BATTERY_LED_3: GPIO 22
- VIBRATION_PIN: GPIO 23

## Development

This project is built using PlatformIO with the Arduino framework for ESP32.

## License

[MIT License](LICENSE)
