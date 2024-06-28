# thermal-cam-ble

The crappy MLX90640 thermal camera connected to an ESP32-C6 that sends frames over BLE to WebBluetooth.

## Hardware used

- An MLX90640 with 55° FOV
- A Beetle C6 dev board
- A 3.7V Li-Ion battery
- An N-channel logic-level MOSFET to power the camera programmaticaly
- I2C pull-up resistors
- 0.1uF and 10uF decoupling capacitors
- A holder to attach the device to a phone?

## Pin assignments

See (config.h)[src/config.h] for the pin assignments.

## Usage
- Pair the device. Confirm pairing by pressing the button at the PAIR_BUTTON_PIN. It needs to be paired in order to read the frames.
- Host (index.html)[index.html] somewhere in a secure context (file:// also seems to work).
- Open the page in a Chromium-based browser.
- The web interface is based on (doct0r0710/Esp32ThermalCam)(https://github.com/doct0r0710/Esp32ThermalCam/blob/main/index.html)
