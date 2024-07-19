# thermal-cam-ble

An MLX90640 thermal camera connected to an ESP32-C6 that sends frames over BLE to WebBluetooth. Uses the espidf framework.

Inspired by (doct0r0710/Esp32ThermalCam)[https://github.com/doct0r0710/Esp32ThermalCam]

MLX90640 driver based on (adafruit/Adafruit_MLX90640)[https://github.com/adafruit/Adafruit_MLX90640] and the original unmodified (mlx90640-library)[https://github.com/melexis/mlx90640-library]

Use `git clone --recurse-submodules https://github.com/kawaiiDango/thermal-cam-ble` to include that MLX90640 driver.

## Hardware used

- MLX90640 with 55° FOV
- Beetle C6 dev board
- 3.7V Li-Ion battery
- Locking switch to physically disconnect the battery
- N-channel logic-level MOSFET to power the camera programmaticaly
- I2C pull-up resistors
- 0.1uF and 10uF decoupling capacitors (was getting a chess pattern noise without them)
- Two sided tripod holder to attach the device to the back of a phone

## Pin assignments

See (config.h)[src/config.h] for the pin assignments.

## Working
- The device keeps advertising BLE even after it is connected to a client to allow multiple clients to connect.
- It sends frames and other data through BLE notifications.
- To confirm pairing, press the button at the PAIR_BUTTON_PIN.
- It needs to be paired in order to read the MLX90640 data.
- The built in LED is used as an indicator for various states. see (led.h)[src/led_indicator.c]

## Sleep
- The device runs in automatic light sleep mode to save battery (which unfortunately makes the C6 hard to debug)
- The camera gets powered off when not in use for n seconds.
- The device goes into deep sleep if nobody is subscribed to BLE notifications, after m seconds.
- The device goes into deep sleep if the battery voltage is too low.

## Web interface
- The web interface is based on (doct0r0710/Esp32ThermalCam)(https://github.com/doct0r0710/Esp32ThermalCam/blob/main/index.html)
- It uses WebBluetooth, so it only works on Chromium based browsers.
- It can auto connect without prompting, if the new WebBluetooth API is enabled.
- The hardware refresh rate of the camera can set through a dropdown. This value gets saved on the ESP32.
- It can overlay the thermal camera frame with a webcam frame if allowed. This is meant to be used when the device is attached to the back of a phone.
- The camera frames and the UI helpers are drawn on a canvas. This also makes it easy to save the canvas an image.
- (index.html)[index.html] needs to be hosted in a secure context. file:// or localhost also works.

## Notes
- I have had good results with a refresh rate of 1Hz (hardware refresh rate of 2Hz, since it captures half a frame at a time).
- It lags at 8Hz (hardware refresh rate of 16Hz), with the ESP32 running at 80MHz. Also there is a lot of noise, which makes it unusable.
- The positioning and scaling of the webcam overlay needs to be adjusted on first use.
- I get a weird noise pattern if the battery voltage is less than ~3.7V.
- The web interface has only been successfully tested on Android.
- BLE notifications seem to be far more stable and jitter free than connection oriented WiFi/WebSockets used by (doct0r0710/Esp32ThermalCam)(https://github.com/doct0r0710/Esp32ThermalCam) and also saves battery (takes about 65ma including the camera's current)