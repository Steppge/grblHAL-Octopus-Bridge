# Octopus-Bridge

ESP32-S3-DevKitC-1 N16R8 at the BTT Octopus Pro. It connects the grblHAL display (CrowPanel 9" ESP32-P4) and the gamepad to the controller, over a USB cable or WiFi.

```
Gamepad -BT-> D1 -UART-> P4 display ==USB or WiFi==> S3 -UART1-> Octopus J26 (display link, USART2)
                                                       -UART2-> Octopus TFT header (gamepad, USART1)
```

## Wiring

| S3 | Signal | Octopus |
|---|---|---|
| GPIO11 | TX link | J26-7 PD6 (RX2) |
| GPIO12 | RX link | J26-9 PD5 (TX2) |
| GPIO13 | TX gamepad | TFT-4 PA10 (RX1) |
| GPIO14 | RX gamepad | TFT-3 PA9 (TX1) |
| 5V / GND | supply | TFT-1 5V / TFT-2 GND |

GPIO9/10 are kept for a later sender UART to USART3 (WIFI socket).

The display's USB-C "USB 2.0" goes to the S3's "USB" socket (GPIO19 D-, GPIO20 D+). For powering the display over the cable, a socket of its own with VBUS straight from the 24 V -> 5 V converter (not through the S3, the display draws about 1 A).

The display link runs at 921600 baud, the gamepad at 230400 (921600 on USART1 stalled the Octopus during SD card
access). In grblHAL's `my_machine.h`: `DISPLAY_LINK_BAUD 921600` and `GAMEPAD_BAUD 230400`.

## Function

- **USB**: the S3 is USB host, the display a USB device with two CDC-ACM channels. Interface 0 is the display link, interface 2 the gamepad. The display announces 64 byte endpoints, the S3 is a full speed host.
- **WiFi**: a hidden access point "grblHAL-Machine" (channel 6, WPA2, a random password made on the first start and kept in NVS). Display link over TCP port 5000 (one client), gamepad lines over UDP port 5001 (one datagram per line).
- **Pairing**: on every plug-in the S3 sends the network name and password to the display over the link channel (`$BW,<ssid>,<password>*<hh>`). Nobody types the password.
- **USB has priority**: while the display is plugged in, the access point is off and the display leaves the machine network.
- Without the display the UARTs stay silent. grblHAL's gamepad watchdog stops jogging after 300 ms; SD jobs keep running.
- RGB LED (GPIO48, GPIO38 on board v1.1):
  - red: no display
  - blue: over USB
  - cyan: over WiFi
  - short green flash a second: data flowing
  - violet: pass-through

## Console

The console is on the USB-C "UART" (CH343, COM19, 115200 baud). Commands:

- `status`: version, channels, byte counters and the WiFi devices with their signal strength.
- `wifi`: name and password of the machine network.
- `link` / `gamepad`: pass-through of that UART to the console, for tests without the display. Ctrl+] ends it.

## Build and flash

PlatformIO, ESP-IDF (pioarduino 55.03.312-1), flashed over the "UART" socket:

```
pio run -t upload
```

## Related

- Display firmware (CrowPanel 9" ESP32-P4) and the grblHAL plugin:
  [Steppge/grblHAL-Plugin-Display](https://github.com/Steppge/grblHAL-Plugin-Display)
- Gamepad plugin and receiver: [Steppge/grblHAL-Plugin-Gamepad](https://github.com/Steppge/grblHAL-Plugin-Gamepad)
- grblHAL for the BTT Octopus Pro: [Steppge/grblHAL-OctopusPro-CNC-Laser](https://github.com/Steppge/grblHAL-OctopusPro-CNC-Laser)

## License

GPL v3 or later, see [LICENSE](LICENSE). Copyright (c) 2026 Steppge.
