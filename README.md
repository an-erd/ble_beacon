# A Bluetooth Low Energy Beacon (ble_beacon)

## Overview

**ble_beacon** is a software for the Nordic Semiconductor SOC NRF52832 and similar. It reads sensor data (in our case temperature, humidity and acceleration data) and send it using Bluetooth Low Energy using advertisement packages. Getting a history of data points is available when establishing a BLE connection to the device.

The code is written for an low power consumption environment. As of today, the average power consumption during unconnectable undirected advertising is 14.4 uA, and with connectable undirected advertising ~19 uA.

### Advertising

The following data will be send during advertising:

- Address type: `RandomStatic`

- Advertising type: `Connectable undirected`

- Services: `1400`

- Flags: `LeGeneralDiscMode BrEdrNotSupported LeOnlyLimitedDiscMode LeOnlyGeneralDiscMode`

- `BLE_GAP_AD_TYPE_TX_POWER_LEVEL`: TxPowerLevel: 00

- `BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA`: ManufacturerSpecificData

- Example: `59-00-00-07-00-08-5D-B4-73-6F-B8-FC-2C-FK-62-3F-0B-6A`

   | Topic              | Bytes    | Example | Result                                   |
   | ------------------ | -------- | ------- | ---------------------------------------- |
   | Company Identifier | [0..1]   | 59 00   | = 0x0059, Nordic Semiconductor           |
   | Major              | [2..3]   | 00 07   | = 0x0007                                 |
   | Minor              | [4..5]   | 00 08   | = 0x0008                                 |
   | Temperature        | [6..7]   | 5D B4   | = 19,05 = (0x5DB4 * 175) / 0xFFFF (° C)  |
   | Humidity           | [8..9]   | 73 6F   | = 45,09 = (0x736F * 100) / 0xFFFF (% RH) |
   | Accel-X            | [10..11] | B8 FD   | = 0xFDB8, 2's complement = -248          |
   | Accel-Y            | [12..13] | 2C FB   | = 0xFB2C, 2's c = -1236                  |
   | Accel-Z            | [14..15] | 62 3F   | = 0x3F62, 2's c = ‭+16542‬                 |
   | Battery Voltage    | [16..17] | 0B 6A   | = 0x0B6A = 2992 mV                       |

Remark: The temperature and humidity data are send using 

### Provided Services


In addition you can connect to the device and can use additional features:

- Download the stored data point from the device
- Get number of data points available
- Delete the data points
- Update software using DFU

### Offline Buffer Functionality

Beside the advertising of the sensor measurements the beacon can store the measurements in an offline buffer. The offline buffer is located  in RAM, i.e., with a reboot the values will be reset, too. Currently 20.000 bytes are reserved for the offline buffer, with an size of 16 bytes for one entry, this gives 20.000/16 bytes = 1.250 entries. With an interval of 15 min/entry the beacon will store data for around 13 days. If the buffer is full, the oldest value will be deleted (i.e. a ring buffer is used). The data of the offline buffer can be accessed and downloaded when connecting to the device.

### Available Sensors on Board

Currently the following sensors are on-board:

- KX022 accelerometer
- SHT03 temperature and humidity sensor

Both sensors use the TWI (i.e. I2C) bus to communicate with the NRF52.

### Additional NFC Antenna and Functionality

An additional NFC (Near field communication) antenna can be attached to allow for an easy paring process with iPhone and Android mobile phones. 

**Important:** If NFC is to be used, you must remove the preprocessor define to not use the NFC pins as GPIO pins. To use NFC, undefine `CONFIG_NFCT_PINS_AS_GPIOS`. If it is defined, the pins P9 and P10 serve as GPIO pins.

**Remark:** NFC functionality is not yet implemented but planned for a future release. The board already provides the point to attach the antenna with.

## Programming the Device

### Preparation Application zip File

**Step 1)** Create zip file with the application

- go to the application build directory with the application already build, e.g. `cd ~/nrf52/nRF5_SDK_15.3.0_59ac345/projects/ble_peripheral/ble_beacon/pca10040/s132/ses/Output/Release/Exe`
- if not done yet, copy the `private.key` to this directory, e.g. `cp ../../../../../../Keys/private.key .`
- generate the package, e.g. `nrfutil pkg generate --hw-version 52 --application-version 1 --application ble_beacon_pca10040_s132.hex --sd-req 0xb7 --key-file private.key app_dfu_package.zip`

### Preparation Bootloader Settings and Merge Settings with Bootloader to one .hex

**Step 2)** Generate settings for the bootloader

- go to bootloader application build directory with the bootloader already build, e.g.  `cd ~/nrf52/nRF5_SDK_15.3.0_59ac345/projects/dfu/secure_bootloader/pca10040_ble/ses/Output/Release/Exe`
- `nrfutil settings generate --family NRF52 --application ../../../../../../../ble_peripheral/ble_beacon/pca10040/s132/ses/Output/Debug/Exe/ble_beacon_pca10040_s132.hex --application-version 0 --bootloader-version 0 --bl-settings-version 1 bootloader_setting.hex`

**Step 3)** Merge the bootloader and settings file to allow flashing the application together with the bootloader/settings during production and without the need to update using DFU

- open a Windows `cmd` shell and go to the bootloader directory, e.g. `c:\msys32\home\AKAEM\nrf52\nRF5_SDK_15.3.0_59ac345\projects\dfu\secure_bootloader\pca10040_ble\ses\Output\Release\Exe>`
- `mergehex -m bootloader_setting.hex secure_bootloader_ble_s132_pca10040.hex --output output.hex`

### Program the Device

**Step 4)** Use nRF Connect Programmer to upload all necessary .hex files

- Open nRF Connect Programmer, connect, read device
- Add files
  - HEX files
  - Softdevice .hex file
  - merged settings and bootloader file generated in Step 3
  - application .hex file generated during application build
- Press `Erase & Write`

**Step 5)** (optional) If necessary, write UICR with the device MAJOR and MINOR address

- to read the current flashed values: `nrfjprog -f NRF52 --memrd 0x10001080 --n 4`
- to write (requires previous delete -> set to 0xFF): `nrfjprog -f nrf52 --memwr 0x10001080 --val 0x000700FF` (adjust!)

### Update the Device using DFU

**Step 6)** to update the application use DFU and the application zip file generated as in Step 1) using nRF Connect Bluetooth Low Energy application on Windows (with nrf52832 dongle) or iPhone/Android app.

#### Remark: Using without bootloader and DFU

If you want to use the software without the bootloader and without buttonless DFU you need to comment out the call to `ble_dfu_buttonless_async_svci_init();` or use `#undef  USE_BUTTONLESS_DFU`. Otherwise you'll get an error  `<error> app: ERROR 4 [NRF_ERROR_NO_MEM]`

## Provided Services by Device

You can connect to the device and use the following additional features:

- Download the stored data point from the device
- Get number of data points available
- Delete data points
- Update software using DFU

### Connect to the Device using nRF Connect 

- You can use the following tools, besides other, to connect to the device:
  - nRF Connect Bluetooth Low Energy tool to connect to the device from Windows with an nrf52832/nrf52840 dongle
  - iPhone nRF Connect app
  - Android nRF Connect app

### Preparation

- Connect to the service 1400
- Pairing - **TBD**
- Turn on **Notification** on characteristic 1401
- Turn on **Indication** on characteristic 2A52 (Random Access Control Point)

### Get Data from device

You can retrieve data from the device or delete the device offline buffer memory using the following commands:

- **write** to 2A52 the following commands:

  - `01 01` (Report Records, All)   -> `2A52: 06 00 01 01` (Response, Operator NULL, ?, Success), `1401: 00-00-01-00-00-00-60-AD-74-15` (2 byte seq. number = 0, 4 byte time (here: not yet real time, thus 1 sec), 2 byte temperature `60 AD`, 2 byte humidity `74 15`)
  - `01 05` (Report Records, First) -> see `01 01`
  - `01 06` (Report Records, Last)  -> see `01 01`
  - `04 01` (Report Number Records, All) -> `2A52: 05 00 01 00` (Number of stored records response, number of records 2 byte  LSB first,  here: 1 record)
  - `02 01` (Delete All Records)

  **Remarks:** the byte order is LSB-MSB

#### Decode sequential number/Number of entries

The number of records in return to the Report Number Records request and the sequential numbers in the records itself are 2 byte, which are coded LSB first, than MSB.

#### Decode time stamp

The time stamp is reported in the records as seconds since 01.01.1970, 00:00:00 (UTC), and is given in 4 Bytes with LSB-MSB ordering. 

The time stamp can be decoded using an appropriate online calculator, for example

- Unix Epoch time calculator, e.g. https://www.unixtimestamp.com/index.php

If the beacon does not have the correct time, the time stamps are reported since boot time, which is 0. If the correct time is available, this will be used.

### Battery Information Service (BAS)

The Bluetooth service "Battery Information Service" is available as service 0x180F, and provides a single value characteristic 0x2A19. This will return the battery level in percentage.

### Device Information Service (DIS)

The Bluetooth service "Device Information Service" is available as service 0x180A, and provides the following characteristics:

| UUID   | Title                    | Example             |
| ------ | ------------------------ | ------------------- |
| 0x2A24 | Model Number String      | 1                   |
| 0x2A25 | Serial Number String     | 38 (="")            |
| 0x2A26 | Firmware Revision String | 0.1a                |
| 0x2A27 | Hardware Revision String | 1.0                 |
| 0x2A28 | Software Revision String | 0.9                 |
| 0x2A29 | Manufacturer Name        | "ansprechendeKunst" |

### Secure DFU 

The Bluetooth service "Secure DFU" is available as service 0xFE59, and provides characteristic to update the device buttonless using Nordic's Secure DFU. 

## Used Services by Device

### Current Time Service (CTS)

As soon as the device is connected, it checks whether a CTS (Current Time Service) server is provided by the central device. If it is available, the following steps will be performed:

- If CTS and thus the current time is available for the **first time** (e. g., directly after booting):
  - The current time is fetched from CTS
  - The internal function (nrf_calendar) to keep the time is set to the current time.
  - If there are any entries stored in the offline buffer, these entries will be updated with the correct time. This is done by calculating the delta, and adding this correction value to all previous entries.
-  For subsequent updates of the time using CTS:
  - The current time is fetched from CTS
  - The internal function (nrf_calendar) will be updated with the current time.
  - The drift will be calculated and for future requests inside the beacon used as a correction factor.
  - Remark: Old values eventually stored in the offline buffer will not be updated/corrected with the drift factor.

## Buttons

The device provides one push button. When attached to the programming JIG, another button is available for testing.

## Power Consumption

todo

## Programming JIG

There is a separate Readme for the programming jig, see [here](Documentation/JIG/README.md)

![Alt text](Documentation/JIG/IMG_JIG_4_resize.jpg?raw=true "Complete JIG")

## Schematics

The schematics for the beacon I use is available here. The product is available from Radioland China using ALIEXPRESS.

![Alt text](Documentation/beacon_hardware/nRF52832+KX022+SHT30%20circuit.jpg?raw=true "Complete JIG")
