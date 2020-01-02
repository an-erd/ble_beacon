

# A Bluetooth Low Energy Beacon (ble_beacon)

## Overview

**ble_beacon** is a software for the Nordic Semiconductor SOC NRF52832 and similar. It reads sensor data (in our case temperature, humidity and acceleration data) and sends it using Bluetooth Low Energy advertisement packages. Getting a history of data points is available when establishing a BLE connection to the device.

The code is power optimized. As of today, the average power consumption during unconnectable undirected advertising is 13.5 &#181;A, and with connectable undirected advertising ~17.4 &#181;A (see below for more detailed figures).

Using the push button, different modes (sensor, advertising, etc.) and deleting Bluetooth bonds can be configured.

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
   | Acceleration X     | [10..11] | B8 FD   | = 0xFDB8, 2's complement = -248          |
   | Acceleration Y     | [12..13] | 2C FB   | = 0xFB2C, 2's c = -1236                  |
   | Acceleration Z     | [14..15] | 62 3F   | = 0x3F62, 2's c = ‭+16542‬                 |
   | Battery Voltage    | [16..17] | 0B 6A   | = 0x0B6A = 2992 mV                       |

Remark: The temperature and humidity data are sent using the sensors native 2 byte format, with the formula given in the table above. See [SHT03 Data Sheet](Documentation/datasheets/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf), section *4.13 Conversion of Signal Output* for more information.

### Provided Services


In addition you can connect to the device and use additional features such as download and deletion of data points or DFU device update. See below for additional information.

### Offline Buffer Functionality

Beside the advertising of the sensor measurements the beacon can store the measurements in an offline buffer. The offline buffer is located  in RAM, i.e. with a reboot the values will be reset, too. Currently 20.000 bytes are reserved for the offline buffer, with an size of 16 bytes for one entry, this gives 20.000/16 bytes = 1.250 entries. With an interval of 15 min/entry the beacon will store data for around 13 days. If the buffer is full, the oldest value will be deleted (i.e. a ring buffer is used). The data of the offline buffer can be accessed and downloaded when connecting to the device.

### Available Sensors on Board

Currently the following sensors are on-board. Both sensors use the TWI (i.e. I2C) bus to communicate with the NRF52. 

| Sensor | Description                                   | Data Sheet                                                   | TWI Address |
| ------ | --------------------------------------------- | ------------------------------------------------------------ | ----------- |
| KX022  | ± 2g / 4g / 8g Tri-axis Digital Accelerometer | [KX022 Data Sheet](Documentation/datasheets/KX022-1020%20Specifications%20Rev4.0%20cn.pdf) | 0x1E        |
| SHT03  | Humidity and Temperature Sensor               | [SHT03 Data Sheet](Documentation/datasheets/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf) | 0x44        |

### Available Buttons and LEDs on Board

There is one tactile push button (connected to P26) and one LED (connected to P25) on-board. 

When attached to the programming JIG, another button and another LED (with a solder bridge) is available for testing. The JIG's button is connected to P9, and the LED is connected to P10, which are used for NFC functionality, too. See the next section for further information.

### NFC Functionality

An additional NFC (Near field communication) antenna can be attached to allow for an easy paring process with iPhone and Android mobile phones. 

**Important:** If NFC is to be used, you must remove the preprocessor define  `CONFIG_NFCT_PINS_AS_GPIOS` to not use the NFC pins as GPIO pins but for NFC. If it is defined, pins P0.09 and P0.10 serve as GPIO pins.

**Remark:** NFC functionality is not yet implemented but planned for a future release. The board already provides the points to attach the antenna with.

## Programming the Device

### Installing nRF Util

If not already done, nRF Util needs to be installed, see [nRF Util](https://infocenter.nordicsemi.com/index.jsp?topic=%2Fug_nrfutil%2FUG%2Fnrfutil%2Fnrfutil_intro.html).

### Install nRF Command Line Tools

If not already done, nRF Command Line Tools needs to be installed, see [nRF Command Line Tools](https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Command-Line-Tools).

### Perform a full erase

Either use the erase function in nRF Connect Programmer or nRFgo Studio, or use:

```
nrfjprog -e
```

### Add Environment Variable for SDK Base Path

For an Windows environment, add an environment variable with `setx`, for example:

```
setx NRF_SDK_PATH "C:\msys32\home\AKAEM\nrf52\nRF5_SDK_16.0.0_98a08e2"
setx NRF_APP_PATH "%NRF_SDK_PATH%\projects\ble_peripheral\ble_beacon"
setx NRF_BL_PATH  "%NRF_SDK_PATH%\projects\dfu\secure_bootloader"
```

In the following steps the SDK base path, application path and bootloader path are referenced by `%NRF_SDK_PATH%`, `%NRF_APP_PATH%` and `%NRF_BL_PATH%` . You can append `/M` to make the environment variable system wide, and not just for the user.

### Detailed Information on DFU Update process

A good starting point on DFU, package creation, validation, etc. is provided by Nordic at infocenter.nordic.com, section [Device Firmware Update Process](https://infocenter.nordicsemi.com/topic/sdk_nrf5_v16.0.0/lib_bootloader_dfu_process.html). In particular for validation and acceptance rules look at [link](https://infocenter.nordicsemi.com/topic/sdk_nrf5_v16.0.0/lib_bootloader_dfu_validation.html).

### Preparation Application zip File

**Step 1)** Create zip file with the application

- Go to the application build directory with the application already build:

  ```
  cd %NRF_APP_PATH%/pca10040/s132/ses/Output/Release/Exe
  ```
  
- If not done yet, copy the `private.key` to this directory:

  ```
  cp %NRF_APP_PATH%/Keys/private.key .
  ```

- Generate the package:

  ```
  nrfutil pkg generate --hw-version 52 --application-version 1 --application ble_beacon_pca10040_s132.hex --sd-req 0xCB --key-file private.key app_dfu_package.zip
  ```

If you want to generate a **package containing SD+BL+APP**, use the command line (example):

```
nrfutil pkg generate --hw-version 52 --application-version 1 --application ble_beacon_pca10040_s132.hex --bootloader-version 1 --bootloader %NRF_BL_PATH%/pca10040_s132_ble/ses/Output/Release/Exe/secure_bootloader_ble_s132_pca10040.hex --sd-req 0xA8,0xAF,0xB7,0xC2,0xCB --sd-id 0xCB --softdevice %NRF_SDK_PATH%/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex --key-file private.key app_bl_sd_dfu_package.zip
```



Command to create package for the HW NRF52832:

```
nrfutil pkg generate --hw-version 52
```

with the arguments

- APP version and HEX file

  ```
  --application-version 1 --application ble_beacon_pca10040_s132.hex
  ```

- BL version and HEX file

  ```
  --bootloader-version 1 --bootloader %NRF_BL_PATH%/.../secure_bootloader_ble_s132_pca10040.hex
  ```

- SD required and new version, and HEX file

  ```
  --sd-req 0xA8,0xAF,0xB7,0xC2,0xCB --sd-id 0xCB --softdevice %NRF_SDK_PATH%/.../s132_nrf52_7.0.1_softdevice.hex
  ```

- Private Key file

  ```
  --key-file private.key
  ```

- and the output ZIP file name

  ```
  app_bl_sd_dfu_package.zip
  ```

**Remark:** 

- A detailed description can be found in this thread on devzone.nordicsemi.com: [link](https://devzone.nordicsemi.com/f/nordic-q-a/40050/softdevice-update-through-dfu).

* The current SoftDevice version string is required using `--sd-req`. Use the NRF Connect Programmer and do a `read` to get the version string. 
* If you upgrade the SoftDevice, `--sd-req` should contain both, current and future version(s) in a comma separated list.
* The future SoftDevice version is given using `--sd-id`.
* List of recent S132 SoftDevices with version strings:

| SDK Version            | SD Version | SD Version String |
| ---------------------- | ---------- | ----------------- |
| SDK 16.0.0             | 7.0.1      | 0xCB              |
|                        | 7.0.0      | 0xC2              |
| SDK 15.3.0             | 6.1.1      | 0xB7              |
| SDK 15.2.0, SDK 15.1.0 | 6.1.0      | 0xAF              |
| SDK 15.0.0             | 6.0.0      | 0xA8              |

### Preparation of Bootloader Settings and Merge Settings with Bootloader to one .hex

**Step 2)** Generate settings for the bootloader

- Go to bootloader application build directory with the bootloader already build: 

  ```
  cd %NRF_BL_PATH%/pca10040_s132_ble/ses/Output/Release/Exe
  ```
  
- Generate the settings:

  ```
  nrfutil settings generate --family NRF52 --application %NRF_APP_PATH%/pca10040/s132/ses/Output/Release/Exe/ble_beacon_pca10040_s132.hex --application-version 0 --bootloader-version 0 --bl-settings-version 2 bootloader_setting.hex
  ```

**Step 3)** Merge the bootloader and settings file to allow flashing the application together with the bootloader/settings during production and without the need to update using DFU

- Go to the bootloader application build directory:

  ```
  cd %NRF_BL_PATH%/pca10040_ble/ses/Output/Release/Exe
  ```
  
- Merge the bootloader and settings file:

  ```
  mergehex -m bootloader_setting.hex secure_bootloader_ble_s132_pca10040.hex --output output.hex
  ```

### Program the Device

**Step 4)** Use nRF Connect Programmer to upload all necessary .hex files

- Open nRF Connect Programmer, `connect`, `read` device
- `Add HEX file`
  - Softdevice .hex file
  - Merged settings and bootloader file generated in Step 3
  - Application .hex file generated during application build
- Press `Erase & Write`

The files are located here:

| File                 | HEX File Path                                                |
| -------------------- | ------------------------------------------------------------ |
| SoftDevice           | %NRF_SDK_PATH%/components/softdevice/s132/hex/s132_nrf52_....hex |
| Merged settings + BL | %NRF_BL_PATH%/pca10040_ble/ses/Output/Release/Exe/output.hex |
| Application          | %NRF_BL_PATH%/pca10040_ble/ses/Output/Release/Exe/<br />ble_beacon_pca10040_s132.hex |

**Step 5)** (optional) If necessary, write 4 byte UICR with the devices MAJOR and MINOR address

- To write (requires previous delete, i.e. set to 0xFF): 

  ```
  nrfjprog -f nrf52 --memwr 0x10001080 --val 0x000700FF (adjust!)
  ```

- To read the current flashed values: 

  ```
  nrfjprog -f NRF52 --memrd 0x10001080 --n 4
  ```

### Update the Device using DFU

**Step 6)** To update the application use DFU and the application zip file generated as in Step 1) using nRF Connect Bluetooth Low Energy application on Windows (with nrf52832 dongle) or iPhone/Android app. The same procedure holds if you want to update APP+BL+SD with one package.

#### Remark: Using without bootloader and DFU

If you want to use the software without bootloader and without buttonless DFU again, you need to comment out the call to `ble_dfu_buttonless_async_svci_init();` or use `#undef USE_BUTTONLESS_DFU`. Otherwise you'll get an error  `<error> app: ERROR 4 [NRF_ERROR_NO_MEM]`.

## User Configuration of device modes

The push button can be used to configure the device mode. To configure the device you first have to enter configuration mode. The configuration mode will be left automatically after an idle time of 10 seconds without a keypress.

### Entering configuration mode

Long push the button (at least 2 seconds) to enter configuration mode. To confirm that you entered configuration mode, you will see the following LED feedback:

| Action           | Description       | LED feedback         |
| ---------------- | ----------------- | -------------------- |
| Button long push | Enter config mode | &#183; &#183; &#183; |

### Change device mode

**Push** the button to change the mode. The number of short LED flashes gives information on the current selected mode.

| Mode | Description                                                  | LED feedback                  |
| ---- | ------------------------------------------------------------ | ----------------------------- |
| 0    | no sensor, no advertising                                    | &horbar;                      |
| 1    | sensor active, store to offline buffer, but no advertising   | &#183; &horbar;               |
| 2    | sensor active, store values to offline buffer, non-scannable non-connectable advertising (advertising interval 1 sec) | &#183; &#183; &horbar;        |
| 3    | sensor active, store values to offline buffer, scannable connectable advertising (advertising interval 1 sec) | &#183; &#183; &#183; &horbar; |

**Remark:** See below for a power consumption comparison for these modes.

### Delete Bluetooth Bonds

If you need to delete Bluetooth bonds, **long press** the push button (>2 seconds) while in configuration mode. The following steps are performed, and you will see the corresponding LED feedback:

- Switch to mode 1 (no advertising)
- Perform Delete bonds
- (Device will stay in mode 1 and needs to be set manually to the desired mode)
- (After idle timeout, config mode will be left, see below)

| Action                                       | Description  | LED feedback                                                 | Steps                                             |
| -------------------------------------------- | ------------ | ------------------------------------------------------------ | ------------------------------------------------- |
| Button long push (during configuration mode) | Delete bonds | &#183; &horbar; <br />&horbar; &horbar; &horbar;<br />&#183; &#183; &#183; &#183; &#183; | Mode 1<br />Delete bonds<br />(Leave config mode) |

### Leaving configuration mode

After 10 seconds without a keypress (i.e., idle), the configuration mode will be left again. To confirm that you left the configuration mode, you will see the following LED feedback:

| Action       | Description       | LED feedback                       |
| ------------ | ----------------- | ---------------------------------- |
| Idle timeout | Leave config mode | &#183; &#183; &#183; &#183; &#183; |

## Provided Services by Device

You can connect to the device and use the following additional features:

- Download the stored data point from the device
- Get number of data points available
- Delete data points
- Get provided features
- Update software using DFU

### Connect to the Device using nRF Connect 

- You can use the following tools, besides other, to connect to the device:
  - nRF Connect Bluetooth Low Energy tool to connect to the device from Windows with an nrf52832/nrf52840 dongle
  - iPhone nRF Connect app
  - Android nRF Connect app

### Preparation

- Connect to the service 0x1400
- Pairing/Bonding - **TBD**
- Turn on **Notification** on characteristic 0x1401 (RACP Measurement Values)
- Turn on **Indication** on characteristic 0x2A52 (Random Access Control Point)

### Get Data from Device with respect to RACP

You can retrieve data from the device or delete the device offline buffer memory using the following commands written to UUID `0x2A52`: 

| Command | Description                | Response 2A52                                          | Response 1401                                                |
| ------- | -------------------------- | ------------------------------------------------------ | ------------------------------------------------------------ |
| 01 ...  | Report Records             | 06-00-01-01<br />(Response, Operator NULL, ?, Success) | 0..n notifications, one for each database entry to be reported |
| 01 05   | Report Records, First      | -                                                      |                                                              |
| 01 06   | Report Records, Last       |                                                        |                                                              |
| 02 ...  | Delete Records             | 06-00-02-01<br />tbd                                   | none                                                         |
| 02 01   | Delete Records, All        |                                                        |                                                              |
| 04 ...  | Report Number Records      | 05-00-0A-00<br />tbd                                   |                                                              |
| 04 01   | Report Number Records, All |                                                        |                                                              |


| Cmd    | Description                | Response 2A52                                          | Response 1401 |
| ------ | -------------------------- | ------------------------------------------------------ | ------------- |
| 01 01  | Report Records, All        | 06-00-01-01<br />(Response, Operator NULL, ?, Success) |               |
| 01 05  | Report Records, First      | (see Report Records, All)                              |               |
| 01 06  | Report Records, Last       | (see Report Records, All)                              |               |
|        |                            | -                                                      | -             |
| 02 01  | Delete Records, All        |                                                        |               |
|        |                            | -                                                      | -             |
| 04 01  | Report Number Records, All | 05 00 0A 00<br />tdb                                   |               |

**Remark:** the byte order is LSB-MSB

- **write** to 0x2A52 the following commands:

  - `01 01` (Report Records, All)   -> `2A52: 06 00 01 01` (Response, Operator NULL, ?, Success), `1401: 00-00-01-00-00-00-60-AD-74-15` (2 byte seq. number = 0, 4 byte time (here: not yet real time, thus 1 sec), 2 byte temperature `60 AD`, 2 byte humidity `74 15`)
  - `01 05` (Report Records, First) -> see `01 01`
  - `01 06` (Report Records, Last)  -> see `01 01`
  - `04 01` (Report Number Records, All) -> `2A52: 05 00 01 00` (Number of stored records response, number of records 2 byte  LSB first,  here: 1 record)
  - `02 01` (Delete All Records)

  

#### Decode sequential number/Number of entries

The number of records in return to the Report Number Records request and the sequential numbers in the records itself are 2 byte, which are coded LSB first, than MSB.

#### Decode time stamp

The time stamp is reported in the records as seconds since 01.01.1970, 00:00:00 (UTC), and is given in 4 Bytes with LSB-MSB ordering. 

The time stamp can be decoded using an appropriate online calculator, for example

- Unix Epoch time calculator, e.g. https://www.unixtimestamp.com/index.php

If the beacon does not have the correct time, the time stamps are reported since boot time, which is 0. If the correct time is available, this will be used.

#### Example

- Value received, see the console log below

- `0A-00-75-D5-0D-5E-5F-17-57-1C` 

  - Sequence number: 0x000A = 10
  - Time stamp: 0x5E0DD575 = ‭1577964917‬ = 01/02/2020 @ 11:35am (UTC)
  - Temperature `60 AD` = 
  - Humidity `74 15` = 

- Corresponding log file entries from NRF Connect console

  Connecting to the device:
  
  ```
  11:44:19.028	Connecting to device
  11:44:29.244	Connected to device D7:59:9D:1D:7B:6B
  11:44:29.521	Attribute value read, handle: 0x03, value (0x): 42-78-30-37-30-38
  11:44:31.298	Connection parameters updated for device D7:59:9D:1D:7B:6B: interval 200ms, timeout 4000ms, latency: 0
  11:44:36.107	Security updated, mode:1, level:2
  ```
  
  Change notification for `handle 0x26` `UUID 1401`:
  
  ```
  11:44:40.906	Attribute value changed, handle: 0x26, value (0x): 01-00
  11:44:40.911	Attribute value written, handle: 0x26, value (0x): 01-00
  ```
  
  Change indication for `handle 0x2B` `UUID 2A52`:
  
  ```
  11:44:42.906	Attribute value changed, handle: 0x2B, value (0x): 02-00
  11:44:42.910	Attribute value written, handle: 0x2B, value (0x): 02-00
  ```
  
  Retrieve last data set using write `01 06` to  `handle 0x2A` `UUID 2A52`:
  
  ```
  11:44:46.707	Attribute value changed, handle: 0x2A, value (0x): 01-06
  11:44:46.715	Attribute value written, handle: 0x2A, value (0x): 01-06
  11:44:46.722	Attribute value changed, handle: 0x25, value (0x): 0A-00-75-D5-0D-5E-5F-17-57-1C
  11:44:47.107	Attribute value changed, handle: 0x2A, value (0x): 06-00-01-01
  ```

- Link to the detailed NRF Connect log file: [Link](Documentation/program_docu/2020-01-02_nrf_connect_log.txt)



### Get Features provided by the Device

The Bluetooth characteristics 0x1402 can be read to get the provided features by the device. Currently the following features are supported:

| Define                           | Value  | Feature                                            |
| -------------------------------- | ------ | -------------------------------------------------- |
| BLE_OS_FEATURE_LOW_BATT          | 0x0001 | Low Battery Detection During Measurement Supported |
| BLE_OS_FEATURE_TEMPERATURE       | 0x0002 | Temperature Measurement Supported                  |
| BLE_OS_FEATURE_TEMPERATURE_ALERT | 0x0004 | Temperature Alert Supported                        |
| BLE_OS_FEATURE_HUMIDITY          | 0x0008 | Humidity Measurement Supported                     |
| BLE_OS_FEATURE_HUMIDITY_ALERT    | 0x0010 | Humidity Alert Supported                           |
| BLE_OS_FEATURE_ACCEL             | 0x0020 | Accel Alert Supported                              |
| BLE_OS_FEATURE_ACCEL_ALERT       | 0x0040 | Accel Alert Supported                              |
| BLE_OS_FEATURE_MALFUNC           | 0x0100 | Sensor Malfunction Detection Supported             |
| BLE_OS_FEATURE_TIME_FAULT        | 0x0200 | Time Fault Supported                               |
| BLE_OS_FEATURE_MULTI_BOND        | 0x0400 | Multiple Bond Supported                            |

### Sensor Status Annunciation

The Bluetooth characteristics 0x1403 can be read to get devices status annunciation. Currently the following annunciations are supported:

| Define                           | Value  | Annunciation                                                 |
| -------------------------------- | ------ | ------------------------------------------------------------ |
| BLE_OS_MEAS_STATUS_BATT_LOW      | 0x0001 | Device battery low at time of measurement                    |
| BLE_OS_MEAS_STATUS_SENSOR_FAULT  | 0x0002 | Sensor malfunction or faulting at time of measurement        |
| BLE_OS_MEAS_STATUS_GENERAL_FAULT | 0x0004 | General device fault has occurred in the sensor              |
| BLE_OS_MEAS_STATUS_TIME_FAULT    | 0x0008 | Time fault has occurred in the sensor and time may be inaccurate |
| BLE_OS_MEAS_STATUS_TIME_NOT_SET  | 0x0010 | Time is not set by CTS, thus showing and using seconds since startup |

### Status data and update commands

The Bluetooth characteristics 0x1404 can be read to get device status data and written to give commands.

Currently the following status data can be read by reading UUID 0x1404, for example ` 00-05-00-4C-00-00-00-04-00-0A-00`. In this example the time is not set using CTS.



| Topic                               | Bytes   | Example     | Result            |
| ----------------------------------- | ------- | ----------- | ----------------- |
| Flags                               | [0]     | 00          | none yet          |
| Sequence number                     | [1..2]  | 05 00       | = 0x0005 = 5      |
| Time stamp                          | [3..6]  | 4C 00 00 00 | = 0x0000004C = 76 |
| Number of free DB entries           | [7..8]  | 04 00       | = 0x0004 = 4      |
| Max. number of DB entries (DB size) | [9..10] | 0A 00       | = 0x000A = 10     |



The following commands can be given to the device by writing to UUID 0x1404:

| Define | Value | Annunciation |
| ------ | ----- | ------------ |
| B      |       |              |
|        |       |              |
|        |       |              |
|        |       |              |

### Battery Information Service (BAS)

The Bluetooth service "Battery Information Service" is available as service 0x180F, and provides a single value characteristic 0x2A19. This will return the battery level in percentage.

### Device Information Service (DIS)

The Bluetooth service "Device Information Service" is available as service 0x180A, and provides the following characteristics:

| UUID   | Title                                                        | Example             |
| ------ | ------------------------------------------------------------ | ------------------- |
| 0x2A24 | Model Number String                                          | 1                   |
| 0x2A25 | Serial Number String                                         | 30 (="1")           |
| 0x2A26 | Firmware Revision String, i.e. version of the SDK used       | 98a08e2             |
| 0x2A27 | Hardware Revision String                                     | 1.0                 |
| 0x2A28 | Software Revision String, i.e. version of the project software used | 0.3-94-g556752e     |
| 0x2A29 | Manufacturer Name                                            | "ansprechendeKunst" |

#### Set values for DIS

To set the values returned by DIS, use the files `ble_beacon_dis.inc` and `ble_beacon_dis_sw.inc`.

| Title                    | File to change          | Update                                                     |
| ------------------------ | ----------------------- | ---------------------------------------------------------- |
| Model Number String      | `ble_beacon_dis.inc`    | Manual update                                              |
| Serial Number String     | `ble_beacon_dis.inc`    | Manual update                                              |
| Firmware Revision String | `ble_beacon_dis.inc`    | Manual update, e.g. after SDK update                       |
| Hardware Revision String | `ble_beacon_dis.inc`    | Manual update, e.g. when using a new HW                    |
| Software Revision String | `ble_beacon_dis_sw.inc` | Currently manual, but retrieve automatically in the future |
| Manufacturer Name        | `ble_beacon_dis.inc`    | Manual update                                              |

**Remark:** The file `ble_beacon_dis.inc` is under version control, `ble_beacon_dis_sw.inc` is not under version control because value changes after check-in and can be retrieved by a `git describe --tags`.

### Secure DFU 

The Bluetooth service "Secure DFU" is available as service 0xFE59, and provides characteristic to update the device buttonless using Nordic's Secure DFU. 

## Used Services by Device

### Current Time Service (CTS)

As soon as the device is connected, it checks whether a CTS (Current Time Service) server is provided by the central device. If it is available, the following steps will be performed:

- If CTS and thus the current time is available for the **first time** (e. g., directly after booting):
  - The current time is fetched from CTS  
  - The internal function (nrf_calendar) to keep the time is set to the current time.  
  - If there are any entries stored in the offline buffer, these entries will be updated with the correct time. This is done by calculating the delta, and adding this correction value to all previous entries.
  
- For **subsequent updates** of the time using CTS:
  - The current time is fetched from CTS
  - The internal function (nrf_calendar) will be updated with the current time.
  - The drift will be calculated and for future requests inside the beacon used as a correction factor.
  - **Remark:** Previous values eventually stored in the offline buffer will not be updated/corrected with the drift factor.

## Power Consumption

The software is power optimized. 

| Mode | Description                                                  | Power Consumption (&#181;A) |
| ---- | ------------------------------------------------------------ | --------------------------- |
| 0    | no sensor, no advertising (but some timer active)            | 3.9                         |
| 1    | sensor active, store to offline buffer, but no advertising   | 5.4                         |
| 2    | sensor active, store values to offline buffer, non-scannable non-connectable advertising (advertising interval 1 sec) | 13.5                        |
| 3    | sensor active, store values to offline buffer, scannable connectable advertising (advertising interval 1 sec) | 17.5                        |
| 3'   | same as 3, but in connected state<br />- during first 2 sec with advertising interval 7.5 ms<br />- after 2 sec with advertising interval 200 ms | <br />450<br />24           |
| all  | in **idle state** (=sleep), i.e. between peaks from sensor handling, <br />advertising, timer, etc. | 3.7                         |

**Remark:**

- The measurement is done with Nordic [Power Profiler Kit](https://www.nordicsemi.com/Software-and-tools/Development-Kits/Power-Profiler-Kit)
- SHT03 sensor
  - The temperature and humidity sensor used has a typical power consumption of 0.2 &#181;A, and a maximum of 2 &#181;A (idle state when in single shot mode). With some devices I experienced a higher consumption but still below the max specifications. 
  - The figures above are measured with a sensor with a consumption of ~0.4 &#181;A. 
  - See [SHT03 Data Sheet](Documentation/datasheets/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf), section *2.1 Electrical Specifications*.
- The mode corresponds to the LED flashing code, i.e. the number of short flashes equals the mode. See section *User Configuration of device modes*

### Mode 0

![Alt text](Documentation/power_consumption/Mode_0.PNG?raw=true "Mode 0 Power Consumption")

### Mode 1

![Alt text](Documentation/power_consumption/Mode_1.PNG?raw=true "Mode 1 Power Consumption")

### Mode 2

![Alt text](Documentation/power_consumption/Mode_2.PNG?raw=true "Mode 1 Power Consumption")



### Mode 3

![Alt text](Documentation/power_consumption/Mode_3.PNG?raw=true "Mode 3 Power Consumption")

### Mode 3 Connected

During the first 2 seconds the connection interval is set to 7.5 ms. Then, by a connection parameter update using `sd_ble_gap_conn_param_update()`, the connection interval is set to 200 ms.

#### Connected state, during first 2 seconds

![Alt text](Documentation/power_consumption/Mode_3a.PNG?raw=true "Mode 3 Connected (first 2 sec) Power Consumption")

#### Connected state, after first 2 seconds

![Alt text](Documentation/power_consumption/Mode_3b.PNG?raw=true "Mode 3 Connected (after 2 sec) Power Consumption")

### Details: Idle state

![Alt text](Documentation/power_consumption/Mode_Idle.PNG?raw=true "Power Consumption single sensor retrieval")

### Details: Single Advertising (Mode 3)

![Alt text](Documentation/power_consumption/Mode_3_single_adv.PNG?raw=true "Mode 3 single advertising Power Consumption")

### Details: Single Sensor retrieval

![Alt text](Documentation/power_consumption/sensor_retrieval.PNG?raw=true "Power Consumption single sensor retrieval")

## Programming JIG

There is a separate Readme for the programming jig, see [here](Documentation/JIG/README.md)

![Alt text](Documentation/JIG/IMG_JIG_4_resize.jpg?raw=true "Complete JIG")

## Pin Assignments and Schematics

### PIN Assignment

| Pin   | Name       | Description                           |
| ----- | ---------- | ------------------------------------- |
| 26    | SWDIO      | TP1                                   |
| 25    | SWDCLK     | TP2                                   |
| 13/48 | VDD        | TP3                                   |
| 45    | VSS        | TP4                                   |
| 31    | GND        | GND                                   |
| 37    | P0.25      | Tactile switch on-board               |
| 38    | P0.26      | LED on-board                          |
| 42    | P0.30      | SCL                                   |
| 43    | P0.31      | SDA                                   |
| 11    | NFC1/P0.09 | NFC antenna **or** JIG tactile switch |
| 12    | NFC2/P0.10 | NFC antenna **or** JIG LED            |

### Schematics

The schematics for the beacon I use is available here. The product is available from Radioland China using ALIEXPRESS.

![Alt text](Documentation/beacon_hardware/nRF52832+KX022+SHT30%20circuit.jpg?raw=true "Complete JIG")

## Technical Development Topics

### Set Makefile.windows path

For the installed tool chain, use the following `Makefile.windows`

 ```
AKAEM@PC MINGW32 ~/nrf52/nRF5_SDK_16.0.0_98a08e2
$ cat ./components/toolchain/gcc/Makefile.windows
GNU_INSTALL_ROOT := /opt/gcc-arm-none-eabi-8-2018-q4-major-win32/bin/
GNU_VERSION := 8.2.1
GNU_PREFIX := arm-none-eabi
 ```

### Set up Nordic Power Profiler Kit

The Nordic Power Profiler Kit (PPK) is configured as follows: See the PPK manual for more information: [link](https://infocenter.nordicsemi.com/pdf/PPK_User_Guide_v2.2.pdf).

The setup is described in section *6.6 Measuring current on custom hardware without nRF5 DK*

- Only the PPK is used, no Nordic Development Kit (DK)
- The External 5V supply USB connector (J1) of the PPK is used to supply power to the onboard analog measurement circuitry and the onboard regulator with 5 V.
- Set the Power select switch (SW4) in the "Reg" position.
- The DUT select switch (SW2) is in the "External" position.
- The custom hardware (DUT) is connected to the External DUT connector (P16) of the PPK
- The additional SEGGER J-Link is connected to the Debug in connector (P21) on the PPK using the 10-pin flat cable. The USB cable is plugged into the SEGGER J-Link and connected to a computer running the Power Profiler application.
- The COM switch (SW3) is in the "EXT" position.

### Strange Issues

#### ERROR 33281 [NRF_ERROR_DRV_TWI_ERR_ANACK] 

I'd a hard time with the error `0> <error> app: ERROR 33281 [NRF_ERROR_DRV_TWI_ERR_ANACK]` which appeared when accessing TWI devices using `nrf_drv_twi_tx()`. Actually, this was caused by an insufficient power supply. On top of that, I had two overlaying problems:

- a bad USB cable, and
- the power provided by the Nordic Power Profiler Kit was ~2.2 V. 

The reason for the latter issue is not clear and needs to be follow up, either the PPK itself or the J-Link Mini Edu used.

#### Peer manager fails with NRF_ERROR_STORAGE_FULL

When receiving this error message, it was not that the flash was full, but that there's some garbage left in flash. Using `nrfjprog -e` to erase all flash and upload worked fine. See this [link](https://devzone.nordicsemi.com/f/nordic-q-a/44931/peer-manager-fails-with-nrf_error_storage_full).

Detailed error message:

```
0> <info> app: ble_beacon - main started.
 0> <error> peer_manager_pds: Could not initialize flash storage. fds_init() returned 0x860A.
 0> <error> peer_manager: pm_init failed because pds_init() returned NRF_ERROR_STORAGE_FULL.
 0> <error> app: ERROR 3 [NRF_ERROR_INTERNAL] at C:\msys32\...\projects\ble_peripheral\ble_beacon\main.c:2317
 0> PC at: 0x00033035
 0> <error> app: End of error report
```


```

```