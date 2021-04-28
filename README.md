

# A Bluetooth Low Energy Beacon (ble_beacon)

## Overview

**ble_beacon** is a software for the Nordic Semiconductor SoC NRF52832 and similar. It reads sensor data (in our case temperature, humidity and acceleration data) and sends it using Bluetooth Low Energy advertisement packet. Getting a history of data points is available when establishing a BLE connection to the device.

The code is power optimized. As of today, the average power consumption during unconnectable undirected advertising is 13.5 &#181;A, and with connectable undirected advertising ~17.4 &#181;A (see below for more detailed figures).

Using the push button, different modes (sensor, advertising, etc.) and deleting Bluetooth bonds can be configured.

### Advertising 

The device supports two different advertising modes:

- non-scannable non-connectable advertising
- scannable connectable advertising

The main target of providing different modes and in particular non-scannable non-connectable advertising is to have a reduced power consumption while sending payload data.

**Both advertising modes** will provide the following payload:

- Major and Minor ID for Identification of the device
- Temperature values
- Humidity values
- Acceleration X/Y/Z readings
- Battery voltage

When in **scannable connectable advertising mode** in addition you will receive using the Scan Response:

- a string (currently just set to `MyTherResp`) (TDB in future)
- the devices full name (e.g. `Bx0708` for Beacon with Major ID 0x07 and Minor ID 0x08)

and you can connect to the device!

In section [Understanding the Advertising Data](#understanding-the-advertising-data) below you will get more information on the Advertising data and how to read it. 

### Provided Services


In addition you can connect to the device and use additional features such as download and deletion of data points or DFU device update. See below in section [Provided Services by Device](#provided-services-by-device) for additional information.

### Offline Buffer Functionality

Beside the advertising of the sensor measurements the beacon can store the measurements in an offline buffer. The offline buffer is located  in RAM, i.e. with a reboot the values will be reset, too. Currently 20.000 bytes are reserved for the offline buffer, with an size of 16 bytes for one entry, this gives 20.000/16 bytes = 1.250 entries. With an interval of 15 min/entry the beacon will store data for around 13 days. If the buffer is full, the oldest value will be deleted (i.e. a ring buffer is used). The data of the offline buffer can be accessed, downloaded and cleared when connected to the device.

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

### Example Data and Beacon used throughout this Document

In this Readme we use a single BLE beacon device with the following address `D7:59:9D:1D:7B:6B`. In the different log files it may also appear in in reverse order without delimiter and in lower case as `6b 7b 1d 9d 59 d7`. The device name for this Beacon is set to `Bx0708` and will be constructed using its major and minor id which will be set to: Major ID `0x07` and Minor ID `0x08`, preceded by `Bx` (as in Beacon).

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
| SDK 17.0.2             | 7.2.0      | 0x0101            |
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
- `Add HEX files`
  - Softdevice .hex file
  - Merged settings and bootloader file generated in Step 3
  - Application .hex file generated during application build
- Press `Erase & Write`

The files are located here:

| File                 | HEX File Path                                                |
| -------------------- | ------------------------------------------------------------ |
| SoftDevice           | %NRF_SDK_PATH%/components/softdevice/s132/hex/s132_nrf52_....hex |
| Merged settings + BL | %NRF_BL_PATH%/pca10040_ble/ses/Output/Release/Exe/output.hex |
| Application          | %NRF_APP_PATH%/pca10040_ble/ses/Output/Release/Exe/<br />ble_beacon_pca10040_s132.hex |

**Step 5)** (optional) If necessary, write 4 byte UICR with the devices MAJOR and MINOR address

- To write (requires previous delete, i.e. set to 0xFF): 

  ```
  nrfjprog -f nrf52 --memwr 0x10001080 --val 0x000700FF (adjust!)
  ```

- To read the current flashed values: 

  ```
  nrfjprog -f NRF52 --memrd 0x10001080 --n 4
  ```

Example: For the test device used throughout this Readme the value `0x00070008` would be used.

### Update the Device using DFU

**Step 6)** To update the application use DFU and the application zip file generated as in Step 1) using nRF Connect Bluetooth Low Energy application on Windows (with nrf52832 dongle) or iPhone/Android app. The same procedure holds if you want to update APP+BL+SD with one package.

#### Remark: Using without bootloader and DFU

If you want to use the software without bootloader and without buttonless DFU again, you need to comment out the call to `ble_dfu_buttonless_async_svci_init();` or use `#undef USE_BUTTONLESS_DFU`. Otherwise you'll get an error  `<error> app: ERROR 4 [NRF_ERROR_NO_MEM]`.

#### Remark: Current packages and corresponding versions

| Github Tag | Package Name (.zip)            | App Version | BL Version | BL Settings | SD Req |
| ---------- | ------------------------------ | ----------- | ---------- | ----------- | ------ |
| 0.4.1      | app2_dfu_package_0.4.1         | 2           | &horbar;   | &horbar;    | 0xCB   |
| 0.4.1      | app2_bl2_sd_dfu_package        | 2           | 2          | 2           | 0xCB   |
| 0.4.1b     | app3_dfu_package_0.4.1b        | 3           | &horbar;   | &horbar;    | 0x0101 |
| 0.4.1b     | app3_bl3_sd_dfu_package_0.4.1b | 3           | 3          | 2           | 0x0101 |

(Version 0.4.1a is only a new SD/BL Version without code changes, so Github tag is 0.4.1)

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



## Understanding the Advertising data

Depending on the configured Advertising Mode (Mode 2 and Mode 3 as discussed in [Change device mode](#change-device-mode)) you will receive non-scannable non-connectable advertising or scannable connectable advertising. Thus we will explain advertising split up in

- the header
- the data retrieved for non-scannable non-connectable advertising mode (Mode 2) and
- the data retrieved for scannable connectable advertising (Mode 3).

I used NRF Connect and the log files provided by the tool to get the detailed logs used below.

**Remark:** 

- For understanding and testing it is necessary to convert between ASCII and numbers. A tool to do the job and convert between hex/binary/decimal numbers and ASCII text can be found [here](https://www.rapidtables.com/convert/number/ascii-hex-bin-dec-converter.html).
- A calculator for 2s complement can be found [here](tototo). (TODO)
- All relevant information on Bluetooth and Advertising can be found in the following documents/links:
  - [Bluetooth Core Specification](https://www.bluetooth.com/specifications/bluetooth-core-specification/)
    - Document "CS Core Specification", Chapter 11 "ADVERTISING AND SCAN RESPONSE DATA FORMAT" 
    - Document "CSS Core Specification Supplement"
    - [BT4 Core Spec, Adv data reference, Chapter 11 and 18](https://www.libelium.com/forum/libelium_files/bt4_core_spec_adv_data_reference.pdf)
  - [List of AD Types](https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/)
  - [KBA_BT_0201: Bluetooth advertising data basics](https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/02/10/bluetooth_advertisin-hGsf)
  - [KBA_BT_0202: Bluetooth advertising using manufacturer specific data](https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/11/14/bluetooth_advertisin-zCHh)
  - [One minute to understand BLE advertising data package](https://github.com/greatscottgadgets/ubertooth/wiki/One-minute-to-understand-BLE-advertising-data-package)

#### BLE Beacon used throughout in this example

See above in section [Example Data and Beacon used throughout this Document](#example-data-and-beacon-used-throughout-this-document) on the address and name used throughout with document and example.

#### Header of Advertising Packet

In our example the header of the advertising packet received is:

- Non-scannable non-connectable advertising (Mode 2):

    ```
    02 1d 00 ff ff 02 6b 7b 1d 9d 59 d7 00 04 99 42 19 eb 84 ce 06 17 01 02 01 06 13 ff
    ```
    | Length  | Type                     | Data              | Result                                                   |
    | ------- | ------------------------ | ----------------- | -------------------------------------------------------- |
    |         |                          | 02 1d 00 ff ff 02 |                                                          |
    | Address | [6..11]                  | 6b 7b 1d 9d 59 d7 | D7:59:9D:1D:7B:6B                                        |
    |         |                          |                   |                                                          |
    |         |                          |                   |                                                          |
    | 0x02    | 0x1 (Flags)              | 0x06              | LE General Discoverable Mode, <br />BR/EDR Not Supported |
    | 0x13    | 0xFF (Manufacturer data) | (see below)       |                                                          |


- Mode 3, advertising packet and scan response:

  ```
  02 1d 00 ff ff 02 6b 7b 1d 9d 59 d7 20 04 99 42 19 eb 84 ca 00 17 01 02 01 06 13 ff 
  02 1d 00 ff ff 02 6b 7b 1d 9d 59 d7 20 04 99 42 19 eb 84 cb 01 17 01 0e ff
  ```

For the further explanation we split up the packets into the following parts:

- Common header
  ``` 
  02 1d 00 ff ff 02 6b 7b 1d 9d 59 d7
  ```



#### Non-Scannable Non-Connectable Advertising (Mode 2)

Received Data as in NRF Connect log file:

```
2020-03-07T11:21:07.708Z DEBUG   110/ 0 <-  [02 1d 00 ff ff 02 6b 7b 1d 9d 59 d7 00 04 99 42 19 eb 84 ce 06 17 01 02 01 06 13 ff 59 00 00 07 00 08 5d 9a 66 6a f4 ff 3e ff 60 3f 0b a6 ] type:     VENDOR_SPECIFIC reliable:yes seq#:3 ack#:5 payload_length:2e data_integrity:1 header_checksum:25 err_code:0x0
```

```
2020-03-07T11:21:07.715Z DEBUG GAP_EVT_ADV_REPORT/ADV_NONCONN_IND time:2020-03-07T11:21:07.707Z connHandle:65535 rssi:50 peerAddr:[address:D7:59:9D:1D:7B:6B type:randomStatic addrIdPeer:0] scanRsp:false advType:advNonconnInd gap:[adTypeFlags:[leGeneralDiscMode,brEdrNotSupported,leOnlyLimitedDiscMode,leOnlyGeneralDiscMode] manufacturerSpecificData:89,0,0,7,0,8,93,154,102,106,244,255,62,255,96,63,11,166]
```

##### Manufacturer Specific Data Mode 2 (Advertising packet)

In our example the manufacturer specific data contained in the advertising packet received is:

```
59 00 00 07 00 08 5d 9a 66 6a f4 ff 3e ff 60 3f 0b a6 (hex numbers)
89,0,0,7,0,8,93,154,102,106,244,255,62,255,96,63,11,166 (decimal numbers)
```

The following table gives information on how to interpret the payload:

| Topic              | Bytes    | Example | Result                                        |
| ------------------ | -------- | ------- | --------------------------------------------- |
| Company Identifier | [0..1]   | 59 00   | = 0x0059, Nordic Semiconductor                |
| Major              | [2..3]   | 00 07   | = 0x0007                                      |
| Minor              | [4..5]   | 00 08   | = 0x0008                                      |
| Temperature        | [6..7]   | 5D 9A   | = 18,99 = -45 + (0x5D9A * 175) / 0xFFFF (° C) |
| Humidity           | [8..9]   | 66 6A   | = 40,01 = (0x666A * 100) / 0xFFFF (% RH)      |
| Acceleration X     | [10..11] | F4 FF   | = 0xFFF4, 2's complement = -11                |
| Acceleration Y     | [12..13] | 3E FF   | = 0xFF3E, 2's c = -193                        |
| Acceleration Z     | [14..15] | 60 3F   | = 0x3F60, 2's c = ‭+16224‬                      |
| Battery Voltage    | [16..17] | 0B A6   | = 0x0B6A = 2922 mV                            |

Remark: The temperature and humidity data are sent using the sensors native 2 byte format, with the formula given in the table above. See [SHT03 Data Sheet](Documentation/datasheets/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf), section *4.13 Conversion of Signal Output* for more information. 

#### Scannable Connectable Advertising (Mode 3)

Received Data as in NRF Connect log file for the advertising packet:

```
2020-03-07T11:03:19.388Z DEBUG    49/ 0 <-  [02 1d 00 ff ff 02 6b 7b 1d 9d 59 d7 20 04 99 42 19 eb 84 ca 00 17 01 02 01 06 13 ff 59 00 00 07 00 08 5d 65 65 c8 b4 ff 2a ff 26 3f 0b 94 ] type:     VENDOR_SPECIFIC reliable:yes seq#:6 ack#:5 payload_length:2e data_integrity:1 header_checksum:22 err_code:0x0
```

```
2020-03-07T11:03:19.396Z DEBUG GAP_EVT_ADV_REPORT/ADV_IND time:2020-03-07T11:03:19.388Z connHandle:65535 rssi:54 peerAddr:[address:D7:59:9D:1D:7B:6B type:randomStatic addrIdPeer:0] scanRsp:false advType:advInd gap:[adTypeFlags:[leGeneralDiscMode,brEdrNotSupported,leOnlyLimitedDiscMode,leOnlyGeneralDiscMode] manufacturerSpecificData:89,0,0,7,0,8,93,101,101,200,180,255,42,255,38,63,11,148]
```

and the scan response:

```
2020-03-07T11:03:19.401Z DEBUG    50/ 0 <-  [02 1d 00 ff ff 02 6b 7b 1d 9d 59 d7 20 04 99 42 19 eb 84 cb 01 17 01 0e ff 59 00 4d 79 54 68 65 72 52 65 73 70 00 07 09 42 78 30 37 30 38 ] type:     VENDOR_SPECIFIC reliable:yes seq#:7 ack#:5 payload_length:2e data_integrity:1 header_checksum:21 err_code:0x0
```

```
2020-03-07T11:03:19.402Z DEBUG GAP_EVT_ADV_REPORT time:2020-03-07T11:03:19.391Z connHandle:65535 rssi:53 peerAddr:[address:D7:59:9D:1D:7B:6B type:randomStatic addrIdPeer:0] scanRsp:true gap:[manufacturerSpecificData:89,0,77,121,84,104,101,114,82,101,115,112,0 completeLocalName:Bx0708]
```

##### Manufacturer Specific Data Mode 3 (Advertising packet)

The manufacturer specific data is the same as for the Non-Scannable Non-Connectable Advertising (Mode 2), thus see [Manufacturer Specific Data Mode 2 (Advertising packet)](#manufacturer-specific-data-mode-2-(advertising-packet)).

##### Manufacturer Specific Data Mode 3 (Scan Response)

In our example the manufacturer specific data contained in the scan response received is: 

```
59 00 4d 79 54 68 65 72 52 65 73 70 00 07 09 42 78 30 37 30 38 (hex numbers)
89,0,77,121,84,104,101,114,82,101,115,112,0 (decimal numbers)
```

The following table gives information on how to interpret the payload:

| Topic              | Bytes    | Example                       | Result                                      |
| ------------------ | -------- | ----------------------------- | ------------------------------------------- |
| Company Identifier | [0..1]   | 59 00                         | = 0x0059, Nordic Semiconductor              |
| Data response      | [2..11]  | 4d 79 54 68 65 72 52 65 73 70 | = "MyTherResp" <br />(hex-ascii conversion) |
| Separator          | [12]     | 00                            |                                             |
| Length             | [13]     | 07                            |                                             |
| Type               | [14]     | 09                            | Complete local name                         |
| Device Name        | [15..20] | 42 78 30 37 30 38             | = "Bx0708"                                  |
| Separator          | [21]     | 00                            |                                             |



## Provided Services by Device

You can connect to the device and use the following additional features:

- Download the stored data point from the device

- Get number of data points available

- Delete data points

- Get provided features

- Update software using DFU

### List of Bluetooth Services provided

| Service                     | UUID   | Handle | Comment |
| --------------------------- | ------ | ------ | ------- |
| Generic Access              | 0x1800 | 0x01   | &horbar; |
| &horbar; Device Name     | 0x2A00 | 0x03   | Read Write |
| &horbar; Appearance  | 0x2A01 | 0x05   | Read |
| &horbar; Peripheral Preferred Connection Parameters | 0x2A04 | 0x07   | Read |
| &horbar; Central Address Resolution | 0x2AA6 | 0x09   | Read |
| Generic Attribute           | 0x1801 | 0x0A | &horbar; |
| &horbar; Service Changed | 0x2A05 | 0x0C | Indicate |
| &horbar; &horbar; CCCD | 0x2902 | 0x0D | &horbar; |
| Device Information Service  | 0x180A | 0x0E       | &horbar; |
| Secure DFU                  | 0xFE59 | 0x1B | &horbar; |
| Battery Information Service | 0x180F | 0x1F | &horbar; |
| &horbar; Battery Level | 0x2A19 | 0x21 | Read Notify |
| &horbar; &horbar; CCCD | 0x2902 | 0x22 | &horbar; |
| Own Beacon Service          | 0x1400 | 0x23 | &horbar; |
| &horbar; RACP Measurement Values | 0x1401 | 0x25 | Notify |
| &horbar; &horbar; CCCD | 0x2902 | 0x26 | &horbar; |
| Features provided by the Device | 0x1402 | 0x28 | Read |
| Random Access Control Point (RACP) | 0x2A52 | 0x2A | Write Indicate |
| &horbar; &horbar; CCCD | 0x2902 | 0x2B | &horbar; |
| Sensor Status Annunciation | 0x1403 | 0x2D | Read Write |
| Status data and update commands | 0x1404 | 0x2F | Read Write |

### Connect to the Device using nRF Connect 

- You can use the following tools, besides other, to connect to the device:
  - nRF Connect Bluetooth Low Energy tool to connect to the device from Windows with an nrf52832/nrf52840 dongle
  - iPhone nRF Connect app
  - Android nRF Connect app

### Preparation

- Connect to the service 0x1400
- Pairing/Bonding - will be done automatically during negotiation process between device and client
- Turn on **Notification** on characteristic 0x1401 (RACP Measurement Values)
- Turn on **Indication** on characteristic 0x2A52 (Random Access Control Point)

### Get Data from Device with respect to RACP

You can retrieve data from the device or delete the device offline buffer memory using the following commands written to UUID `0x2A52`: 

| Command | Description                | Response 2A52                                          | Response 1401                                                |
| ------- | -------------------------- | ------------------------------------------------------ | ------------------------------------------------------------ |
| 01 ...  | Report Records             | 06-00-01-01<br />(Response, Operator NULL, ?, Success) | 0..n notifications, one for each database entry to be reported |
| 01 01   | Report Records, All        | 06-00-01-01<br />(Response, Operator NULL, ?, Success) |                                                              |
| 01 05   | Report Records, First      | -                                                      |                                                              |
| 01 06   | Report Records, Last       |                                                        |                                                              |
| 02 ...  | Delete Records             | 06-00-02-01<br />tbd                                   | none                                                         |
| 02 01   | Delete Records, All        |                                                        |                                                              |
| 04 ...  | Report Number Records      | 05-00-0A-00<br />tbd                                   |                                                              |
| 04 01   | Report Number Records, All |                                                        |                                                              |

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

#### Decode Temperature and Humidity

Temperature and Humidity are given in 2 byte MSB-LSB ordering and are calculated using the sensors native 2 byte format. See [SHT03 Data Sheet](Documentation/datasheets/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf), section *4.13 Conversion of Signal Output* for more information.

- Temperature T in ° C where ST denotes the raw sensor output for temperature in decimal representation:
  <a href="https://www.codecogs.com/eqnedit.php?latex=T&space;[^{\circ}&space;C]&space;=&space;-&space;45&space;&plus;&space;175&space;\cdot&space;\frac{S_{T}}{2^{16}-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?T&space;[^{\circ}&space;C]&space;=&space;-&space;45&space;&plus;&space;175&space;\cdot&space;\frac{S_{T}}{2^{16}-1}" title="T [^{\circ} C] = - 45 + 175 \cdot \frac{S_{T}}{2^{16}-1}" /></a>


- Humidity RH in % RH where SRH denotes the raw sensor output for humidity in decimal representation:
  <a href="https://www.codecogs.com/eqnedit.php?latex=RH&space;=&space;100&space;\cdot&space;\frac{S_{RH}}{2^{16}-1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?RH&space;=&space;100&space;\cdot&space;\frac{S_{RH}}{2^{16}-1}" title="RH = 100 \cdot \frac{S_{RH}}{2^{16}-1}" /></a>
#### Example

- Value `0A-00-75-D5-0D-5E-5F-17-57-1C` received, see the console log below:

  - Sequence number: 0x000A = 10
  - Time stamp: 0x5E0DD575 = ‭1577964917‬ = 01/02/2020 @ 11:35am (UTC)
  - Temperature `0x5F17` = 24.343; results in T = -45 + (24.343  * 175) / 65535 = 20,00 ° C
  - Humidity `0x751C` = 29.980; results in RH = 45,75 % RH
#### Corresponding log file entries from NRF Connect console

- Connecting to the device:

  ```
  11:44:19.028	Connecting to device
  11:44:29.244	Connected to device D7:59:9D:1D:7B:6B
  11:44:29.521	Attribute value read, handle: 0x03, value (0x): 42-78-30-37-30-38
  11:44:31.298	Connection parameters updated for device D7:59:9D:1D:7B:6B: interval 200ms, timeout 4000ms, latency: 0
  11:44:36.107	Security updated, mode:1, level:2
  ```

- Change notification for `handle 0x26` `UUID 1401`:

  ```
  11:44:40.906	Attribute value changed, handle: 0x26, value (0x): 01-00
  11:44:40.911	Attribute value written, handle: 0x26, value (0x): 01-00
  ```

- Change indication for `handle 0x2B` `UUID 2A52`:

  ```
  11:44:42.906	Attribute value changed, handle: 0x2B, value (0x): 02-00
  11:44:42.910	Attribute value written, handle: 0x2B, value (0x): 02-00
  ```

- Retrieve last data set using write `01 06` to  `handle 0x2A` `UUID 2A52`:

  ```
  11:44:46.707	Attribute value changed, handle: 0x2A, value (0x): 01-06
  11:44:46.715	Attribute value written, handle: 0x2A, value (0x): 01-06
  11:44:46.722	Attribute value changed, handle: 0x25, value (0x): 0A-00-75-D5-0D-5E-5F-17-57-1C
  11:44:47.107	Attribute value changed, handle: 0x2A, value (0x): 06-00-01-01
  ```

**Remark:** Link to the detailed NRF Connect log file: [Link](Documentation/program_docu/2020-01-02_nrf_connect_log.txt)


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

| Command | Description                                                  | xxx  |
| ------- | ------------------------------------------------------------ | ---- |
| 01 01   | (Re-)Read time and update DB entries, take no drift into account |      |
|         |                                                              |      |
|         |                                                              |      |
|         |                                                              |      |

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

Using this figures the **battery life time** can be calculated:

- In Mode 2 (sensor active, offline buffer but non-scannable non-connectable advertising):
  220 mAh / 0,0135 mA * 0,7 = 11.407 h = ~1.3 Years
- In Mode 3 (in addition scannable connectable advertising):
  220 mAh / 0,0175 mA * 0,7 = 8.800 h = ~1 Year

**Remark:**

- The measurement is done with Nordic [Power Profiler Kit](https://www.nordicsemi.com/Software-and-tools/Development-Kits/Power-Profiler-Kit)
- SHT03 sensor
  - The temperature and humidity sensor used has a typical power consumption of 0.2 &#181;A, and a maximum of 2 &#181;A (idle state when in single shot mode). With some devices I experienced a higher consumption but still below the max specifications. 
  - The figures above are measured with a sensor with a consumption of ~0.4 &#181;A. 
  - See [SHT03 Data Sheet](Documentation/datasheets/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital-971521.pdf), section *2.1 Electrical Specifications*.
- The mode corresponds to the LED flashing code, i.e. the number of short flashes equals the mode. See section *User Configuration of device modes*
- In comparison, the supplier’s original firmware of that devices has a consumption of 577 μA, which corresponds to ~11 days (220mAh/577μA * 70% = ~267h), and which is not acceptable

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

### Long Term Visualization 04/2019-04/2020

![Alt text](Documentation/power_consumption/Sensor_data_temp_20190415-20200420.png?raw=true "Long Term Visualization")

![Alt text](Documentation/power_consumption/Sensor_data_hum_20190415-20200420.png?raw=true "Long Term Visualization")

![Alt text](Documentation/power_consumption/Sensor_data_battery_20190415-20200420.png?raw=true "Long Term Visualization")

**Remarks:** 

- The battery life time of beacon Bx0701 ended on 2020-03-22, and was replaced by 2020-03-24. The graph is configured to use a "moving average" of the last 40 values. This is the reason why there is an increase in battery power again. Without "moving average" the see the following graph. (There is a measurement issues when the sensor values fallen down to the lowest value. These cases are now excluded as "outliers".)

  ![Alt text](Documentation/power_consumption/Sensor_data_battery_Bx0701_20190417-20200420.png?raw=true "One year of battery life")

- If the device shuts down and doesn't come up again, probably the power supply/battery voltage is too low. In this case one of the sensors (SHT3, KX022) does not come up, and so the complete device. 
  - Specifications for SHT3 power-up/down level V_POR is min./typ./max.: 2.1, 2.3, 2.4 V
  - Specifications for KX022 operating power is min./typ./max.: 1.71, 2.5, 3.6 V
  
- Another view on the end-of-battery for two beacons ("From the cradle to the grave"), and a pretty new battery:
  ![Alt text](Documentation/power_consumption/Sensor_data_battery_20200323-20200423.png?raw=true "From the cradle to the grave")



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

### Reset device with nrfjprog

To reset a device with a J-Link attached, use

```
nrfjprog --reset
```



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


#### GATT Client not able to pair, error code = 0x52

On the GATT Client, in this case an ESP32 running esp-idf, shows a `pair status = fail`.  The error code `0x52` corresponds to `BTA_DM_AUTH_SMP_PAIR_NOT_SUPPORT`.

On the GATT Client the log file reads:

```
I (6690) GATTC_DEMO: remote BD_ADDR: dbaebaab672e
I (6700) GATTC_DEMO: address type = 1
I (6700) GATTC_DEMO: pair status = fail
I (6710) GATTC_DEMO: fail reason = 0x52
```


On the NRF52 Beacon device is shown:

```
00> <info> peer_manager_handler: Connection security failed: role: Peripheral, conn_handle: 0x1, procedure: Bonding, error: 133
```

**Deleting bonds** on the beacon should correct the problem, see [Delete Bluetooth Bonds](#delete_bluetooth_bonds). 

On the NRF52 Beacon device is then shown:

```
I (7850) GATTC_DEMO: remote BD_ADDR: dbaebaab672e
I (7860) GATTC_DEMO: address type = 1
I (7860) GATTC_DEMO: pair status = success
I (7870) GATTC_DEMO: auth mode = ESP_LE_AUTH_BOND
```

#### GATT Client SMP_ENC_FAIL, error code = 0x61

On the GATT Client, in this case an ESP32 running esp-idf, shows a `pair status = fail`.  The error code `0x61` corresponds to `SMP_ENC_FAIL` (=smp_api.h, SMP_ENC_FAIL).

On the GATT Client the log file reads:

```
I (147769) BLEMQTTPROXY: remote BD_ADDR: dbaebaab672e
I (147769) BLEMQTTPROXY: address type = 1
I (147769) BLEMQTTPROXY: pair status = fail
I (147779) BLEMQTTPROXY: fail reason = 0x61
```

The problem appears to happen if the distance is to high or the connection is disturbed and thus the authentication cannot be completed.

#### ESP_GATTC_DISCONNECT_EVT, reason = 8

On the GATT Client, in this case a disconnect w/reason 8 occurs. Again, if the distance is to high or connection disturbed this error occurs.

```
W (3547729) BT_APPL: bta_gattc_conn_cback() - cif=3 connected=0 conn_id=3 reason=0x0008
D (3547729) BLEMQTTPROXY: ESP_GATTC_DISCONNECT_EVT
I (3547729) BLEMQTTPROXY: ESP_GATTC_DISCONNECT_EVT, reason = 8
```

#### SEGGER RTT VIEWER Debug output broken 

If you have additional empty lines or a non-working debug log in RTT Viewer you can work around this by adding/setting in `sdk_config.h`, see [nrf_log-not-working-on-segger-embedded-studio](https://devzone.nordicsemi.com/f/nordic-q-a/45985/nrf_log-not-working-on-segger-embedded-studio/182742#182742):

```
#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED 0
```

#### Reference: GATT Client error codes

See `esp-idf\components\bt\bluedroid\bta\include\bta\bta_api.h` and [List of error codes](https://github.com/chegewara/esp32-ble-wiki/issues/5).