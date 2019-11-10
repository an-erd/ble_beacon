# ble_beacon

## Programming the device

### Preparation Application zip file

Step 1) Create zip file with the application
- go to the application build directory with the application already build, e.g. cd ~/nrf52/nRF5_SDK_15.3.0_59ac345/projects/ble_peripheral/ble_beacon/pca10040/s132/ses/Output/Release/Exe
- if not done yet, copy the private.key to this directory, e.g. cp ../../../../../../Keys/private.key .
- generate the package, e.g. nrfutil pkg generate --hw-version 52 --application-version 1 --application ble_beacon_pca10040_s132.hex --sd-req 0xb7 --key-file private.key app_dfu_package.zip

### Preparation Bootloader settings and merge settings with bootloader to one .hex

Step 2) Generate settings for the bootloader
- go to bootloader application build directory with the bootloader already build, e.g.  ~/nrf52/nRF5_SDK_15.3.0_59ac345/projects/dfu/secure_bootloader/pca10040_ble/ses/Output/Release/Exe
- nrfutil settings generate --family NRF52 --application ../../../../../../../ble_peripheral/ble_beacon/pca10040/s132/ses/Output/Debug/Exe/ble_beacon_pca10040_s132.hex --application-version 0 --bootloader-version 0 --bl-settings-version 1 bootloader_setting.hex

Step 3) Merge the bootloader and settings file to loater allow to flash the application together with the bootloader/settings during production and without the need to update using DFU
- open a Windows cmd shell and go to the bootloader directory, e.g. c:\msys32\home\AKAEM\nrf52\nRF5_SDK_15.3.0_59ac345\projects\dfu\secure_bootloader\pca10040_ble\ses\Output\Release\Exe>
- mergehex -m bootloader_setting.hex secure_bootloader_ble_s132_pca10040.hex --output output.hex

### Program the device

Step 4) Use nRF Connect Programmer to upload all necessary .hex files
- Open nRF Connect Programmer, connect, read device
- Add HEX files
* Softdevice .hex file
* merged settings and bootloader file generated in Step 3
* application .hex file generated during application build

Step 5) (optional) If necessary, write UICR with the device MAJOR and MINOR address
- to read the current flashed values: nrfjprog -f NRF52 --memrd 0x10001080 --n 4
- to write (requires previous delete -> set to 0xFF): nrfjprog -f nrf52 --memwr 0x10001080 --val 0x000700FF (adjust!)

### Update the device using DFU

Step 6) to update the application use DFU and the application zip file generated as in Step 1) using nRF Connect Bluetooth Low Energy application on Windows (with nrf52832 dongle) or iPhone/Android app.

## Get offline buffer data from device

### Connect to the device using nRF Connect 

- use nRF Connect Bluetooth Low Energy tool to connect to the device from Windows with an nrf52832/nrf52840 dongle

- or use the iPhone nRF Connect app

- or use the Android nRF Connect app

- go to the service 1400

- turn on notification on 1401

- turn on indication on 2A52/Random Access Control Point

- write to 2A52 the following commands

* 01 01 (Report Records, All)   -> 2A52: 06 00 01 01 (Response, Operator NULL, ?, Success), 1401: 00-00-01-00-00-00-60-AD-74-15 (2 byte seq, 4 byte time, 2 byte temperature, 2 byte humidity)

* 01 05 (Report Records, First) -> see 01 01

* 01 06 (Report Records, Last)  -> see 01 01

* 04 01 (Report Num Records, All) -> 2A52: 05 00 01 00 (Number of stored records response, ?, num records 2B LSB first)

* 02 01 (Delete All Records)

### Decode sequential number

- just coded 2 byte, LSB first, than MSB

### Decode time stamp

- 4 bytes, LSB-MSB

- decode using a epoch Unix time calculator, e.g. https://www.unixtimestamp.com/index.php

### Old stuff, to be sorted

critical:
for the beacon PCB use the preprocessor define CONFIG_NFCT_PINS_AS_GPIOS

adv flags	
	len	 	02 				length
	type 	01 				type (flags)
	val 	04 				value (BR/EDR Not Supported)
adv header	
	len 	1a 
	type 	ff 				(custum manufacturer packet)
			59 00 			(manufacturer ID)
																APP_BEACON_INFO_LENGTH 0x17			
dev type	02  (iBeacon)
Data Len	0x15  = 21

UUID 		01 12 23 34 45 56 67 78 89 9a ab bc cd de ef f0
MAJ 		01 02
MIN			03 04 
RSSI		c3


0x0059	Nordic Semiconductor


new advertisement packet
adv flags		02 01 04			3
adv header		1a ff 59 00			4
dev type		02					1
data len		XX					1
UUID			xx xx xx xx			4
major id		xx xx				2
minor id 		xx xx				2
rssi			xx					1
temp			xx xx				2
humidity		xx xx				2
x				xx xx				2
y				yy yy				2
z				zz zz				2
battery			xx xx				2

SUM									30

result:
20190317-09:10 (0x00070001) rssi -59 | temp  19.0 | hum  23 | x   +154 | y   -250 | z +16242 | batt 2808
18:34	2814
20190318-07:00 (0x00070001) rssi -45 | temp  18.0 | hum  34 | x    -32 | y   -399 | z +16259 | batt 2784

nrfjprog -f NRF52 --memrd 0x10001080 --n 4



FC:4C:B4:F2:10:03	FC:4C:B4:F2:10:04	beac1
EB:30:72:AB:35:37	EB:30:72:AB:35:38 	beac2
E8:1C:81:77:36:E4	beac3
D4:21:12:9D:E7:F6	beac4
F1:A0:51:21:A0:13	beac5
DB:AE:BA:AB:67:2E	DB:AE:BA:AB:67:2F	beac6

F1:A0:51:21:A0:12





#define RACP_OPCODE_REPORT_RECS             1       /**< Record Access Control Point opcode - Report stored records. */
#define RACP_OPCODE_DELETE_RECS             2       /**< Record Access Control Point opcode - Delete stored records. */
#define RACP_OPCODE_ABORT_OPERATION         3       /**< Record Access Control Point opcode - Abort operation. */
#define RACP_OPCODE_REPORT_NUM_RECS         4       /**< Record Access Control Point opcode - Report number of stored records. */
#define RACP_OPCODE_NUM_RECS_RESPONSE       5       /**< Record Access Control Point opcode - Number of stored records response. */
#define RACP_OPCODE_RESPONSE_CODE           6       /**< Record Access Control Point opcode - Response code. */

/**@brief Record Access Control Point operators. */
#define RACP_OPERATOR_NULL                   0       /**< Record Access Control Point operator - Null. */
#define RACP_OPERATOR_ALL                    1       /**< Record Access Control Point operator - All records. */
#define RACP_OPERATOR_LESS_OR_EQUAL          2       /**< Record Access Control Point operator - Less than or equal to. */
#define RACP_OPERATOR_GREATER_OR_EQUAL       3       /**< Record Access Control Point operator - Greater than or equal to. */
#define RACP_OPERATOR_RANGE                  4       /**< Record Access Control Point operator - Within range of (inclusive). */
#define RACP_OPERATOR_FIRST                  5       /**< Record Access Control Point operator - First record (i.e. oldest record). */
#define RACP_OPERATOR_LAST                   6       /**< Record Access Control Point operator - Last record (i.e. most recent record). */
#define RACP_OPERATOR_RFU_START              7       /**< Record Access Control Point operator - Start of Reserved for Future Use area. */

/**@brief Record Access Control Point Operand Filter Type Value. */
#define RACP_OPERAND_FILTER_TYPE_TIME_OFFSET 1       /**< Record Access Control Point Operand Filter Type Value - Time Offset. */
#define RACP_OPERAND_FILTER_TYPE_FACING_TIME 2       /**< Record Access Control Point Operand Filter Type Value - User Facing Time. */

/**@brief Record Access Control Point response codes. */
#define RACP_RESPONSE_RESERVED               0       /**< Record Access Control Point response code - Reserved for future use. */
#define RACP_RESPONSE_SUCCESS                1       /**< Record Access Control Point response code - Successful operation. */
#define RACP_RESPONSE_OPCODE_UNSUPPORTED     2       /**< Record Access Control Point response code - Unsupported op code received. */
#define RACP_RESPONSE_INVALID_OPERATOR       3       /**< Record Access Control Point response code - Operator not valid for service. */
#define RACP_RESPONSE_OPERATOR_UNSUPPORTED   4       /**< Record Access Control Point response code - Unsupported operator. */
#define RACP_RESPONSE_INVALID_OPERAND        5       /**< Record Access Control Point response code - Operand not valid for service. */
#define RACP_RESPONSE_NO_RECORDS_FOUND       6       /**< Record Access Control Point response code - No matching records found. */
#define RACP_RESPONSE_ABORT_FAILED           7       /**< Record Access Control Point response code - Abort could not be completed. */
#define RACP_RESPONSE_PROCEDURE_NOT_DONE     8       /**< Record Access Control Point response code - Procedure could not be completed. */
#define RACP_RESPONSE_OPERAND_UNSUPPORTED    9       /**< Record Access Control Point response code - Unsupported operand. */
