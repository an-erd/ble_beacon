# ble_beacon

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


Step 1)	 "Erase all" using nRFgo Studio by nRF5x Programming 
Step 2)  Flash SoftDevice using nRFgo Studio by nRF5x Programming 
Step 3)  Flash bootloader using nRFgo Studio by nRF5x Programming 
Step 4)  Write UICR using DOS Prompt: nrfjprog -f nrf52 --memwr 0x10001080 --val 0x00070005 (adjust!)
Step 4b) (optional) Verify UICR using DOS Prompt: nrfjprog -f NRF52 --memrd 0x10001080 --n 4
Step 5)  Use DFU to install app, e.g. with nRF Connect on PC or Mobile device

Create zip with app
Step 1) go to _build directory with the app already build

	AKAEM@PC MINGW32 ~/nrf52/nRF5_SDK_15.3.0_59ac345/projects/ble_peripheral/ble_beacon/pca10040/s132/ses/Output/Release/Exe
	$ cp ../../../../../../Keys/private.key .^C

	AKAEM@PC MINGW32 ~/nrf52/nRF5_SDK_15.3.0_59ac345/projects/ble_peripheral/ble_beacon/pca10040/s132/ses/Output/Release/Exe
	$ nrfutil pkg generate --hw-version 52 --application-version 1 --application ble_beacon_pca10040_s132.hex --sd-req 0xb7 --key-file private.key app_dfu_package.zip


AKAEM@PC MINGW32 ~/nrf52/nRF5_SDK_15.3.0_59ac345/projects/ble_peripheral/ble_beacon/pca10040/s132/ses/Output/Debug/Exe
$ nrfutil pkg generate --hw-version 52 --application-version 0 --application ble_beacon_pca10040_s132.hex --sd-req 0xb7 --key-file private.key app_dfu_package1.zip
Zip created at app_dfu_package1.zip

AKAEM@PC MINGW32 ~/nrf52/nRF5_SDK_15.3.0_59ac345/projects/dfu/secure_bootloader/pca10040_ble/ses/Output/Release/Exe
$ nrfutil settings generate --family NRF52 --application ../../../../../../../ble_peripheral/ble_beacon/pca10040/s132/ses/Output/Debug/Exe/ble_beacon_pca10040_s132.hex --application-version 0 --bootloader-version 0 --bl-settings-version 1 bootloader_setting.hex

Note: Generating a DFU settings page with backup page included.
This is only required for bootloaders from nRF5 SDK 15.1 and newer.
If you want to skip backup page generation, use --no-backup option.

Generated Bootloader DFU settings .hex file and stored it in: bootloader_setting.hex

Bootloader DFU Settings:
* File:                     bootloader_setting.hex
* Family:                   nRF52
* Start Address:            0x0007F000
* CRC:                      0xE6EF6F2A
* Settings Version:         0x00000001 (1)
* App Version:              0x00000000 (0)
* Bootloader Version:       0x00000000 (0)
* Bank Layout:              0x00000000
* Current Bank:             0x00000000
* Application Size:         0x00020638 (132664 bytes)
* Application CRC:          0x29E0DEEC
* Bank0 Bank Code:          0x00000001
* Softdevice Size:          0x00000000 (0 bytes)
* Boot Validation CRC:      0x00000000
* SD Boot Validation Type:  0x00000000 (0)
* App Boot Validation Type: 0x00000000 (0)

c:\msys32\home\AKAEM\nrf52\nRF5_SDK_15.3.0_59ac345\projects\dfu\secure_bootloader\pca10040_ble\ses\Output\Release\Exe>mergehex -m bootloader_setting.hex secure_bootloader_ble_s132_pca10040.hex --output output.hex

Mit nRF Connect Programmer in einem Rutsch:
	SoftDevice
	output.hex (settings für die App (siehe unten) und Bootloader)
	offen UICR?!
	die passende App
	

FC:4C:B4:F2:10:03	FC:4C:B4:F2:10:04	beac1
EB:30:72:AB:35:37	EB:30:72:AB:35:38 	beac2
E8:1C:81:77:36:E4	beac3
D4:21:12:9D:E7:F6	beac4
F1:A0:51:21:A0:13	beac5
DB:AE:BA:AB:67:2E	DB:AE:BA:AB:67:2F	beac6

F1:A0:51:21:A0:12

Sketch
	- advertising w/o connecting -> broadcast temperature/humidity/accel to everyone
	- BLE connection -> successful
		CTS Update
		1) if on_cts_c_evt = BLE_CTS_C_EVT_DISCOVERY_COMPLETE 
			initiate time update
		1a) if on_cts_c_evt = BLE_CTS_C_EVT_CURRENT_TIME
			update buffer entries w/o time? TODO
		1b) if BLE_CTS_C_EVT_DISCOVERY_FAILED or BLE_CTS_C_EVT_INVALID_TIME
		
		

20:13:54.732	Attribute value changed, handle: 0x2A, value (0x): 01-03-01-02-00
20:13:54.738	Attribute value written, handle: 0x2A, value (0x): 01-03-01-02-00
20:13:54.750	Attribute value changed, handle: 0x25, value (0x): 02-00-03-00-00-00-61-FF-7D-B6
20:13:54.750	Attribute value changed, handle: 0x25, value (0x): 03-00-04-00-00-00-62-05-7D-B0
20:13:54.750	Attribute value changed, handle: 0x25, value (0x): 04-00-05-00-00-00-61-FA-7D-AD
20:13:54.750	Attribute value changed, handle: 0x2A, value (0x): 06-00-01-01

https://www.unixtimestamp.com/index.php