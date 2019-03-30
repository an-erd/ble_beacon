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

