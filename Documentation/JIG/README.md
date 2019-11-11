# Beacon JIG to program Bluetooth Low Energy Beacons

To easily mount the beacon to allow a comfortable and stable programming and debugging connection I use a JIG made out of Acrylic material. It is a low cost solution, too. After hours of using the jig I can give a very positive feedback with test needles pricisely hitting the test points - anyway, some ideas for improvements at the end. 

![Alt text](IMG_JIG_4_resize.jpg?raw=true "Complete JIG")

## Mount 

The jig to mount the beacon PCB and the test needles to program the beacon are cut with a laser from a 4 mm Acrylic GS
clear, transparent material. There are several services available with short delivery times.

The design used are in the files "P1_beacon_jig_MOUNT.svg". I use the right column of the design, i.e. a base plate, two plates which hold the PCB, and two top plates which hold the test needles. I prepared multiple plates with different hole diameter for the test needles and a small twist drill to finally get a fixed and precise hold of the complete JIG.

![Alt text](laser_cut_JIG.PNG?raw=true "Laser cut template")

For the test needles I use a product from PTR (Präzisionsprüfstift 1025/E-1.5N-AU-1.0), with a diameter of 1.37 mm. I found that a laser cut hole with a diameter of 1.21 mm works pretty well. See the following [Data sheet](737238-da-01-de-PRAEZISIONSPRUEFST_1025_B_1_5N_AU_1_0.pdf)

I use screws to fix the plates, while I glued the two bottom and the two top plates together, respectively. Unfortunately, I glued them in the middle, too, which is not good loking. Lessons learned...

![Alt text](IMG_JIG_3_resize.jpg?raw=true "JIG Parts")
![Alt text](IMG_JIG_5_resize.jpg?raw=true "JIG with mounted beacon PCB")
![Alt text](IMG_JIG_6_resize.jpg?raw=true "Zoom to JIG with mounted beacon PCB")

## PCB

The PCB is set on top of the test needles and is connected to the programming device. Since the button on the beacon is not accessible when mounted, I added an additional push button (P09). Also, one additional led with a solder bridge is available (P10).  

![Alt text](P1_beacon_jig_PCB.png?raw=true "JIG PCB")

## Futher enhancements and bugs

- Bug: The programming device pin connector is mirror-inverted (at least for my adapter)
- Enhancement: There should be a connector for power supply
- Enhancement: Use a clamp instead of screws to allow faster mounting, e.g. [Adafruit Toggle Clamp](https://www.adafruit.com/product/2456)
- Enhancement: Use shorter test needles / pogo pins
- Enhancement: Make an open mount, e.g. as a horse shoe, to allow accessing the button and thus have free pins for NFC antenna.
