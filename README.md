## Balancing cube

![The cube balances on its corner using three integrated reaction wheels](media/cube.jpg)

This repository contains all design, documentation, configuration and other files related to the balancing cube that I have designed and built.
* Mechanical parts are designed in [Fusion 360](https://www.autodesk.com/products/fusion-360/personal).
* Circuit boards are designed in [KiCad 7](https://www.kicad.org/download/windows/).
* Code has been written using [Arduino IDE](https://docs.arduino.cc/software/ide-v2/tutorials/getting-started/ide-v2-downloading-and-installing/).
* Motor drivers are commissioned and tuned using [Escon Studio](https://www.maxongroup.nl/maxon/view/content/ESCON-Detailsite?isoCode=nl).
* Motors and motor controllers are available from [Maxon](https://www.maxongroup.com/maxon/view/content/index).
* Circuit boards and custom mechanical parts can be ordered from [PCBWay](https://www.pcbway.com/rapid-prototyping/), for example.

A YouTube video demonstrating the balancing cube is available [here](https://youtu.be/zGclFqkZBsk).

### Want to build one? Read this!

This repository should contain everything you need to order and/or build parts for this cube. Take into account the following:

* The Molex cable assemblies which deliver power to the motors (part number [215325-1081](https://mouser.com/ProductDetail/Molex/215325-1081?qs=DuoOggApuK9vh2w3ntfmSQ%3D%3D)) need to be modified. The design requires that each pin is mapped to the same pin number on the other end (1-1, 2-2, etc) but this is by default not the case, as can be seen in the [datasheet](https://www.molex.com/content/dam/molex/molex-dot-com/products/automated/en-us/salesdrawingpdf/215/215325/2153251081_sd.pdf). You can remove the circuits from their housing using [this](https://www.digikey.nl/nl/products/detail/molex/0011030044/210947) tool. Cheaper (generic) "terminal removal kits" are available too. I made a small tool myself from two folded pieces of sheet metal which I cut from a SMT soldering stencil. Alternatively, you can modify the motherboard schematic to compensate for the default circuit arrangement of the cable assembly, or build your own cable assembly from scratch using Molex part numbers [39012085](https://www.molex.com/en-us/products/part-detail/39012085) and [39000077](https://www.molex.com/en-us/products/part-detail/39000077). This is the cheapest and arguably best (from a technical point of view) option!
* I replaced four of the eight 16AWG wires in each cable assembly by 22AWG wires since only four wires carry large currents. The others carry +5V and hall sensor signals. This reduces weight and makes the cables easier to handle, as well as reducing strain on the mating connectors. The motor connection is rather fragile and might break if stressed too much.
* The ESP32 board does not go into flashing mode automatically. I fixed this by adding a 10 uF capacitor between the enable and reset pin, as described [here](https://randomnerdtutorials.com/solved-failed-to-connect-to-esp32-timed-out-waiting-for-packet-header/).
* In my blog post, I describe some errors in the design of the motherboard. These have been resolved in the files that are available in this repository.
* For your own safety, I highly recommend taking precautions to reduce the risk of trapping fingers between the spinning flywheels. One option is to attach very thin (spring steel) discs to each wheel, which may be (laser)cut from larger sheets that are readily available. I do not have designs available for such safety devices yet.

### More information

More information about this project and my other projects is available on my [website](https://willempennings.nl/balancing-cube/).

