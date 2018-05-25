# earthquake_detector

Hardware low power device to trigger messages when detect earthquakes.

## Getting Started

These instructions will get you a copy of the project up and running on your device.

### Prerequisites

Follow all instructions here to get hidnseek board working on arduino IDE:
https://github.com/hidnseek/hidnseek/tree/master/arduino

Libraries needed:
Hidnseek library: https://github.com/hidnseek/hidnseek

Accelerometer library: https://github.com/Flummy/FluMMA865x

Streaming library from: https://github.com/kachok/arduino-libraries

USB-Serial driver download: http://www.prolific.com.tw/US/ShowProduct.aspx?p_id=229&pcid=41

### Installing

Steps to flash Hidnseek device:

Long(First setup):
Follow instructions here: https://github.com/Bucknalla/sigfox-hidnseek-tutorial/blob/master/README.md

Short(Already setup):
1. Take off top
2. Plug device into power source
3. Short G and R pins
4. Quickly upload firmware as programmer (Arduino IDE: Sketch->Upload using programmer)
(Important that step 4 is done quickly after step 3, while red light flashing on device)

## Running the tests

In DEBUG mode, prints to serial.
To use USB-Serial port download(mentioned earlier): http://www.prolific.com.tw/US/ShowProduct.aspx?p_id=229&pcid=41

All easily modifiable values are in def.h:

* Variable clock fix loop
* Variable device sleep loop
* Variable gravity scale for accelerometer
* Variable accelerometer sensitivity threshold,
* Variable accelerometer sleep count after interrupt
* Variable accelerometer data rate for sleep and wake modes

## Built With

* C/C++ (Using Hidnseek device)

## Authors

* **Louis Moreau** - *Initial work* - [Earthquake_detector](https://github.com/luisomoreau)
* **Harrison Smith** - *Initial work* - [Earthquake_detector](https://github.com/dstar1)

See also the list of [contributors](https://github.com/sigfox-earthquake/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Sigfox
* Hidnseek
