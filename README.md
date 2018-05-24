# earthquake_detector

Hardware low power device to trigger messages when detect earthquakes

## Getting Started

These instructions will get you a copy of the project up and running on your device.

### Prerequisites

Libraries needed:
Accelerometer library: https://github.com/Flummy/FluMMA865x

### Installing

Steps to flash Hidnseek device:

Long(First setup):
Follow instructions here: https://github.com/Bucknalla/sigfox-hidnseek-tutorial/blob/master/README.md

Short:
1. Take off top
2. Plug device into power source
3. Short G and R pins
4. Quickly upload firmware as programmer (Arduino IDE: Sketch->Upload using programmer)
(Important that step 4 is done quickly after step 3, while red light flashing on device)

## Running the tests

In DEBUG mode, prints to serial.

All easily modifiable values are in def.h:

*Variable clock fix loop
*Variable device sleep loop
*Variable gravity scale for accelerometer
*Variable accelerometer sensitivity threshold,
*Variable accelerometer sleep count after interrupt
*Variable accelerometer data rate for sleep and wake modes

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* C/C++ (Using Hidnseek device)

## Authors

* **Harrison Smith** - *Initial work* - [Earthquake_detector](https://github.com/dstar1)
* **Louis Moreau** - *Initial work* - [Earthquake_detector](https://github.com/luisomoreau)

See also the list of [contributors](https://github.com/sigfox-earthquake/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Sigfox
* Hidnseek
