# NINJA-ECU-Firmware
Firmware for ECU board in order to control sensors and actuators of ARPG's self driving vehicle

## How firmware versions work?
each ECU-Firmware has a version format of **v.**_X_**.**_Y_ in which
* X represents the major change in functionality of the board
* Y represents minor changes like fixing errors in PCB or adding simple functions like an LED

## How to add a new firmware  with major version change?
versions of firmware should be in sync with [NINJA-ECU](https://github.com/arpg/NINJA-ECU) board versions. so make sure that branch names and tags match with the ones in NINJA-ECU repository. 
* branch from current master (this should only add some readme files and folders)
* create your project in the folders created when you branched from master
* when you are ready to push your changes, create a new branch and name it as 'v.major_number' (ex. **v.3** )
* after you are done with your code add a minor tag of 1 to your last commit (ex. add tag **v.3.1**)

## How to add minor changes for last major board?
* apply and commit your changes when you were done add a minor tag for your commit (ex. add tag **v.3.2**)
