# Scope Firmware (ARM)
----
Processor: [NRF51822](https://github.com/avatech-inc/Scope_Firmware_ARM/blob/documentation/doc/Scope%20Datasheets/nRF51_Series_Reference_manual%20v3.0.pdf)

----

## Revision History

### Version 0.4
* general bug fixes

### Version 0.3
* **Basic Errors**: too fast and too slow errors
* fixed bug where squal trigger can be triggered by pulling up
* **Status**: Status is now communicated from the device to the phone
* added debug file to be sent to phone on startup

### Version 0.2 (OR Demos)
* made advertisement packet shorter so that full name always displays

### Version 0.1
* **Basic features**: 
   * Taking a test, transfer to phone

----
## Documentation
1. [Development Setup](doc/dev_setup.md)
2. [Known Issues and Next Steps](doc/next_steps.md)
3. Software Outline
    * [State Machine](#state-machine)
    * [PIC-ARM Communication](#pic-arm-communication)
    * [BLE](#ble)
    * [Power and Sleep](#power-and-sleep)
    * [Firmware Update](#firmware-update)
    * [Accelerometer and Gyro](accelerometer-and-gyro)
    * [Calibration](#calibration)

----

### State Machine
The system is designed as an event-driven state machine. We chose this due its simplicity and ease of debugging as compared to an RTOS type of system. Maybe the system requirements have changed enought to revisit this. 

The ARM is always on. This chip is on when out of the pole but only sleeping when in the pole. 

### Test Flow
The point of the device to take tests. We have tried to make this as easy as possible. The device only works when it is tipped vertically up and the LED illumintes and the user feels a vibration. Accelerometer values are constantly being sent from the ARM to the PIC. Once the device has been tipped vertical, a circular buffer collects data from all sensors. At this point we automatically recognize when the device enters the snow from windowing algorithms on the SQUAL signal. After the snow is entered, TEST_TIME-CAL_TIME seconds of data are collected and processed. 

### PIC-ARM Communication
All of the communication with the phone or with the ARM chip must be done througha SPI communication channel between the PIC and the ARM boards. This is a part of the 10-pin header connector between the main board and the BLE board. We designed a custom communication protocol to meet our needs. In software we have created a structure type called pic_arm_pack_t, which consists of a code, a pointer to the data, and a data size in bytes. These packs are used to pass this info on to send_data_to_ARM() which initiates a transfer. There is an identical system on the ARM to send info to the PIC. In this case, however, we also two hardlines that are importnat for successful communication, the ARM_DRY line and the ARM_REQ line. We use the ARM_RDY line to signal tot he PIC that the ARM is ready to receive a packet. This is necessary because the PIC is the master in the SPI relationship and controls when to pull CS low or high which clocks data in/out on both platform no matter what. The ARM_REQ line is used by the ARM to signal to the PIC that it has data ready to be transferred. This allows us to have a somewhat multi-master system. Receiving data is done by first receiving a header to alert the chip about where the data is going and what it is saying.There is haeder pack that has code to dictate where to store the data (rx_data_ptr) and sets where the program flow will go (next_state), the system then wait for the expected amoutn of data to be recieved and change states when completed.

The ARM is the slave on this SPI bus, meaning that it cannot control when data is sent or received. It's sending function can only set the right pin to signal to the PIC to 'pull' the data in. After that it is all interrupt based on when the CS line goes low.

This system is very fragile and often breaks due to collisions and timing errors. Improving this, in my opinion would add the most value to future development.

### BLE
Ble is the reason why we have this chip. This SoC integrate a ble radio with ARM Cortex-MO for running the BLE stuff. We originally designed the system to work with the nrf8001 which is only the radio. We choose this chip because it can send up to 6 packets per connection interval as opposed to only on on the 8001. Even after this switch we are still out of date: they have released an nrf52 which supposedely will help us with BLE and power consumption.

The BLE communication work via a profile of services and characteristics. The [Scope Data Exchange doc](https://docs.google.com/spreadsheets/d/11Skohx51fUKSo8CJepi-iRT_FO2PIE0_o6SkqYJRFlI/edit#gid=1879330772) contains outline of all services and what infomation is transferred for each. The softdevice hides most of the BLE stuff but the stuff we can control is in main.c. Look at any of the service files to see how to set up a service with the correct permissions, characteristics, and update functions.

### Power and Sleep
Scope has a somewhat complicated sleep/power system due to its low power requirements and the fact that it's in a ski pole. The on/off switch is a magnet embedded in the handle. This magnet interacts with two magnetic sensors mounted on the board to tell if the device is in the handle or not. These sensor outputs go through an AND gate and then to the ARM board. The ARM controls the on/off of the PIC, in such a way that when the pole is in the handle the ARM diables the power supply that powers the PIC. IMPORTANT: there is an electrical mistake on the board in which the HALL signal connects to a PIC pin before going to the ARM. When we try to power down the PIC the voltage on this pin attmepts to power the PIC which drains currnet. To get ideal sleep currents, this pin on the PIC will need to be lifted or the baord will need to be rebuilt. I think we have already made this change in the PCB design files.

TODO: need to detail out the sleep stuff.

### Firmware Update
Updating the FW on the ARM is pretty easy once it's set up. FWU requires a bootloader which contains custom FW and a BLE Profile that connects to a special service we have set up on the app. Like the PIC, the ARM has a dual bank configuration so it can boot into either bank.The bootloader code handles the transfer of data from the phone to the opposite bank and booting into it. We still need to test that we can get back to the other bank in case of the FWU failure.

For updating firmware from the app, the code will need to be packaged as a zip. We use a utility called nrfutil. This is easily downlaoded and installed on Windows, but took a little inginuity to get it working on OSX. To generate the zip file need by the DFU api run the following command from the command line or terminal:
```
>>> nrfutil dfu genpkg --application path_to_hex_file.hex name_of_created_zip.zip
```


### Accelerometer and Gyro
The ARM board contains an Accelerometer ([LSM303D](https://github.com/avatech-inc/Scope_Firmware_ARM/blob/documentation/doc/Scope%20Datasheets/LSM303D.pdf)) and Gyro ([L3GD20H](https://github.com/avatech-inc/Scope_Firmware_ARM/blob/documentation/doc/Scope%20Datasheets/L3GD20H.pdf) for detemining orientation and helping with depth accuracy. They share a SPI bus connected to that ARM. Unfortunately, the ARm doesn't really need this data, it just sends it to the PIC. The device samples the accelerometer and gyro at 500Hz and sends this info to the PIC, which basically occupies the entire bandwhich of that SPI comms channel. We have run into lots of issues with sending accelerometer while trying to do something else. We currently have a system that disbales it if we are trying to do something that needs to control the PIC-ARM link for a little while. The accelerometer is sampled and sent on timer depending on if it is enabled or not. The PIC has the ability to enable and disbale it depending on when it needs acc data. The Arm also has the ability to override this if it wants to change the state flow. There is a lot of optimizatoin to be donw with this system.

NOTE: we currently not using or sending gyro info to the PIC. 

### Calibration
Unlike the PIC, the ARM firmware needs to be recompiled in a special mode to calibrate the device. To set the ARM in calibration mode the change `#define CALIBRATION 0` to `#define CALIBRATION 1`. Once in calibration mode, the reset functionality is disabled to be able to properly test the magnetic sensor system. The test flow is also disabled, so it will not vibrate and light up when vertically oriented. Furthermore, the BLE profile is different than in normal operations so there is a seperate calibration app that connects to this profile. The device will not work with the normal app when in calibration mode. The ARM mostly coordinates information transfer between the PIC and the phone. The two most important calibration procedures are the Optical Sensor and the Force Sensor. These sensors actually need to use a calibration value to be accurate as the sensors depend in the manufacturing practices. The other processes are for testing different systems.

### PCB Test
Another part of our QA procedure involves testing the PCB assembly before assmebling the entire product and calibrating it. This allows us to reject bad PCBs early in the process. The code lives in pcb_test.c. To execute the tests, the boards should be in the PCB test jig, which includes a UART link to a computer. The results fo the test are displayed over UART. The tests are run by `run_pcb_tests()` when the PIC sends over a command to tell the ARM that the device is in the PCB test jig and running the tests. This function has an array of function pointers that point to the actual tests. If adding a new test, write the test as a seperate function with no parameters and returning a uint8_t, then add its header as a funtion pointer to the array in `run_pcb_tests()`. These test are designed to check if the ICs are turning on and generally working.

