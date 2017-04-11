# Scope Development on ARM

### Setup

[Nordic tutorials](https://devzone.nordicsemi.com/tutorials/)

#### Keil uVision IDE with MDK Lite
Note this software is Windows only! I have mine runnign in a virtual machine but it is not as clean. Download Keil uVision IDE with the MD Lite. This means that you have code size limit of 32kB. We are very close to this and have had to make numerous concessions in order to work around this constraint. Also download the NRF5 SDK.

#### NRF51 Development Kit
In order to falsh the nrf board we use a cheap dev kit from nordic with debugger built in. It connects to the ARM using the standard ARM 10-pin connector. This board can program both the onboard nrf chip or an external. You can cut Solder Bridge 9 (SB9 in silkscreen) to disbale being able to program the onboard nrf without a jumper. 

#### nrfGo Studio
nrfGo studio is used to flash bootloader, application, and softdevice to an nrf board.

#### S130 Softdevice 
We use the [S130 Softdevice](https://github.com/avatech-inc/Scope_Firmware_ARM/blob/documentation/doc/Scope%20Datasheets/S130_SDS_v2.0.pdf) with our code. This is Nordic's packaged up BLE library. It gets loaded on to he device using nrfGo Studio. We do not access to the code but can interact like any other API.

#### RTT Viewer
Instead of using the UART port on the ARM connector, we are using the RTT functionality.


