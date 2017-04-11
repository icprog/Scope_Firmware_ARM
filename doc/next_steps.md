
## Scope Next Steps
* **Long UUIDS**:
scope needs to change its Blurtooth profile to be compliant with BLE-SIG stuff. This is done and in the feature/long_uuid branch
* **Name change**:
should be in the feature/long_uuid branch
* **Time**:
time and location are currently being synced from the phone
* **Raw data**:
we've been working to reliably get raw data from the device to the phone/computer. I think fixing the PIC-ARM comms problems will help with this

* **Sleep**: 
The HALL signal (see eagle files) get routed through a PIC pin, but causes current draw when we try to disbale the PIC. This need to be lifted on current bords and rerouted on future boards. We have already made the change in the eagle files. Furthermore the current [MIC5323](https://github.com/avatech-inc/Scope_Firmware/blob/documentation/doc/Scope%20Datasheets/MIC5323.pdf) regulator on the ARM board is always on but it quiescient current is ~ 70uA which is the majority of our low power current consumption, protoypes of replacing this with the pin and circuit compatible [NCP551](https://github.com/avatech-inc/Scope_Firmware/blob/documentation/doc/Scope%20Datasheets/NCP551-D.PDF) which has quiescent current of only a few uAs.  Similarly, a better implementation of sleep state has been built and is ready to integrated from another branch. Without these changes, battery life is much worse when in sleep or in the pole. These changes reduce current from 300uA to about 30uA.

* **ADC**: 
After switching from EC PIC family to the EF, an exeternal ADC may no longer be necessary. We have yet to experiment with this but could save money and board space.

* **Battery Monitor**:
Our current hardare for monitoring battery life is insuficient. Our sense resistor is connected to traces that are too long, essentially doubling the resistance and cause very bad current readings. We do, however, read voltage, and provide the user witha "battery critically low" laert if voltage drops too low. This system shoudl be replaced by culoumb counting system that will help us get closer toa ctually being able to determein battery life.

## Known Issues
* **PIC-ARM Communication**: 
The way the PIc communicates to the ARM is via  custom protocol that is using SPI as a mulit-master system. It does not work well and should be redesigned. Timing and collision issues often cause the device to freeze or miss packets or misinterpret information.
* **FWU reliability**:
The FWU does not autoamtically restart after finishing anymore. WE have also experienced issues of an incomplete update resulting bricking devices. Tests will need to be done 
* **Old profile syncing**: 
We need a way for users to get past profiles off of the device. This situation arises when someone phoen dies or multiple people want to use Scope on different devices. The current system syncs up to 5 previous profiles then asks the user if it wants more. 
Another problem here is encountered when syncing the current profile. If you disconnect, it will look like it transferring an old profile so we can't just say you need to request old profiles. It would be nice to keep time between disconnects and if there are any out stadning profiles that need to be transferred within a certain time. Then we can just bury the old profile transfer and not worry about the new ones getting lost due to a disconnect during the probe.

## Sketchy Things
* **Delays**: 
There are delays everywhere to get communication and state transitions to work. A cleaner system would allow for better efficiency.

* **The extra wire**:
We have an extra wire connecting the TX pin on the ARM to the pin labeled GPIO2 in silkscreen to help out with our PIC-ARM communication. This line acts as out ARM_RDY line, signal to the PIC when the ARM is ready for a SPI transaction. This wire is added after the boards are soldered and is required for the device to function.
