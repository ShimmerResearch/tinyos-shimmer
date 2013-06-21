This is the Shimmer Research repository for TinyOS applications.  

For more information about shimmer wireless sensor motes see
[http://www.shimmer-research.com](http://www.shimmer-research.com)

For more information about the TinyOS operating system see
[http://www.tinyos.net](http://www.tinyos.net)

============================================================================

A (very) brief description of the contents follows:  

* apps/BluetoothMaster  
   Simple application that demonstrates how to use Bluetooth master mode  

* apps/BtStream  
   All purpose configurable Bluetooth sensing and streaming application 

* apps/FirmwareIdentifireList.txt  
   List of identifiers used by applications to identify themselves  

* apps/GpsSleep  
   Simple application that shows how to save power on a shimer with a GPS
   daughter card attached (when the daughter card's sensors are not in use)  

* apps/JustFATLogging  
   Simple application that logs accelerometer data to the SD card  

* apps/SDLog  
   Fully configurable logging to SD card with relative time synchronisation

* apps/SimpleAccel  
   Simple application that streams accelerometer data over Bluetooth  

* apps/SimpleAccelGyro  
   Simple application that streams accelerometer and gyroscope data over the
   802.15.4 radio  

* apps/SimpleGSR  
   Simple application that streams GSR data over Bluetooth  

* apps/SyncedRadioTx  
   Application that demonstrates how to coordinate multiple shimmers
   transmitting over the 802.15.4 radio at (relatively) high data rates  

* apps/TestBluetooth  
   Simple application to demonstrate how to use the Bluetooth radio (in
   slave mode)  

* apps/TestGPS  
   Simple application that demonstrates how to use the GPS daughter card

* apps/TestSerialTime  
   Simple application to demonstrate how to send a "real world" timestamp to
   a shimmer of the serial port  

* apps/TestTiltSwitch  
   Simple application to demonstrate how to use the tilt switch on shimmer2
   and shimmer2r devices  

* apps/TestUserButton  
   Simple application to demonstrate how to make use of the user button on
   the dock or external expansion board  

* apps/Legacy  
   This directory contains applications that have been superseded by newer
   applications  

* apps/Legacy/AccelECG  
   Streams accelerometer and ECG data over Bluetooth using the "BioMOBIUS"
   packet format  
   Superseded by BtStream  

* apps/Legacy/AccelGyro  
   Streams accelerometer and gyroscope data over Bluetooth using the
   "BioMOBIUS" packet format  
   Superseded by BtStream  

* apps/Legacy/BoilerPlate  
   All purpose configurable Bluetooth sensing and streaming application  
   Became BtStream  

* apps/Legacy/GyroMag  
   Streams gyroscope and magnetometer data over Bluetooth using the
   "BioMOBIUS" packet format  
   Superseded by BtStream  

* apps/Legacy/HostTimeLogging  
   Logs accelerometer data to SD card  
   Also demonstrates how to read a "real world" timestamp over the shimmer's
   serial port  
   Superseded by SdLog  

* apps/Legacy/ParamLogging  
   Configurable sensing application that logs to SD card  
   Superseded by SdLog  

* apps/Legacy/TestGyroBoard  
   Logs gyroscope data to SD card  
   Superseded by SdLog  
