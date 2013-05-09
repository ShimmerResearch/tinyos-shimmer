This is a general purpose configurable application to be used with shimmer and any add-on daughter-cards supplied by shimmer-research.


By default this application samples the 3-axis accelerometer at 50Hz and sends the data over the Bluetooth radio, using a data buffer size of 1. If buffer size is greater than 1 then the timestamp and channel data is repeated that number of times. 

Buffer size 1:
   Data Packet Format:
          Packet Type | TimeStamp | chan1 | chan2 | ... |  chanX 
   Byte:       0      |    1-2    |  3-4  |  5-6  | ... | (x-1)-x

Buffer size 2:
   Data Packet Format:
          Packet Type | TimeStamp | chan1 | chan2 | ... |  chanX  |  TimeStamp  |    chan1    | ... |   chanX   |
   Byte:       0      |    1-2    |  3-4  |  5-6  | ... | (x-1)-x | (x+1)-(x+2) | (x+3)-(x+4) | ... | (2x-1)-2x |
                      |----------------- buffer 0 ----------------|------------------ buffer 1 -----------------|

When the application receives an Inquiry command it responds with the following packet. The value in the channel fields indicate exactly what data from which sensor is contained in this field of the data packet:

Inquiry Response Packet Format:
          Packet Type | ADC Sampling rate | Accel Range | Config Byte 0 |Num Chans | Buf size | Chan1 | Chan2 | ... | ChanX
   Byte:       0      |         1         |      2      |       3       |    4     |     5    |   6   |   7   | ... |   x 


Currently the following parameters can be configured. This configuration is stored in the Infomem so survives a reset/power off (but does not survive reprogramming):
   - Sampling rate
   - Which sensors are sampled
   - Accelerometer range
   - The state of the 5V regulator on the AnEx board
   - The power mux
   - calibration values for accel, gyro, mag, emg and ecg
   - GSR range
      - Special cases: 
         - GSR autorange
            - When set the GSR range is controlled on the shimmer
            - The two most significant bits of the GSR channel value are overloaded to indicate which resistor is active (i.e. the range)
               - e.g. if HW_RES_1M is selected (val 2), then the GSR channel will be 10xxxxxxxxxxxxxx
         - GSRx4
            - not currently used
   - Magnetometer gain and sampling rate
   - The mux that swaps between Gyro X, Y and Z axes vs. Gyro temperature and VREF


The following commands are available:
   - INQUIRY_COMMAND
   - GET_SAMPLING_RATE_COMMAND
   - SET_SAMPLING_RATE_COMMAND
   - TOGGLE_LED_COMMAND
   - START_STREAMING_COMMAND
   - SET_SENSORS_COMMAND
   - SET_ACCEL_RANGE_COMMAND
   - GET_ACCEL_RANGE_COMMAND
   - SET_5V_REGULATOR_COMMAND
   - SET_PMUX_COMMAND
   - SET_CONFIG_SETUP_BYTE0_COMMAND
   - GET_CONFIG_SETUP_BYTE0_COMMAND
   - SET_ACCEL_CALIBRATION_COMMAND
   - GET_ACCEL_CALIBRATION_COMMAND
   - SET_GYRO_CALIBRATION_COMMAND
   - GET_GYRO_CALIBRATION_COMMAND
   - SET_MAG_CALIBRATION_COMMAND
   - GET_MAG_CALIBRATION_COMMAND
   - STOP_STREAMING_COMMAND
   - SET_GSR_RANGE_COMMAND
   - GET_GSR_RANGE_COMMAND
   - GET_SHIMMER_VERSION_COMMAND
   - SET_EMG_CALIBRATION_COMMAND
   - GET_EMG_CALIBRATION_COMMAND
   - SET_ECG_CALIBRATION_COMMAND
   - GET_ECG_CALIBRATION_COMMAND
   - GET_ALL_CALIBRATION_COMMAND
   - GET_FW_VERSION_COMMAND
   - SET_BLINK_LED_COMMAND
   - GET_BLINK_LED_COMMAND
   - SET_GYRO_TEMP_VREF_COMMAND
   - SET_BUFFER_SIZE_COMMAND
   - GET_BUFFER_SIZE_COMMAND
   - SET_MAG_GAIN_COMMAND
   - GET_MAG_GAIN_COMMAND
   - SET_MAG_SAMPLING_RATE_COMMAND
   - GET_MAG_SAMPLING_RATE_COMMAND


Config Setup Byte 0:
   - Bit 7: 5V regulator               # When 0 5V regulator on AnEx board is disabled, when 1 5V regulator is enabled
   - Bit 6: PMUX                       # When 0 AnEx channels are read, when 1 power values are read
   - Bit 5: GYRO Axes vs temp/vref mux # When 0 Gyro axes, when 1 temperature and vref values
   - Bit 4: Not yet assigned
   - Bit 3: Not yet assigned
   - Bit 2: Not yet assigned
   - Bit 1: Not yet assigned
   - Bit 0: Not yet assigned
Config Setup Byte 1-4:
   - Not yet assigned


The format of the configuration data stored in Infomem is as follows:
   - 90 bytes starting from address 0
      Byte 0: Sampling rate
      Byte 1: Buffer Size
      Byte 2 - 11: Selected Sensors (Allows for up to 80 different sensors)
      Byte 12: Accel Range
      Byte 13: Magnetometer configuration (gain in upper 4 bits, sampling rate in lower 4 bits)
      Byte 14 - 17: Config Bytes (Allows for 32 individual boolean settings)
      Byte 18 - 38: Accelerometers calibration values
      Byte 39 - 59: Gyroscopes calibration values
      Byte 60 - 80: Magnetometer calibration values
      Byte 81: GSR range
      Byte 82 - 85: EMG calibration (unit16_t offset, uint16_t gain)
      Byte 86 - 93: ECG calibration (uint16_t offset_chan0, uint16_t offset_chan1, uint16_t gain_chan0, uint16_t gain_chan1)


The assignment of the selected sensors field is a follows:
   - 1 bit per sensor. When there is a conflict priority is most significant bit -> least significant bit
      Byte2:
         Bit 7: Accel
         Bit 6: Gyro
         Bit 5: Magnetometer
         Bit 4: ECG
         Bit 3: EMG
         Bit 2: GSR
         Bit 1: AnEx ADC Channel 7
         Bit 0: AnEx ADC Channel 0
      Byte3
         Bit 7: Strain Gauge
         Bit 6: Heart Rate
         Bit 5: Not yet assigned
         Bit 4: Not yet assigned
         Bit 3: Not yet assigned
         Bit 2: Not yet assigned
         Bit 1: Not yet assigned
         Bit 0: Not yet assigned
      Byte4 - Byte11
         Not yet assigned


The GET_SHIMMER_VERSION_COMMAND returns a 1 byte value, based on the shimmer revision as follows:
   0 = shimmer1
   1 = shimmer2
   2 = shimmer2r
   

The GET_FW_VERSION_COMMAND returns 6 bytes:
   Byte0+Byte1                   = Firmware identifier (see FirmwareIdentifierList.txt in tinyos-2.x-contrib/shimmer/apps/)
                                 = 1 for this application
   Byte2+Byte3 (little endian)   = Major version number
   Byte4                         = Minor version number
   Byte5                         = Release candidate number (internal use only. All externally released
                                                             firmware images will have 0 here)
These values are set in the application's Makefile


The GET_ALL_CALIBRATION_COMMAND returns all the stored calibration values (71 bytes) in the following order:
   Accel (21 bytes)
   Gyro  (21 bytes)
   Mag   (21 bytes)
   EMG   ( 4 bytes)
   ECG   ( 8 bytes)
The breakdown of the kinematic (accel, gyro and mag) calibration values is as follows:
   Each of the 3 offset bias vector values (one for each axis) are stored as 16-bit signed integers (big endian) and are contained in bytes 0-5. Each of the 3 sensitivity vector values (again one for each axis) are stored as 16-bit signed integers (big endian) and are contained in bytes 6-11. Each of the 9 alignment matrix values are stored as 8-bit signed integers and are contained in bytes 12-20.  
The breakdown of the EMG calibration values is as follows: 
   The 16-bit signed offset value is stored in bytes 0 and 1 (big endian). The 16-bit signed gain value is stored in bytes 2 and 3 (big endian).
The breakdown of the ECG calibration values is as follows: 
   The 16-bit signed offset value is stored in bytes 0 and 1 (big endian) for the first channel and bytes 2 and 3 (big endian) for the second channel. The 16-bit signed gain value is stored in bytes 4 and 5 (big endian) for the first channel and bytes 6 and 7 (big endian) for the second channel.


The SET_BLINK_LED_COMMAND changes which LED is used to indicate the shimmer's state:
- selected LED operation
   - Occasional flash of selected LED when on but not connected
      - On for approx 50ms
      - Off for 2s
   - Solid LED when connected but not streaming
   - Flash LED @1Hz when connected and streaming
   - Flash LED @10Hz to indicate activity on the Shimmer
      - e.g. when Gryo is being initialised
- The SET_BLINK_LED command takes a single 1 byte argument
   - 0 for green
   - 1 for yellow
   - 2-255 for red
- Selected LED resets to green at power up
- Resets to green if docked but not streaming
- The GET_BLINK_LED_COMMAND returns a single byte, indicating selected LED as above 


The SET_BUFFER_SIZE_COMMAND sets the number of sample periods that arrives per data packet. It takes a single 1 byte argument. The maximum allowed value depends on which sensors are enabled. The maximum data packet size (as shown above) cannot exceed 255 bytes. If the buffer size setting causes the packet to exceed this number of bytes then streaming/sensing cannot be started (the START_STREAMING_COMMAND will still return an acknowledgement but no data will be transmitted). 


When not streaming pressing the user button (on the dock or AnEx board) will reset TimerB (i.e. the source of the TimeStamp in the data packet) to zero. Currently this functionality is disabled if the heart rate sensor is configured. This is necessary because there is no power control on the (prototype) RMCM01 boards, so, if the module is attached to the shimmer and there is a active transmitter in range then the shimmer will see the interrupt (which is shared with the user button signal).


Changelog:
- 10 Jan 2011
   - initial release
   - support for Accelerometer, Gyroscope, Magnetometer, ECG, EMG, AnEx sensors
   - Sampling rate, Accel range, 5V regulator and PMUX (voltage monitoring) configurable
   - save configuration to InfoMem
- 29 Mar 2011
   - support for Strain Gauge and Heart Rate sensors
   - fixed EMG problem
      - a second EMG channel was being added erroneously
   - support for transmitting 8-bit data channels instead of just 16-bit 
- 21 Apr 2011
   - Fixed bug in heart rate support
   - Fixed bug in timestamping
   - changed SampleTimer to an Alarm
- 4 May 2011
   - removed a lot of unnecessary atomic commands
   - added support for writing and reading accel, gyro and mag calibration data
- 13 May 2011
   - GSR support
   - added command to get shimmer version (revision)
- 4 July 2011
   - fixed bug which caused a command sent while shimmer was streaming data to not always receive an acknowledgement
- 09 May 2013 (v1.1.0)
   - changed name from BoilerPlate to BtStream
   - added support for ECG and EMG calibration values
   - moved gyro startup delay from startSensing() to configure_channels()
      - indicate gyro startup using warning LED pattern
   - single command to retrieve all sensors calibration values
      - i.e. accel, gyro, mag, emg, ecg (in that order)
   - command to get firmware identifier/type and version number
      - set in Makefile
      - Firmware identier for this application is 1
      - starts as version 1.1.0 for this revision
   - changed indicator LED operation
      - as described above
      - support for GET_BLINK_LED and SET_BLINK_LED commands
      - support to reset to green when docked and not streaming
   - documentation for USE_230400_BAUD preprocessor flag
   - modified Msp430DcoSpec.h so that correct TARGET_DCO_KHZ setting is used if 8MHz crystal is not enabled
   - include active GSR resistor setting in upper two bits of GSR field even if autorange is not selected
   - eliminate spikes in GSR resistance value at autorange transitions (previous value is maintained until circuitry has settled) 
   - added support to allow gyro temperature and vref to be read
   - modified heart rate to send beat-time as 16-bit value instead of attempting to calculate heart rate and sending 8-bit value
   - reset TimerB (32.768kHz) using user button
      - only when not streaming data
      - and when heart rate sensor is not enabled
   - support for variable data buffer size
   - make the STOP_STREAMING_COMMAND synchronous with respect to the sample period
      - seems to fix the issue with the mag causing the shimmer to lock up when stopping streaming
   - added commands to get and set the mag gain and mag sampling rate
