/*
 * Copyright (c) 2010, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:

 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Shimmer Research, Ltd. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Mike Healy
 * @date   November, 2010
 */


/***********************************************************************************

   Data Packet Format:
          Packet Type | TimeStamp | chan1 | chan2 | ... | chanX 
   Byte:       0      |    1-2    |  3-4  |  5-6  | ... | chanX

   Inquiry Response Packet Format:
          Packet Type | ADC Sampling rate | Accel Range | Config Byte 0 |Num Chans | Buf size | Chan1 | Chan2 | ... | ChanX
   Byte:       0      |         1         |       2     |       3       |    4     |     5    |   6   |   7   | ... |   x

***********************************************************************************/
#include "Timer.h"
#include "Mma_Accel.h"
#include "RovingNetworks.h"
#include "Gsr.h"
#include "Shimmer.h"
#include "UserButton.h"
#include "Magnetometer.h"

module BtStreamC {
   uses {
      interface Boot;
      interface Init as BluetoothInit;
      interface Init as AccelInit;
      interface Init as GyroInit;
      interface Init as MagInit;
      interface Init as StrainInit;
      interface Init as DigitalHeartInit;
      interface Init as GsrInit;
      interface Init as SampleTimerInit;
      interface Init as BlinkTimerInit;
      interface GyroBoard;
      interface Magnetometer;
      interface StrainGauge;
      interface DigitalHeartRate;
      interface StdControl as GyroStdControl;
      interface Mma_Accel as Accel;
      interface Gsr;
      interface Leds;
      interface shimmerAnalogSetup;
      interface Timer<TMilli> as SetupTimer;
      interface Alarm<TMilli, uint16_t> as BlinkTimer;
      interface Alarm<TMilli, uint16_t> as SampleTimer;
      interface LocalTime<T32khz>;
      interface StdControl as BTStdControl;
      interface Bluetooth;
      interface Msp430DmaChannel as DMA0;
      interface BtCommandParser;
      interface InternalFlash;
      interface HplMsp430Interrupt as DockInterrupt;
      interface Notify<button_state_t> as UserButton;

#ifdef USE_8MHZ_CRYSTAL
      interface Init as FastClockInit;
      interface FastClock;
#endif
   }
} 

implementation {
   void init();
   void configure_channels();
   void prepareDataPacket();

   uint8_t res_packet[RESPONSE_PACKET_SIZE], readBuf[7];
   uint8_t tx_packet0[255], tx_packet1[255]; //max packet size that can be transmitted
   uint8_t *current_tx_packet, buffer_iteration;

   norace uint8_t nbr_adc_chans, nbr_1byte_digi_chans, nbr_2byte_digi_chans, current_buffer;
   int8_t sbuf0[(MAX_NUM_2_BYTE_CHANNELS*2) + MAX_NUM_1_BYTE_CHANNELS], sbuf1[(MAX_NUM_2_BYTE_CHANNELS*2) + MAX_NUM_1_BYTE_CHANNELS]; 
   norace uint16_t timestamp0, timestamp1;

   norace bool sensing, enable_sending, bt_connected, command_pending, command_mode_complete, mag_read_pending, stop_sensing_pending;
   bool wasSensing, configureChannels, changeMagSamplingRate, magStartContinuousConversion, sendAck, inquiryResponse, samplingRateResponse, accelRangeResponse, 
        configSetupByte0Response, gsrRangeResponse, shimmerVersionResponse, fwVersionResponse, blinkLedResponse, bufferSizeResponse, stop_sensing, 
        magGainResponse, magSamplingRateResponse;

   uint8_t g_arg_size, g_arg[MAX_COMMAND_ARG_SIZE];
   norace uint8_t g_action;

   // Configuration
   norace uint8_t stored_config[NV_NUM_CONFIG_BYTES];
   uint8_t channel_contents[MAX_NUM_CHANNELS], buffered_buffer_size;

   bool accelCalibrationResponse, gyroCalibrationResponse, magCalibrationResponse, emgCalibrationResponse, ecgCalibrationResponse, allCalibrationResponse;

   //GSR
   uint8_t gsr_active_resistor;
   uint16_t last_GSR_val;

   //Leds
   uint8_t selectedLed, warningLedCount;
   bool ledOn, activityLed, warningLed; //Memory is plentiful, so using full bytes rather than bit fields for speed

   //Heart rate
   bool beat_flag;
   uint32_t beat_time;
   

   inline void SetLedFunc(void) {
      switch(selectedLed) { 
         case 0:
            call Leds.led2On();
            break;
         case 1:
            call Leds.led1On();
            break;
         default:
            call Leds.led0On();
      }
   }

   inline void ClearLedFunc(void) {
      switch(selectedLed) { 
         case 0:
            call Leds.led2Off();
            break;
         case 1:
            call Leds.led1Off();
            break;
         default:
            call Leds.led0Off();
      }
   }

   inline void ToggleLedFunc(void) {
      switch(selectedLed) { 
         case 0:
            call Leds.led2Toggle();
            break;
         case 1:
            call Leds.led1Toggle();
            break;
         default:
            call Leds.led0Toggle();
      }
   }


   task void ToggleLed() {
      ToggleLedFunc();
   }


   void startStreaming() {
      call BlinkTimer.start(1024);
      activityLed = TRUE;
      ToggleLedFunc();

      buffer_iteration = 0;

      if(stored_config[NV_SAMPLING_RATE] != SAMPLING_0HZ_OFF)
         call SampleTimer.start(stored_config[NV_SAMPLING_RATE]); 
      sensing = TRUE;
   }

   task void startSensors() {
      atomic buffered_buffer_size = stored_config[NV_BUFFER_SIZE]; //to ensure it doesn't change while streaming
      
      if (((((nbr_adc_chans+nbr_2byte_digi_chans)*2)+nbr_1byte_digi_chans+2)*buffered_buffer_size) <= 254) {
         //buffer size can be accommodated by max bluetooth packet size

         call UserButton.disable(); //don't want to be able to reset TimerB to 0 while streaming

         if(stored_config[NV_SENSORS0] & SENSOR_GYRO){
            call GyroStdControl.start();
         }

         if(stored_config[NV_SENSORS0] & SENSOR_ACCEL) {
            call Accel.setSensitivity(stored_config[NV_ACCEL_RANGE]);
            call Accel.wake(TRUE);
         }

         if(stored_config[NV_SENSORS0] & SENSOR_GSR) {
            call GsrInit.init();
            if(stored_config[NV_GSR_RANGE] <= HW_RES_3M3) {
               call Gsr.setRange(stored_config[NV_GSR_RANGE]);
               gsr_active_resistor = stored_config[NV_GSR_RANGE];
            }
            else {
               call Gsr.setRange(HW_RES_40K);
               gsr_active_resistor = HW_RES_40K;
            }

         }

         if(stored_config[NV_SENSORS1] & SENSOR_STRAIN) {
            call StrainInit.init();
            call StrainGauge.powerOn();
            // Sets 5V reg
            stored_config[NV_CONFIG_SETUP_BYTE0] |= CONFIG_5V_REG;
            call InternalFlash.write((void*)NV_CONFIG_SETUP_BYTE0, (void*)&stored_config[NV_CONFIG_SETUP_BYTE0], 1);
         }

         if(stored_config[NV_SENSORS1] & SENSOR_HEART) {
            call DigitalHeartInit.init(); //needed because of UserButton.disable() above
            beat_flag = FALSE;
         }

         if(stored_config[NV_SENSORS0] & SENSOR_MAG) {
            call MagInit.init();
            magStartContinuousConversion = TRUE;

            if((stored_config[NV_MAG_CONFIG]&0x0F) != TEN_HZ) 
               changeMagSamplingRate = TRUE;
            if(((stored_config[NV_MAG_CONFIG]&0xF0)>>4) != ONE_GAUSS) {
               call Magnetometer.setGain(((stored_config[NV_MAG_CONFIG]&0xF0)>>4));
            } else if(changeMagSamplingRate) {
               changeMagSamplingRate = FALSE;
               call Magnetometer.setOutputRate(stored_config[NV_MAG_CONFIG]&0x0F);
            } else {
               magStartContinuousConversion = FALSE;
               call Magnetometer.runContinuousConversion();
            }
         } else {
            startStreaming();
         }
      }
   }


   task void ConfigureChannelsTask() {
      configure_channels();
   }


   task void stopSensing() {
      call SetupTimer.stop();
      call SampleTimer.stop();
      atomic sensing = FALSE;

      atomic {
         if(mag_read_pending)
            stop_sensing_pending = TRUE;
      }

      while (mag_read_pending) nop();
      stop_sensing_pending = FALSE;

      call BlinkTimer.stop();
      activityLed = FALSE;
      call shimmerAnalogSetup.stopConversion();
      call DMA0.stopTransfer();
      if(stored_config[NV_SENSORS0] & SENSOR_ACCEL)
         call Accel.wake(FALSE);
      if((stored_config[NV_SENSORS0] & SENSOR_GYRO) || (stored_config[NV_SENSORS0] & SENSOR_MAG)){
         call GyroStdControl.stop();
         call Magnetometer.disableBus();
      }
      if(stored_config[NV_SENSORS1] & SENSOR_STRAIN) {
         call StrainGauge.powerOff();
         // clears 5V reg
         stored_config[NV_CONFIG_SETUP_BYTE0] &= ~CONFIG_5V_REG;
         call InternalFlash.write((void*)NV_CONFIG_SETUP_BYTE0, (void*)&stored_config[NV_CONFIG_SETUP_BYTE0], 1);
      }
      if (!call DockInterrupt.getValue() == TRUE){  // if on the dock
         atomic {
            // reset the selected LED to green
            ClearLedFunc();
            selectedLed = 0;
         }
      }
      if(bt_connected) {
         //still connected
         SetLedFunc();
      } else {
         //not connected
         ClearLedFunc();
         ledOn = FALSE;
         call BlinkTimer.start(2048);
      }
      if(stored_config[NV_SENSORS1] & SENSOR_HEART) {
         beat_flag = FALSE;
         call UserButton.disable();
      } else {
         call UserButton.enable();
      }
      stop_sensing = FALSE;

      if(configureChannels) {
         configureChannels = FALSE;
         post ConfigureChannelsTask();
      } else if(wasSensing) {
         wasSensing = FALSE;
         post startSensors();
      }
   }


   task void sendSensorData() {
      prepareDataPacket();

      if(stop_sensing)
         post stopSensing();
      else {
         if(buffer_iteration >= buffered_buffer_size) {
            if(enable_sending) {
               // send data over the air
               call Bluetooth.write(current_tx_packet, (1 + ((((nbr_adc_chans+nbr_2byte_digi_chans)*2)+ nbr_1byte_digi_chans + 2)*buffer_iteration)));
               enable_sending = FALSE;
               //flip buffers. Only one pending send is allowed at a time
               current_tx_packet = (current_tx_packet==tx_packet0) ? tx_packet1 : tx_packet0;
            }
            buffer_iteration = 0;
         }
      }
   }


   task void sendResponse() {
      uint16_t packet_length = 0;
      uint8_t i;
      
      command_pending = FALSE;
      
      if(enable_sending) {
         if(sendAck) {
            // if sending other response prepend acknowledgement to start
            // else just send acknowledgement by itself 
            *(res_packet + packet_length++) = ACK_COMMAND_PROCESSED;
            sendAck = FALSE;
         }
         if(inquiryResponse) {
            *(res_packet + packet_length++) = INQUIRY_RESPONSE;                                             //packet type
            *(res_packet + packet_length++) = stored_config[NV_SAMPLING_RATE];                              //ADC sampling rate
            *(res_packet + packet_length++) = stored_config[NV_ACCEL_RANGE];                                //Accel range
            *(res_packet + packet_length++) = stored_config[NV_CONFIG_SETUP_BYTE0];                         //Config Setup Byte 0
            *(res_packet + packet_length++) = nbr_adc_chans + nbr_1byte_digi_chans + nbr_2byte_digi_chans;  //number of data channels
            *(res_packet + packet_length++) = stored_config[NV_BUFFER_SIZE];                                //buffer size 
            memcpy((res_packet + packet_length), channel_contents, (nbr_adc_chans + nbr_1byte_digi_chans + nbr_2byte_digi_chans));
            packet_length += nbr_adc_chans + nbr_1byte_digi_chans + nbr_2byte_digi_chans;
            inquiryResponse = FALSE;
         }
         else if(samplingRateResponse) {
            *(res_packet + packet_length++) = SAMPLING_RATE_RESPONSE;                     // packet type
            *(res_packet + packet_length++) = stored_config[NV_SAMPLING_RATE];            // ADC sampling rate
            samplingRateResponse = FALSE;
         }
         else if(accelRangeResponse) {
            *(res_packet + packet_length++) = ACCEL_RANGE_RESPONSE;                       // packet type
            *(res_packet + packet_length++) = stored_config[NV_ACCEL_RANGE];              // Accel range
            accelRangeResponse = FALSE;
         }
         else if(configSetupByte0Response) {
            *(res_packet + packet_length++) = CONFIG_SETUP_BYTE0_RESPONSE;                // packet type
            *(res_packet + packet_length++) = stored_config[NV_CONFIG_SETUP_BYTE0];       // Config setup byte 0
            configSetupByte0Response = FALSE;
         }
         else if(accelCalibrationResponse){
            *(res_packet + packet_length++) = ACCEL_CALIBRATION_RESPONSE;                 // packet type
            for(i=0; i<21; i++)
               *(res_packet + packet_length++) = stored_config[NV_ACCEL_CALIBRATION + i]; // Calibration values
            accelCalibrationResponse = FALSE;
         }
         else if(gyroCalibrationResponse){
            *(res_packet + packet_length++) = GYRO_CALIBRATION_RESPONSE;                  // packet type
            for(i=0; i<21; i++)
               *(res_packet + packet_length++) = stored_config[NV_GYRO_CALIBRATION + i];  // Calibration values
            gyroCalibrationResponse = FALSE;
         }
         else if(magCalibrationResponse){
            *(res_packet + packet_length++) = MAG_CALIBRATION_RESPONSE;                   // packet type
            for(i=0; i<21; i++)
               *(res_packet + packet_length++) = stored_config[NV_MAG_CALIBRATION + i];   // Calibration values
            magCalibrationResponse = FALSE;
         }
         else if(gsrRangeResponse) {
            *(res_packet + packet_length++) = GSR_RANGE_RESPONSE;                          // packet type
            *(res_packet + packet_length++) = stored_config[NV_GSR_RANGE];                 // GSR range 
            gsrRangeResponse = FALSE;
         }
         else if(shimmerVersionResponse) {
            *(res_packet + packet_length++) = SHIMMER_VERSION_RESPONSE;                   // packet type
            *(res_packet + packet_length++) = SHIMMER_VER;                                // Shimmer Version
            shimmerVersionResponse = FALSE;
         }
         else if(emgCalibrationResponse){
            *(res_packet + packet_length++) = EMG_CALIBRATION_RESPONSE;                   // packet type
            for(i=0; i<4; i++)
               *(res_packet + packet_length++) = stored_config[NV_EMG_CALIBRATION + i];   // Calibration values
            emgCalibrationResponse = FALSE;
         }
         else if(ecgCalibrationResponse){
            *(res_packet + packet_length++) = ECG_CALIBRATION_RESPONSE;                   // packet type
            for(i=0; i<8; i++)
               *(res_packet + packet_length++) = stored_config[NV_ECG_CALIBRATION + i];   // Calibration values
            ecgCalibrationResponse = FALSE;
         }
         else if(allCalibrationResponse){
            *(res_packet + packet_length++) = ALL_CALIBRATION_RESPONSE;                   // packet type
            memcpy((res_packet + packet_length), (stored_config+NV_ACCEL_CALIBRATION), 75);
            packet_length += 75;

            allCalibrationResponse = FALSE;
         }
         else if(fwVersionResponse) {
            *(res_packet + packet_length++) = FW_VERSION_RESPONSE;                        // packet type
            *(res_packet + packet_length++) = FW_IDENTIFIER & 0xFF;
            *(res_packet + packet_length++) = (FW_IDENTIFIER & 0xFF00) >> 8;
            *(res_packet + packet_length++) = FW_VER_MAJOR & 0xFF;
            *(res_packet + packet_length++) = (FW_VER_MAJOR & 0xFF00) >> 8;
            *(res_packet + packet_length++) = FW_VER_MINOR;
            *(res_packet + packet_length++) = FW_VER_REL;
            fwVersionResponse = FALSE;
         }
         else if(blinkLedResponse) {
            *(res_packet + packet_length++) = BLINK_LED_RESPONSE;                         // packet type
            *(res_packet + packet_length++) = selectedLed;
            blinkLedResponse = FALSE;
         }
         else if(bufferSizeResponse) {
            *(res_packet + packet_length++) = BUFFER_SIZE_RESPONSE;                       // packet type
            *(res_packet + packet_length++) = stored_config[NV_BUFFER_SIZE];
            bufferSizeResponse = FALSE;
         } 
         else if(magGainResponse) {
            *(res_packet + packet_length++) = MAG_GAIN_RESPONSE;                          // packet type
            *(res_packet + packet_length++) = (stored_config[NV_MAG_CONFIG] & 0xF0) >> 4;
            magGainResponse = FALSE;
         } 
         else if(magSamplingRateResponse) {
            *(res_packet + packet_length++) = MAG_SAMPLING_RATE_RESPONSE;                 // packet type
            *(res_packet + packet_length++) = stored_config[NV_MAG_CONFIG] & 0x0F;
            magSamplingRateResponse = FALSE;
         }
         else {}

         // send data over the air
         call Bluetooth.write(res_packet, packet_length);
         enable_sending = FALSE;
      }
      else
         post sendResponse();
   }


   task void startConfigTimer() {
      call SetupTimer.startPeriodic(5000);
   }


   task void ProcessCommand() {
      switch(g_action){
         case INQUIRY_COMMAND:
            inquiryResponse = TRUE;
            break;
         case GET_SAMPLING_RATE_COMMAND:
            samplingRateResponse = TRUE;
            break;
         case SET_SAMPLING_RATE_COMMAND:
            stored_config[NV_SAMPLING_RATE] = g_arg[0];
            call InternalFlash.write((void*)NV_SAMPLING_RATE, (void*)&stored_config[NV_SAMPLING_RATE], 1);
            if(sensing) {
               wasSensing = TRUE;
               stop_sensing = TRUE;
            }
            break;
         case TOGGLE_LED_COMMAND:
            call Leds.led0Toggle();
            break;
         case START_STREAMING_COMMAND:
            // start capturing on ^G
            if(command_mode_complete && !sensing)
               post startSensors();
            else
               // give config a chance, wait 5 secs
               post startConfigTimer();
            break;
         case SET_SENSORS_COMMAND:
            stored_config[NV_SENSORS0] = g_arg[0];
            stored_config[NV_SENSORS1] = g_arg[1];
            call InternalFlash.write((void*)NV_SENSORS0, (void*)&stored_config[NV_SENSORS0], 2);

            if(sensing) {
               configureChannels = TRUE;
               wasSensing = TRUE;
               stop_sensing = TRUE;
            }
            else
               post ConfigureChannelsTask();
            break;
         case SET_ACCEL_RANGE_COMMAND:
            if((g_arg[0] == RANGE_1_5G) || (g_arg[0] == RANGE_2_0G) || (g_arg[0] == RANGE_4_0G) || (g_arg[0] == RANGE_6_0G)) {
               stored_config[NV_ACCEL_RANGE] = g_arg[0];
               call InternalFlash.write((void*)NV_ACCEL_RANGE, (void*)&stored_config[NV_ACCEL_RANGE], 1);
               if(sensing) {
                  wasSensing = TRUE;
                  stop_sensing = TRUE;
               }
            }
            break;
         case GET_ACCEL_RANGE_COMMAND:
            accelRangeResponse = TRUE;
            break;
         case SET_5V_REGULATOR_COMMAND:
            if(g_arg[0]) {
               TOSH_SET_SER0_RTS_PIN();
               stored_config[NV_CONFIG_SETUP_BYTE0] |= CONFIG_5V_REG;
            }
            else {
               TOSH_CLR_SER0_RTS_PIN();
               stored_config[NV_CONFIG_SETUP_BYTE0] &= ~CONFIG_5V_REG;
            }
            call InternalFlash.write((void*)NV_CONFIG_SETUP_BYTE0, (void*)&stored_config[NV_CONFIG_SETUP_BYTE0], 1);
            break;
         case SET_PMUX_COMMAND:
#ifdef PWRMUX_UTIL
            if(g_arg[0]) {
               TOSH_SET_PWRMUX_SEL_PIN();
               stored_config[NV_CONFIG_SETUP_BYTE0] |= CONFIG_PMUX;
            }
            else {
               TOSH_CLR_PWRMUX_SEL_PIN();
               stored_config[NV_CONFIG_SETUP_BYTE0] &= ~CONFIG_PMUX;
            }
            call InternalFlash.write((void*)NV_CONFIG_SETUP_BYTE0, (void*)&stored_config[NV_CONFIG_SETUP_BYTE0], 1);
#endif
            break;
         case SET_GYRO_TEMP_VREF_COMMAND:
            if(g_arg[0]) {
               TOSH_SET_SER0_CTS_PIN();
               stored_config[NV_CONFIG_SETUP_BYTE0] |= CONFIG_GYRO_TEMP_VREF;
            }
            else {
               TOSH_CLR_SER0_CTS_PIN();
               stored_config[NV_CONFIG_SETUP_BYTE0] &= ~CONFIG_GYRO_TEMP_VREF;
            }
            call InternalFlash.write((void*)NV_CONFIG_SETUP_BYTE0, (void*)&stored_config[NV_CONFIG_SETUP_BYTE0], 1);
            break;
         case SET_CONFIG_SETUP_BYTE0_COMMAND:
            stored_config[NV_CONFIG_SETUP_BYTE0] = g_arg[0];
            if(stored_config[NV_CONFIG_SETUP_BYTE0] & CONFIG_5V_REG)
               TOSH_SET_SER0_RTS_PIN();
            else
               TOSH_CLR_SER0_RTS_PIN();
#ifdef PWRMUX_UTIL
            if(stored_config[NV_CONFIG_SETUP_BYTE0] & CONFIG_PMUX)
               TOSH_SET_PWRMUX_SEL_PIN();
            else
               TOSH_CLR_PWRMUX_SEL_PIN();
#else
            // make sure PMUX bit is not set
            stored_config[NV_CONFIG_SETUP_BYTE0] &= ~CONFIG_PMUX;

#endif
            if(stored_config[NV_CONFIG_SETUP_BYTE0] & CONFIG_GYRO_TEMP_VREF)
               TOSH_SET_SER0_CTS_PIN();
            else
               TOSH_CLR_SER0_CTS_PIN();
            call InternalFlash.write((void*)NV_CONFIG_SETUP_BYTE0, (void*)&stored_config[NV_CONFIG_SETUP_BYTE0], 1);
            break;
         case GET_CONFIG_SETUP_BYTE0_COMMAND:
            configSetupByte0Response = TRUE;
            break;
         case SET_ACCEL_CALIBRATION_COMMAND:
            memcpy(&stored_config[NV_ACCEL_CALIBRATION], g_arg, 21);
            call InternalFlash.write((void*)NV_ACCEL_CALIBRATION, (void*)&stored_config[NV_ACCEL_CALIBRATION], 21);
            break;
         case GET_ACCEL_CALIBRATION_COMMAND:
            accelCalibrationResponse = TRUE;
            break; 
         case SET_GYRO_CALIBRATION_COMMAND:
            memcpy(&stored_config[NV_GYRO_CALIBRATION], g_arg, 21);
            call InternalFlash.write((void*)NV_GYRO_CALIBRATION, (void*)&stored_config[NV_GYRO_CALIBRATION], 21);
            break;
         case GET_GYRO_CALIBRATION_COMMAND:
            gyroCalibrationResponse = TRUE;
            break; 
         case SET_MAG_CALIBRATION_COMMAND:
            memcpy(&stored_config[NV_MAG_CALIBRATION], g_arg, 21);
            call InternalFlash.write((void*)NV_MAG_CALIBRATION, (void*)&stored_config[NV_MAG_CALIBRATION], 21);
            break;
         case GET_MAG_CALIBRATION_COMMAND:
            magCalibrationResponse = TRUE;
            break; 
         case STOP_STREAMING_COMMAND:
            if(sensing) stop_sensing = TRUE;
            break;
         case SET_GSR_RANGE_COMMAND:
            if((g_arg[0] == HW_RES_40K) || (g_arg[0] == HW_RES_287K) || 
               (g_arg[0] == HW_RES_1M) || (g_arg[0] == HW_RES_3M3) ||
               (g_arg[0] == GSR_AUTORANGE) || (g_arg[0] == GSR_X4)) {
               stored_config[NV_GSR_RANGE] = g_arg[0];
               call InternalFlash.write((void*)NV_GSR_RANGE, (void*)&stored_config[NV_GSR_RANGE], 1);
               if(sensing) {
                  wasSensing = TRUE;
                  stop_sensing = TRUE;
               }
            }
            break;
         case GET_GSR_RANGE_COMMAND:
            gsrRangeResponse = TRUE;
            break;
         case GET_SHIMMER_VERSION_COMMAND:
            shimmerVersionResponse = TRUE;
            break;
         case SET_EMG_CALIBRATION_COMMAND:
            memcpy(&stored_config[NV_EMG_CALIBRATION], g_arg, 4);
            call InternalFlash.write((void*)NV_EMG_CALIBRATION, (void*)&stored_config[NV_EMG_CALIBRATION], 4);
            break;
         case GET_EMG_CALIBRATION_COMMAND:
            emgCalibrationResponse = TRUE;
            break; 
         case SET_ECG_CALIBRATION_COMMAND:
            memcpy(&stored_config[NV_ECG_CALIBRATION], g_arg, 8);
            call InternalFlash.write((void*)NV_ECG_CALIBRATION, (void*)&stored_config[NV_ECG_CALIBRATION], 8);
            break;
         case GET_ECG_CALIBRATION_COMMAND:
            ecgCalibrationResponse = TRUE;
            break; 
         case GET_ALL_CALIBRATION_COMMAND:
            allCalibrationResponse = TRUE;
            break;
         case GET_FW_VERSION_COMMAND:
            fwVersionResponse = TRUE;
            break;
         case SET_BLINK_LED_COMMAND:
            atomic {
               ClearLedFunc();
               selectedLed = g_arg[0];
            }
            if(bt_connected) SetLedFunc();
            break;
         case GET_BLINK_LED_COMMAND:
            blinkLedResponse = TRUE;
            break;
         case SET_BUFFER_SIZE_COMMAND:
            stored_config[NV_BUFFER_SIZE] = g_arg[0];
            call InternalFlash.write((void*)NV_BUFFER_SIZE, (void*)&stored_config[NV_BUFFER_SIZE], 1);
            if(sensing) {
               wasSensing = TRUE;
               stop_sensing = TRUE;
            }
            break;
         case GET_BUFFER_SIZE_COMMAND:
            bufferSizeResponse = TRUE;
            break;
         case SET_MAG_GAIN_COMMAND:
            stored_config[NV_MAG_CONFIG] = (g_arg[0]<<4) + (stored_config[NV_MAG_CONFIG] & 0x0F);
            call InternalFlash.write((void*)NV_BUFFER_SIZE, (void*)&stored_config[NV_MAG_CONFIG], 1);
            if(sensing) {
               wasSensing = TRUE;
               stop_sensing = TRUE;
            }
            break;
         case GET_MAG_GAIN_COMMAND:
            magGainResponse = TRUE;
            break;
         case SET_MAG_SAMPLING_RATE_COMMAND:
            stored_config[NV_MAG_CONFIG] = g_arg[0] + (stored_config[NV_MAG_CONFIG] & 0xF0);
            call InternalFlash.write((void*)NV_BUFFER_SIZE, (void*)&stored_config[NV_MAG_CONFIG], 1);
            if(sensing) {
               wasSensing = TRUE;
               stop_sensing = TRUE;
            }
            break;
         case GET_MAG_SAMPLING_RATE_COMMAND:
            magSamplingRateResponse = TRUE;
            break;
         default:
      }
      sendAck = TRUE;
      post sendResponse();
   }

   
   task void collect_results() {
      int16_t realVals[3];
      register uint8_t i, j=0;
      
      if(stored_config[NV_SENSORS0] & SENSOR_MAG) {
         call Magnetometer.convertRegistersToData(readBuf, realVals);

         if(current_buffer == 0) {
            for(i = 0; i < 3; i++)
               *((uint16_t *)sbuf0 + j++ + nbr_adc_chans) = *(realVals + i);
         }
         else {
            for(i = 0; i < 3; i++)
               *((uint16_t *)sbuf1 + j++ + nbr_adc_chans) = *(realVals + i);
         }
      }

      if(stored_config[NV_SENSORS1] & SENSOR_HEART) {
         uint16_t b_t;
         
         if(beat_flag)
            b_t = beat_time >> 5;
         else
            b_t = 0;

         if(current_buffer == 0) 
            *((uint16_t *)sbuf0 + j + nbr_adc_chans) = b_t;
         else
            *((uint16_t *)sbuf1 + j + nbr_adc_chans) = b_t;

         beat_flag = FALSE;
      }

      post sendSensorData();
   }


   task void clockin_result() {
      if(stored_config[NV_SENSORS0] & SENSOR_MAG) {
         mag_read_pending = TRUE;
         call Magnetometer.readData();
      }
      else 
         post collect_results();
   }


   task void gsr_range() {
      // Fill the current active resistor into the upper two bits of the GSR value
      // if autorange is enabled, switch active resistor if required
      // If during resistor transition period use old ADC and resistor values
      // as determined by Gsr.smoothTransition()

      uint8_t current_active_resistor = gsr_active_resistor;
      uint16_t ADC_val;

      // GSR channel will always be last ADC channel
      if(current_buffer == 0) {
         ADC_val = *((uint16_t *)sbuf0 + (nbr_adc_chans - 1));
         if(stored_config[NV_GSR_RANGE] == GSR_AUTORANGE) {
            gsr_active_resistor = call Gsr.controlRange(ADC_val, gsr_active_resistor);
            if(call Gsr.smoothTransition(&current_active_resistor, stored_config[NV_SAMPLING_RATE])) {
               ADC_val = last_GSR_val;
            }
         }
         *((uint16_t *)sbuf0 + (nbr_adc_chans - 1)) =  ADC_val | (current_active_resistor << 14);
      }
      else {
         ADC_val = *((uint16_t *)sbuf1 + (nbr_adc_chans - 1));
         if(stored_config[NV_GSR_RANGE] == GSR_AUTORANGE) {
            gsr_active_resistor = call Gsr.controlRange(ADC_val, gsr_active_resistor);
            if(call Gsr.smoothTransition(&current_active_resistor, stored_config[NV_SAMPLING_RATE])) {
               ADC_val = last_GSR_val;
            }
         }
         *((uint16_t *)sbuf1 + (nbr_adc_chans - 1)) =  ADC_val | (current_active_resistor << 14);
      }

      last_GSR_val = ADC_val;
   }


   void init() {
#ifdef USE_8MHZ_CRYSTAL
      call FastClockInit.init();
      call FastClock.setSMCLK(1);  // set smclk to 1MHz
#endif // USE_8MHZ_CRYSTAL
   
      enable_sending = FALSE;
      bt_connected = FALSE;
      command_mode_complete = FALSE;
      sensing = FALSE;
      wasSensing = FALSE;
      configureChannels = FALSE;
      changeMagSamplingRate = FALSE;
      magStartContinuousConversion = FALSE;
      sendAck = FALSE;
      command_pending = FALSE;
      mag_read_pending = FALSE;
      stop_sensing_pending = FALSE;
      current_buffer = 0;
      nbr_1byte_digi_chans = 0;
      nbr_2byte_digi_chans = 0;
      nbr_adc_chans = 0;
      samplingRateResponse = FALSE;
      accelRangeResponse = FALSE;
      configSetupByte0Response = FALSE;
      gsrRangeResponse = FALSE;
      shimmerVersionResponse = FALSE;
      inquiryResponse = FALSE;
      accelCalibrationResponse = FALSE;
      gyroCalibrationResponse = FALSE;
      magCalibrationResponse = FALSE;
      emgCalibrationResponse = FALSE;
      ecgCalibrationResponse = FALSE;
      allCalibrationResponse = FALSE;
      fwVersionResponse = FALSE;
      blinkLedResponse = FALSE;
      bufferSizeResponse = FALSE;
      magGainResponse = FALSE;
      magSamplingRateResponse = FALSE;
      selectedLed = 0;
      activityLed = FALSE;
      warningLed = FALSE;
      warningLedCount = 0;
      beat_flag = 0;
      last_GSR_val = 0;
      *tx_packet0 = DATA_PACKET;
      *tx_packet1 = DATA_PACKET;
      current_tx_packet = tx_packet0;
      buffer_iteration = 0;
      stop_sensing = FALSE;

      call BluetoothInit.init();
      call Bluetooth.disableRemoteConfig(TRUE);

      call SampleTimerInit.init();
      call BlinkTimerInit.init();

      call InternalFlash.read((void*)0, (void*)stored_config, NV_NUM_CONFIG_BYTES);
      if(stored_config[NV_SENSORS0] == 0xFF) {
         // if config was never written to Infomem write default
         // Accel only, 50Hz, buffer size of 1, GSR range as HW_RES_40K, 5V reg, PMUX and gyro temperature/vref off
         stored_config[NV_SAMPLING_RATE] = SAMPLING_50HZ;
         stored_config[NV_BUFFER_SIZE] = 1;
         stored_config[NV_SENSORS0] = SENSOR_ACCEL;
         stored_config[NV_SENSORS1] = 0;
         stored_config[NV_ACCEL_RANGE] = RANGE_1_5G;
         stored_config[NV_MAG_CONFIG] = (ONE_GAUSS<<4) + TEN_HZ;
         stored_config[NV_GSR_RANGE] = HW_RES_40K;
         stored_config[NV_CONFIG_SETUP_BYTE0] = 0;
         call InternalFlash.write((void*)0, (void*)stored_config, NV_NUM_CONFIG_BYTES);
      }

      // Set SER0_RTS pin to be I/O in order to control 5V regulator on AnEx board
      TOSH_MAKE_SER0_RTS_OUTPUT();
      TOSH_SEL_SER0_RTS_IOFUNC();
      if(stored_config[NV_CONFIG_SETUP_BYTE0] & CONFIG_5V_REG)
	      TOSH_SET_SER0_RTS_PIN();
      else
         TOSH_CLR_SER0_RTS_PIN();
#ifdef PWRMUX_UTIL
      // Set PMUX
      if(stored_config[NV_CONFIG_SETUP_BYTE0] & CONFIG_PMUX)
         TOSH_SET_PWRMUX_SEL_PIN();
      else
         TOSH_CLR_PWRMUX_SEL_PIN();
#endif
      if(stored_config[NV_CONFIG_SETUP_BYTE0] & CONFIG_GYRO_TEMP_VREF)
         TOSH_SET_SER0_CTS_PIN();
      else
         TOSH_CLR_SER0_CTS_PIN();

      configure_channels();

      if(!call DockInterrupt.getValue()){ // on the dock
         call DockInterrupt.edge(TRUE);   // watch for it to go high, off the dock
      } else {                            // off the dock
         call DockInterrupt.edge(FALSE);  // watch for it to go high, off the dock
      }
      call DockInterrupt.enable();
      call DockInterrupt.clear();
   }


   void configure_channels() {
      uint8_t *channel_contents_ptr = channel_contents;

      nbr_adc_chans = 0;
      nbr_1byte_digi_chans = 0;
      nbr_2byte_digi_chans = 0;

      call shimmerAnalogSetup.reset();
      // Accel
      if(stored_config[NV_SENSORS0] & SENSOR_ACCEL) {
         call AccelInit.init();
         call shimmerAnalogSetup.addAccelInputs();
         *channel_contents_ptr++ = X_ACCEL;
         *channel_contents_ptr++ = Y_ACCEL;
         *channel_contents_ptr++ = Z_ACCEL;
      }
      // Gyro
      if(stored_config[NV_SENSORS0] & SENSOR_GYRO){
         call shimmerAnalogSetup.addGyroInputs();
         *channel_contents_ptr++ = X_GYRO;
         *channel_contents_ptr++ = Y_GYRO;
         *channel_contents_ptr++ = Z_GYRO;
         //turn on red led to indicate inialisation
         ToggleLedFunc();
         warningLedCount = 0;
         warningLed = TRUE;
         call BlinkTimer.stop();
         call BlinkTimer.start(102);
         call GyroInit.init();
         // initialising the gyro also starts it
         // so turn it off now to save power
         call GyroStdControl.stop();
         call BlinkTimer.stop();
         warningLed = FALSE;
         if(bt_connected) SetLedFunc(); //connected
         else ClearLedFunc();           //not connected

         //need to check/set GYRO_TEMP_VREF here as GyroInit.init() resets it to low
         if(stored_config[NV_CONFIG_SETUP_BYTE0] & CONFIG_GYRO_TEMP_VREF)
            TOSH_SET_SER0_CTS_PIN();
      }
      // ECG
      else if(stored_config[NV_SENSORS0] & SENSOR_ECG){
         call shimmerAnalogSetup.addECGInputs();
         *channel_contents_ptr++ = ECG_RA_LL;
         *channel_contents_ptr++ = ECG_LA_LL;
      }
      // EMG
      else if(stored_config[NV_SENSORS0] & SENSOR_EMG){
         call shimmerAnalogSetup.addEMGInput();
         *channel_contents_ptr++ = EMG;
      }
      // AnEx A7
      if(stored_config[NV_SENSORS0] & SENSOR_ANEX_A7) {
         call shimmerAnalogSetup.addAnExInput(7);
         *channel_contents_ptr++ = ANEX_A7;
      }
      // AnEx A0
      if(stored_config[NV_SENSORS0] & SENSOR_ANEX_A0) {
         call shimmerAnalogSetup.addAnExInput(0);
         *channel_contents_ptr++ = ANEX_A0;
      }
      // Strain Gauge
      if(stored_config[NV_SENSORS1] & SENSOR_STRAIN) {
         call shimmerAnalogSetup.addStrainGaugeInputs();
         *channel_contents_ptr++ = STRAIN_HIGH;
         *channel_contents_ptr++ = STRAIN_LOW;
      }
      // GSR
      // needs to be the last analog channel
      else if(stored_config[NV_SENSORS0] & SENSOR_GSR){
         call shimmerAnalogSetup.addGSRInput();
         *channel_contents_ptr++ = GSR_RAW;
      }
      // Digital channels need to be after ADC channels to allow for DMA
      // Mag
      if(stored_config[NV_SENSORS0] & SENSOR_MAG) {
         *channel_contents_ptr++ = X_MAG;
         *channel_contents_ptr++ = Y_MAG;
         *channel_contents_ptr++ = Z_MAG;
         nbr_2byte_digi_chans += 3;
      }
      // Heart Rate
      if(stored_config[NV_SENSORS1] & SENSOR_HEART) {
         *channel_contents_ptr++ = HEART_RATE;
         nbr_2byte_digi_chans += 1;
         call UserButton.disable(); //ensure the UserButton is not enabled when the HR sensor is configured
      } else {
         call UserButton.enable();
      }

      call shimmerAnalogSetup.finishADCSetup((uint16_t *)sbuf0);
      nbr_adc_chans = call shimmerAnalogSetup.getNumberOfChannels();


      if(wasSensing) {
         wasSensing = FALSE;
         post startSensors();
      }
   }


   // The MSP430 CPU is byte addressed and little endian/
   // packets are sent little endian so the word 0xABCD will be sent as bytes 0xCD 0xAB
   void prepareDataPacket() {
      uint8_t sample_size;
      
      sample_size = ((nbr_adc_chans+nbr_2byte_digi_chans)*2)+nbr_1byte_digi_chans+2;
      if(current_buffer == 0) {
         memcpy(current_tx_packet+1+(sample_size*buffer_iteration), (uint8_t *)&timestamp0, 2);
         memcpy(current_tx_packet+3+(sample_size*buffer_iteration), sbuf0, sample_size-2);
         current_buffer = 1;
      }
      else {
         memcpy(current_tx_packet+1+(sample_size*buffer_iteration), (uint8_t *)&timestamp1, 2);
         memcpy(current_tx_packet+3+(sample_size*buffer_iteration), sbuf1, sample_size-2);
         current_buffer = 0;
      }
      buffer_iteration++;
   }


   event void Boot.booted() {
      init();
      call BTStdControl.start(); 
   }


   async event void Bluetooth.connectionMade(uint8_t status) { 
      enable_sending = TRUE;
      bt_connected = TRUE;
      call BlinkTimer.stop();
      SetLedFunc();
   }

   
   async event void Bluetooth.commandModeEnded() { 
      command_mode_complete = TRUE;
      SetLedFunc();
      ledOn = TRUE;
      call BlinkTimer.start(51);
   }
    

   async event void Bluetooth.connectionClosed(uint8_t reason){
      enable_sending = FALSE;
      bt_connected = FALSE;
      if(sensing)
         stop_sensing = TRUE;
      else
         post stopSensing();
   }


   async event void Bluetooth.dataAvailable(uint8_t data){
      if(bt_connected)
         call BtCommandParser.handleByte(data);
   }


   event void Bluetooth.writeDone(){
      enable_sending = TRUE;
   }


   event void SetupTimer.fired() {
      atomic if(command_mode_complete){
         post startSensors();
      }
   }


   async event void BlinkTimer.fired() {
      if(activityLed) { //most common case, so check first
         //Blink at 1Hz to indicate streaming
         call BlinkTimer.start(1024);
         post ToggleLed();
      } else if (warningLed) {
         //Blink at 10Hz to indicate shimmer is active
         if(warningLedCount) {
            if(!(--warningLedCount)) {
               warningLed = FALSE;
               SetLedFunc();
               if(!bt_connected) {
                  call BlinkTimer.start(51);
                  ledOn = TRUE;
               }
               return;
            }
         }
         call BlinkTimer.start(102);
         ToggleLedFunc();
      } else {
         //power on indication
         if(ledOn) {
            call BlinkTimer.start(2048);
            ClearLedFunc();
            ledOn = FALSE;
         } else {
            call BlinkTimer.start(51);
            SetLedFunc();
            ledOn = TRUE;
         }
      }
   }


   //event void SampleTimer.fired() {
   async event void SampleTimer.fired() {
      call SampleTimer.start(stored_config[NV_SAMPLING_RATE]); 
      if(current_buffer == 0){
         timestamp0 = call LocalTime.get();
      }
      else { 
         timestamp1 = call LocalTime.get();
      }
      if(nbr_adc_chans > 0)
         call shimmerAnalogSetup.triggerConversion();
      else if((nbr_1byte_digi_chans > 0) || (nbr_2byte_digi_chans > 0))
         post clockin_result();
   }


   async event void DMA0.transferDone(error_t success) {
      if(current_buffer == 0){
         call DMA0.repeatTransfer((void*)ADC12MEM0_, (void*)sbuf1, nbr_adc_chans);
      }
      else { 
         call DMA0.repeatTransfer((void*)ADC12MEM0_, (void*)sbuf0, nbr_adc_chans);
      }
      if(stored_config[NV_SENSORS0] & SENSOR_GSR) {
         post gsr_range();
      }
      if((nbr_1byte_digi_chans > 0) || (nbr_2byte_digi_chans > 0))
         post clockin_result();
      else
         post sendSensorData();      
   }


   async event void BtCommandParser.activate(uint8_t action, uint8_t arg_size, uint8_t *arg){
      bool temp;

      atomic temp = command_pending; // to keep atomic section as short as possible
      if(!temp) {
         g_arg_size = arg_size;
         memcpy(g_arg, arg, arg_size);
         g_action = action;
         command_pending = TRUE;
         post ProcessCommand();
      }
   }


   async event void GyroBoard.buttonPressed() {
   }


   async event void Magnetometer.readDone(uint8_t * data, error_t success) {
      memcpy(readBuf, data, 7);
      mag_read_pending = FALSE;
      if(!stop_sensing_pending)
         post collect_results();
   }


   async event void Magnetometer.writeDone(error_t success){
      if(changeMagSamplingRate) {
         changeMagSamplingRate = FALSE;
         call Magnetometer.setOutputRate(stored_config[NV_MAG_CONFIG]&0x0F);
      } else if(magStartContinuousConversion) {  
         magStartContinuousConversion = FALSE;
         call Magnetometer.runContinuousConversion();
      } else {
         startStreaming();
      }
   }

   async event void DigitalHeartRate.beat(uint32_t time) {
      beat_flag = TRUE;
      beat_time = time;
   }

   async event void DockInterrupt.fired() {
      if (call DockInterrupt.getValue() == TRUE){  // off the dock
         call DockInterrupt.edge(FALSE);
      } else {                                     // on the dock
         call DockInterrupt.edge(TRUE);      
         if(!sensing) {
            atomic {
               // reset the selected LED to green
               ClearLedFunc();
               selectedLed = 0;
            }
            if(bt_connected) SetLedFunc();
         }
      }
      call DockInterrupt.clear();
   }

   event void UserButton.notify(button_state_t val) {
      if(!(stored_config[NV_SENSORS1] & SENSOR_HEART)) {
         TBR = 0;
         warningLedCount = 10;
         warningLed = TRUE;
         call BlinkTimer.stop();
         call BlinkTimer.start(102);
      }
   }
}
