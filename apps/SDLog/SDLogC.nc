/*
 * Copyright (c) 2012, Shimmer Research, Ltd.
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
 * @author  Niamh O'Mahony
 * @date    September, 2012
 * (Adapted from ParamLogging (author: Steve Ayer))
 */

#include "Mma_Accel.h"
#include "msp430usart.h"
#include "ShimmerSD.h"
#include "Gsr.h"
#include "TimeBroadcast.h"
#include "LoggingBroadcast.h"
#include "UserButton.h"
#include "msp430usart.h"

module SDLogC {
  uses {
    interface Boot;

    interface FatFs;

    interface shimmerAnalogSetup;

    interface Msp430DmaChannel as DMA0;

    interface ReadId48 as IDChip;

    interface Init as AccelInit;
    interface Mma_Accel as Accel;

    interface Init as GyroInit;
    interface StdControl as GyroStdControl;
    interface GyroBoard;

    interface Init as MagInit;
    interface Magnetometer;

    interface Init as GsrInit;
    interface Gsr;

    interface Init as StrainGaugeInit;
    interface StrainGauge;

    interface Init as DigitalHeartInit;
    interface DigitalHeartRate;

    interface Time;
    interface LocalTime<T32khz>;

    interface Leds;

    interface Init as sampleTimerInit;
    interface Alarm<TMilli, uint16_t> as sampleTimer;
    
    interface Init as broadcastTimerInit;
    interface Alarm<TMilli, uint16_t> as broadcastTimer;
    
    interface Timer<TMilli> as listenTimer;
    interface Timer<TMilli> as ledTimer;
    interface Timer<TMilli> as warningTimer;

    interface Notify<button_state_t> as UserButton;
    interface Notify<button_state_t> as GyroButton;

    interface HplMsp430Usart as UARTControl;
    interface HplMsp430UsartInterrupts as UARTData;

    interface SplitControl as AMRadioControl;
    interface AMSend as AMRadioSend;
    interface Receive;
    interface Packet;

    interface PacketTimeStamp<T32khz, uint32_t>;
    interface TimeSyncPacket<T32khz, uint32_t>;
  }
}

implementation {
  extern int sprintf(char *str, const char *format, ...) __attribute__ ((C));
  extern int snprintf(char *str, size_t len, const char *format, ...) __attribute__ ((C));

  void do_stores(uint32_t);
  task void startup();
  task void prepareNewSession();
  task void getSamplingConfig();
  void calibrate();
  void writeConfigHeader(uint32_t);
  void storeConfigBytes();
  char * fgets(char * buff, int len, FIL* fp);
  void busControlToMag();
  void busControlToSD();
  void collect_results();
  void dummy_results();
  void startLogging();
  void sendTimestamp();
  void sendSerial();
  void sendLoggingMessage(bool);
  void default_calibration(uint8_t);
  void read_calibration(uint8_t);

  // Boolean variables
  bool directory_set = FALSE, bad_opendir = FALSE, bad_mkdir = FALSE, justbooted = FALSE, config_stored = FALSE, file_error = FALSE;
  bool gsr_autorange = FALSE, gsr_selected = FALSE, gyro_selected = FALSE;
  bool user_button_enable = FALSE, gyro_button_enable = FALSE;
  bool docked = TRUE, logging = FALSE, first_sample = FALSE, first_write_done = FALSE;
  bool heart_rate = FALSE, beat_flag = FALSE;
  bool set_5V_reg = FALSE, set_PMUX = FALSE;
  bool mag_requested = FALSE, doin_the_mag = FALSE, got_analog = FALSE, mag_change_rate = FALSE, mag_run_continuous = FALSE, mag_ready = FALSE;
  bool iAmMaster = FALSE, time_sync = FALSE, timestamps = TRUE, realtime = FALSE, singletouch = FALSE, timestamp_received = FALSE, timestamp_requested = FALSE, waiting_for_hosttime = FALSE;
  bool radio_enabled = FALSE, radio_locked = FALSE, update_received = FALSE, first_timestamp_recvd = FALSE;
  bool pending_start_logging = FALSE, logging_message = FALSE, ledOff = TRUE, warningLedOn = TRUE, error = FALSE; 
  bool write_ongoing = FALSE;
 
  // 1-bye variables
  uint8_t longAddress[6], NUM_ADC_CHANS = 0, dirname[64], exp_dir_name[32], filename[64], idname[12], exp_id_name[12], max_chars = 12, shimmername[12]; 
  uint8_t readBuf[8], bytes_per_sample, offset_sign = 0, sensor_array[16], mag_channels;
  uint8_t accel_range = 0, gsr_range = 0, mag_range = 1, mag_rate = 4, active_resistor; 
  uint8_t data_to_send[8], max_time_bytes = 8, total_bytes = 0, bytes_sent = 0, host_time[8], time_bytes_received = 0;
  uint8_t num_shimmers_in_trial = 1, my_trial_id = 1;
  uint8_t stop_message_count = 0;
  uint8_t broadcast_interval = 120, count_seconds = 0, listening_seconds = 0, listen_window = 60, min_listen_window = 1;
  norace uint8_t stored_config[NV_NUM_CONFIG_BYTES];
  norace uint8_t current_buffer = 0, current_dig_buffer = 0;
  
  norace uint16_t dma_samples = 0, digital_samples = 0;

  // 2-byte variables
  uint16_t sequence_number = 0, sample_period, dir_counter, fs_block_size, block_samples;
  int16_t sbuf0[NV_SD_WRITE_SIZE], sbuf1[NV_SD_WRITE_SIZE], *mag_slot, last_mag_x = 0, last_mag_y = 0, last_mag_z = 0; 
  uint16_t timestamps0[NV_SD_WRITE_SIZE], timestamps1[NV_SD_WRITE_SIZE];
  uint16_t last_ADC_val = 500;
  const uint16_t one_second = 1024, one_hour = 3600, fifty_ms = 51; 

  float fsample;
 
  // 4-byte variables
  uint32_t my_timestamp = 0xFFFFFFFF, master_timestamp = 0, initial_timestamp0 = 0, initial_timestamp1 = 0, config_time = 0, time_offset = 0xFFFFFFFF, beat_time, my_time_at_host_time = 0;

  message_t radio_packet;

  FATFS gfs;
  FIL gfp; 
  DIR gdp;// data

#define SPI_TX_DONE  while(call UARTControl.isTxEmpty() == FALSE);
#define CS_LOW()  TOSH_CLR_SD_CS_N_PIN(); // card select
#define CS_HIGH() SPI_TX_DONE; TOSH_SET_SD_CS_N_PIN();

  void powerCycle() {
      register uint8_t i;

      //wait until the tx buf is clear before killing the card
      CS_HIGH();

      // this connects the path from mcu to card
      TOSH_MAKE_DOCK_N_OUTPUT();
      TOSH_SET_DOCK_N_PIN();

      TOSH_SET_SW_SD_PWR_N_PIN();
      TOSH_CLR_SD_CS_N_PIN();

      // clear all input pins to the card
      call UARTControl.disableSpi();
      TOSH_CLR_SD_DI_PIN();
      TOSH_CLR_SD_DO_PIN();
      TOSH_CLR_SD_CLK_PIN();

      for(i = 0; i < 10; i ++)
          TOSH_uwait(6000);

      TOSH_SET_SD_CS_N_PIN();
      TOSH_CLR_SW_SD_PWR_N_PIN();

      // undo the override above
      TOSH_MAKE_DOCK_N_INPUT();
  }

  inline float pow(float number, uint8_t power){
      float result = 1;
      uint8_t i = 0;

      if(power < 0){
          for(i = 0; i < -power; i++)
              result /= number;
      }
      else{
          for(i = 0; i < power; i ++)
              result *= number;
      }
      return result;
  }

  inline float atof(char *string){
      char *decimal;
      uint8_t decimal_string[12], num_chars;
      int32_t integer;
      float fraction;
      
      integer = atoi(string);
      
      decimal = strchr(string, '.');
      if(!decimal)
          decimal = strchr(string, ','); // compatibility with European locale settings
      if(decimal){
          decimal++;
          num_chars = snprintf(decimal_string, 4, "%s", decimal); // copy up to three decimal places

          fraction = ((float)atoi(decimal_string))/pow(10.0, num_chars<4?num_chars-1:3);

          if(strchr(string, '-')) // check the sign to correctly apply the fraction part
              return (float)integer - fraction;
          else
              return (float)integer + fraction;
      }
      else // it's an integer
          return (float)integer;
  }

  error_t set_basedir() { 
    FILINFO gfi;
    error_t res;
    uint16_t tmp_counter = 0;
    char lfn[_MAX_LFN + 1], * fname, * scout, * dash, dirnum[8];

    // first we'll make the shimmer mac address into a string
     if(strlen(shimmername)==0) // if name hasn't been assigned by user, use default (from Shimmer mac address)
        sprintf(shimmername, "ID%02x%02x", longAddress[1], longAddress[0]); // the "x" will be deleted in the next step

     if(strlen(exp_id_name)==0) // if name hasn't been assigned by user, use default (from Shimmer mac address)
        sprintf(exp_id_name, "default_exp"); 

    sprintf(idname, "%s", shimmername);

    gfi.lfname = lfn;
    gfi.lfsize = sizeof(lfn);

    if((res = call FatFs.opendir(&gdp, "/data"))){
      if(res == FR_NO_PATH)      // we'll have to make /data first
          res = call FatFs.mkdir("/data");
      if(res)         // in every case, we're toast
          return FAIL;

      // try one more time
      if((res = call FatFs.opendir(&gdp, "/data")))
          return FAIL;
    }

    sprintf(exp_dir_name, "data/%s_%ld", exp_id_name, config_time);
    if((res = call FatFs.opendir(&gdp, exp_dir_name))){
      if(res == FR_NO_PATH)      // we'll have to make the experiment folder first
          res = call FatFs.mkdir(exp_dir_name);
      if(res)         // in every case, we're toast
          return FAIL;

      // try one more time
      if((res = call FatFs.opendir(&gdp, exp_dir_name)))
          return FAIL;
    }

    dir_counter = 0;   // this might be the first log for this shimmer

    /*
     * file name format
     * shimmername    as defined in sdlog.cfg
     * -              separator
     * 000
     *
     * we want to create a new directory with a sequential run number each power-up/reset for each shimmer
     */
    while(call FatFs.readdir(&gdp, &gfi) == FR_OK){
      if(*gfi.fname == 0)
	break;
      else if(gfi.fattrib & AM_DIR){      
          fname = (*gfi.lfname) ? gfi.lfname : gfi.fname;
         
          //if(!strncmp(fname, idname, strlen(idname))){  
          if(!strncmp(fname, idname, strlen(fname)-4)){ // -4 because of the -000 etc. 
              if((scout = strchr(fname, '-'))){   // if not, something is seriously wrong!
                  scout++;
                  while((dash = strchr(scout, '-'))) // In case the shimmer name contains '-'
                      scout = dash + 1;
                  strcpy(dirnum, scout);
                  tmp_counter = atoi(dirnum);
                  if(tmp_counter >= dir_counter){
                      dir_counter = tmp_counter;
                      dir_counter++;                   // start with next in numerical sequence
                  }
              }
              else
                  return FAIL;
          }
      }
    }

    // at this point, we have the id string and the counter, so we can make a directory name
    return SUCCESS;
  }

  error_t make_basedir() { 
    sprintf(dirname, "%s/%s-%03d", exp_dir_name, idname, dir_counter);

    if(call FatFs.mkdir(dirname))
      return FAIL;
    
    sequence_number = 0;
    first_sample = FALSE;
    first_write_done = FALSE;
    config_stored = FALSE;
    first_timestamp_recvd = FALSE;
   
    return SUCCESS;
  }

  task void store_contents0() {
    uint bytesWritten;
    uint16_t i, j, packet[fs_block_size + 5*time_sync]; // fs_block_size = bytes_per_sample + 2byte timestamp per block
    uint8_t *p_packet = (uint8_t *)&packet[0];
    uint8_t *pbuffer;
    uint32_t max_file_size;
    uint32_t file_position;
    uint32_t initial_timestamp;

    if(current_buffer == 0){ // too late, the buffer is already being re-filled
        return;
    }
    else{
        atomic{        
            initial_timestamp = initial_timestamp0;
            pbuffer = (uint8_t *)&sbuf0;
            for(i = 0; i < block_samples; i++){
                if(timestamps){
                    *(p_packet + 5*time_sync + i*(bytes_per_sample+2)) = timestamps0[i] & 0xFF;
                    *(p_packet + 5*time_sync + i*(bytes_per_sample+2) + 1) = (timestamps0[i] >> 8) & 0xFF;
                }
                for(j = 0; j < bytes_per_sample; j+=2){
                    *(p_packet + 5*time_sync + i*(bytes_per_sample+2*timestamps) + 2*timestamps + j) = *(pbuffer + i*bytes_per_sample + j) & 0xFF;
                    *(p_packet + 5*time_sync + i*(bytes_per_sample+2*timestamps) + 2*timestamps + j + 1) = (*(pbuffer + i*bytes_per_sample + j + 1)) & 0xFF;
                }
            }
        }

        if(time_sync){
            *(p_packet) = offset_sign & 0xFF;
            *(p_packet + 1) = time_offset & 0xFF;
            *(p_packet + 2) = (time_offset >> 8) & 0xFF;
            *(p_packet + 3) = (time_offset >> 16) & 0xFF;
            *(p_packet + 4) = (time_offset >> 24) & 0xFF;
            if(!iAmMaster){
                time_offset = 0xFFFFFFFF;
                offset_sign = 0;
            }
        }
 
        if(doin_the_mag){
            i = 0;
            while(mag_requested || (i > 10000)){
                i++;
            }
            if(mag_requested)
                call Leds.led0On();
            busControlToSD();
        }
        else
            atomic{write_ongoing = TRUE;}

        if(first_write_done == FALSE){
            do_stores(initial_timestamp);

            atomic{
                TOSH_MAKE_DOCK_N_OUTPUT();    // we need these to disable the dock interrupt because do_stores enabled it.
                TOSH_SET_DOCK_N_PIN();
            }

            first_write_done = TRUE;
        }
        else{
            max_file_size = (uint32_t)one_hour*( uint32_t)fsample*(uint32_t)(bytes_per_sample + 2*timestamps);
            file_position = gfp.fptr;
            if(file_position > max_file_size){  // @ SampRate hz * NUM_ADC_CHANS channels, one hour
                call FatFs.fclose(&gfp);                             // close this file
                do_stores(initial_timestamp);

                atomic{
                    TOSH_MAKE_DOCK_N_OUTPUT();  // we need these to disable the dock interrupt because do_stores enabled it.
                    TOSH_SET_DOCK_N_PIN();
                }
            }
        }
      
        call FatFs.fwrite(&gfp, (uint8_t *)p_packet, fs_block_size + 5*time_sync, &bytesWritten);    
        call FatFs.fsync(&gfp);

        /*
         * ugly way to poll dock, since we disabled driver's ability to get 
         * a hw interrupt.  better than blowing out the filesystem with a broken write
         * to disk.
         */

        // enable the dock again
        TOSH_MAKE_DOCK_N_INPUT();
        atomic if(!TOSH_READ_DOCK_N_PIN()){ // sitting on the dock
            call sampleTimer.stop();

            /*
             * the next thing the apps sees should be 
             * an fatfs.available event.  unmount
             * will call diskiostdcontrol.init, which will
             * powercycle the card, put it in sd mode for the dock,
             * and re-initialize the interrupt
             */
            call FatFs.unmount();      
        }

        if(doin_the_mag){
            busControlToMag();
        }
        else
            atomic{write_ongoing = FALSE;}
    }
  }

  task void store_contents1() {
    uint bytesWritten;
    uint16_t i, j, packet[fs_block_size + 5*time_sync]; // fs_block_size = bytes_per_sample + 2byte timestamp per block
    uint8_t *p_packet = (uint8_t *)&packet[0];
    uint8_t *pbuffer;
    uint32_t max_file_size;
    uint32_t file_position; 
    uint32_t initial_timestamp;

    if(current_buffer == 1){ // too late, the buffer is already being re-filled
        return;
    }
    else{
        atomic{        
            initial_timestamp = initial_timestamp1;
            pbuffer = (uint8_t *)&sbuf1;
            for(i = 0; i < block_samples; i++){
                if(timestamps){
                    *(p_packet + 5*time_sync + i*(bytes_per_sample+2)) = timestamps1[i] & 0xFF;
                    *(p_packet + 5*time_sync + i*(bytes_per_sample+2) + 1) = (timestamps1[i] >> 8) & 0xFF;
                }
                for(j = 0; j < bytes_per_sample; j+=2){
                    *(p_packet + 5*time_sync + i*(bytes_per_sample+2*timestamps) + 2*timestamps + j) = *(pbuffer + i*bytes_per_sample + j) & 0xFF;
                    *(p_packet + 5*time_sync + i*(bytes_per_sample+2*timestamps) + 2*timestamps + j + 1) = (*(pbuffer + i*bytes_per_sample + j + 1)) & 0xFF;
                }
            }
        }

        if(time_sync){
            *(p_packet) = offset_sign & 0xFF;
            *(p_packet + 1) = time_offset & 0xFF;
            *(p_packet + 2) = (time_offset >> 8) & 0xFF;
            *(p_packet + 3) = (time_offset >> 16) & 0xFF;
            *(p_packet + 4) = (time_offset >> 24) & 0xFF;
            if(!iAmMaster){
                time_offset = 0xFFFFFFFF;
                offset_sign = 0;
            }
        }
    
        if(doin_the_mag){
            i = 0;
            while(mag_requested || (i > 10000)){
                i++;
            }
            if(mag_requested)
                call Leds.led0On();
            busControlToSD();
        }
        else
            atomic{write_ongoing = TRUE;}

        if(first_write_done == FALSE){ 
            do_stores(initial_timestamp);

            atomic{
                TOSH_MAKE_DOCK_N_OUTPUT();                // we need these because do_stores has its own output/input transition
                TOSH_SET_DOCK_N_PIN();
            }
      
            first_write_done = TRUE;
        }
        else{
            max_file_size = (uint32_t)one_hour*( uint32_t)fsample*(uint32_t)(bytes_per_sample + 2*timestamps);
            file_position = gfp.fptr;

            if(file_position > max_file_size){  // @ SampRate hz * NUM_ADC_CHANS channels, one hour
                call FatFs.fclose(&gfp);                             // close this file
                do_stores(initial_timestamp);

                atomic{
                    TOSH_MAKE_DOCK_N_OUTPUT();                // we need these because do_stores has its own output/input transition
                    TOSH_SET_DOCK_N_PIN();
                }
            }
        }
       
        call FatFs.fwrite(&gfp, (uint8_t *)p_packet, fs_block_size + 5*time_sync, &bytesWritten);    
        call FatFs.fsync(&gfp);
        /*
         * ugly way to poll dock, since we disabled driver's ability to get 
         * a hw interrupt.  better than blowing out the filesystem with a broken write
         * to disk.
         */
        // enable the dock interrupt
        TOSH_MAKE_DOCK_N_INPUT();
        atomic if(!TOSH_READ_DOCK_N_PIN()){ // sitting on the dock
            call sampleTimer.stop();

            /*
             * the next thing the apps sees should be 
             * an fatfs.available event.  unmount
             * will call diskiostdcontrol.init, which will
             * powercycle the card, put it in sd mode for the dock,
             * and re-initialize the interrupt
             */
            call FatFs.unmount();      
        }

        if(doin_the_mag){
            busControlToMag();
        }
        else
            atomic{write_ongoing = FALSE;}

    }
  }

  void initialize_directories() {
    atomic directory_set = 1;

    bad_opendir = bad_mkdir = 0;

    TOSH_MAKE_DOCK_N_OUTPUT();
    TOSH_SET_DOCK_N_PIN();

    if(set_basedir() != SUCCESS)
      bad_opendir = 1;

    if(make_basedir() != SUCCESS)
      bad_mkdir = 1;

    TOSH_MAKE_DOCK_N_INPUT();
  }
  
  void setupUART(){
      msp430_uart_union_config_t RN_uart_config = { { ubr: UBR_4MHZ_115200, umctl: UMCTL_4MHZ_115200,
          ssel: 0x02, pena: 0, pev: 0, spb: 0, clen: 1, listen: 0,
          mm: 0, ckpl: 0, urxse: 0, urxeie: 0,
          urxwie: 0, utxe: 1, urxe: 1} };

      call UARTControl.setModeUart(&RN_uart_config);

      call UARTControl.enableTxIntr();
      call UARTControl.enableRxIntr();
  }

  event void Boot.booted() {
      justbooted = 1;

      sample_period = 20;   // 51.2 hz
      fsample = floor(1024/sample_period + 0.5);
      stored_config[NV_SAMPLING_RATE] = (uint8_t)((uint16_t)fsample >> 8);
      stored_config[NV_SAMPLING_RATE + 1] = (uint8_t)fsample;

      call sampleTimerInit.init();
      call broadcastTimerInit.init();

      // we'll use this to id which shimmer wrote the files
      call IDChip.read(longAddress);

      post startup();
  }

  task void startup(){
    timestamps = TRUE;
    error = FALSE;
    
    radio_enabled = FALSE;
    call AMRadioControl.stop(); // just in case 

 
    // just a flag so we only do this once
    directory_set = 0;

    dma_samples = 0;
    digital_samples = 0;

    // Disable both buttons to begin with to prevent floating values triggering them
    call UserButton.disable();
    call GyroButton.disable();

    call FatFs.mount(&gfs);

    call ledTimer.startOneShot(2*one_second);
  }

  void busControlToSD(){
      atomic write_ongoing = TRUE;

      call UARTControl.setModeSpi(&msp430_spi_default_config);
      call UARTControl.enableRxIntr();
    
      while(call UARTControl.isTxEmpty() == FALSE);

      TOSH_CLR_SD_CS_N_PIN(); 
  }

  void busControlToMag(){
    TOSH_SET_SD_CS_N_PIN();  
   
    call Magnetometer.enableBus();
    
    atomic write_ongoing = FALSE;
  }

  /*
   * simple function to erase mutually-exclusive selection errors
   * i.e., you can only have one daughter card at a time, so we
   * should eliminate selecting more than one input combo
   * note that accel can operate with any of these...
   */
  void errorCheckArray() {
    if(*(sensor_array + 1) || *(sensor_array + 2))    // gyro and/or mag board: eliminate ecg, emg, gsr, str
      *(sensor_array + 3) = *(sensor_array + 4) = *(sensor_array + 5) = *(sensor_array + 8) = 0;
    else if(*(sensor_array + 3))  // ecg: eliminate emg, gsr, str
      *(sensor_array + 4) = *(sensor_array + 5) = *(sensor_array + 8) = 0;
    else if(*(sensor_array + 4))  // emg: eliminate gsr, str
      *(sensor_array + 5) = *(sensor_array + 8) = 0;
    else if(*(sensor_array + 5))  // gsr: eliminate gsr
      *(sensor_array + 8) = 0;

    if(!set_PMUX && (*(sensor_array + 6) || *(sensor_array + 7))) // AnEx (except Battery monitor): eliminate HeartRate
      *(sensor_array + 9) = 0;

    if(*(sensor_array + 9))
        user_button_enable = FALSE;

    // If both buttons are enabled, user button takes precedence
    if(user_button_enable && gyro_button_enable){
        gyro_button_enable = FALSE;
    }

    // If singletouch is enabled and neither button is enabled, enable user button by default
    if(singletouch && !user_button_enable && !gyro_button_enable)
        user_button_enable = TRUE;

    if(singletouch && !time_sync)
        time_sync == TRUE;
  }

  void setSamplingConfig() { 
    register uint16_t i;

    call shimmerAnalogSetup.reset(); 
    errorCheckArray();
    for(i = 0; i < 10; i++){
      switch(i){
          case 0:
              if(*sensor_array){
                  call AccelInit.init();
                  if((accel_range == RANGE_1_5G) || (accel_range == RANGE_2_0G) || (accel_range == RANGE_4_0G) || (accel_range == RANGE_6_0G))
                      call Accel.setSensitivity(accel_range);
                  else
                      call Accel.setSensitivity(RANGE_1_5G);
                  call shimmerAnalogSetup.addAccelInputs();
                  got_analog = 1;
              }
              break;
          case 1:
              if(*(sensor_array + 1)){
                  gyro_selected = TRUE;
                  call GyroInit.init();
                  call GyroStdControl.start();
                  call shimmerAnalogSetup.addGyroInputs();
                  got_analog = 1;
              }

              break;
          case 2:
              if(*(sensor_array + 2)){
                  TOSH_SET_SD_CS_N_PIN();  
                  call MagInit.init();
	 
                  mag_change_rate = TRUE;
                  mag_run_continuous = TRUE;
                  call Magnetometer.setGain(mag_range);
      
                  doin_the_mag = 1;
              }
              break;
          case 3:
              if(*(sensor_array + 3)){
                  call shimmerAnalogSetup.addECGInputs();
                  got_analog = 1;
              }
              break;
          case 4:
              if(*(sensor_array + 4)){
                  call shimmerAnalogSetup.addEMGInput();
                  got_analog = 1;
              }
              break;
          case 5:
              if(*(sensor_array + 5)){
                  call GsrInit.init();
                  if(gsr_range == 4) { // autorange
                      gsr_autorange = TRUE;
                      active_resistor = HW_RES_40K; // as set in GSR.Init()
                  }
                  else if(gsr_range == 0 || gsr_range == 1 || gsr_range == 2 || gsr_range == 3){
                      call Gsr.setRange(gsr_range);
                      active_resistor = gsr_range;
                      gsr_autorange = FALSE;
                  }
                  call shimmerAnalogSetup.addGSRInput();
                  got_analog = 1;
              }
              break;
          case 6:
              if(*(sensor_array + 6)){
                  call shimmerAnalogSetup.addAnExInput(7);
                  got_analog = 1;
              }
              break;
          case 7:
              if(*(sensor_array + 7)){
                  call shimmerAnalogSetup.addAnExInput(0);
                  got_analog = 1;
              }
              break;
          case 8:
              if(*(sensor_array + 8)){
                  call StrainGaugeInit.init();
                  call StrainGauge.powerOn();
                  call shimmerAnalogSetup.addStrainGaugeInputs();
                  set_5V_reg = TRUE;
                  got_analog = 1;
              }
              break;
          case 9:
              if(*(sensor_array + 9)){
                  call DigitalHeartInit.init();
                  heart_rate = TRUE;
                  beat_flag = FALSE;
              }
              break;
          default:
              break;
      }
    }
    if(got_analog){
      call shimmerAnalogSetup.finishADCSetup(sbuf0);
      NUM_ADC_CHANS = call shimmerAnalogSetup.getNumberOfChannels();
    }
    
    if(doin_the_mag){
      bytes_per_sample = NUM_ADC_CHANS * 2 + 6;
      mag_channels = 3;
      mag_slot = sbuf0 + NUM_ADC_CHANS; 
    }
    else{ 
        mag_channels = 0;
        bytes_per_sample = NUM_ADC_CHANS * 2;
    }
    if(heart_rate){
        bytes_per_sample = ( NUM_ADC_CHANS + mag_channels ) * 2 + 2;
    }
    block_samples = (2*NV_SD_WRITE_SIZE-6*time_sync) / (bytes_per_sample + 2*timestamps);
    fs_block_size = block_samples * (bytes_per_sample + 2*timestamps);

    // Turn off the warning if you previously had a bad config.
    error = FALSE;
    call Leds.led0Off();
  }
 
  char * fgets(char * buff, int len, FIL * fp) {
    uint16_t i = 0;
    char * p = buff;
    uint bytesRead;
    
    file_error = 0;
    while (i < len - 1) {			/* Read bytes until buffer gets filled */
      if(call FatFs.fread(fp, p, 1, &bytesRead)){
	file_error = 1;
	return NULL;
      }
      if(bytesRead != 1) 
	break;			/* Break when no data to read */

      if(*p == '\r') 
	continue;	/* Strip '\r' */

      i++;
      if (*p++ == '\n') 
	break;	/* Break when reached end of line */
    }
    *p = 0;
    return i ? buff : NULL;
  }


  task void configFailureWarning(){
      call FatFs.enableDock();
      error = TRUE;
      
      call Leds.led0On();
      warningLedOn = TRUE;
      call warningTimer.startOneShot(fifty_ms);
  }

  void calibrate(){
    if(stored_config[NV_SENSORS0] & SENSOR_ACCEL){
        read_calibration(S_ACCEL);
    }
    if(stored_config[NV_SENSORS0] & SENSOR_GYRO)
        read_calibration(S_GYRO);
    if(stored_config[NV_SENSORS0] & SENSOR_MAG)
        read_calibration(S_MAG);
    if(stored_config[NV_SENSORS0] & SENSOR_EMG)
        read_calibration(S_EMG);
    if(stored_config[NV_SENSORS0] & SENSOR_ECG)
        read_calibration(S_ECG);
  }


  void read_calibration(uint8_t sensor){
    char buffer[66], * equals, keyword[20];
    uint8_t i = 0, num_2byte_params = 6, num_1byte_params = 9, cal_file[32];
    uint16_t address;
    bool sensor_found = FALSE;
    float value;
    int16_t rounded_value;
    error_t res;

    DIR gdc;
    FIL gfc;
    
    if(sensor == S_GYRO){
        sprintf(keyword, "Gyro");
        address = NV_GYRO_CALIBRATION;
    }
    else if(sensor == S_MAG){
        sprintf(keyword, "Mag");
        address = NV_MAG_CALIBRATION;
    }
    else if(sensor == S_ACCEL){
        if(accel_range == RANGE_1_5G)
            sprintf(keyword, "Accel 1.5g");
        else if(accel_range == RANGE_2_0G)
            sprintf(keyword, "Accel 2.0g");
        else if(accel_range == RANGE_4_0G)
            sprintf(keyword, "Accel 4.0g");
        else //(accel_range == RANGE_6_0G)
            sprintf(keyword, "Accel 6.0g");
        address = NV_ACCEL_CALIBRATION;
    }
    else if(sensor == S_EMG){
        sprintf(keyword, "Emg");
        address = NV_EMG_CALIBRATION;
        num_2byte_params = 2;
        num_1byte_params = 0;
    }
    else if(sensor == S_ECG){
        sprintf(keyword, "Ecg");
        address = NV_ECG_CALIBRATION;
        num_2byte_params = 4;
        num_1byte_params = 0;
    }
    else { 
        return;
    }
    sprintf(cal_file, "/Calibration/calibParams.ini");
    if((res = call FatFs.opendir(&gdc, "/Calibration"))){
        if((res = call FatFs.opendir(&gdc, "/calibration")))
            default_calibration(sensor);
        else
            sprintf(cal_file, "/calibration/calibParams.ini");
    }
    else if(call FatFs.fopen(&gfc, cal_file, (FA_OPEN_EXISTING | FA_READ))) // no calibration file, use default
        default_calibration(sensor);
    else{ // look for sensor in calibration file. 
        while(fgets(buffer, 64, &gfc)){
            if(file_error){
                call FatFs.fclose(&gfc);
            }
            if(!strstr(buffer, keyword))
                continue;
            else{ // found the right sensor
                sensor_found = TRUE;
                for(i = 0; i < num_2byte_params; i++){ 
                    fgets(buffer, 64, &gfc);
                    if(!(equals = strchr(buffer, '='))){
                        sensor_found = FALSE; // there's an error, use the default
                        break;
                    }
                    equals ++;
                    value = atof(equals);
                    if((sensor == S_GYRO) & (i >= 3))
                        value *= 100;
                    rounded_value = (int16_t)(value>=0?value+0.5:value-0.5);
                    stored_config[address + 2*i] = (rounded_value & 0xFF00) >> 8;
                    stored_config[address + 2*i + 1] = (rounded_value & 0xFF);
                }
                for(i = 0; i < num_1byte_params; i++){ 
                    fgets(buffer, 64, &gfc);
                    if(!(equals = strchr(buffer, '='))){
                        sensor_found = FALSE; // there's an error, use the default
                        break;
                    }
                    equals ++;
                    value = atof(equals)*100;
                    rounded_value = (int16_t)(value>=0?value+0.5:value-0.5);
                    stored_config[address + 2*num_2byte_params + i] = (int8_t)(rounded_value);
                }
                call FatFs.fclose(&gfc);
                break;
            }
        }
    }
    if(!sensor_found)
        default_calibration(sensor);
  }

   void default_calibration(uint8_t sensor){
       int16_t bias, sensitivity;
       uint8_t bias_byte_one, bias_byte_two, sens_byte_one, sens_byte_two, number_axes = 1;
       int8_t align_xx, align_xy, align_xz, align_yx, align_yy, align_yz, align_zx, align_zy, align_zz, i = 0;
       uint16_t address;
       bool align = FALSE;

       if(sensor==S_ACCEL){
           number_axes = 3;
           bias = 2048;
           if(accel_range == RANGE_1_5G) 
               sensitivity = 101;
           else if(accel_range == RANGE_2_0G)
               sensitivity = 76;
           else if (accel_range == RANGE_4_0G)
               sensitivity = 38;
           else //(accel_range == RANGE_6_0G)
               sensitivity = 25;
           align = TRUE;
           align_xx = -100;
           align_xy = 0;
           align_xz = 0;
           align_yx = 0;
           align_yy = -100;
           align_yz = 0;
           align_zx = 0;
           align_zy = 0;
           align_zz = 100;
           address = NV_ACCEL_CALIBRATION;
       }
       else if(sensor==S_GYRO){
           number_axes = 3;
           bias = 1843;
           sensitivity = 273;
           align = TRUE;
           align_xx = 0;
           align_xy = -100;
           align_xz = 0;
           align_yx = -100;
           align_yy = 0;
           align_yz = 0;
           align_zx = 0;
           align_zy = 0;
           align_zz = -100;
           address = NV_GYRO_CALIBRATION;
       }
       else if(sensor==S_MAG){
           number_axes = 3;
           bias = 0;
           sensitivity = 580;
           align = TRUE;
           align_xx = 100;
           align_xy = 0;
           align_xz = 0;
           align_yx = 0;
           align_yy = 100;
           align_yz = 0;
           align_zx = 0;
           align_zy = 0;
           align_zz = -100;
           address = NV_MAG_CALIBRATION;
       }
       else if(sensor==S_ECG){
           number_axes = 2;
           bias = 2060;
           sensitivity = 175;
           address = NV_ECG_CALIBRATION;
       }
       else if(sensor==S_EMG){
           bias = 2060;
           sensitivity = 640;
           address = NV_EMG_CALIBRATION;
       }
       else{
           return;
       }

       bias_byte_one = (uint8_t)(bias >> 8);
       bias_byte_two = (uint8_t)(bias);
       sens_byte_one = (uint8_t)(sensitivity >> 8);
       sens_byte_two = (uint8_t)(sensitivity);
 
       // offset
       for(i = 0; i < number_axes; i++){
           stored_config[address + 2*i] = bias_byte_one;
           stored_config[address + 2*i + 1] = bias_byte_two;
       }
       // sensitivity
       for(i = 0; i < number_axes; i++){
           stored_config[address + 2*(number_axes + i)] = sens_byte_one;
           stored_config[address + 2*(number_axes + i) + 1] = sens_byte_two;
       }
       // alignment
       if(align){
           stored_config[address+ 12] = align_xx;
           stored_config[address+ 13] = align_xy;
           stored_config[address+ 14] = align_xz;
           stored_config[address+ 15] = align_yx;
           stored_config[address+ 16] = align_yy;
           stored_config[address+ 17] = align_yz;
           stored_config[address+ 18] = align_zx;
           stored_config[address+ 19] = align_zy;
           stored_config[address+ 20] = align_zz;
       }
   }

  uint8_t parseConfig() {
    char buffer[66], * equals;
    uint8_t string_length = 0;

    while(fgets(buffer, 64, &gfp)){
      if(file_error){
          call FatFs.fclose(&gfp);
          return 0;
      }
      if(!(equals = strchr(buffer, '=')))
          continue;
      equals++;     // this is the value
      if(strstr(buffer, "accel="))
 	    *sensor_array = atoi(equals);
      else if(strstr(buffer, "gyro="))
	    *(sensor_array + 1) = atoi(equals);
      else if(strstr(buffer, "mag="))
	    *(sensor_array + 2) = atoi(equals);
      else if(strstr(buffer, "ecg="))
	    *(sensor_array + 3) = atoi(equals);
      else if(strstr(buffer, "emg="))
	    *(sensor_array + 4) = atoi(equals);
      else if(strstr(buffer, "gsr="))
	    *(sensor_array + 5) = atoi(equals);
      else if(strstr(buffer, "anex7="))
        *(sensor_array + 6) = atoi(equals);
      else if(strstr(buffer, "anex0="))
        *(sensor_array + 7) = atoi(equals);
      else if(strstr(buffer, "str="))
	    *(sensor_array + 8) = atoi(equals);
      else if(strstr(buffer, "heart="))
        *(sensor_array + 9) = atoi(equals);
      else if(strstr(buffer, "sample_rate=")){
          fsample = atof(equals);
          sample_period = (uint16_t)ceil((float)1024.0 / (float)fsample);
          fsample = (float)1024.0 / (float)sample_period;
      }
      else if(strstr(buffer, "mg_internal_rate="))
        mag_rate = atoi(equals);
      else if(strstr(buffer, "mg_range="))
	    mag_range = atoi(equals);
      else if(strstr(buffer, "acc_range="))
        accel_range = atoi(equals);
      else if(strstr(buffer, "gs_range="))
        gsr_range = atoi(equals);
      else if(strstr(buffer, "user_button_enable="))
        user_button_enable = (atoi(equals)==0)?FALSE:TRUE;
      else if(strstr(buffer, "gyr_button_enable="))
        gyro_button_enable = (atoi(equals)==0)?FALSE:TRUE;
      else if(strstr(buffer, "PMUX_enable="))
        set_PMUX = (atoi(equals)==0)?FALSE:TRUE;
      else if(strstr(buffer, "5V_enable="))
        set_5V_reg = (atoi(equals)==0)?FALSE:TRUE;
      else if(strstr(buffer, "iammaster="))
        iAmMaster = (atoi(equals)==0)?FALSE:TRUE;
      else if(strstr(buffer, "sync="))
        time_sync = (atoi(equals)==0)?FALSE:TRUE;
      else if(strstr(buffer, "interval=")){
        broadcast_interval = atoi(equals)>255?255:atoi(equals);
        listen_window = broadcast_interval >> 1;
      }
      else if(strstr(buffer, "realtime="))
          realtime = (atoi(equals)==0)?FALSE:TRUE;
      else if(strstr(buffer, "singletouch"))
          singletouch = (atoi(equals)==0)?FALSE:TRUE;
      else if(strstr(buffer, "myid"))
          my_trial_id = atoi(equals);
      else if(strstr(buffer, "Nshimmer="))
          num_shimmers_in_trial = atoi(equals);
      else if(strstr(buffer, "shimmername=")){
          string_length = max_chars<strlen(equals)?max_chars:strlen(equals);
          snprintf(shimmername, string_length, "%s", equals);
      }
      else if(strstr(buffer, "experimentid=")){
          string_length = max_chars<strlen(equals)?max_chars:strlen(equals);
          snprintf(exp_id_name, string_length, "%s", equals);
      }
      else if(strstr(buffer, "configtime=")){
          config_time = atol(equals);
          stored_config[NV_CONFIG_TIME] = (config_time >> 24) && 0xFF;
          stored_config[NV_CONFIG_TIME + 1] = (config_time >> 16) && 0xFF;
          stored_config[NV_CONFIG_TIME + 2] = (config_time >> 8) & 0xFF;
          stored_config[NV_CONFIG_TIME + 3] = config_time && 0xFF;
      }
     }
    stored_config[NV_SAMPLING_RATE] = (uint8_t)((uint16_t)fsample >> 8 );
    stored_config[NV_SAMPLING_RATE + 1] = (uint8_t)fsample;

    if(*(sensor_array + 5)) // gsr
        gsr_selected = TRUE;

    // minimum sync broadcast interval is one second
    if(broadcast_interval < 1){
        broadcast_interval = 1;
        listen_window = 1;
    }

    if(user_button_enable && gyro_button_enable)
        gyro_button_enable = FALSE;
    call FatFs.fclose(&gfp);
    
    return 1;
  }

  void writeConfigHeader(uint32_t initial_timestamp) {
    uint bytesWritten;

    TOSH_MAKE_DOCK_N_OUTPUT();
    TOSH_SET_DOCK_N_PIN();

    calibrate();

    call FatFs.fwrite(&gfp, (uint8_t *)stored_config, NV_NUM_CONFIG_BYTES, &bytesWritten);
    call FatFs.fwrite(&gfp, (uint8_t *)&initial_timestamp, 4, &bytesWritten);
    call FatFs.fsync(&gfp);
    
    TOSH_MAKE_DOCK_N_INPUT();
    
    atomic if(!TOSH_READ_DOCK_N_PIN()){
      call sampleTimer.stop();

      /*
       * the next thing the apps sees should be 
       * an fatfs.available event.  unmount
       * will call diskiostdcontrol.init, which will
       * powercycle the card, put it in sd mode for the dock,
       * and re-initialize the interrupt
       */
      call FatFs.unmount();      
    }
   }

  void writeHostTime() {
    uint bytesWritten;

    TOSH_MAKE_DOCK_N_OUTPUT();
    TOSH_SET_DOCK_N_PIN();

    call FatFs.fwrite(&gfp, (uint8_t *)host_time, 8, &bytesWritten);
    call FatFs.fwrite(&gfp, (uint32_t *)&my_time_at_host_time, 4, &bytesWritten);
    call FatFs.fsync(&gfp);
    
    TOSH_MAKE_DOCK_N_INPUT();
    
    atomic if(!TOSH_READ_DOCK_N_PIN()){
      call sampleTimer.stop();

      /*
       * the next thing the apps sees should be 
       * an fatfs.available event.  unmount
       * will call diskiostdcontrol.init, which will
       * powercycle the card, put it in sd mode for the dock,
       * and re-initialize the interrupt
       */
      call FatFs.unmount();      
    }
   }

   void storeConfigBytes() {
       uint8_t sensor_byte_0 = 0, sensor_byte_1 = 0, config_byte_1 = 0, config_byte_0 = 0;
       
       if(*sensor_array) // accel
           sensor_byte_0 |= 0x80;
       if(*(sensor_array+1)) // gyro
           sensor_byte_0 |= 0x40;
       if(*(sensor_array+2)) // mag
           sensor_byte_0 |= 0x20;
       if(*(sensor_array+3)) // ecg
           sensor_byte_0 |= 0x10;
       if(*(sensor_array+4)) // emg
           sensor_byte_0 |= 0x08;
       if(*(sensor_array+5)) // gsr
           sensor_byte_0 |= 0x04;
       if(*(sensor_array+6)) // anex7
           sensor_byte_0 |= 0x02;
       if(*(sensor_array+7)) // anex0
           sensor_byte_0 |= 0x01;
       stored_config[NV_SENSORS0] = sensor_byte_0;

       if(*(sensor_array+8)) // strain
           sensor_byte_1 |= 0x80;
       if(*(sensor_array+9)) // heartrate
           sensor_byte_1 |= 0x40;
       stored_config[NV_SENSORS1] = sensor_byte_1;
       
       stored_config[NV_ACCEL_RANGE] = accel_range;
       stored_config[NV_GSR_RANGE] = gsr_range;
       stored_config[NV_MAG_RANGE] = mag_range;
       stored_config[NV_MAG_RATE] = mag_rate; 
 
       if(set_5V_reg)
           config_byte_0 |= 0x80;
       if(set_PMUX)
           config_byte_0 |= 0x40;
       if(user_button_enable)
           config_byte_0 |= 0x20;
       if(gyro_button_enable)
           config_byte_0 |= 0x10;
       if(timestamps)
           config_byte_0 |= 0x08;
       if(time_sync)
           config_byte_0 |= 0x04;
       if(iAmMaster)
           config_byte_0 |= 0x02;
       if(realtime)
           config_byte_0 |= 0x01;
       stored_config[NV_CONFIG_SETUP0] = config_byte_0;

       if(singletouch)
           config_byte_1 |= 0x80;
       stored_config[NV_CONFIG_SETUP1] = config_byte_1;
       
       stored_config[NV_CONFIG_SETUP5] = broadcast_interval;

       stored_config[NV_FW_VER_BYTE0] = (BOILERPLATE_VER_TYPE & 0XFF00) >> 8;
       stored_config[NV_FW_VER_BYTE0 + 1] = BOILERPLATE_VER_TYPE & 0xFF;
       stored_config[NV_FW_VER_BYTE0 + 2] = (BOILERPLATE_VER_MAJOR & 0XFF00) >> 8;
       stored_config[NV_FW_VER_BYTE0 + 3] = BOILERPLATE_VER_MAJOR & 0xFF;
       stored_config[NV_FW_VER_BYTE0 + 4] = BOILERPLATE_VER_MINOR & 0xFF;
       stored_config[NV_FW_VER_BYTE0 + 5] = BOILERPLATE_VER_REL & 0xFF;
       
       stored_config[NV_SHIMMER_VER_BYTE0] = (SHIMMER_VER & 0xFF00) >> 8;
       stored_config[NV_SHIMMER_VER_BYTE0 + 1] = SHIMMER_VER & 0xFF;
 
       stored_config[NV_MYTRIALID] = my_trial_id & 0xFF;
       stored_config[NV_NUMSHIMMERS] = num_shimmers_in_trial & 0xFF;
       
      config_stored = TRUE;
   }
   
   task void getSamplingConfig() {
    uint16_t i;
    char configfile[32];

    call Leds.led2On();

    sprintf(configfile, "/sdlog.cfg");

    call FatFs.disableDock();
   
    for(i = 0; i < 5000; i++)
      TOSH_uwait(1000);
    
    if(call FatFs.fopen(&gfp, configfile, (FA_OPEN_EXISTING | FA_READ))){
      post configFailureWarning();
      return;
    }
    else if(!parseConfig()){  // if this returns 0 the file has a glitch
      post configFailureWarning();
      return;
    }
    else
      setSamplingConfig();

    TOSH_MAKE_SER0_RTS_OUTPUT();
    TOSH_SEL_SER0_RTS_IOFUNC();
    if(set_5V_reg)
        TOSH_SET_SER0_RTS_PIN();
    else
        TOSH_CLR_SER0_RTS_PIN();
#ifdef PWRMUX_UTIL
    if(set_PMUX)
        TOSH_SET_PWRMUX_SEL_PIN();
    else
        TOSH_CLR_PWRMUX_SEL_PIN();
#endif

    call FatFs.enableDock();
        
    if(user_button_enable) {
        call UserButton.enable();
        call GyroButton.disable();
    }
    else if(gyro_button_enable){
        call GyroButton.enable();
        if(!heart_rate) // do not disable UserButton if heart rate is enabled
            call UserButton.disable();
    }


    if(!user_button_enable && !gyro_button_enable && !singletouch){ // Nothing to wait for...
        if(!doin_the_mag) // the mag will start logging if it's being used.
            startLogging();
    }
    else{ // one of the buttons or singletouch is enabled...wait for an action to start logging
        if(singletouch && !iAmMaster){ // Slave: turn on radio and wait for start message
            if(!radio_enabled)
                call AMRadioControl.start();
        }
    }

   }
    
  event void Time.tick() {}

  void do_stores(uint32_t initial_timestamp){
    uint8_t r;

    // disable dock interrupt
    TOSH_MAKE_DOCK_N_OUTPUT();
    TOSH_SET_DOCK_N_PIN();

    if(timestamp_received){
        sprintf(filename, "%s/HostTime", dirname);
        r = call FatFs.fopen(&gfp, filename, (FA_OPEN_ALWAYS | FA_WRITE));
        writeHostTime();
        call FatFs.fclose(&gfp);
        timestamp_received = FALSE;
    }

    sprintf(filename, "%s/%03d", dirname, sequence_number++);

    if(!config_stored)
        storeConfigBytes();
    
    r = call FatFs.fopen(&gfp, filename, (FA_OPEN_ALWAYS | FA_WRITE | FA_READ));

    writeConfigHeader(initial_timestamp);

    // enable dock interrupt
    TOSH_MAKE_DOCK_N_INPUT();

    if(r){
        error = TRUE;
        call Leds.led0On();
    }
  }
  
  task void clockin_result() {
     if(mag_requested || (NUM_ADC_CHANS && !mag_ready) || write_ongoing){ // For various reasons we can miss a mag sample
         // Even though we missed the mag sample, we still need to update everything
         // so that the next sample will be stored in the right place. The previous 
         // mag value will be reused.
      
         uint32_t current_time = 0;

         if(!NUM_ADC_CHANS && timestamps){ //we have no analog channels so must update timestamps here
             current_time = call LocalTime.get();
             if(current_dig_buffer == 0)
                 *(timestamps0 + digital_samples) = (uint16_t)(current_time);
             else
                 *(timestamps1 + digital_samples) = (uint16_t)(current_time);
        
             if(digital_samples==0){
                 if(current_dig_buffer==0)
                     initial_timestamp0 = current_time;
                 else
                     initial_timestamp1 = current_time;
             }
             if(!first_sample){
                 first_sample = TRUE;
             }
         }

         collect_results();
     }
     else if(mag_ready){
             busControlToMag(); // just in case 
             
             atomic{mag_requested = TRUE;} // Flag that there is a pending request for a mag sample
             call Magnetometer.readData();  
     }
  }

 async event void sampleTimer.fired() {
    if(NUM_ADC_CHANS) // This will cause the DMA to transfer a sample and then we will request a mag sample in transferDone().
      call shimmerAnalogSetup.triggerConversion();
    else // if there are no analog channels, we must request the mag sample directly.
      post clockin_result(); 

    call sampleTimer.start(sample_period);
  }

  void collect_results() {
      uint16_t i = 0, channels_per_sample = bytes_per_sample >> 1, *temp_mag_slot;
      uint16_t mag_x = 0, mag_y = 0, mag_z = 0;
      digital_samples ++;
      
      mag_x = (uint8_t)(*(readBuf));
      mag_x <<= 8;
      mag_x += (uint8_t)(*(readBuf + 1));

      mag_y = (uint8_t)(*(readBuf + 2));
      mag_y <<= 8;
      mag_y += (uint8_t)(*(readBuf + 3));

      mag_z = (uint8_t)(*(readBuf + 4));
      mag_z <<= 8;
      mag_z += (uint8_t)(*(readBuf + 5));

      if(digital_samples == dma_samples || (digital_samples == block_samples && dma_samples == 0) || !NUM_ADC_CHANS){
          // copy the mag sample just read in readBuf into the appropriate sbuf location for writing to SD later.
          // Remember that mag_slot points to the first mag channel of the current buffer (sbuf0 of sbuf1).
          *(mag_slot + (digital_samples - 1)*(channels_per_sample)) = mag_x; 
          *(mag_slot + (digital_samples - 1)*(channels_per_sample) + 1) = mag_y;
          *(mag_slot + (digital_samples - 1)*(channels_per_sample) + 2) = mag_z; 
      }
      else{ // we're out of sync. Put the new sample at the "now" of the analog clock and fill in replicas of the last value for the missed samples.
          if(digital_samples < dma_samples){ //buffer hasn't wrapped around...easy case
              for(i = digital_samples; i < dma_samples; i ++){
                  *(mag_slot + (i - 1)*(channels_per_sample)) = last_mag_x;
                  *(mag_slot + (i - 1)*(channels_per_sample) + 1) = last_mag_y;
                  *(mag_slot + (i - 1)*(channels_per_sample) + 2) = last_mag_z;
              }
              *(mag_slot + (dma_samples - 1)*(channels_per_sample)) = mag_x;
              *(mag_slot + (dma_samples - 1)*(channels_per_sample) + 1) = mag_y;
              *(mag_slot + (dma_samples - 1)*(channels_per_sample) + 2) = mag_z;
          }
          else{ // analog buffer has wrapped around...needs more care.
              for(i = digital_samples; i <= block_samples; i ++){
                  *(mag_slot + (i - 1)*(channels_per_sample)) = last_mag_x;
                  *(mag_slot + (i - 1)*(channels_per_sample) + 1) = last_mag_y;
                  *(mag_slot + (i - 1)*(channels_per_sample) + 2) = last_mag_z;
              }

              if(current_dig_buffer == 0)
                  temp_mag_slot = sbuf1 + NUM_ADC_CHANS;
              else
                  temp_mag_slot = sbuf0 + NUM_ADC_CHANS;

              for(i = 1; i < dma_samples; i++){
                  *(temp_mag_slot + (i - 1)*(channels_per_sample)) = last_mag_x;
                  *(temp_mag_slot + (i - 1)*(channels_per_sample) + 1) = last_mag_y;
                  *(temp_mag_slot + (i - 1)*(channels_per_sample) + 2) = last_mag_z;
              }
              *(temp_mag_slot + (dma_samples - 1)*(channels_per_sample)) = mag_x;
              *(temp_mag_slot + (dma_samples - 1)*(channels_per_sample) + 1) = mag_y;
              *(temp_mag_slot + (dma_samples - 1)*(channels_per_sample) + 2) = mag_z;
          }
          digital_samples = dma_samples;
      }
          
      last_mag_x = mag_x;
      last_mag_y = mag_y;
      last_mag_z = mag_z;

      if(digital_samples == block_samples)
          digital_samples = 0;

    if(!NUM_ADC_CHANS){ //we have no analog channels so must update buffers here
      if(digital_samples == 0){
          // swap over the buffer for new block
          if(current_dig_buffer == 0){ 
	        mag_slot = sbuf1; // (NUM_ADC_CHANS == 0) 
	        current_dig_buffer = 1;
            current_buffer = 1; // for writing to the SD card
            post store_contents0();
          }
          else { 
	        mag_slot = sbuf0; // (NUM_ADC_CHANS == 0)
	        current_dig_buffer = 0;           
            current_buffer = 0; // for writing to the SD card
            post store_contents1();
          }
      }
    }
    else{ // everything is already taken care of unless we have a full block; in that case we must swap the buffers and call store_contents
        if(digital_samples == 0){
            if(current_dig_buffer == 0){
                  mag_slot = sbuf1 + NUM_ADC_CHANS; 
                  current_dig_buffer = 1;
                  post store_contents0();
            }
            else{
                mag_slot = sbuf0 + NUM_ADC_CHANS; 
                current_dig_buffer = 0;
                post store_contents1();
            }
          }
   }
  }

  async event void Magnetometer.readDone(uint8_t * data, error_t success){
      uint32_t current_time;

      if(!NUM_ADC_CHANS && timestamps){ //we have no analog channels so must update timestamps here
         current_time = call LocalTime.get();
        if(current_dig_buffer == 0)
            *(timestamps0 + digital_samples) = (uint16_t)(current_time);
        else
            *(timestamps1 + digital_samples) = (uint16_t)(current_time);
        
        if(digital_samples==0){
            if(current_dig_buffer==0)
                initial_timestamp0 = current_time;
            else
                initial_timestamp1 = current_time;
        }
        if(!first_sample){
            first_sample = TRUE;
        }
     }
     memcpy(readBuf, data, 6);
     atomic{mag_requested = FALSE;} // Flag that there is no pending request for a mag sample
     
     collect_results();
  }


  void collect_heart_rate(){
      uint16_t b_t = 0;
      
      if(beat_flag){
          b_t = beat_time >> 5;
      }
      else 
          b_t = 0;
      if(current_buffer == 0)
          *((uint16_t *)sbuf0 + dma_samples*(bytes_per_sample >> 1) - 1) = b_t;
      else
          *((uint16_t *)sbuf1 + dma_samples*(bytes_per_sample >> 1) - 1) = b_t;
      beat_flag = FALSE;
  }


  void autorange_gsr() {
      uint8_t current_active_resistor = active_resistor, dummy_active_resistor = active_resistor;
      uint16_t ADC_val; 
      bool transient = FALSE;

      // GSR channel will always be last ADC channel 
      if(current_buffer == 0) {
          ADC_val = *((uint16_t *)sbuf0 + (dma_samples - 1)*(bytes_per_sample >> 1) + NUM_ADC_CHANS - 1);
          active_resistor = call Gsr.controlRange(ADC_val, active_resistor);
          
          dummy_active_resistor = current_active_resistor; // Old resistor value must be written to file if during a transition: determined by Gsr.smoothTransition
          transient = call Gsr.smoothTransition(&dummy_active_resistor, sample_period);  
          if(transient){
              ADC_val = last_ADC_val;
          }
          
          *((uint16_t *)sbuf0 + (dma_samples - 1)*(bytes_per_sample >> 1) + NUM_ADC_CHANS - 1) = (ADC_val & 0x3FFF) | (dummy_active_resistor << 14);
          last_ADC_val = ADC_val;
      }
      else {
          ADC_val = *((uint16_t *)sbuf1 + (dma_samples - 1)*(bytes_per_sample >> 1) + NUM_ADC_CHANS - 1);
          active_resistor = call Gsr.controlRange(ADC_val, active_resistor);
          
          dummy_active_resistor = current_active_resistor; // Old resistor value must be written to file if during a transition: handled by Gsr.smoothTransition
          transient = call Gsr.smoothTransition(&dummy_active_resistor, sample_period);
          if(transient){
              ADC_val = last_ADC_val;
          }
          
          *((uint16_t *)sbuf1 + (dma_samples - 1)*(bytes_per_sample >> 1) + NUM_ADC_CHANS - 1) = (ADC_val & 0x3FFF) | (dummy_active_resistor << 14);
          last_ADC_val = ADC_val;
      }
  }
           

  async event void DMA0.transferDone(error_t success) {
    uint32_t current_time;
      if(timestamps){
          current_time = call LocalTime.get();
          if(current_buffer == 0)
              *(timestamps0 + dma_samples) = (uint16_t)(current_time);
          else
              *(timestamps1 + dma_samples) = (uint16_t)(current_time);
          if(dma_samples==0){
              if(current_buffer==0)
                  initial_timestamp0 = current_time;
              else
                  initial_timestamp1 = current_time;
          }
          if(!first_sample){
            first_sample = TRUE;
          }
      }

    dma_samples++; // dma_samples now has a value between 1 and block_samples
    atomic DMA0DA += bytes_per_sample;

    if(doin_the_mag) // Ask the mag for a sample if it has been selected.
        post clockin_result();

     if(gsr_selected && gsr_autorange) // update gsr resistor if autorange has been selected
        autorange_gsr();

     if(heart_rate)
         collect_heart_rate();

   if(dma_samples == block_samples){ // we have filled the buffer so swap buffers to start again and call store_contents
      dma_samples = 0;
      if(current_buffer == 0){
          call DMA0.repeatTransfer((void *)ADC12MEM0_, (void *)sbuf1, NUM_ADC_CHANS);
          current_buffer = 1;
          if(!doin_the_mag)
              post store_contents0();
      }
      else { 
          call DMA0.repeatTransfer((void *)ADC12MEM0_, (void *)sbuf0, NUM_ADC_CHANS);
          current_buffer = 0;
          if(!doin_the_mag)
              post store_contents1();
      }
    }
   }

  void rollTheBall() {
      // This is where logging really starts
    uint16_t i;

    for(i = 0; i < 500; i++)
      TOSH_uwait(1000);
    
    if(doin_the_mag)
      busControlToMag();

    if(gyro_selected)
        call GyroStdControl.start();

    // Turn 5V reg and PWRMUX back on if they were selected
    if(set_5V_reg)
        TOSH_SET_SER0_RTS_PIN();
    else
        TOSH_CLR_SER0_RTS_PIN();
#ifdef PWRMUX_UTIL
    if(set_PMUX)
        TOSH_SET_PWRMUX_SEL_PIN();
    else
        TOSH_CLR_PWRMUX_SEL_PIN();
#endif

    call sampleTimer.start(sample_period);
    atomic{logging = TRUE;}
    
    if(time_sync){
        if(iAmMaster){ 
            // start the broadcast timer 
            count_seconds = broadcast_interval - 10 > 0?broadcast_interval - 10:0; // wait 10 seconds to send first timestamp
            call broadcastTimer.start(one_second);
        }
        else{ // slave
            if(!radio_enabled) // Turn on the AM channel and leave it on until the first timestamp is received
                call AMRadioControl.start();
        }
    }
  }

  task void prepareNewSession() {
      // stop sampling (in case you were previously sampling)
      call sampleTimer.stop();
      atomic{logging = FALSE;}

      first_write_done = FALSE; // This will make sure we open a new file when logging starts
      directory_set = FALSE; // This will make sure we create a new folder when logging starts

      atomic mag_requested = FALSE;
      write_ongoing = FALSE;
      mag_ready = FALSE;

      call broadcastTimer.stop(); // we don't want to send/receive timestamps in between sessions
   
      if(!iAmMaster && singletouch)
          call listenTimer.stop(); // otherwise it will turn off the radio

      call GyroStdControl.stop(); // to prevent draining the battery
      
      // Turn off 5V regulator and clear PMUX pin while we're in between session
      TOSH_CLR_SER0_RTS_PIN();
#ifdef PWRMUX_UTIL
      TOSH_CLR_PWRMUX_SEL_PIN();
#endif

      // This will allow the host time to be received if requested (by user button on dock)
      if(docked){
//          setupUART();
//          call UARTControl.enableUart();
//          call UARTControl.enableIntr();
//          waiting_for_hosttime = TRUE;
      }
      else if(singletouch && !iAmMaster){ // Slave: turn on radio and wait for start message
            if(!radio_enabled)
                call AMRadioControl.start();
      }
  }


  void startLogging(){
      if(!error){
          if(!directory_set){ // Have we set up the folders?
              if(doin_the_mag)
                  busControlToSD();
              initialize_directories();
              if(bad_opendir || bad_mkdir){
                  error = TRUE;
                  call Leds.led0On();
                  return;
              }
          }
          // Folders are ready...get going with logging
          if(!error)
              rollTheBall();
      }
  }

  async event void FatFs.mediaAvailable() {    
      uint16_t i, j, k;
      TOSH_MAKE_DOCK_N_INPUT();
      atomic if(TOSH_READ_DOCK_N_PIN()){ // we've been pulled off the dock
          docked = FALSE;

          // we can't receive a hosttime without sitting on the dock
          waiting_for_hosttime = FALSE;
          call UARTControl.disableIntr();

          i = 0;
          while(TOSH_READ_DOCK_N_PIN() && i < 5){ // wait five seconds or until docked
              i++;
              call Leds.led2On();
              for(j = 0; j < 400; j ++) // 500 ms (approx)
                  __delay_cycles(1000);
              call Leds.led2Off();
              for(k = 0; k < 19; k++){
                  for(j = 0; j < 400; j ++) // 500 ms (approx)
                      __delay_cycles(1000);
              }
          }
          atomic if(TOSH_READ_DOCK_N_PIN()){ // make sure that we haven't been put back on the dock
              CS_LOW(); // card select (SD)
              post getSamplingConfig();
          }
          else{ // we're back on the dock but interrupt won't have been received so prepare the next session here.
              docked = TRUE;
              
              powerCycle();
              
              post startup();

              post prepareNewSession();

              call FatFs.enableDock();
          }
      }
  }
  
  async event void FatFs.mediaUnavailable() {
      TOSH_MAKE_DOCK_N_INPUT();
      atomic if(!TOSH_READ_DOCK_N_PIN()){ // we're in the dock
          docked = TRUE;
          call Leds.led2Off();

          powerCycle();

          call AMRadioControl.stop();

          post startup();

          post prepareNewSession();
      }
  }

  async event void Magnetometer.writeDone(error_t success){
      if(mag_change_rate){
          mag_change_rate = FALSE;
          call Magnetometer.setOutputRate(mag_rate);
      }
      else if(mag_run_continuous){
          mag_run_continuous = FALSE;
          call Magnetometer.runContinuousConversion();
      }
      else{
          mag_ready = TRUE;
          if(!logging && !docked)
              startLogging();
      }
  }

  async event void GyroBoard.buttonPressed() {}

  async event void UARTData.rxDone(uint8_t data){
/*      if(waiting_for_hosttime){
          *data_to_send = 1;
          total_bytes = 1;
          bytes_sent = 0; 
          sendSerial();
          waiting_for_hosttime = FALSE;
          timestamp_requested = TRUE;
          timestamp_received = FALSE;
          time_bytes_received = 0;
      }
      else if(timestamp_requested && !timestamp_received){
          if(time_bytes_received == 0)
              my_time_at_host_time = (call LocalTime.get()) & 0xFFFFFFFF;
          if(time_bytes_received < max_time_bytes) {
              host_time[7-time_bytes_received] = data & 0xFF;
          }
          time_bytes_received++;
          if(time_bytes_received == max_time_bytes){
              timestamp_received = TRUE;
              timestamp_requested = FALSE;
              *((uint32_t *)data_to_send) = my_time_at_host_time;
              if(!first_sample)
                  *((uint16_t *)data_to_send + 2) = (uint16_t)(1000);
              else
                  *((uint16_t *)data_to_send + 2) = (uint16_t)(dir_counter);
              total_bytes = 6;
              bytes_sent = 0;
              sendSerial();
          }
      }*/
   }

/*  void sendSerial(){
      while(!call UARTControl.isTxEmpty());
      call UARTControl.tx((*(data_to_send + bytes_sent) & 0xFF));
  }
*/
  async event void UARTData.txDone() {
/*      bytes_sent++;
      if(bytes_sent < total_bytes)
          sendSerial(); */
  }


  event void UserButton.notify(button_state_t val) {
      /* Start/stop logging if button start/stop is 
       * enabled and we are not sitting in the dock.
       * Else request host time from PC.
       */

      if(docked){ // we're sitting on the dock so ignore the button press
      }
      else if(user_button_enable){ // we're not on the dock and the button is enabled ...
          // ...so the button press means we must start or stop logging

          if(singletouch && iAmMaster){ // need to tell all shimmers to start logging
              pending_start_logging = TRUE;
              if(logging)
                  stop_message_count = 0;
              logging_message = !logging;
              if(!radio_enabled)
                  call AMRadioControl.start();
          }
          
          // Only need button to tell us to start...if not already logging, we want to start logging
          if(!logging){
              startLogging();
          }
          else if(logging){ // We are currently logging so we want to stop
              post prepareNewSession(); // This will stop the logging
          }
      }
  }

  event void GyroButton.notify(button_state_t val) {
      /* We only want to act on this if button start/stop is 
       * enabled and we are not sitting in the dock.
       */
     if(gyro_button_enable && !docked){ // we're not on the dock and the button is enabled ...
          // ...so the button press means we must start or stop logging

          if(singletouch && iAmMaster){ // need to tell all shimmers to start logging
              pending_start_logging = TRUE;
              if(logging)
                  stop_message_count = 0;
              logging_message = !logging;
              if(!radio_enabled)
                  call AMRadioControl.start();
          }
          
          // Only need button to tell us to start...if not already logging, we want to start logging
          if(!logging){
              startLogging();
          }
          else if(logging){ // We are currently logging so we want to stop
              post prepareNewSession(); // This will stop the logging
          }
      }
  }

  async event void DigitalHeartRate.beat(uint32_t time) {
      beat_flag = TRUE;
      beat_time = time;
  }

  event void AMRadioControl.startDone(error_t err){
      if(err == SUCCESS){
          call Leds.led1On();
          radio_enabled = TRUE;
          radio_locked = FALSE;
          if(!iAmMaster&&first_timestamp_recvd){
              listening_seconds = 0;
              call listenTimer.startOneShot(one_second);
          }
          if(iAmMaster){
              if(pending_start_logging)
                  sendLoggingMessage(logging_message);
              else if(time_sync)
                  sendTimestamp();
          }
      }
     else{
          call AMRadioControl.start();
      }
  }

  event void AMRadioControl.stopDone(error_t err){
      call Leds.led1Off();
      radio_enabled = FALSE;
      radio_locked = FALSE;
  }

  event void AMRadioSend.sendDone(message_t *msg, error_t err){
      radio_locked = FALSE;
      
      if(radio_enabled){
          call AMRadioControl.stop();
      }
  }

  event message_t * Receive.receive(message_t *msg, void *payload, uint8_t len) {
      if(!iAmMaster){ // the master should never receive a message, only the slaves
          if(singletouch && len == sizeof(start_logging_msg_t)){ // this is a start/stop logging message
              start_logging_msg_t *r_packet = (start_logging_msg_t *)payload;
              if(r_packet->trial_id == config_time){ // the message belongs to this trial
                  if(r_packet->start_stop){ // the message says start logging
                      if(!logging){ // check that we're not already logging
                          startLogging();
                      }
                  }
                  else{ // the message says stop logging
                      if(logging){ // check that we are actually logging
                          post prepareNewSession();
                      }
                  }
              }
          }
          else{ // this should be a timestamp message
              if(logging){ //a slave doesn't know how to cope with a timestamp if it's not logging yet (i.e. it's waiting for a start logging message)
                  if(call PacketTimeStamp.isValid(msg) && len == sizeof(time_broadcast_msg_t)){ // the message is a time broadcast,and has a valid timestamp
                      time_broadcast_msg_t *r_packet = (time_broadcast_msg_t *)payload;
                      if(!(r_packet->trial_id == config_time)){ // the message didn't come from the master for this trial
                          return msg;
                      }
                      else{
                          my_timestamp = call PacketTimeStamp.timestamp(msg);

                          count_seconds = 0;
                          call broadcastTimer.start(900); // this will allow the slave to wake up (approximately) 100 ms before the master
                          master_timestamp = r_packet->timestamp;
                    
                          if(my_timestamp >= master_timestamp){
                              offset_sign = 0;
                              time_offset = (uint32_t)fmod(4294967296+(uint64_t)my_timestamp - (uint64_t)master_timestamp,4294967296);
                          }
                          else{
                              offset_sign = 1;
                              time_offset = (uint32_t)fmod(4294967296+(uint64_t)master_timestamp - (uint64_t)my_timestamp,4294967296);
                          }
                          update_received = TRUE;
                          if(!first_timestamp_recvd){
                              first_timestamp_recvd = TRUE;
                          }
                      }
                      // we're done, turn off the radio now.
                      if(radio_enabled)
                          call AMRadioControl.stop();
                  }
                  else{
                      update_received = FALSE;
                      offset_sign = 0;
                      time_offset = 0xFFFFFFFF;
                  }
              }
          }
      }
      return msg;
  }

  void sendTimestamp(){
      time_broadcast_msg_t *rcm;
      error_t send_success = FALSE;

      count_seconds = 0;
      call broadcastTimer.start(one_second);
      
      if(radio_locked)
          return;
      else{
          rcm = (time_broadcast_msg_t *)call Packet.getPayload(&radio_packet, sizeof(time_broadcast_msg_t));
          if(rcm == NULL){
              return;
          }
      }
      atomic{
          rcm->timestamp = (call LocalTime.get()) & 0xFFFFFFFF;
          rcm->trial_id = config_time;
          send_success = call AMRadioSend.send(AM_BROADCAST_ADDR, &radio_packet, sizeof(time_broadcast_msg_t));
      }

      if(send_success == SUCCESS){
          radio_locked = TRUE;
          // Set the listenTimer to timeout just in case we don't get a sendDone signal and we need to manually stop the radio.
          listening_seconds = 0;
          call listenTimer.startOneShot(one_second);
      }
  }

  void sendLoggingMessage(bool start_stop){
      start_logging_msg_t *rcm;
      error_t send_success = FALSE;

      if(!start_stop){ // message is stop logging...need to repeateadly send during the broadcast interval.
          stop_message_count ++;
          if(stop_message_count == broadcast_interval)
              pending_start_logging = FALSE;
      }
      else // message is start logging...only needs to be sent once
          pending_start_logging = FALSE;

      if(radio_locked)
          return;
      else{
          rcm = (start_logging_msg_t *)call Packet.getPayload(&radio_packet, sizeof(start_logging_msg_t));
          if(rcm == NULL){
              return;
          }
      }
      atomic{
          rcm->start_stop = start_stop;
          rcm->trial_id = config_time;
          send_success = call AMRadioSend.send(AM_BROADCAST_ADDR, &radio_packet, sizeof(start_logging_msg_t));
      }
      if(send_success == SUCCESS){
          radio_locked = TRUE;
          // Set the listenTimer to timeout just in case we don't get a sendDone signal and we need to manually stop the radio.
          listening_seconds = 0;
          call listenTimer.startOneShot(one_second);
      }
  }

  async event void broadcastTimer.fired(){
      count_seconds ++;
      if(count_seconds >= broadcast_interval){
          count_seconds = 0;
          call broadcastTimer.start(one_second);
           if(!radio_enabled)
              call AMRadioControl.start();
      }
      else{
          call broadcastTimer.start(one_second);
      }
  }

  // The window is over; close the comms channel. BUt if an update wasn't received, make the window bigger for next time.
  event void listenTimer.fired(){
      if(iAmMaster && pending_start_logging) // we need to repeat the stop logging message
          call AMRadioControl.start();
      else{
          listening_seconds ++;
          if(listening_seconds >= listen_window){
              listening_seconds = 0;
              if(iAmMaster){
                  if(radio_enabled){ // We didn't get a sendDone signal - timeout and turn off the radio.
                      call AMRadioControl.stop();
                  }
              }
              else{ //Slave
                  if(!update_received){
                      offset_sign = 0;
                      time_offset = 0xFFFFFFFF;
                      count_seconds = listen_window;
                      call broadcastTimer.start(900);
                      listen_window = broadcast_interval < (listen_window+1)?broadcast_interval:(listen_window+1);
                  }
                  else{
                      listen_window = listen_window-1 < min_listen_window?min_listen_window:listen_window-1;
                  }
                  update_received = FALSE;
                  if(radio_enabled)
                      call AMRadioControl.stop();
              }
          }
          else{
              call listenTimer.startOneShot(one_second);
          }
      }
  }
  
  event void ledTimer.fired(){
      if(!error){
          if(logging){
              call Leds.led2Toggle();
              call ledTimer.startOneShot(one_second);
          }
          else{
              if(ledOff){
                  call Leds.led2On();
                  call ledTimer.startOneShot(fifty_ms);
                  ledOff = FALSE;
              }
              else{
                  call Leds.led2Off();
                  call ledTimer.startOneShot(2*one_second);
                  ledOff = TRUE;
              }
          }
      }
      else
          call Leds.led2Off();
  }

  event void warningTimer.fired(){
      if(warningLedOn){
          call Leds.led0Off();
          warningLedOn = FALSE;
          call warningTimer.startOneShot(2*one_second);
      }
      else{
          call Leds.led0On();
          warningLedOn = TRUE;
          call warningTimer.startOneShot(fifty_ms);
      }
  }
 

}






