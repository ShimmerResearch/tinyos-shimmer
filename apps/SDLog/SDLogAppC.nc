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
 * @author Niamh O'Mahony
 * @date   September, 2012
 */

#include "FatFs.h"
#include "TimeBroadcast.h"

configuration SDLogAppC {
}
implementation {
  components MainC, SDLogC;
  SDLogC -> MainC.Boot;

  // this auto-wires in the 8mhz xt2, running smclk at 4mhz
  components FastClockC;
  MainC.SoftwareInit -> FastClockC;

  components LedsC;
  SDLogC.Leds -> LedsC;
  
  components new AlarmMilli16C() as sampleTimer;
  SDLogC.sampleTimerInit -> sampleTimer;
  SDLogC.sampleTimer   -> sampleTimer;

  components new AlarmMilli16C() as broadcastTimer;
  SDLogC.broadcastTimerInit -> broadcastTimer;
  SDLogC.broadcastTimer     -> broadcastTimer;
  
  components new TimerMilliC() as listenTimer;
  SDLogC.listenTimer   -> listenTimer;

  components new TimerMilliC() as ledTimer;
  SDLogC.ledTimer   -> ledTimer;

 components new TimerMilliC() as warningTimer;
  SDLogC.warningTimer   -> warningTimer;

  components shimmerAnalogSetupC, Msp430DmaC;
  MainC.SoftwareInit -> shimmerAnalogSetupC.Init;
  SDLogC.shimmerAnalogSetup -> shimmerAnalogSetupC;
  SDLogC.DMA0 -> Msp430DmaC.Channel0;

  components AccelC;
  SDLogC.AccelInit    -> AccelC;
  SDLogC.Accel        -> AccelC;

  components GyroBoardC;
  SDLogC.GyroInit       -> GyroBoardC;
  SDLogC.GyroStdControl -> GyroBoardC;
  SDLogC.GyroBoard      -> GyroBoardC;

  components MagnetometerC;
  SDLogC.MagInit -> MagnetometerC;
  SDLogC.Magnetometer     -> MagnetometerC;

  components GsrC;
  SDLogC.GsrInit -> GsrC.Init;
  SDLogC.Gsr     -> GsrC;

  components StrainGaugeC;
  SDLogC.StrainGaugeInit -> StrainGaugeC.Init;
  SDLogC.StrainGauge     -> StrainGaugeC.StrainGauge;

  components DigitalHeartRateC;
  SDLogC.DigitalHeartInit -> DigitalHeartRateC.Init;
  SDLogC.DigitalHeartRate -> DigitalHeartRateC;

  components FatFsP, diskIOC;
  FatFsP.diskIO             -> diskIOC;
  FatFsP.diskIOStdControl   -> diskIOC;
  SDLogC.FatFs     -> FatFsP;

  components Ds2411C;
  SDLogC.IDChip     -> Ds2411C.ReadId48;

  components TimeC;
  MainC.SoftwareInit   -> TimeC;
  SDLogC.Time -> TimeC;

  components Counter32khz32C as Counter;
  components new CounterToLocalTimeC(T32khz);
  CounterToLocalTimeC.Counter -> Counter;
  SDLogC.LocalTime     -> CounterToLocalTimeC;

  components UserButtonC;
  SDLogC.UserButton -> UserButtonC;

  components GyroButtonC;
  SDLogC.GyroButton -> GyroButtonC;

  components HplMsp430Usart0C;
  SDLogC.UARTControl -> HplMsp430Usart0C.HplMsp430Usart;
  SDLogC.UARTData -> HplMsp430Usart0C.HplMsp430UsartInterrupts;

  components ActiveMessageC;
  components TimeSyncMessageC;
  SDLogC.AMRadioControl -> ActiveMessageC;
  SDLogC.PacketTimeStamp -> ActiveMessageC;
  SDLogC.TimeSyncPacket -> TimeSyncMessageC;

  components new AMReceiverC(AM_TIME_BROADCAST_MSG);
  SDLogC.Receive -> AMReceiverC;

  components new AMSenderC(AM_TIME_BROADCAST_MSG);
  SDLogC.Packet -> AMSenderC;
  SDLogC.AMRadioSend -> AMSenderC;
}
