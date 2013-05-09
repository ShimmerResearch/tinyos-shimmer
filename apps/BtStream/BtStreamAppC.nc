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

configuration BtStreamAppC {
}
implementation {
   components MainC, BtStreamC as App;
   App -> MainC.Boot; 

   components LedsC;
   App.Leds -> LedsC;

   components new AlarmMilli16C() as SampleTimer;
   App.SampleTimerInit  -> SampleTimer;
   App.SampleTimer      -> SampleTimer;
   components new TimerMilliC() as SetupTimer;
   App.SetupTimer       -> SetupTimer;
   components new AlarmMilli16C() as BlinkTimer;
   App.BlinkTimerInit   -> BlinkTimer;
   App.BlinkTimer       -> BlinkTimer;
  
   components Counter32khz32C as Counter;
   components new CounterToLocalTimeC(T32khz);
   CounterToLocalTimeC.Counter -> Counter;
   App.LocalTime -> CounterToLocalTimeC;
  
   components RovingNetworksC;
   App.BluetoothInit -> RovingNetworksC.Init;
   App.BTStdControl  -> RovingNetworksC.StdControl;
   App.Bluetooth     -> RovingNetworksC;

   components AccelC;
   App.AccelInit -> AccelC;
   App.Accel     -> AccelC;

   components shimmerAnalogSetupC, Msp430DmaC;
   MainC.SoftwareInit      -> shimmerAnalogSetupC.Init;
   App.shimmerAnalogSetup  -> shimmerAnalogSetupC;
   App.DMA0                -> Msp430DmaC.Channel0;

   components GyroBoardC;
   App.GyroInit       -> GyroBoardC.Init;
   App.GyroStdControl -> GyroBoardC.StdControl;
   App.GyroBoard      -> GyroBoardC.GyroBoard;
   
   components MagnetometerC;
   App.MagInit       -> MagnetometerC.Init;
   App.Magnetometer  -> MagnetometerC.Magnetometer;

   components StrainGaugeC;
   App.StrainInit    -> StrainGaugeC.Init;
   App.StrainGauge   -> StrainGaugeC.StrainGauge;

   components DigitalHeartRateC;
   App.DigitalHeartInit -> DigitalHeartRateC.Init;
   App.DigitalHeartRate -> DigitalHeartRateC;

   components GsrC;
   App.GsrInit -> GsrC.Init;
   App.Gsr     -> GsrC;

   components BtCommandParserC;
   App.BtCommandParser -> BtCommandParserC;

   components InternalFlashC;
   App.InternalFlash -> InternalFlashC;

   components HplMsp430InterruptP;
   //Port 23 on shimmer2 and shimmer2r, port 25 on shimmer
   App.DockInterrupt -> HplMsp430InterruptP.Port23;

   components UserButtonC;
   App.UserButton -> UserButtonC;
 
#ifdef USE_8MHZ_CRYSTAL
   components FastClockC;
   App.FastClockInit -> FastClockC;
   App.FastClock     -> FastClockC;
#endif
}
