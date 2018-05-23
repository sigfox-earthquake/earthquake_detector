/*
 * Author: Louis Moreau (https://github.com/luisomoreau)
 * Contributors: Harrison Smith (https://github.com/DStar1), Daniella Gerard (https://github.com/dgerard42)
 * Date: 2018/04/18
 * Description:
 * This program is using a modified HidnSeek device (accelerometer changed to the MMA8652FC) 
 * (https://github.com/hidnseek/hidnseek) to detect earthquakes and send the relative information using Sigfox network.
 * 
 */

void InitializeFluMMA865x::begin()
{
  accel.fullScaleRange = FluMMA865xR::XyzDataCfgRegT::ACCEL_MODE;
  accel.outputDataRate = FluMMA865xR::CtrlReg1T::DATA_RATE;
  convFactMicrograv = accel.getConversionFactorMicrograv();
  Serial << F("convFact=") << convFactMicrograv << "ug\n";

  Wire.begin();
  accel.reset();
  while(!accel.devicePresent())
  {
    Serial << F("ERR: FluMMA865x not present!");
    delay(20000);
  }

  accel.calibrate();
  Serial << F("bias x=") << (int8_t)2 * (int8_t)comms.readByte(FluMMA865xR::OFF_X)
         << F("\ty=")    << (int8_t)2 * (int8_t)comms.readByte(FluMMA865xR::OFF_Y)
         << F("\tz=")    << (int8_t)2 * (int8_t)comms.readByte(FluMMA865xR::OFF_Z) << F("mg\n");
  
  init();
  delay(2000); // wait for device to settle
  lastIntSourceR.v = 0;  
  accel.clearInterrupts();    
}


void InitializeFluMMA865x::init()
{
  FluMMA865xR::HpFilterCutoffRegT hpFilterCutoffR;
  FluMMA865xR::XyzDataCfgRegT     xyzDataCfgR;
  FluMMA865xR::CtrlReg1T          ctrlReg1, ctrlReg1Old;
  FluMMA865xR::CtrlReg2T          ctrlReg2;
  FluMMA865xR::CtrlReg4T          ctrlReg4;
  FluMMA865xR::TransientCfgRegT   transientCfgR;
  FluMMA865xR::PulseCfgRegT       pulseCfgR;
  FluMMA865xR::FfMtCfgRegT        ffMtCfgR;
  FluMMA865xR::FfMtThsRegT        ffMtThsR;
  FluMMA865xR::TransientThsRegT   transientThsR;
  
  (void) accel.standby(); // Must be in standby mode to change most registers

  // Set up high pass filter: removes static acceleration(gravitation) and leaves only dynamic acceleration:
  // taps, flicks, shakes, steps, ... . offset(DC) and low freqs well below the cutoff freq are removed
  hpFilterCutoffR.f.SEL           = B11;   // 11 = weakest filter
  hpFilterCutoffR.f.Pulse_LPF_EN  = false; // low-pass filter disabled
  hpFilterCutoffR.f.Pulse_HPF_BYP = false; // high-pass filter enabled ( = bypass disabled)
  comms.writeByte(FluMMA865xR::HP_FILTER_CUTOFF, hpFilterCutoffR.v);

  xyzDataCfgR.f.FS = accel.fullScaleRange; // FluMMA865x.cpp sets default AFS_2g
  xyzDataCfgR.f.HPF_OUT = true;            // Output data is high pass filtered
  comms.writeByte(FluMMA865xR::XYZ_DATA_CFG, xyzDataCfgR.v);

  ctrlReg1Old.v = comms.readByte(FluMMA865xR::CTRL_REG1);
  ctrlReg1.v = 0;
//ctrlReg1.f.F_READ = 0 (default) // Normal read mode (1 = fast read mode)
//ctrlReg1.f.LNOISE = true;//not in main data sheet, only in Appnote AN4083 
  ctrlReg1.f.DR = accel.outputDataRate; // FluMMA865x.cpp sets degault AODR_50HZ
  ctrlReg1.f.ASLP_RATE = SLEEP_DATA_RATE;//B11; // Asleep data rate for low power 11 = 1.56Hz see Datasheet Table 95, p.54
  comms.writeByte(FluMMA865xR::CTRL_REG1, ctrlReg1.v);

  ctrlReg2.v = 0;
  ctrlReg2.f.MODS = FluMMA865xR::CtrlReg2T::HighResolution; // HiRes pulls 184uA, LowPower 7uA@1-12Hz, 49uA@200Hz
ctrlReg2.f.SLPE = true; // Enable auto SLEEP
//ctrlReg2.f.SMODS = FluMMA865xR::CtrlReg2T::LowNoiseLowPower;//FluMMA865xR::CTRL_REG2::LowNoiseLowPower;// / LowPower // set sleep power mode scheme
  comms.writeByte(FluMMA865xR::CTRL_REG2, ctrlReg2.v);

comms.writeByte(0x29, (SLP_CNT*3));//0x40); // sleep after ~36 seconds of inactivity at 6.25 Hz ODR (D6=40)

// CTRL_REG3 register is used to control the Auto-WAKE/SLEEP function by setting the orientation or Freefall/Motion
// as an interrupt to wake. CTRL_REG3 register also configures the interrupt pins INT1 and INT2.    Datasheet p. 56
comms.writeByte(FluMMA865xR::CTRL_REG3, 0x68);
// Bit 7: FIFO_GATE 0 default fifo flushed on transition between WAKE and SLEEP modes
//     6: WAKE_TRANS Wake from Transient interrupt 0
//     5: WAKE_LNDPRT
//     4: WAKE_PULSE
//     3: WAKE_FF_MT
//     2: RSVD
//     1: IPOL Interrupt polarity 0 ACTIVE low default

// CTRL_REG4 egister enables the following interrupts: Auto-WAKE/SLEEP, Orientation Detection, Freefall/Motion, 
// and Data Ready.

// Bit 7: INT_EN_ASLP
//     6: INT_EN_FIFO
//     5: INT_EN_TRANS
//     4: INT_EN_LNDPRT
//     3: INT_EN_PULSE
//     2: INT_EN_FF_MT
//     1: 0
//     0 INT_EN_DRDY     Data Ready

  // Enable Auto-SLEEP/WAKE interrupt
  // accelWriteByte(CTRL_REG4, accelReadByte(CTRL_REG4) & ~(0x80)); // clear bit 7
  // accelWriteByte(CTRL_REG4, accelReadByte(CTRL_REG4) |  (0x80)); // select  Auto-SLEEP/WAKE interrupt enable



   
  // Set up portrait/landscape registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  // accelWriteByte(PL_CFG, 0x40);        // 1. Enable P/L
  // dis // accelWriteByte(PL_BF_ZCOMP, 0x44); // 2. 29deg z-lock (don't think this register is actually writable)
  // dis // accelWriteByte(P_L_THS_REG, 0x84); // 3. 45deg thresh, 14deg hyst (don't think this register is writable either)
  // accelWriteByte(PL_COUNT, 0x50);      // 4. debounce counter at 100ms (at 800 hz)


  /* Set up single and double tap - 5 steps:
  1. Set up single and/or double tap detection on each axis individually.
  2. Set the threshold - minimum required acceleration to cause a tap.
  3. Set the time limit - the maximum time that a tap can be above the threshold
  4. Set the pulse latency - the minimum required time between one pulse and the next
  5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
  for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  pulseCfgR.f.XSPEFE = 1; // Enable x/y/z single pulse events
  pulseCfgR.f.YSPEFE = 1; 
  pulseCfgR.f.ZSPEFE = 1;
  pulseCfgR.f.XDPEFE = 0; // Disable x/y/z double pulse events
  pulseCfgR.f.YDPEFE = 0;
  pulseCfgR.f.ZDPEFE = 0;
  pulseCfgR.f.DPA    = 0; // Double Pulse Abort
  pulseCfgR.f.ELE    = 1; // Pulse event flags are latched into PULSE_SRC
  comms.writeByte(FluMMA865xR::PULSE_CFG, pulseCfgR.v);
  comms.writeByte(FluMMA865xR::PULSE_THSX, 0x20);  // 0x20 = 32 lsb * 0.0625 g/lsb = 2g
  comms.writeByte(FluMMA865xR::PULSE_THSY, 0x20); 
  comms.writeByte(FluMMA865xR::PULSE_THSZ, 0x20);
  comms.writeByte(FluMMA865xR::PULSE_TMLT, 0x30);  // 3. 2.55s time limit at 100Hz odr, this is very dependent on data rate, see the app note
  comms.writeByte(FluMMA865xR::PULSE_LTCY, 0xA0);  // 4. 5.1s 100Hz odr between taps min, this also depends on the data rate
  comms.writeByte(FluMMA865xR::PULSE_WIND, 0xFF);  // 5. 10.2s (max value)  at 100 Hz between taps max

  
  ffMtCfgR.v = 0; // Set up motion detection
  ffMtCfgR.f.XEFE  = 1; // Set motion flag on x/y axes
  ffMtCfgR.f.YEFE  = 1;
  ffMtCfgR.f.ZEFE  = 0; // gravity is always triggering Z event even with high pass filter enabled
  ffMtCfgR.f.OAE   = 1; // =1 Logical OR = Motion detection ( = 0 Logical AND = Freefall detection)
  ffMtCfgR.f.ELE   = 1; // Event latch Enable = 0: realtime status; =1 reading FF_MT_SRC clears EA and all FF_MT_SRC bits
  comms.writeByte(FluMMA865xR::FF_MT_CFG, ffMtCfgR.v);
  
  ffMtThsR.f.THS    = 0x08; // Set treshhold to 8 * 0.063g/LSB == 0.5g (independent of selected g range)
  ffMtThsR.f.DBCNTM = 1; // Debounce counter mode 1: counter cleared to 0 when event no longer active
  comms.writeByte(FluMMA865xR::FF_MT_THS, ffMtThsR.v);                                                
  comms.writeByte(FluMMA865xR::FF_MT_COUNT, 0x4); // Set 8 bit debounce counter to 4 (0.04 s at 100 Hz in ___ mode?)

  //Set up (high-pass-filtered) transient detection --hw
  transientCfgR.v   = 0;
//transientCfgR.f.HPF_BYP = false; // 0 = High-pass filter active (bypass disabled)
  transientCfgR.f.XTEFE = true;
  transientCfgR.f.YTEFE = true;
  transientCfgR.f.ZTEFE = true;
  transientCfgR.f.ELE   = true; // event flags are latched into TRANSIENT_SRC
  comms.writeByte(FluMMA865xR::TRANSIENT_CFG, transientCfgR.v);


// Transient
  transientThsR.v = 0;
  transientThsR.f.THS   = ACCEL_TRIG;//3;    // treshhold 0-127 x 0.063 g/LSB == 0 - 8 g (+/-): 3lsb = 0.189g
  comms.writeByte(FluMMA865xR::TRANSIENT_THS, transientThsR.v); 
  comms.writeByte(FluMMA865xR::TRANSIENT_COUNT, 0x8); // debounce count
 

// Set up interrupt 1 and 2
  //accelWriteByte(CTRL_REG3, accelReadByte(CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
  //accelWriteByte(CTRL_REG3, accelReadByte(CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts
  // dis // accelWriteByte(0x2C, 0x02);  // Active high, push-pull interrupts

  ctrlReg4.v = 0;
  ctrlReg4.f.INT_EN_DRDY   = true; // Only ints enabled in CtrlReg4 are visible in IntSourceReg --hw
  ctrlReg4.f.INT_EN_FF_MT  = true;
  ctrlReg4.f.INT_EN_PULSE  = true;
  ctrlReg4.f.INT_EN_LNDPRT = true;
  ctrlReg4.f.INT_EN_TRANS  = true;
//ctrlReg4.f.INT_EN_FIFO   = true;
  ctrlReg4.f.INT_EN_ASLP   = true;
  comms.writeByte(FluMMA865xR::CTRL_REG4, ctrlReg4.v);
   
  //accelWriteByte(CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2

  accel.active();  // Set to active mode to start reading
}

