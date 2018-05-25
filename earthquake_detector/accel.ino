/*
 * Author: Louis Moreau (https://github.com/luisomoreau)
 * Contributors: Harrison Smith (https://github.com/DStar1), Daniella Gerard (https://github.com/dgerard42)
 * Date: 2018/04/18
 * Description:
 * This program is using a modified HidnSeek device (accelerometer changed to the MMA8652FC) 
 * (https://github.com/hidnseek/hidnseek) to detect earthquakes and send the relative information using Sigfox network.
 * 
 */

void Accel::work()
{
  FluMMA865xR::IntSourceRegT intSourceR;
  FluMMA865xR::StatusRegT    statusR;
  intSourceR.v = comms.readByte(FluMMA865xR::INT_SOURCE); // check if any interrupts are pending

  if(0 != intSourceR.v && intSourceR.v != lastIntSourceR.v)
  {
    //intSourceR.print();
    lastIntSourceR.v = intSourceR.v;
  }

  if(intSourceR.f.SRC_DRDY) // data ready
  {
    statusR.v = comms.readByte(FluMMA865xR::F_STATUS); // clear all int flags by reading F_STATUS and data output regs
    if(statusR.f.ZYXDR) accelDataHandler();
    else if (DEBUG) Serial.print("spurious SRC_DRDY interrupt\n");
  }

  if(intSourceR.f.SRC_ASLP) // Int source: sleep/wake Int
  {
    sleepFuction();
    if (DEBUG) Serial.println((String)"ACCEL SLEEP/ACTIVE");
    comms.readByte(FluMMA865xR::SYSMOD); // clear sleep interrupt by reading SYSMOD
  }

  if(intSourceR.f.SRC_TRANS)  {interruptFuction();transientHandler();}   // handles transient event = transient IRQ
}

void Accel::accelDataHandler()
{
  accelLsb = accel.readData();
  if(abs(accelLsb.x) > 1 || abs(accelLsb.y) > 1 || abs(accelLsb.z) > 1) {
      if (DEBUG) Serial.print((String)"," + accelLsb.x + "," + accelLsb.y + "," + accelLsb.z + "\n");
           
      check_largest(accelLsb.x, accelLsb.y, accelLsb.z);
  }
}

void Accel::transientHandler()
{
  timeLastTransient = 0;
  FluMMA865xR::TransientSrcRegT transientSrcR;
  
  if(millis() - timeLastTransient   <   100) return;

  transientSrcR.v = comms.readByte(FluMMA865xR::TRANSIENT_SRC);
  if(!transientSrcR.f.EA && DEBUG) Serial.print("spurious transient interrupt\n");
  
//  transientSrcR.print();//uncomment to print when transient event
  timeLastTransient = millis();
}

