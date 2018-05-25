/*
 * Author: Louis Moreau (https://github.com/luisomoreau)
 * Contributors: Harrison Smith (https://github.com/DStar1), Daniella Gerard (https://github.com/dgerard42)
 * Date: 2018/04/18
 * Description:
 * This program is using a modified HidnSeek device (accelerometer changed to the MMA8652FC) 
 * (https://github.com/hidnseek/hidnseek) to detect earthquakes and send the relative information using Sigfox network.
 *
 */


/***************************************************************************//**
 * @brief
   This is where the official algorythm will be placed (not official algo at the moment)
   Finds largest x, y, z
   Called in Accel::accelDataHandler()
   Resets on next accel interrupt
 *
 * @param[in] none at the moment

 ******************************************************************************/

 void check_largest(int x, int y, int z)
 {
    //find largest x, y, z
    x = abs(x);
    y = abs(y);
    z = abs(z);
    if (x > largest_x) largest_x = x;
    if (y > largest_y) largest_y = y;
    if (z > largest_z) largest_z = z;
 }
 
 /*
  * dir:  0 = all
  *       1 = x
  *       2 = y
  *       3 = z
  *       4 = x & y
  *       5 = x & z
  *       6 = y & z
  * 
  */
// void send_largest(int dir)
// {
////  if (EEPROM.read
////  if (x1 < x2)
////    xres = x2;
////  if (y1 < y2)
////    yres = y2;
////  if (z1 < z2)
////    zres = z2;
// }

