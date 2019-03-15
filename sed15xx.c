/**
  *@file sed15xx.c
  *@brief driver for SED 1500 series lcd driver
  *@author Jason Berger
  *@date 03/02/2019
  */

#include "sed15xx.h"


/*Memory mapping to the buffer
*
* The lcd buffer stores pixel data 'sideways'
*   0[4] = byte 0, bit 4
*
* lcd ram:
*
* 0[0] 1[0] 2[0] 3[0] 4[0] 5[0] 6[0] 7[0] .....
* 0[1] 1[1] 2[1] 3[1] 4[1] 5[1] 6[1] 7[1] .....
* 0[2] 1[2] 2[2] 3[2] 4[2] 5[2] 6[2] 7[2] .....
* 0[3] 1[3] 2[3] 3[3] 4[3] 5[3] 6[3] 7[3] .....
* 0[4] 1[4] 2[4] 3[4] 4[4] 5[4] 6[4] 7[4] .....
* 0[5] 1[5] 2[5] 3[5] 4[5] 5[5] 6[5] 7[5] .....
* 0[6] 1[6] 2[6] 3[6] 4[6] 5[6] 6[6] 7[6] .....
* 0[7] 1[7] 2[7] 3[7] 4[7] 5[7] 6[7] 7[7] .....
*
* local buffer:
*
* 0[7] 0[6] 0[5] 0[4] 0[3] 0[2] 0[1] 0[0] ,
* 1[7] 1[6] 1[5] 1[4] 1[3] 1[2] 1[1] 1[0] ,
* 2[7] 2[6] 2[5] 2[4] 2[3] 2[2] 2[1] 2[0] ,
* 3[7] 3[6] 3[5] 3[4] 3[3] 3[2] 3[1] 3[0] ,
* 4[7] 4[6] 4[5] 4[4] 4[3] 4[2] 4[1] 4[0] ,
* 5[7] 5[6] 5[5] 5[4] 5[3] 5[2] 5[1] 5[0] ,
* 6[7] 6[6] 6[5] 6[4] 6[3] 6[2] 6[1] 6[0] ,
* 7[7] 7[6] 7[5] 7[4] 7[3] 7[2] 7[1] 7[0] ,
*
* So before writing to the device, we take a 'block' which is 8 pixels by 8 pixels,
* and rotate it to match the lcd ram
*
*/

/**
  *@brief rotates an 8x8 matrix of bits
  *@param A input array
  *@param B output array
  *
  *@notes 'Hackers Delight' page 108, Henry S. Warren
  */
inline void transpose8(uint8_t A[8], uint8_t B[8])
{
   uint32_t x, y, t;

   // Load the array and pack it into x and y.

   x = (A[0]<<24)   | (A[1]<<16)   | (A[2]<<8) | A[3];
   y = (A[4]<<24) | (A[5]<<16) | (A[6]<<8) | A[7];

   t = (x ^ (x >> 7)) & 0x00AA00AA;  x = x ^ t ^ (t << 7);
   t = (y ^ (y >> 7)) & 0x00AA00AA;  y = y ^ t ^ (t << 7);

   t = (x ^ (x >>14)) & 0x0000CCCC;  x = x ^ t ^ (t <<14);
   t = (y ^ (y >>14)) & 0x0000CCCC;  y = y ^ t ^ (t <<14);

   t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F);
   y = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F);
   x = t;

   B[0]=x>>24;    B[1]=x>>16;    B[2]=x>>8;  B[3]=x;
   B[4]=y>>24;  B[5]=y>>16;  B[6]=y>>8;  B[7]=y;
}

mrt_status_t sed15xx_init(sed15xx_t* dev, sed15xx_hw_cfg_t* hw,  int width, int height )
{
  //copy in hw config
  memcpy(&dev->mHW, hw, sizeof(sed15xx_hw_cfg_t));

  dev->mInverted = false;

  // initialize canvas as buffered canvas
  mono_gfx_init_buffered(&dev->mCanvas, width, height);

/*
   * Initialize display.  This code follows the sequence in the
   * example code from the display manufacturer.
   */
   sed15xx_cmd( dev , SED15XX_CMD_RESET);
   sed15xx_cmd( dev , SED15XX_CMD_ADC_SELECT_NORMAL);
   sed15xx_cmd( dev , SED15XX_CMD_COM_REVERSE);
   sed15xx_cmd( dev , SED15XX_CMD_LCD_BIAS_1_7);
   sed15xx_cmd( dev , SED15XX_CMD_POWER_CONTROL(0x7));
   sed15xx_cmd( dev , SED15XX_CMD_REGULATOR_RESISTOR_RATIO_SET(0x5));
   sed15xx_cmd( dev , SED15XX_CMD_CONTRAST_SET);
   sed15xx_cmd( dev , 19);
   sed15xx_cmd( dev , SED15XX_CMD_START_LINE_SET(0x0));
   sed15xx_cmd( dev , SED15XX_CMD_BOOSTER_RATIO_SET);
   sed15xx_cmd( dev , 0x0);
   sed15xx_cmd( dev , SED15XX_CMD_DISPLAY_ON);

  return MRT_STATUS_OK;
}

void sed15xx_cmd(sed15xx_t* dev, uint8_t cmd)
{
  uint32_t dataMask = 0xFF << dev->mHW.mDataOffset;

  //set bus to command/ctrl mode
  MRT_GPIO_WRITE(dev->mHW.mA0, LOW);

  //set WR low
  MRT_GPIO_WRITE(dev->mHW.mWR, LOW);

  //write command byte
  MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, (cmd << (dev->mHW.mDataOffset)));

  //set WR high to clock in cmd
  MRT_GPIO_WRITE(dev->mHW.mWR, HIGH);

}

mrt_status_t sed15xx_refresh(sed15xx_t* dev)
{
  uint32_t dataMask = 0xFF << dev->mHW.mDataOffset;

  //set cursor to 0,0
  sed15xx_cmd(dev,SED15XX_CMD_START_LINE_SET(0));
  sed15xx_cmd(dev,SED15XX_CMD_PAGE_ADDRESS_SET(0));
  sed15xx_cmd(dev,SED15XX_CMD_COLUMN_ADDRESS_SET_LOW(0));
  sed15xx_cmd(dev,SED15XX_CMD_COLUMN_ADDRESS_SET_HIGH(0));

  //disable display while we write the data
  sed15xx_enable(dev, false);

  // make sure we are in data mode
  MRT_GPIO_WRITE(dev->mHW.mA0, HIGH);

  //local buffer to store transposed block
  uint8_t transposedBuffer[8];

  for(int i=0; i < dev->mCanvas.mBufferSize/8; i++)
  {

    //rotate block of pixels to match lcd memory structure. see README.md for more information
    transpose8(&dev->mCanvas.mBuffer[i*8], transposedBuffer);

    for(int a=0; a <8; a++)
    {
      //set WR low
      MRT_GPIO_WRITE(dev->mHW.mWR, LOW);

      //write data byte (invert pixels if mInverted is set)
      if(dev->mInverted)
        MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, ~(dev->mCanvas.mBuffer[i] << (dev->mHW.mDataOffset)));
      else
        MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, (dev->mCanvas.mBuffer[i] << (dev->mHW.mDataOffset)));
      //set WR high to clock in cmd
      MRT_GPIO_WRITE(dev->mHW.mWR, HIGH);

    }

  }

  //re-enable display
  sed15xx_enable(dev, true);

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_enable(sed15xx_t* dev, bool val)
{
  if(val)
    sed15xx_cmd( dev, SED15XX_CMD_DISPLAY_ON);
  else
    sed15xx_cmd( dev , SED15XX_CMD_DISPLAY_OFF);

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_sw_reset(sed15xx_t* dev)
{
  sed15xx_cmd( dev , SED15XX_CMD_RESET);

  return MRT_STATUS_OK;
}


mrt_status_t sed15xx_draw_bmp(sed15xx_t* dev, uint16_t x, uint16_t y, GFXBmp* bmp)
{
  return mono_gfx_draw_bmp(&dev->mCanvas, x,y,bmp);
}

mrt_status_t sed15xx_print( sed15xx_t* dev, uint16_t x, uint16_t y, const char * text)
{

  return mono_gfx_print(&dev->mCanvas,x,y,text);
}

mrt_status_t sed15xx_fill(sed15xx_t* dev, uint8_t val)
{
  return mono_gfx_fill(&dev->mCanvas,val);
}
