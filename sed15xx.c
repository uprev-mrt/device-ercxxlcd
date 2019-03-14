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

  memcpy(&dev->mHW, hw, sizeof(sed15xx_hw_cfg_t));
  dev->mWidth = width;
  dev->mHeight = height;
  dev->mCursor =0;
  dev->mBufferSize = (width * height)/8;
  dev->mBuffer = (uint8_t*)malloc( dev->mBufferSize);
  dev->mInverted = false;


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
  sed15xx_set_cursor(dev, 0,0);

  //disable display while we write the data
  sed15xx_enable(dev, false);

  // make sure we are in data mode
  MRT_GPIO_WRITE(dev->mHW.mA0, HIGH);

  //local buffer to store transposed block
  uint8_t transposedBuffer[8];

  for(int i=0; i < dev->mBufferSize/8; i++)
  {

    //rotate block of pixels to match lcd memory structure. see README.md for more information
    transpose8(&dev->mBuffer[i*8], transposedBuffer);

    for(int a=0; a <8; a++)
    {
      //set WR low
      MRT_GPIO_WRITE(dev->mHW.mWR, LOW);

      //write data byte (invert pixels if mInverted is set)
      if(dev->mInverted)
        MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, ~(dev->mBuffer[i] << (dev->mHW.mDataOffset)));
      else
        MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, (dev->mBuffer[i] << (dev->mHW.mDataOffset)));
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

mrt_status_t sed15xx_set_cursor(sed15xx_t* dev, uint16_t x, uint16_t y)
{
  dev->mCursor = (y * dev->mWidth) + x;

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_pixel(sed15xx_t* dev,uint16_t x, uint16_t y, bool val )
{
  dev->mCursor  = (y * dev->mWidth) + x;
  uint32_t byteOffset = (dev->mCursor  / 8);
  uint8_t bitOffset = dev->mCursor  % 8;

  if( val)
    dev->mBuffer[byteOffset] |= (1 << bitOffset);
  else
    dev->mBuffer[byteOffset] &= (~(1 << bitOffset));

  //advance
  dev->mCursor++;

  //wrap
  if(dev->mCursor == (dev->mWidth * dev->mHeight))
    dev->mCursor = 0;

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_write_buffer(sed15xx_t* dev, uint8_t* data, int len, bool wrap)
{
  //get number of bits off of alignment in case we are not writing on a byte boundary
  uint32_t byteOffset = (dev->mCursor  / 8);
  uint8_t bitOffset = dev->mCursor % 8;

  //get number of bytes before we would wrap to next row
  int nextRow = (dev->mWidth - (dev->mCursor % dev->mWidth));
  if((nextRow < len) && (wrap == false))
  {
    len = nextRow;
  }


  uint8_t prevByte; //used for shifting in data when not aligned
  uint8_t mask;


  //If we are byte aligned , just memcpy the data in
  if(bitOffset == 0)
  {
    memcpy(&dev->mBuffer[byteOffset], data, len);
  }
  //If we are not byte aligned, we have to mask and shift in data
  else
  {
    mask = 0xFF << (8-bitOffset);
    prevByte = dev->mBuffer[byteOffset] & mask;

    for(int i=0; i < len; i++)
    {
      dev->mBuffer[byteOffset++] = prevByte | (data[i] >> bitOffset);
      prevByte = data[i] << (8-bitOffset);

      if(byteOffset >= dev->mBufferSize)
        byteOffset = 0;
    }
  }


  //advance cursor
  dev->mCursor += len;

  // If its gone over, wrap
  while(dev->mCursor >= (dev->mWidth * dev->mHeight))
    dev->mCursor -=  (dev->mWidth * dev->mHeight);

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_draw_bmp(sed15xx_t* dev, uint16_t x, uint16_t y, GFXBmp* bmp)
{
  uint32_t bmpIdx = 0;
  for(int i=0; i < bmp->height; i ++)
  {
    sed15xx_set_cursor(dev,x,y+i);
    sed15xx_write_buffer(dev, &bmp->data[bmpIdx], bmp->width, false);
    bmpIdx += bmp->width;
  }

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_print( sed15xx_t* dev, uint16_t x, uint16_t y, const char * text)
{

  //if a font has not been set, return error
  if(dev->mFont == NULL)
    return MRT_STATUS_ERROR;

  uint16_t xx =x;     //current position for writing
  uint16_t yy = y;
  GFXglyph* glyph;    //pointer to glyph for current character
  GFXBmp bmp;         //bitmap struct used to draw glyph
  char c = *text++;   //grab first character from string

  //run until we hit a null character (end of string)
  while(c != 0)
  {
    if(c == '\n')
    {
      //if character is newline, we advance the y, and reset x
      yy+= dev->mFont->yAdvance;
      xx = x;
    }
    else if((c >= dev->mFont->first) && (c <= dev->mFont->last))// make sure the font contains this character
    {
      //grab the glyph for current character from our font
      glyph = &dev->mFont->glyph[c - dev->mFont->first]; //index in glyph array is offset by first printable char in font

      //map glyph to a bitmap that we can draw
      bmp.data = &dev->mFont->bitmap[glyph->bitmapOffset];
      bmp.width = glyph->width;
      bmp.height = glyph->height;

      //draw the character
      sed15xx_draw_bmp(dev, xx + glyph->xOffset, yy + glyph->yOffset, &bmp );
      xx += glyph->xOffset + glyph->xAdvance;
    }


    //get next character
    c = *text++;
  }

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_fill(sed15xx_t* dev, uint8_t val)
{
  memset(dev->mBuffer, val, dev->mBufferSize);
  return MRT_STATUS_OK;
}
