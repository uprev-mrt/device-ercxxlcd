/**
  *@file sed15xx.c
  *@brief driver for SED 1500 series lcd driver
  *@author Jason Berger
  *@date 03/02/2019
  */

#include "sed15xx.h"

mrt_status_t sed15xx_init(sed15xx_t* dev, mrt_pbus_handle_t handle,  int width, int height )
{

  dev->mHandle = handle;
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
  //set bus to command/ctrl mode
  MRT_PBUS_MODE(dev->mHandle, PBUS_MODE_CMD);

  //write command byte
  MRT_PBUS_WRITE(dev->mHandle, cmd);
}

mrt_status_t sed15xx_refresh(sed15xx_t* dev)
{
  //set cursor to 0,0
  sed15xx_set_cursor(dev, 0,0);

  //disable display while we write the data
  sed15xx_enable(dev, false);

  // make sure we are in data mode
  MRT_PBUS_MODE(dev->mHandle, PBUS_MODE_DATA);

  //write data buffer
  for(int i=0; i < dev->mBufferSize; i++)
  {
    MRT_PBUS_WRITE(dev->mHandle, dev->mBuffer[i]);
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
  if(dev->mFont == NULL)
    return MRT_STATUS_OK;

  uint16_t xx =x;
  uint16_t yy = y;
  GFXglyph* glyph;
  GFXBmp bmp;
  char c = *text++;

  while(c != 0)
  {
    if(c == '\n')
    {
      yy+= dev->mFont->yAdvance;
    }
    else
    {
      glyph = &dev->mFont->glyph[c - dev->mFont->first]; //index in glyph array is offset by first printable char in font

      //map glyph to a bitmap that we can draw
      bmp.data = &dev->mFont->bitmap[glyph->bitmapOffset];
      bmp.width = glyph->width;
      bmp.height = glyph->height;

      //draw the character
      sed15xx_draw_bmp(dev, xx + glyph->xOffset, yy + glyph->yOffset, &bmp );
      xx += glyph->xOffset + glyph->xAdvance;
    }

    char c = *text++;
  }

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_fill(sed15xx_t* dev, uint8_t val)
{
  memset(dev->mBuffer, val, dev->mBufferSize);
  return MRT_STATUS_OK;
}
