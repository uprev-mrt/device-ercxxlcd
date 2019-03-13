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
  //TODO set cursor
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_pixel(sed15xx_t* dev,uint16_t x, uint16_t y, bool val )
{
  //TODO
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_write_buffer(sed15xx_t* dev, uint8_t* data, int len)
{
  //TODO
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_draw_bmp(sed15xx_t* dev, uint16_t x, uint16_t y, GFXBmp bmp)
{
  //TODO
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_print( sed15xx_t* dev, uint16_t x, uint16_t y, const char * text)
{
  //TODO
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_fill(sed15xx_t* dev, uint8_t val)
{
  memset(dev->mBuffer, val, dev->mBufferSize);
  return MRT_STATUS_OK;
}
