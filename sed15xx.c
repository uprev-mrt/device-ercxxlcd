/**
  *@file sed15xx.c
  *@brief driver for SED 1500 series lcd driver
  *@author Jason Berger
  *@date 03/02/2019
  */

#include "sed15xx.h"


/**
  *@brief custom pixel writing function
  */
mrt_status_t sed15xx_write_pixel(mono_gfx_t* gfx, int x, int y, uint8_t val)
{
  if(( x < 0) || (x >= gfx->mWidth) || (y < 0) || (y>= gfx->mHeight))
    return MRT_STATUS_OK;

    if(val)
      gfx->mBuffer[x + (y/8)*gfx->mWidth] |=  (1 << (y&7));
    else
      gfx->mBuffer[x + (y/8)*gfx->mHeight] &= ~(1 << (y&7));

  return MRT_STATUS_OK;
}


mrt_status_t sed15xx_init(sed15xx_t* dev, sed15xx_hw_cfg_t* hw,  int width, int height )
{
  //copy in hw config
  memcpy(&dev->mHW, hw, sizeof(sed15xx_hw_cfg_t));

  // make sure we are in data mode
  MRT_GPIO_WRITE(dev->mHW.mRST, LOW);
  //MRT_DELAY_MS(1);
  MRT_GPIO_WRITE(dev->mHW.mRST, HIGH);
  MRT_GPIO_WRITE(dev->mHW.mCS, LOW);
  MRT_GPIO_WRITE(dev->mHW.mRD, HIGH);
  MRT_GPIO_WRITE(dev->mHW.mWR, HIGH);

  uint32_t dataMask = 0xFF << dev->mHW.mDataOffset;


  //release data pins
  //MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, dataMask);

  
  dev->mInverted = false;

  // initialize canvas as buffered canvas
  mono_gfx_init_buffered(&dev->mCanvas, width, height);

  //use custom pixel function to handle pixel mapping
  dev->mCanvas.fWritePixel = &sed15xx_write_pixel;

  uint8_t status = sed15xx_get_status(dev);

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
   sed15xx_cmd( dev, SED15XX_CMD_DISPLAY_ALL_POINTS_OFF);
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

  //MRT_DELAY_MS(1);
  //set WR high to clock in cmd
  MRT_GPIO_WRITE(dev->mHW.mWR, HIGH);

  //release data pins
  MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, dataMask);

  //MRT_DELAY_MS(1);

}

uint8_t sed15xx_get_status(sed15xx_t* dev)
{
  uint32_t stat;

  MRT_GPIO_PORT_SET_DIR(dev->mHW.mPort,(0xFF << dev->mHW.mDataOffset ), MRT_GPIO_MODE_INPUT );

  //set bus to command/ctrl mode
  MRT_GPIO_WRITE(dev->mHW.mA0, LOW);

  //MRT_DELAY_MS(1);

  //set RD LOW
  MRT_GPIO_WRITE(dev->mHW.mRD, LOW);
  //MRT_DELAY_MS(1);

  //write command byte
  stat = MRT_GPIO_PORT_READ(dev->mHW.mPort);
  stat = (stat >> dev->mHW.mDataOffset) & 0xFF;

  //set RD HIGH
  MRT_GPIO_WRITE(dev->mHW.mRD, HIGH);

  MRT_GPIO_PORT_SET_DIR(dev->mHW.mPort,(0xFF << dev->mHW.mDataOffset ), MRT_GPIO_MODE_OUTPUT );

  return stat;
}

mrt_status_t sed15xx_refresh(sed15xx_t* dev)
{
  uint32_t dataMask = 0xFF << dev->mHW.mDataOffset;


  //disable display while we write the data
  sed15xx_enable(dev, false);

  int addr = 0;

  for (int i = 0; i < (dev->mCanvas.mHeight/8); i++)
  {

    //set cursor to next page column 0
    sed15xx_cmd(dev,SED15XX_CMD_PAGE_ADDRESS_SET(i));
    sed15xx_cmd(dev,SED15XX_CMD_COLUMN_ADDRESS_SET_LOW(0));
    sed15xx_cmd(dev,SED15XX_CMD_COLUMN_ADDRESS_SET_HIGH(0));

    // make sure we are in data mode
    MRT_GPIO_WRITE(dev->mHW.mA0, HIGH);

    for(int a =0; a < dev->mCanvas.mWidth; a++)
    {
      //set WR low
      MRT_GPIO_WRITE(dev->mHW.mWR, LOW);

      addr = (i*dev->mCanvas.mWidth) + a;

      //write data byte (invert pixels if mInverted is set)
      if(dev->mInverted)
        MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, ~(dev->mCanvas.mBuffer[addr] << (dev->mHW.mDataOffset)));
      else
        MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, (dev->mCanvas.mBuffer[addr] << (dev->mHW.mDataOffset)));

      //MRT_DELAY_MS(1);

      //set WR high to clock in data
      MRT_GPIO_WRITE(dev->mHW.mWR, HIGH);
    }
  }

  //re-enable display
  sed15xx_enable(dev, true);

  //release data pins
  MRT_GPIO_PORT_WRITE(dev->mHW.mPort, dataMask, dataMask);

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
