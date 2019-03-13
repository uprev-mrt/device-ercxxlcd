/**
  *@file sed15xx.c
  *@brief driver for SED 1500 series lcd driver
  *@author Jason Berger
  *@date 03/02/2019
  */


mrt_status_t sed15xx_init(sed15xx_t* dev, mrt_8080_handle_t handle,  int width, int height)
{


  dev->mHandle = handle;
  dev->mWidth = width;
  dev->mHeight = height;
  dev->mCursor =0;
  dev->mBufferSize = (width * height)/8;
  dev->mBuffer = (uint8_t*)malloc( dev->mBufferSize);
  dev->mInverted = false;

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_refresh(sed15xx_t* dev)
{
  //set cursor to 0,0
  sed15xx_set_cursor(dev, 0,0);

  //disable display while we write the data
  sed15xx_enable(dev, false);

  //write data buffer
  MRT_8080_DATA_WRITE(dev->mBuffer, dev->mBufferSize);

  //re-enable display
  sed15xx_enable(dev, true);

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_enable(sed15xx_t* dev, bool val)
{
  if(val)
    MRT_8080_CMD(SED15XX_CMD_ENABLE(1));
  else:
    MRT_8080_CMD(SED15XX_CMD_ENABLE(0));

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_sw_reset(sed15xx_t* dev)
{
  MRT_8080_CMD(SED15XX_CMD_RESET);

  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_set_cursor(sed15xx_t* dev, uin16_t x, uin16_t y)
{
  //TODO set cursor
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_pixel(sed15xx_t* dev,uin16_t x, uin16_t y, bool val )
{
  //TODO
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_write_buffer(sed15xx_t* dev, uint8_t* data, int len)
{
  //TODO
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_draw_bmp(sed15xx_t* dev, uin16_t x, uin16_t y, GFXBmp bmp)
{
  //TODO
  return MRT_STATUS_OK;
}

mrt_status_t sed15xx_print( uin16_t x, uint16_t y, const char * text)
{
  //TODO
  return MRT_STATUS_OK;
}
