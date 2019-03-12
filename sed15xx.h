/**
  *@file sed15xx.h
  *@brief driver for SED 1500 series lcd driver
  *@author Jason Berger
  *@date 03/02/2019
  */


#include "Platforms/Common/mrt_platform.h"
#include "Utilities/GFX/Fonts/gfxfont.h"



//Macros for commands
#define SED15XX_CMD_ENABLE(x) (0xAE | (x))
#define SED15XX_CMD_START_LINE(x) (0xC0 | (x))
#define SED15XX_CMD_PAGE_ADDR(x) (0xAC | (x))
#define SED15XX_CMD_SET_COLUMN(x) (x)
#define SED15XX_CMD_MODIFY 0xE0
#define SED15XX_CMD_END 0xEE
#define SED15XX_CMD_RESET 0xE2

#define

typedef struct{
  int mWidth;                   //width of display in pixels
  int mHeight;                  //height of display in pixels
  uint32_t mCursor;             //cursor in buffer of display
  GFXfont* mFont;               //font to use for printing
  uint8_t* mBuffer;             //buffer of pixel data
  int mBufferSize;
  mrt_8080_handle_t mHandle;    //platform dependent handle for parallel bus
  bool mInverted;
}sed15xx_t;


/**
  *@brief initialize lcd driver
  *@param dev ptr to device descriptor
  *@param handle platform handle for parallel bus
  *@param width width (in pixels) of display buffer
  *@param height height (in pixels) of display buffer
  *@return MRT_STATUS_OK
  */
mrt_status_t sed15xx_init(sed15xx_t* dev, mrt_8080_handle_t handle,  int width, int height);

/**
  *@brief updates the device with the local buffer
  *@param dev ptr to device descriptor
  *@return MRT_STATUS_OK
  */
mrt_status_t sed15xx_refresh(sed15xx_t* dev);

/**
  *@brief enables/disables the display
  *@param dev ptr to device descriptor
  *@param val True/False = Enable/Disable
  *@return status of operation
  */
mrt_status_t sed15xx_enable(sed15xx_t* dev, bool val);

/**
  *@brief performs software reset of the device by setting reset command
  *@param dev ptr to device descriptor
  *@return status of operation
  */
mrt_status_t sed15xx_sw_reset(sed15xx_t* dev);

/**
  *@brief sets the cursor in the buffer for reading/writing
  *@param dev ptr to device descriptor
  *@param x x coordinate
  *@param y y coordinate
  *@return status of operation
  */
mrt_status_t sed15xx_set_cursor(sed15xx_t* dev, uin16_t x, uin16_t y);

/**
  *@brief writes a single pixel in the buffer
  *@param dev ptr to device descriptor
  *@param x x coordinate
  *@param y y coordinate
  *@param val val for pixel
  *@return status of operation
  */
mrt_status_t sed15xx_pixel(sed15xx_t* dev,uin16_t x, uin16_t y, bool val );

/**
  *@brief writes an array of bytes to the buffer
  *@param dev ptr to device descriptor
  *@param data ptr to data being written
  *@param len number of bytes being written
  *@return status of operation
  */
mrt_status_t sed15xx_write_buffer(sed15xx_t* dev, uint8_t* data, int len);

/**
  *@brief Draws a bitmap to the buffer
  *@param dev ptr to device descriptor
  *@param x x coord to begin drawing at
  *@param y y coord to begin drawing at
  *@param bmp bitmap to draw
  *@return status of operation
  */
mrt_status_t sed15xx_draw_bmp(sed15xx_t* dev, uin16_t x, uin16_t y, GFXBmp bmp);

/**
  *@brief Draws rendered text to the buffer
  *@param dev ptr to device descriptor
  *@param x x coord to begin drawing at
  *@param y y coord to begin drawing at
  *@param text text to be written
  *@return status of operation
  */
mrt_status_t sed15xx_print( uin16_t x, uint16_t y, const char * text);
