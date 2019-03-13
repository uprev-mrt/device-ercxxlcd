/**
  *@file sed15xx.h
  *@brief driver for SED 1500 series lcd driver
  *@author Jason Berger
  *@date 03/02/2019
  */


#include "Platforms/Common/mrt_platform.h"
#include "Utilities/GFX/Fonts/gfxfont.h"
#include <string.h>



//Macros for commands
#define SED15XX_CMD_REGULATOR_RESISTOR_RATIO_SET(s)  (0x20 | (s))
#define SED15XX_CMD_POWER_CONTROL(s)                 (0x28 | (s))
#define SED15XX_CMD_CONTRAST_SET                     (0x81)
#define SED15XX_CMD_ADC_SELECT_NORMAL                (0xA0)
#define SED15XX_CMD_ADC_SELECT_REVERSE               (0xA1)
#define SED15XX_CMD_LCD_BIAS_1_9                     (0xA2)
#define SED15XX_CMD_LCD_BIAS_1_7                     (0xA3)
#define SED15XX_CMD_DISPLAY_ALL_POINTS_OFF           (0xA4)
#define SED15XX_CMD_DISPLAY_ALL_POINTS_ON            (0xA5)
#define SED15XX_CMD_DISPLAY_NORMAL                   (0xA6)
#define SED15XX_CMD_DISPLAY_REVERSE                  (0xA7)
#define SED15XX_CMD_DISPLAY_OFF                      (0xAE)
#define SED15XX_CMD_DISPLAY_ON                       (0xAF)
#define SED15XX_CMD_COM_NORMAL                       (0xC0)
#define SED15XX_CMD_COM_REVERSE                      (0xC8)
#define SED15XX_CMD_READ_MODIFY_WRITE                (0xE0)
#define SED15XX_CMD_END                              (0xEE)
#define SED15XX_CMD_RESET                            (0xE2)
#define SED15XX_CMD_BOOSTER_RATIO_SET                (0xF8)
#define SED15XX_CMD_START_LINE_SET(line)             (0x40 | (line))
#define SED15XX_CMD_PAGE_ADDRESS_SET(page)           (0xD0 | (page))
#define SED15XX_CMD_COLUMN_ADDRESS_SET_HIGH(column)  (0x10 | ((column >> 4) & 0x0F))
#define SED15XX_CMD_COLUMN_ADDRESS_SET_LOW(column)   (0x00 | ((column >> 0) & 0x0F))


typedef struct{
  int mWidth;                   //width of display in pixels
  int mHeight;                  //height of display in pixels
  uint32_t mCursor;             //cursor in buffer of display (by bit)
  GFXfont* mFont;               //font to use for printing
  uint8_t* mBuffer;             //buffer of pixel data
  int mBufferSize;
  mrt_pbus_handle_t mHandle;    //platform dependent handle for parallel bus
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
mrt_status_t sed15xx_init(sed15xx_t* dev, mrt_pbus_handle_t handle,  int width, int height);

/**
  *@brief sends command over parallel bus to device
  *@param dev ptr to sed15xx device
  *@param cmd command to send
  */
void sed15xx_cmd(sed15xx_t* dev, uint8_t cmd);

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
mrt_status_t sed15xx_set_cursor(sed15xx_t* dev, uint16_t x, uint16_t y);

/**
  *@brief writes a single pixel in the buffer
  *@param dev ptr to device descriptor
  *@param x x coordinate
  *@param y y coordinate
  *@param val val for pixel
  *@return status of operation
  */
mrt_status_t sed15xx_pixel(sed15xx_t* dev,uint16_t x, uint16_t y, bool val );

/**
  *@brief writes an array of bytes to the buffer
  *@param dev ptr to device descriptor
  *@param data ptr to data being written
  *@param len number of bytes being written
  *@return status of operation
  */
mrt_status_t sed15xx_write_buffer(sed15xx_t* dev, uint8_t* data, int len, bool wrap);

/**
  *@brief Draws a bitmap to the buffer
  *@param dev ptr to device descriptor
  *@param x x coord to begin drawing at
  *@param y y coord to begin drawing at
  *@param bmp bitmap to draw
  *@return status of operation
  */
mrt_status_t sed15xx_draw_bmp(sed15xx_t* dev, uint16_t x, uint16_t y, GFXBmp* bmp);

/**
  *@brief Draws rendered text to the buffer
  *@param dev ptr to device descriptor
  *@param x x coord to begin drawing at
  *@param y y coord to begin drawing at
  *@param text text to be written
  *@return status of operation
  */
mrt_status_t sed15xx_print(sed15xx_t* dev, uint16_t x, uint16_t y, const char * text);

/**
  *@brief fill buffer with value
  *@param dev ptr to device
  *@param val value to write
  *@return status of operation
  */
mrt_status_t sed15xx_fill(sed15xx_t* dev, uint8_t val);
