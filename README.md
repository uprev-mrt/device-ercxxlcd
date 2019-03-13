# ERCxxLcd #

>Datasheet: https://www.mikrocontroller.net/attachment/10245/SED1565.pdf
>Partnumber: Unkown

This module is a driver for the ERC monochromatic lcd displays driven by the SED15xx driver IC

This module handles mapping the pixels to the device pages/rows/columns in a logical order. So byte 0 of the buffer represents the first 8 pixels on the first row (top) of the display, and continues until it wraps at the end of the row
