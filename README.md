# ERCxxLcd #

>Datasheet: https://www.mikrocontroller.net/attachment/10245/SED1565.pdf
<br/>
>Partnumber: Unkown

> Requires: Modules/Utilities/GFX/MonoGfx

This module is a driver for the ERC monochromatic lcd displays driven by the SED15xx driver IC

This module handles mapping the pixels to the device pages/rows/columns in a logical order. So byte 0 of the buffer represents the first 8 pixels on the first row (top) of the display, and continues until it wraps at the end of the row


 The lcd buffer stores pixel data 'sideways'
   0[4] = byte 0, bit 4

 lcd ram:

 >0[0] 1[0] 2[0] 3[0] 4[0] 5[0] 6[0] 7[0] .....<br/>
 >0[1] 1[1] 2[1] 3[1] 4[1] 5[1] 6[1] 7[1] .....<br/>
 >0[2] 1[2] 2[2] 3[2] 4[2] 5[2] 6[2] 7[2] .....<br/>
 >0[3] 1[3] 2[3] 3[3] 4[3] 5[3] 6[3] 7[3] .....<br/>
 >0[4] 1[4] 2[4] 3[4] 4[4] 5[4] 6[4] 7[4] .....<br/>
 >0[5] 1[5] 2[5] 3[5] 4[5] 5[5] 6[5] 7[5] .....<br/>
 >0[6] 1[6] 2[6] 3[6] 4[6] 5[6] 6[6] 7[6] .....<br/>
 >0[7] 1[7] 2[7] 3[7] 4[7] 5[7] 6[7] 7[7] .....<br/>

 local buffer:

 >0[7] 0[6] 0[5] 0[4] 0[3] 0[2] 0[1] 0[0] ,<br/>
 >1[7] 1[6] 1[5] 1[4] 1[3] 1[2] 1[1] 1[0] ,<br/>
 >2[7] 2[6] 2[5] 2[4] 2[3] 2[2] 2[1] 2[0] ,<br/>
 >3[7] 3[6] 3[5] 3[4] 3[3] 3[2] 3[1] 3[0] ,<br/>
 >4[7] 4[6] 4[5] 4[4] 4[3] 4[2] 4[1] 4[0] ,<br/>
 >5[7] 5[6] 5[5] 5[4] 5[3] 5[2] 5[1] 5[0] ,<br/>
 >6[7] 6[6] 6[5] 6[4] 6[3] 6[2] 6[1] 6[0] ,<br/>
 >7[7] 7[6] 7[5] 7[4] 7[3] 7[2] 7[1] 7[0] ,<br/>

 So before writing to the device, we take a 'block' which is 8 pixels by 8 pixels,
 and rotate it to match the lcd ram
