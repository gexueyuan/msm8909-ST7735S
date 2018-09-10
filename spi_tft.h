/*****************************************************************************
 Copyright(C) Tendyron Corporation
 All rights reserved.
 
 @file   : spi_tft.h
 @brief  : spi_tft header
 @author : gexueyuan
 @history:
           2018-9-3    gexueyuan    Created file
           ...
******************************************************************************/


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
extern "C" {

extern void spi_tft_init(void); 
extern void dislpay_tft_init(void); 
extern int display_image(unsigned char*  image_data,int len);

}

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

