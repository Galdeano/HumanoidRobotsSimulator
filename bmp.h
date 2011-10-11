#ifndef H_bmp
#define H_bmp

/** \file bmp.h
 *  \brief read and write bmp images
 *
 *  Distributed with Xplanet.
 *  Copyright (C) 2002 Hari Nair <hari@alumni.caltech.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

/**
 * \struct BMPHeader bmp.h
 * \brief BMPHeader contain header for a bitmap file
 */
struct BMPHeader
{
    char bfType[2];       /*!< "BM" */
    int bfSize;           /*!< Size of file in bytes */
    int bfReserved;       /*!< set to 0 */
    int bfOffBits;        /*!< Byte offset to actual bitmap data (= 54) */
    int biSize;           /*!< Size of BITMAPINFOHEADER, in bytes (= 40) */
    int biWidth;          /*!< Width of image, in pixels */
    int biHeight;         /*!< Height of images, in pixels */
    short biPlanes;       /*!< Number of planes in target device (set to 1) */
    short biBitCount;     /*!< Bits per pixel (24 in this case) */
    int biCompression;    /*!< Type of compression (0 if no compression) */
    int biSizeImage;      /*!< Image size, in bytes (0 if no compression) */
    int biXPelsPerMeter;  /*!< Resolution in pixels/meter of display device */
    int biYPelsPerMeter;  /*!< Resolution in pixels/meter of display device */
    int biClrUsed;        /*!< Number of colors in the color table (if 0, use
                             maximum allowed by biBitCount) */
    int biClrImportant;   /*!< Number of important colors.  If 0, all colors
                             are important */
};


/**
 * \fn int read_bmp(const char *filename, int *width, int *height, unsigned char *rgb)
 * \brief not implemented
 *
 * \param filename Name of bmp file
 * \param width width of image
  * \param height height of image
 * \param rgb buffer of the image to save
 */
int read_bmp(const char *filename, int *width, int *height, unsigned char *rgb);

/**
 * \fn int write_bmp(const char *filename, int width, int height, char *rgb)
 * \brief Write images in a bmp file
 *
 * \param filename Name of bmp file
 * \param width width of image
  * \param height height of image
 * \param rgb buffer of the image to save
 */
int write_bmp(const char *filename, int width, int height, char *rgb);

#endif
