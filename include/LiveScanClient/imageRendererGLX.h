//------------------------------------------------------------------------------
// <copyright file="ImageRenderer.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// Manages the drawing of image data

#pragma once
#include <stdint.h>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "viewer.h"

class ImageRendererGLX
{
public:
    /// <summary>
    /// Constructor
    /// </summary>
    ImageRendererGLX();

    /// <summary>
    /// Destructor
    /// </summary>
    virtual ~ImageRendererGLX();

    /// <summary>
    /// Set the window to draw to as well as the video format
    /// Implied bits per pixel is 32
    /// </summary>
    /// <param name="sourceWidth">width (in pixels) of image data to be drawn</param>
    /// <param name="sourceHeight">height (in pixels) of image data to be drawn</param>
    /// <param name="sourceStride">length (in bytes) of a single scanline</param>
    /// <returns>indicates success or failure</returns>
    bool Initialize(int sourceWidth, int sourceHeight, int sourceStride);

    /// <summary>
    /// Draws a 32 bit per pixel image of previously specified width, height, and stride to the associated hwnd
    /// </summary>
    /// <param name="pImage">image data in RGBX format</param>
    /// <param name="cbImage">size of image data in bytes</param>
    /// <returns>indicates success or failure</returns>
    bool Draw(uint8_t* pImage, unsigned long cbImage);

private:
    // Format information
    uint32_t                     m_sourceHeight;
    uint32_t                     m_sourceWidth;
    long                         m_sourceStride;
	Viewer viewer;

};
