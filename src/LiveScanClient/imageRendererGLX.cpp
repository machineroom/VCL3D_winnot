//------------------------------------------------------------------------------
// <copyright file="ImageRenderer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "imageRendererGLX.h"
#include "viewer.h"

/// <summary>
/// Constructor
/// </summary>
ImageRendererGLX::ImageRendererGLX() : 
    m_sourceWidth(0),
    m_sourceHeight(0),
    m_sourceStride(0)
{
    viewer.initialize();
}

/// <summary>
/// Destructor
/// </summary>
ImageRendererGLX::~ImageRendererGLX()
{
}


/// <summary>
/// Set the window to draw to as well as the video format
/// Implied bits per pixel is 32
/// </summary>
/// <param name="hWnd">window to draw to</param>
/// <param name="pD2DFactory">already created D2D factory object</param>
/// <param name="sourceWidth">width (in pixels) of image data to be drawn</param>
/// <param name="sourceHeight">height (in pixels) of image data to be drawn</param>
/// <param name="sourceStride">length (in bytes) of a single scanline</param>
/// <returns>indicates success or failure</returns>
bool ImageRendererGLX::Initialize(int sourceWidth, int sourceHeight, int sourceStride)
{
    // Get the frame size
    m_sourceWidth  = sourceWidth;
    m_sourceHeight = sourceHeight;
    m_sourceStride = sourceStride;

    return true;
}

/// <summary>
/// Draws a 32 bit per pixel image of previously specified width, height, and stride to the associated hwnd
/// </summary>
/// <param name="pImage">image data in RGBX format</param>
/// <param name="cbImage">size of image data in bytes</param>
/// <param name="vBodies">vector of bodies to draw</param>
/// <returns>indicates success or failure</returns>
bool ImageRendererGLX::Draw(uint8_t* pImage, unsigned long cbImage)
{
	viewer.render_colour(pImage, m_sourceWidth, m_sourceHeight, 4);
}


