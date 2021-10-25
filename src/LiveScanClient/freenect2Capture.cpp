//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#include "freenect2Capture.h"
#include <libfreenect_registration.h>
#include <chrono>
#include <cassert>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cmath>


Freenect2Capture::Freenect2Capture()
{
	libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
	//These are just known facts about the kinect2. From freenect2/frame_listener.hpp:
    //Color = 1, ///< 1920x1080. BGRX or RGBX.
    //Depth = 4  ///< 512x424 float, unit: millimeter. Non-positive, NaN, and infinity are invalid or missing data.
	nDepthFrameWidth = 512;
	nDepthFrameHeight = 424;
	nColorFrameWidth = 1920;
	nColorFrameHeight = 1080;
	// The buffers accessed by livescanclient
	pColorRGBX = (RGB*)malloc(nColorFrameWidth*nColorFrameHeight*sizeof(*pColorRGBX));	//32 bits per RGBX sample
	pDepth = (UINT16*)malloc(nDepthFrameWidth*nDepthFrameHeight*sizeof(*pDepth));	//16 bits per depth sample
	undistorted = new libfreenect2::Frame (nDepthFrameWidth, nDepthFrameHeight, sizeof(float));
	registered = new libfreenect2::Frame (nDepthFrameWidth, nDepthFrameHeight, sizeof(float));
}

Freenect2Capture::~Freenect2Capture()
{
	dev->stop();
	dev->close();
}

bool Freenect2Capture::Initialize()
{
	std::string serial = "";
		
	std::cout << "freenect2 init" << std::endl;

    //TODO do we need to override the default pipeline? if so do it now.
    /*pipeline = new libfreenect2::CpuPacketPipeline();
	if(pipeline == 0)
	{
		std::cout << "failure creating CPU pipeline!" << std::endl;
		return false;
	}*/

	/// [discovery]
	if(freenect2.enumerateDevices() == 0)
	{
	    std::cout << "no device connected!" << std::endl;
	    return false;
	}

	//todo support >1 kinect
	serial = freenect2.getDefaultDeviceSerialNumber();
	
	if(pipeline)
	{
		/// [open]
		dev = freenect2.openDevice(serial, pipeline);
	}
	else
	{
		dev = freenect2.openDevice(serial);
	}

	if(dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return false;
	}

	/// [listeners]
	int types = libfreenect2::Frame::Color | libfreenect2::Frame::Depth;
	listener = new libfreenect2::SyncMultiFrameListener (types);

	dev->setColorFrameListener (listener);
	dev->setIrAndDepthFrameListener(listener);


	/// [start]
    if (!dev->start()) {
		std::cout << "failure starting device!" << std::endl;
		return false;
	}
	
	registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

	bInitialized = true;

	return bInitialized;
}

bool Freenect2Capture::AcquireFrame()
{
	libfreenect2::FrameMap frames;
    if (!listener->waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
		std::cout << "timeout!" << std::endl;
		return false;
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    std::cout << "frame!" << std::endl;

	//copy frame data into our buffers so livescan can process it
	//TODO check dimensions won't overwrite our buffer
	memcpy (pColorRGBX, rgb->data, rgb->width * rgb->height * rgb->bytes_per_pixel);

	//livescan3d expects depth as shorts. freenect2 gives as floats
	long pixelCounter = 0;
	for (int y=0; y < depth->height; y++) {
		for (int x=0; x < depth->width; x++) {
			pDepth[pixelCounter] = depth->data[pixelCounter];
			pixelCounter++;
		}
	}
	registration->apply(rgb, depth, undistorted, registered);
    listener->release(frames);

	return true;
}


// for info in MS kinect mappings see https://ed.ilogues.com/Tutorials/kinect2/kinect3.html
// mapping between depth pixel coordinates and 3D point coordinates
void Freenect2Capture::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	//MS kinect pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
	//MS Kinect CameraSpacePoint { float X, Y, Z; };
	Point3f *out = pCameraSpacePoints;
	for (int row=0; row < nDepthFrameHeight; row++) {
		for (int col=0; col < nDepthFrameWidth; col++) {
			float x, y, z;
 		  /* freenect2 getPointXYZ doc states:
 		    Construct a 3-D point in a point cloud.
		   * @param undistorted Undistorted depth frame from apply().
		   * @param r Row (y) index in depth image.
		   * @param c Column (x) index in depth image.
		   * @param[out] x X coordinate of the 3-D point (meter).
		   * @param[out] y Y coordinate of the 3-D point (meter).
		   * @param[out] z Z coordinate of the 3-D point (meter).
		   */
			registration->getPointXYZ (undistorted, row, col, x, y, z);
			// convert metres to mm as expected by livescan
			out->X = x*1000.0f;
			out->Y = y*1000.0f;
			out->Z = z*1000.0f;
		}	
	}	
}

// mapping between depth pixel coordinates to the corresponding pixel in the color image
// TODO freenect2 provides the mapping functions that livescan needs so this can be heavily optimised - but the current iCapture interface isn't flexible enough
void Freenect2Capture::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
	//MS kinect pCoordinateMapper->MapDepthFrameToColorSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
	//MS Kinect ColorSpacePoint { float X, Y; };
	Point2f *out = pColorSpacePoints;
	for (int row=0; row < nDepthFrameHeight; row++) {
		for (int col=0; col < nDepthFrameWidth; col++) {
			float x, y, z;
			registration->getPointXYZ (undistorted, row, col, x, y, z);
 		  	/* freenect2 getPointXYZ doc states:
			/** Undistort and register a single depth point to color camera.
			   * @param dx Distorted depth coordinate x (pixel)
			   * @param dy Distorted depth coordinate y (pixel)
			   * @param dz Depth value (millimeter)
			   * @param[out] cx Undistorted color coordinate x (normalized)
			   * @param[out] cy Undistorted color coordinate y (normalized)
			   */
			   
			if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
				out->X = NAN;
				out->Y = NAN;
			} else {
	  			float cx, cy;	//color coords
				registration->apply (x,y,z,cx,cy);
				out->X = cx;
				out->Y = cy;
			}
		}	
	}	
}

void Freenect2Capture::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	bool implemented=false;
	assert (implemented);
}

void Freenect2Capture::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
{
	bool implemented=false;
	assert (implemented);
}

