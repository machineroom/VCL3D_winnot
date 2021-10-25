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


Freenect2Capture::Freenect2Capture()
{
	libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
	nDepthFrameWidth = 640;
	nDepthFrameHeight = 480;
	nColorFrameWidth = 640;
	nColorFrameHeight = 480;
	pColorRGBX = (RGB*)malloc(nColorFrameWidth*nColorFrameHeight*sizeof(*pColorRGBX));	//32 bits per RGBX sample
	pDepth = (UINT16*)malloc(nDepthFrameWidth*nDepthFrameHeight*sizeof(*pDepth));	//16 bits per depth sample
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

    //TODO do we need cuda/Gl pipelines? or a pipeline at all?
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

    listener->release(frames);

	return true;
}


// for info in MS kinect mappings see https://ed.ilogues.com/Tutorials/kinect2/kinect3.html
// We're using depth mode FREENECT_DEPTH_REGISTERED so the depth readings are aligned with the RGB data by freenect library

// Maps depth pixels (mm readings) to 3d coordinates (X,Y,Z in mm)
void Freenect2Capture::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	//MS kinect pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
	//MS Kinect CameraSpacePoint { float X, Y, Z; };
	uint16_t *depthp = pDepth;
	Point3f *out = pCameraSpacePoints;
	for (int y=0; y < nDepthFrameHeight; y++) {
		for (int x=0; x < nDepthFrameWidth; x++) {
			double wx, wy;
//			freenect_camera_to_world(f_dev, x, y, *depthp, &wx, &wy);
			out->X = (float)wx;
			out->Y = (float)wy;
			out->Z = (float)*depthp;
			depthp++;
		}	
	}	
}

// Maps depth pixels (???) to RGB coordinates
void Freenect2Capture::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
	//MS kinect pCoordinateMapper->MapDepthFrameToColorSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
	//MS Kinect ColorSpacePoint { float X, Y; };
	uint16_t *depthp = pDepth;
	Point2f	*out = pColorSpacePoints;
	for (int y=0; y < nDepthFrameHeight; y++) {
		for (int x=0; x < nDepthFrameWidth; x++) {
			out->X = (float)x;
			out->Y = (float)y;
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

