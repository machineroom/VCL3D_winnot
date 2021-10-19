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
#include "freenectCapture.h"
#include <chrono>

FreenectCapture::FreenectCapture()
{
}

FreenectCapture::~FreenectCapture()
{
}

bool FreenectCapture::Initialize()
{
	printf ("freenect init\n");
	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return false;
	}
	/* TODO support other devices */
	if (freenect_open_device(f_ctx, &f_dev, 0/*device_number*/) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return false;
	}
	//set Kinect LED green to say we're awake
	freenect_set_led(f_dev, LED_GREEN);
	return true;
}

bool FreenectCapture::AcquireFrame()
{
}

void FreenectCapture::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
}

void FreenectCapture::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
}

void FreenectCapture::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
}

void FreenectCapture::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
{
}

