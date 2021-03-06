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
#pragma once

#include "utils.h"
#ifdef KINECT
#include "Kinect.h"
#endif
#include <stdint.h>
typedef uint16_t UINT16;

#ifdef KINECT
struct Body
{
	Body()
	{
		bTracked = false;
		vJoints.resize(JointType_Count);
		vJointsInColorSpace.resize(JointType_Count);
	}
	bool bTracked;
	std::vector<Joint> vJoints;
	std::vector<Point2f> vJointsInColorSpace;
};
#else
struct Body
{
};
#endif

class ICapture
{
public:
	ICapture();
	virtual ~ICapture();

	virtual bool Initialize() = 0;
	virtual bool Initialize(std::string serial) = 0;
	virtual bool AcquireFrame() = 0;
	virtual void MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints) = 0;
	virtual void MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints) = 0;
	virtual void MapDepthFrameToColorSpace(Point2f *pColorSpacePoints) = 0;
	virtual void MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints) = 0;
	virtual std::string Identifier() = 0;	// return some unique identifier of this device for diagnostics (e.g. serial #)

	bool bInitialized;

	int nColorFrameHeight, nColorFrameWidth;
	int nDepthFrameHeight, nDepthFrameWidth;

	UINT16 *pDepth;			// raw mm readings from depth sensor
	BYTE *pBodyIndex;
	RGB *pColorRGBX;		// raw RGB pixels from colour sensor
	int *depthToColourMap;	// index of color pixel for each depth pixel (same dimensions as pDepth)
	std::vector<Body> vBodies;
};
