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

#include "iCapture.h"
#include <libfreenect.h>
#include "utils.h"


class FreenectCapture : public ICapture
{
public:
	FreenectCapture();
	~FreenectCapture();

	bool Initialize();
	bool AcquireFrame();
	void MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints);
	void MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints);
	void MapDepthFrameToColorSpace(Point2f *pColorSpacePoints);
	void MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints);
	//used in C callback so must be public. Yuk.
	freenect_context *f_ctx;
	freenect_device *f_dev;
	uint8_t *f_video_buffer;
	uint16_t *f_depth_buffer;
	pthread_mutex_t f_video_mutex;
	pthread_cond_t f_video_cond;
	pthread_mutex_t f_depth_mutex;
	pthread_cond_t f_depth_cond;
	
	/*ICoordinateMapper* pCoordinateMapper;
	IKinectSensor* pKinectSensor;
	IMultiSourceFrameReader* pMultiSourceFrameReader;

	void GetDepthFrame(IMultiSourceFrame* pMultiFrame);
	void GetColorFrame(IMultiSourceFrame* pMultiFrame);
	void GetBodyFrame(IMultiSourceFrame* pMultiFrame);
	void GetBodyIndexFrame(IMultiSourceFrame* pMultiFrame);*/
};
