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
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/frame_listener_impl.h>

#include "utils.h"


class Freenect2Capture : public ICapture
{
public:
	Freenect2Capture();
	~Freenect2Capture();

	bool Initialize();
	bool Initialize(std::string serial);	//serial is hex string serial number of kinect
	bool AcquireFrame();
	void MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints);
	void MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints);
	void MapDepthFrameToColorSpace(Point2f *pColorSpacePoints);
	void MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints);
	std::string Identifier();
private:
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	libfreenect2::SyncMultiFrameListener *listener = 0;
	libfreenect2::Registration *registration = 0;
	libfreenect2::Frame *undistorted;
	libfreenect2::Frame *registered;
};
