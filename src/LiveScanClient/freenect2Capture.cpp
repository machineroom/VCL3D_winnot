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
	std::cout << "freenect2 close" << std::endl;
	dev->stop();
	dev->close();
}
bool Freenect2Capture::Initialize()
{
	return Initialize("");
}

bool Freenect2Capture::Initialize(std::string serial)
{
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
	if (serial == "") {
		serial = freenect2.getDefaultDeviceSerialNumber();
	}
	
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


	/// [start all streams (rgb & depth)]
    if (!dev->start()) {
		std::cout << "failure starting device!" << std::endl;
		return false;
	}
	
	registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

	bInitialized = true;

	return bInitialized;
}

std::string Freenect2Capture::Identifier() {
	return "s " + dev->getSerialNumber() + " f " + dev->getFirmwareVersion();
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

	/** Map color images onto depth images
	* @param rgb Color image (1920x1080 BGRX)
	* @param depth Depth image (512x424 float mm)
	* @param[out] undistorted Undistorted depth image (JW note 512x424 float mm)
	* @param[out] registered Color image for the depth image (JW note 512x424 ARGB)
	* @param enable_filter Filter out pixels not visible to both cameras.
	* @param[out] bigdepth If not `NULL`, return mapping of depth onto colors (c float). **1082** not 1080, with a blank top and bottom row.
	* @param[out] color_depth_map Index of mapped color pixel for each depth pixel (512x424).
	*/
  	//void apply(const Frame* rgb, const Frame* depth, Frame* undistorted, Frame* registered, const bool enable_filter = true, Frame* bigdepth = 0, int* color_depth_map = 0) const;
	registration->apply(rgb, depth, undistorted, registered);

	//copy frame data into our buffers so livescan can process it
	//TODO check dimensions won't overwrite our buffer
	memcpy (pColorRGBX, rgb->data, rgb->width * rgb->height * rgb->bytes_per_pixel);

	//livescan3d expects depth as shorts. freenect2 gives as floats
	long pixelCounter = 0;
	float *fdata = (float *)depth->data;
	for (int y=0; y < depth->height; y++) {
		for (int x=0; x < depth->width; x++) {
			pDepth[pixelCounter] = (UINT16)fdata[pixelCounter];
			pixelCounter++;
		}
	}


	//now that we've taken a copy of the data the freenect2 buffers can be released
    listener->release(frames);

	return true;
}


// TODO freenect2 provides the mapping functions that livescan needs so this can be heavily optimised - but the current iCapture interface isn't flexible enough

// for info in MS kinect mappings see https://ed.ilogues.com/Tutorials/kinect2/kinect3.html
// mapping between depth pixel coordinates and 3D point coordinates
// pCameraSpacePoints 512*424 {X,Y,Z} in cartesian coordinate space
void Freenect2Capture::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	//MS kinect pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
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
			if (std::isnan(x) || std::isinf(x) || std::isnan(y) || std::isinf(y) || std::isnan(z) || std::isinf(z)) {
				out->X = 0.f;
				out->Y = 0.f;
				out->Z = 0.f;
			} else {
				// convert to mm as expected by livescan
				out->X = x*1000.0f;
				out->Y = y*1000.0f;
				out->Z = z*1000.0f;
			}
			out++;
		}	
	}	
}

// mapping between depth pixel coordinates to the corresponding pixel in the color image
// pColorSpacePoints 512*424 {X,Y,Z}
void Freenect2Capture::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
	//MS kinect pCoordinateMapper->MapDepthFrameToColorSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
	//MS Kinect ColorSpacePoint = Point2f = { float X, Y }
	Point2f *out = pColorSpacePoints;
	for (int row=0; row < nDepthFrameHeight; row++) {
		for (int col=0; col < nDepthFrameWidth; col++) {
			float cx, cy;
		    /** Undistort and register a single depth point to color camera.
		     * @param dx Distorted depth coordinate x (pixel)
  		     * @param dy Distorted depth coordinate y (pixel)
	  	     * @param dz Depth value (millimeter)
		     * @param[out] cx Undistorted color coordinate x (normalized)
		     * @param[out] cy Undistorted color coordinate y (normalized)
  		  	 void apply(int dx, int dy, float dz, float& cx, float &cy) const;
		     */
  		  	//TODO is use of undistorted correct here? since apply expects distored coords
		  	registration->apply (col, row, undistorted->data[row*nDepthFrameWidth+col], cx, cy);
		  	if (std::isnan(cx) || std::isinf(cx) || std::isnan(cy) || std::isinf(cy)) {
		  		out->X = 0.f;
				out->Y = 0.f;
    	    } else {
		  		out->X = cx;
		  		out->Y = cy;
			}	
	  		out++;
	  	}
	}	
}

void Freenect2Capture::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	// TODO maybe? like seriously I have no idea.
	Point3f *out = pCameraSpacePoints;
	for (int row=0; row < nDepthFrameHeight; row++) {
		for (int col=0; col < nDepthFrameWidth; col++) {
			/** Construct a 3-D point with color in a point cloud.
			   * @param undistorted Undistorted depth frame from apply().
			   * @param registered Registered color frame from apply().
			   * @param r Row (y) index in depth image.
			   * @param c Column (x) index in depth image.
			   * @param[out] x X coordinate of the 3-D point (meter).
			   * @param[out] y Y coordinate of the 3-D point (meter).
			   * @param[out] z Z coordinate of the 3-D point (meter).
			   * @param[out] rgb Color of the 3-D point (BGRX). To unpack the data, use
			   *
			   *     const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
			   *     uint8_t b = p[0];
			   *     uint8_t g = p[1];
			   *     uint8_t r = p[2];
			   */
			//void getPointXYZRGB (const Frame* undistorted, const Frame* registered, int r, int c, float& x, float& y, float& z, float& rgb) const; 		  
			float x, y, z, rgb;
			registration->getPointXYZRGB (undistorted, registered, row, col, x, y, z, rgb);
			// convert metres to mm as expected by livescan
			out->X = x*1000.0f;
			out->Y = y*1000.0f;
			out->Z = z*1000.0f;
			// RGB data is ignored - livescan only wants the coordinates
			out++;
		}	
	}	
}

// pDepthSpacePoints is colour resolution (1920x1080) and gets filled with XY coord of pixel in depth image corresponding to pixel in colour image
//XXX ths only used by depth preview in livescan so less important to fix
void Freenect2Capture::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
{
	// TODO maybe? like seriously I have no idea.
	Point2f *out = pDepthSpacePoints;
	for (int row=0; row < nDepthFrameHeight; row++) {
		for (int col=0; col < nDepthFrameWidth; col++) {
			float x, y, z;
			registration->getPointXYZ (registered, row, col, x, y, z);
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
			out++;
		}	
	}	
}

