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
	pKinectSensor = NULL;
	pCoordinateMapper = NULL;
	pMultiSourceFrameReader = NULL;
}

FreenectCapture::~FreenectCapture()
{
	SafeRelease(pKinectSensor);
	SafeRelease(pCoordinateMapper);
	SafeRelease(pMultiSourceFrameReader);
}

bool FreenectCapture::Initialize()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&pKinectSensor);
	if (FAILED(hr))
	{
		bInitialized = false;
		return bInitialized;
	}

	if (pKinectSensor)
	{
		pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
		hr = pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color | 
				FrameSourceTypes::FrameSourceTypes_Depth | 
				FrameSourceTypes::FrameSourceTypes_Body |
				FrameSourceTypes::FrameSourceTypes_BodyIndex, 
				&pMultiSourceFrameReader);
		}
	}

	bInitialized = SUCCEEDED(hr);

	if (bInitialized)
	{
		std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
		bool bTemp;
		do
		{
			bTemp = AcquireFrame();
			
			std::chrono::duration<double> elapsedSeconds = std::chrono::system_clock::now() - start;
			if (elapsedSeconds.count() > 5.0)
			{
				bInitialized = false;
				break;
			}

		} while (!bTemp);
	}

	return bInitialized;
}

bool FreenectCapture::AcquireFrame()
{
	if (!bInitialized)
	{
		return false;
	}

	//Multi frame
	IMultiSourceFrame* pMultiFrame = NULL;
	HRESULT hr = pMultiSourceFrameReader->AcquireLatestFrame(&pMultiFrame);

	if (!SUCCEEDED(hr))
	{
		return false;
	}

	GetDepthFrame(pMultiFrame);
	GetColorFrame(pMultiFrame);
	GetBodyFrame(pMultiFrame);
	GetBodyIndexFrame(pMultiFrame);

	SafeRelease(pMultiFrame);

	return true;
}

void FreenectCapture::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void FreenectCapture::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	pCoordinateMapper->MapColorFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nColorFrameWidth * nColorFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void FreenectCapture::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
	pCoordinateMapper->MapDepthFrameToColorSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
}

void FreenectCapture::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
{
	pCoordinateMapper->MapColorFrameToDepthSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nColorFrameWidth * nColorFrameHeight, (DepthSpacePoint*)pDepthSpacePoints);;
}

