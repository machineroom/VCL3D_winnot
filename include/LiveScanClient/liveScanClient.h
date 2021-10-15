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

//#include "resource.h"
//#include "ImageRenderer.h"
#include "socketCS.h"
#include "calibration.h"
#include "utils.h"
#ifdef KINECT
#include "kinectCapture.h"
#else
#include "freenectCapture.h"
#endif

#include "frameFileWriterReader.h"
#include <thread>
#include <mutex>

#include <stdint.h>
typedef uint32_t DWORD;
typedef int64_t INT64;
typedef char WCHAR;
typedef uint16_t USHORT;
#define _In_z_


class LiveScanClient
{
public:
    LiveScanClient();
    ~LiveScanClient();


    int                     Run();

	bool m_bSocketThread;
private:
	Calibration calibration;

	bool m_bCalibrate;
	bool m_bFilter;
	bool m_bStreamOnlyBodies;

	ICapture *pCapture;

	int m_nFilterNeighbors;
	float m_fFilterThreshold;

	bool m_bCaptureFrame;
	bool m_bConnected;
	bool m_bConfirmCaptured;
	bool m_bConfirmCalibrated;
	bool m_bShowDepth;
	bool m_bFrameCompression;
	int m_iCompressionLevel;

	FrameFileWriterReader m_framesFileWriterReader;


	SocketClient *m_pClientSocket;
	std::vector<float> m_vBounds;

	std::vector<Point3s> m_vLastFrameVertices;
	std::vector<RGB> m_vLastFrameRGB;
#ifdef KINECT
	std::vector<Body> m_vLastFrameBody;
#endif

    INT64 m_nLastCounter;
    double m_fFreq;
    INT64 m_nNextStatusTime;
    DWORD m_nFramesSinceUpdate;	

	Point3f* m_pCameraSpaceCoordinates;
	Point2f* m_pColorCoordinatesOfDepth;
	Point2f* m_pDepthCoordinatesOfColor;

    // TODO port off Direct2D
    //ImageRenderer* m_pDrawColor;
    //ID2D1Factory* m_pD2DFactory;
	RGB* m_pDepthRGBX;

	void UpdateFrame();
    void ProcessColor(RGB* pBuffer, int nWidth, int nHeight);
	void ProcessDepth(const UINT16* pBuffer, int nHeight, int nWidth);

    bool SetStatusMessage(WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

	void HandleSocket();
#ifdef KINECT
	void SendFrame(vector<Point3s> vertices, vector<RGB> RGB, vector<Body> body);
#else
	void SendFrame(vector<Point3s> vertices, vector<RGB> RGB);
#endif

	void SocketThreadFunction();

#ifdef KINECT
	void StoreFrame(Point3f *vertices, Point2f *mapping, RGB *color, vector<Body> &bodies, BYTE* bodyIndex);
#else
	void StoreFrame(Point3f *vertices, Point2f *mapping, RGB *color);
#endif
	void ShowFPS();
};

