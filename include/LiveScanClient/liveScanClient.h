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

#include "viewer.h"

#include "socketCS.h"
#include "calibration.h"
#include "utils.h"
#ifdef KINECT
#include "kinectCapture.h"
#else
#include "freenect2Capture.h"
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
    LiveScanClient(std::string kinectSerial, std::string server, int port);
    ~LiveScanClient();

    int                     Run();

	bool m_bSocketThread;
	bool m_bCalibrate;
private:
	std::string m_sKinectSerial;
	std::string m_sServer;
	Calibration calibration;

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

    INT64 m_nLastCounter;
    double m_fFreq;
    INT64 m_nNextStatusTime;
    DWORD m_nFramesSinceUpdate;	

	Point3f* m_pCameraSpaceCoordinates;
	Point2f* m_pDepthCoordinatesOfColor;

    Viewer m_viewer;
	RGB* m_pDepthRGBX;

	void UpdateFrame();
    void ProcessColor();
	void ProcessDepth();
	void ShowRawDepth();

    bool SetStatusMessage(WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

	void HandleSocket();
	void SendFrame(vector<Point3s> vertices, vector<RGB> RGB);

	void SocketThreadFunction();

	void StoreFrame(Point3f *vertices, RGB *color);
	void ShowFPS();
	void ShowStatus();
	void VisualiseDepthMapping();
};

