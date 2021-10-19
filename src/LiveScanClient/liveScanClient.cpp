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
//#include "resource.h"
#include "liveScanClient.h"
#include "filter.h"
#include <chrono>
#include <fstream>
#include <zstd.h>

std::mutex m_mSocketThreadMutex;

int main (int argc, char **argv)
{
    LiveScanClient application;
    application.Run();
}

LiveScanClient::LiveScanClient() :
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
	// TODO port off Direct2D
    //m_pD2DFactory(NULL),
    //m_pDrawColor(NULL),
	m_pDepthRGBX(NULL),
	m_pCameraSpaceCoordinates(NULL),
	m_pColorCoordinatesOfDepth(NULL),
	m_pDepthCoordinatesOfColor(NULL),
	m_bCalibrate(false),
	m_bFilter(false),
#ifdef KINECT
	m_bStreamOnlyBodies(false),
#endif
	m_bCaptureFrame(false),
	m_bConnected(false),
	m_bConfirmCaptured(false),
	m_bConfirmCalibrated(false),
	m_bShowDepth(false),
	m_bSocketThread(true),
	m_bFrameCompression(true),
	m_iCompressionLevel(2),
	m_pClientSocket(NULL),
	m_nFilterNeighbors(10),
	m_fFilterThreshold(0.01f)
{
	#ifdef KINECT
	pCapture = new KinectCapture();
	#else
	pCapture = new FreenectCapture();
	#endif
	pCapture->Initialize();

    /*TODO port from windows
	LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }*/

	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(0.5);
	m_vBounds.push_back(0.5);
	m_vBounds.push_back(0.5);

	calibration.LoadCalibration();
}
  
LiveScanClient::~LiveScanClient()
{
    // clean up Direct2D renderer
	// TODO port off Direct2D
    /*if (m_pDrawColor)
    {
        delete m_pDrawColor;
        m_pDrawColor = NULL;
    }*/

	if (pCapture)
	{
		delete pCapture;
		pCapture = NULL;
	}

	if (m_pDepthRGBX)
	{
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
	}

	if (m_pCameraSpaceCoordinates)
	{
		delete[] m_pCameraSpaceCoordinates;
		m_pCameraSpaceCoordinates = NULL;
	}

	if (m_pColorCoordinatesOfDepth)
	{
		delete[] m_pColorCoordinatesOfDepth;
		m_pColorCoordinatesOfDepth = NULL;
	}

	if (m_pDepthCoordinatesOfColor)
	{
		delete[] m_pDepthCoordinatesOfColor;
		m_pDepthCoordinatesOfColor = NULL;
	}

	if (m_pClientSocket)
	{
		delete m_pClientSocket;
		m_pClientSocket = NULL;
	}
    // clean up Direct2D
    //SafeRelease(m_pD2DFactory);
}

int LiveScanClient::Run()
{
	std::thread t1(&LiveScanClient::SocketThreadFunction, this);
    // Main message loop
    while (true)
    {
		//HandleSocket();
		UpdateFrame();
    }

	m_bSocketThread = false;
	t1.join();
}



void LiveScanClient::UpdateFrame()
{
	if (!pCapture->bInitialized)
	{
		return;
	}

	bool bNewFrameAcquired = pCapture->AcquireFrame();

	if (!bNewFrameAcquired)
		return;

	pCapture->MapDepthFrameToCameraSpace(m_pCameraSpaceCoordinates);
	pCapture->MapDepthFrameToColorSpace(m_pColorCoordinatesOfDepth);
	{
		std::lock_guard<std::mutex> lock(m_mSocketThreadMutex);
#ifdef KINECT
		StoreFrame(m_pCameraSpaceCoordinates, m_pColorCoordinatesOfDepth, pCapture->pColorRGBX, pCapture->vBodies, pCapture->pBodyIndex);
#else
		StoreFrame(m_pCameraSpaceCoordinates, m_pColorCoordinatesOfDepth, pCapture->pColorRGBX);
#endif

		if (m_bCaptureFrame)
		{
			m_framesFileWriterReader.writeFrame(m_vLastFrameVertices, m_vLastFrameRGB);
			m_bConfirmCaptured = true;
			m_bCaptureFrame = false;
		}
	}

	if (m_bCalibrate)
	{		
		std::lock_guard<std::mutex> lock(m_mSocketThreadMutex);
		Point3f *pCameraCoordinates = new Point3f[pCapture->nColorFrameWidth * pCapture->nColorFrameHeight];
		pCapture->MapColorFrameToCameraSpace(pCameraCoordinates);

		bool res = calibration.Calibrate(pCapture->pColorRGBX, pCameraCoordinates, pCapture->nColorFrameWidth, pCapture->nColorFrameHeight);

		delete[] pCameraCoordinates;

		if (res)
		{
			m_bConfirmCalibrated = true;
			m_bCalibrate = false;
		}
	}

	if (!m_bShowDepth)
		ProcessColor(pCapture->pColorRGBX, pCapture->nColorFrameWidth, pCapture->nColorFrameHeight);
	else
		ProcessDepth(pCapture->pDepth, pCapture->nDepthFrameWidth, pCapture->nDepthFrameHeight);

	ShowFPS();
}


void LiveScanClient::ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight)
{
	// Make sure we've received valid data
	if (m_pDepthRGBX && m_pDepthCoordinatesOfColor && pBuffer && (nWidth == pCapture->nDepthFrameWidth) && (nHeight == pCapture->nDepthFrameHeight))
	{
		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		pCapture->MapColorFrameToDepthSpace(m_pDepthCoordinatesOfColor);

		for (int i = 0; i < pCapture->nColorFrameWidth * pCapture->nColorFrameHeight; i++)
		{
			Point2f depthPoint = m_pDepthCoordinatesOfColor[i];
			BYTE intensity = 0;
			
			if (depthPoint.X >= 0 && depthPoint.Y >= 0)
			{
				int depthIdx = (int)(depthPoint.X + depthPoint.Y * pCapture->nDepthFrameWidth);
				USHORT depth = pBuffer[depthIdx];
				intensity = static_cast<BYTE>(depth % 256);
			}

			m_pDepthRGBX[i].rgbRed = intensity;
			m_pDepthRGBX[i].rgbGreen = intensity;
			m_pDepthRGBX[i].rgbBlue = intensity;
		}

		// Draw the data with Direct2D
		// TODO port off Direct2D
		//m_pDrawColor->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), pCapture->nColorFrameWidth * pCapture->nColorFrameHeight * sizeof(RGB), pCapture->vBodies);
	}
}

void LiveScanClient::ProcessColor(RGB* pBuffer, int nWidth, int nHeight) 
{
    // Make sure we've received valid data
	if (pBuffer && (nWidth == pCapture->nColorFrameWidth) && (nHeight == pCapture->nColorFrameHeight))
    {
        // Draw the data with Direct2D
		// TODO port off Direct2D
		//m_pDrawColor->Draw(reinterpret_cast<BYTE*>(pBuffer), pCapture->nColorFrameWidth * pCapture->nColorFrameHeight * sizeof(RGB), pCapture->vBodies);
    }
}

	/* TODO port from windows
bool LiveScanClient::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}
*/

void LiveScanClient::SocketThreadFunction()
{
	while (m_bSocketThread)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		HandleSocket();
	}
}

void LiveScanClient::HandleSocket()
{
	char byteToSend;
	std::lock_guard<std::mutex> lock(m_mSocketThreadMutex);

	if (!m_bConnected)
	{
		return;
	}

	string received = m_pClientSocket->ReceiveBytes();
	for (unsigned int i = 0; i < received.length(); i++)
	{
		//capture a frame
		if (received[i] == MSG_CAPTURE_FRAME)
			m_bCaptureFrame = true;
		//calibrate
		else if (received[i] == MSG_CALIBRATE)
			m_bCalibrate = true;
		//receive settings
		//TODO: what if packet is split?
		else if (received[i] == MSG_RECEIVE_SETTINGS)
		{
			vector<float> bounds(6);
			i++;
			int nBytes = *(int*)(received.c_str() + i);
			i += sizeof(int);

			for (int j = 0; j < 6; j++)
			{
				bounds[j] = *(float*)(received.c_str() + i);
				i += sizeof(float);
			}
				
			m_bFilter = (received[i]!=0);
			i++;

			m_nFilterNeighbors = *(int*)(received.c_str() + i);
			i += sizeof(int);

			m_fFilterThreshold = *(float*)(received.c_str() + i);
			i += sizeof(float);

			m_vBounds = bounds;

			int nMarkers = *(int*)(received.c_str() + i);
			i += sizeof(int);

			calibration.markerPoses.resize(nMarkers);

			for (int j = 0; j < nMarkers; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					for (int l = 0; l < 3; l++)
					{
						calibration.markerPoses[j].R[k][l] = *(float*)(received.c_str() + i);
						i += sizeof(float);
					}
				}

				for (int k = 0; k < 3; k++)
				{
					calibration.markerPoses[j].t[k] = *(float*)(received.c_str() + i);
					i += sizeof(float);
				}

				calibration.markerPoses[j].markerId = *(int*)(received.c_str() + i);
				i += sizeof(int);
			}

			m_bStreamOnlyBodies = (received[i] != 0);
			i += 1;

			m_iCompressionLevel = *(int*)(received.c_str() + i);
			i += sizeof(int);
			if (m_iCompressionLevel > 0)
				m_bFrameCompression = true;
			else
				m_bFrameCompression = false;

			//so that we do not lose the next character in the stream
			i--;
		}
		//send stored frame
		else if (received[i] == MSG_REQUEST_STORED_FRAME)
		{
			byteToSend = MSG_STORED_FRAME;
			m_pClientSocket->SendBytes(&byteToSend, 1);

			vector<Point3s> points;
			vector<RGB> colors; 
			bool res = m_framesFileWriterReader.readFrame(points, colors);
			if (res == false)
			{
				int size = -1;
				m_pClientSocket->SendBytes((char*)&size, 4);
			} else
#ifdef KINECT
				SendFrame(points, colors, m_vLastFrameBody);
#else
				SendFrame(points, colors);
#endif
		}
		//send last frame
		else if (received[i] == MSG_REQUEST_LAST_FRAME)
		{
			byteToSend = MSG_LAST_FRAME;
			m_pClientSocket->SendBytes(&byteToSend, 1);

#ifdef KINECT
			SendFrame(m_vLastFrameVertices, m_vLastFrameRGB, m_vLastFrameBody);
#else
			SendFrame(m_vLastFrameVertices, m_vLastFrameRGB);
#endif
		}
		//receive calibration data
		else if (received[i] == MSG_RECEIVE_CALIBRATION)
		{
			i++;
			for (int j = 0; j < 3; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					calibration.worldR[j][k] = *(float*)(received.c_str() + i);
					i += sizeof(float);
				}
			}
			for (int j = 0; j < 3; j++)
			{
				calibration.worldT[j] = *(float*)(received.c_str() + i);
				i += sizeof(float);
			}

			//so that we do not lose the next character in the stream
			i--;
		}
		else if (received[i] == MSG_CLEAR_STORED_FRAMES)
		{
			m_framesFileWriterReader.closeFileIfOpened();
		}
	}

	if (m_bConfirmCaptured)
	{
		byteToSend = MSG_CONFIRM_CAPTURED;
		m_pClientSocket->SendBytes(&byteToSend, 1);
		m_bConfirmCaptured = false;
	}

	if (m_bConfirmCalibrated)
	{
		int size = (9 + 3) * sizeof(float) + sizeof(int) + 1;
		char *buffer = new char[size];
		buffer[0] = MSG_CONFIRM_CALIBRATED;
		int i = 1;

		memcpy(buffer + i, &calibration.iUsedMarkerId, 1 * sizeof(int));
		i += 1 * sizeof(int);
		memcpy(buffer + i, calibration.worldR[0].data(), 3 * sizeof(float));
		i += 3 * sizeof(float);
		memcpy(buffer + i, calibration.worldR[1].data(), 3 * sizeof(float));
		i += 3 * sizeof(float);
		memcpy(buffer + i, calibration.worldR[2].data(), 3 * sizeof(float));
		i += 3 * sizeof(float);
		memcpy(buffer + i, calibration.worldT.data(), 3 * sizeof(float));
		i += 3 * sizeof(float);

		m_pClientSocket->SendBytes(buffer, size);
		m_bConfirmCalibrated = false;
	}
}

#ifdef KINECT
void LiveScanClient::SendFrame(vector<Point3s> vertices, vector<RGB> RGB, vector<Body> body)
#else
void LiveScanClient::SendFrame(vector<Point3s> vertices, vector<RGB> RGB)
#endif
{
	int size = RGB.size() * (3 + 3 * sizeof(short)) + sizeof(int);

	vector<char> buffer(size);
	char *ptr2 = (char*)vertices.data();
	int pos = 0;

	int nVertices = RGB.size();
	memcpy(buffer.data() + pos, &nVertices, sizeof(nVertices));
	pos += sizeof(nVertices);

	for (unsigned int i = 0; i < RGB.size(); i++)
	{
		buffer[pos++] = RGB[i].rgbRed;
		buffer[pos++] = RGB[i].rgbGreen;
		buffer[pos++] = RGB[i].rgbBlue;

		memcpy(buffer.data() + pos, ptr2, sizeof(short)* 3);
		ptr2 += sizeof(short) * 3;
		pos += sizeof(short) * 3;
	}
	
#ifdef KINECT
	int nBodies = body.size();
	size += sizeof(nBodies);
	for (int i = 0; i < nBodies; i++)
	{
		size += sizeof(body[i].bTracked);
		int nJoints = body[i].vJoints.size();
		size += sizeof(nJoints);
		size += nJoints * (3 * sizeof(float) + 2 * sizeof(int));
		size += nJoints * 2 * sizeof(float);
	}
	buffer.resize(size);
	
	memcpy(buffer.data() + pos, &nBodies, sizeof(nBodies));
	pos += sizeof(nBodies);

	for (int i = 0; i < nBodies; i++)
	{
		memcpy(buffer.data() + pos, &body[i].bTracked, sizeof(body[i].bTracked));
		pos += sizeof(body[i].bTracked);

		int nJoints = body[i].vJoints.size();
		memcpy(buffer.data() + pos, &nJoints, sizeof(nJoints));
		pos += sizeof(nJoints);

		for (int j = 0; j < nJoints; j++)
		{
			//Joint
			memcpy(buffer.data() + pos, &body[i].vJoints[j].JointType, sizeof(JointType));
			pos += sizeof(JointType);
			memcpy(buffer.data() + pos, &body[i].vJoints[j].TrackingState, sizeof(TrackingState));
			pos += sizeof(TrackingState);
			//Joint position
			memcpy(buffer.data() + pos, &body[i].vJoints[j].Position.X, sizeof(float));
			pos += sizeof(float);
			memcpy(buffer.data() + pos, &body[i].vJoints[j].Position.Y, sizeof(float));
			pos += sizeof(float);
			memcpy(buffer.data() + pos, &body[i].vJoints[j].Position.Z, sizeof(float));
			pos += sizeof(float);

			//JointInColorSpace
			memcpy(buffer.data() + pos, &body[i].vJointsInColorSpace[j].X, sizeof(float));
			pos += sizeof(float);
			memcpy(buffer.data() + pos, &body[i].vJointsInColorSpace[j].Y, sizeof(float));
			pos += sizeof(float);
		}
	}
#endif // KINECT

	int iCompression = static_cast<int>(m_bFrameCompression);

	if (m_bFrameCompression)
	{
		// *2, because according to zstd documentation, increasing the size of the output buffer above a 
		// bound should speed up the compression.
		int cBuffSize = ZSTD_compressBound(size) * 2;	
		vector<char> compressedBuffer(cBuffSize);
		int cSize = ZSTD_compress(compressedBuffer.data(), cBuffSize, buffer.data(), size, m_iCompressionLevel);
		size = cSize; 
		buffer = compressedBuffer;
	}
	char header[8];
	memcpy(header, (char*)&size, sizeof(size));
	memcpy(header + 4, (char*)&iCompression, sizeof(iCompression));

	m_pClientSocket->SendBytes((char*)&header, sizeof(int) * 2);
	m_pClientSocket->SendBytes(buffer.data(), size);
}

#ifdef KINECT
void LiveScanClient::StoreFrame(Point3f *vertices, Point2f *mapping, RGB *color, vector<Body> &bodies, BYTE* bodyIndex)
#else
void LiveScanClient::StoreFrame(Point3f *vertices, Point2f *mapping, RGB *color)
#endif
{
	std::vector<Point3f> goodVertices;
	std::vector<RGB> goodColorPoints;

	unsigned int nVertices = pCapture->nDepthFrameWidth * pCapture->nDepthFrameHeight;

	for (unsigned int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++)
	{
#ifdef KINECT
		if (m_bStreamOnlyBodies && bodyIndex[vertexIndex] >= bodies.size())
			continue;
#endif
		if (vertices[vertexIndex].Z >= 0 && mapping[vertexIndex].Y >= 0 && mapping[vertexIndex].Y < pCapture->nColorFrameHeight)
		{
			Point3f temp = vertices[vertexIndex];
			RGB tempColor = color[(int)mapping[vertexIndex].X + (int)mapping[vertexIndex].Y * pCapture->nColorFrameWidth];
			if (calibration.bCalibrated)
			{
				temp.X += calibration.worldT[0];
				temp.Y += calibration.worldT[1];
				temp.Z += calibration.worldT[2];
				temp = RotatePoint(temp, calibration.worldR);

				if (temp.X < m_vBounds[0] || temp.X > m_vBounds[3]
					|| temp.Y < m_vBounds[1] || temp.Y > m_vBounds[4]
					|| temp.Z < m_vBounds[2] || temp.Z > m_vBounds[5])
					continue;
			}

			goodVertices.push_back(temp);
			goodColorPoints.push_back(tempColor);
		}
	}

#ifdef KINECT
	vector<Body> tempBodies = bodies;

	for (unsigned int i = 0; i < tempBodies.size(); i++)
	{
		for (unsigned int j = 0; j < tempBodies[i].vJoints.size(); j++)
		{
			if (calibration.bCalibrated)
			{
				tempBodies[i].vJoints[j].Position.X += calibration.worldT[0];
				tempBodies[i].vJoints[j].Position.Y += calibration.worldT[1];
				tempBodies[i].vJoints[j].Position.Z += calibration.worldT[2];

				Point3f tempPoint(tempBodies[i].vJoints[j].Position.X, tempBodies[i].vJoints[j].Position.Y, tempBodies[i].vJoints[j].Position.Z);

				tempPoint = RotatePoint(tempPoint, calibration.worldR);

				tempBodies[i].vJoints[j].Position.X = tempPoint.X;
				tempBodies[i].vJoints[j].Position.Y = tempPoint.Y;
				tempBodies[i].vJoints[j].Position.Z = tempPoint.Z;
			}
		}
	}
#endif //KINECT

	if (m_bFilter)
		filter(goodVertices, goodColorPoints, m_nFilterNeighbors, m_fFilterThreshold);

	vector<Point3s> goodVerticesShort(goodVertices.size());

	for (unsigned int i = 0; i < goodVertices.size(); i++)
	{
		goodVerticesShort[i] = goodVertices[i];
	}

#ifdef KINECT
	m_vLastFrameBody = tempBodies;
#endif
	m_vLastFrameVertices = goodVerticesShort;
	m_vLastFrameRGB = goodColorPoints;
}

void LiveScanClient::ShowFPS()
{
	/* TODO port from windows
	if (m_hWnd)
	{
		double fps = 0.0;

		LARGE_INTEGER qpcNow = { 0 };
		if (m_fFreq)
		{
			if (QueryPerformanceCounter(&qpcNow))
			{
				if (m_nLastCounter)
				{
					m_nFramesSinceUpdate++;
					fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
				}
			}
		}

		WCHAR szStatusMessage[64];
		StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f", fps);

		if (SetStatusMessage(szStatusMessage, 1000, false))
		{
			m_nLastCounter = qpcNow.QuadPart;
			m_nFramesSinceUpdate = 0;
		}
	}*/
}

