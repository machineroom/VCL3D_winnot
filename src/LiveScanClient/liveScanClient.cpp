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
#include "liveScanClient.h"
#include "filter.h"
#include <chrono>
#include <fstream>
#include <zstd.h>
#include <gflags/gflags.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

DEFINE_string(server, "localhost", "LiveScan server address");
DEFINE_int32(port,48001, "LiveScan server port");
DEFINE_string(serial, "", "Kinect serial number (underside long decimal string) - default is 1st discovered");

std::mutex m_mSocketThreadMutex;

int main (int argc, char **argv)
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);
    LiveScanClient application (FLAGS_serial, FLAGS_server, FLAGS_port);
    application.Run();
}

LiveScanClient::LiveScanClient(std::string kinectSerial, std::string server, int port) :
	m_pDepthRGBX(NULL),
	m_pCameraSpaceCoordinates(NULL),
	m_pDepthCoordinatesOfColor(NULL),
	m_bCalibrate(false),
	m_bFilter(false),
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
	m_fFilterThreshold(0.01f),
	m_sKinectSerial(kinectSerial),
	m_sServer(server)
{
	try {
		m_pClientSocket = new SocketClient(m_sServer, port);
		m_bConnected = true;
	} 					
	catch (...) {
		std::cout << "Failed server connection" << std::endl;
	}


	pCapture = new Freenect2Capture();
	pCapture->Initialize(m_sKinectSerial);
	
	//m_pDepthRGBX is generically used for visualuising stuff and will be frequently overritten
	m_pDepthRGBX = new RGB[pCapture->nColorFrameWidth * pCapture->nColorFrameHeight];

	m_pCameraSpaceCoordinates = new Point3f[pCapture->nDepthFrameWidth * pCapture->nDepthFrameHeight];
	m_pDepthCoordinatesOfColor = new Point2f[pCapture->nColorFrameWidth * pCapture->nColorFrameHeight];

	// Create and initialize a new image renderer (take a look at ImageRenderer.h)
	// We'll use this to draw the data we receive from the Kinect to the screen
	m_viewer.initialize();


	m_lastClockReading = std::chrono::steady_clock::now();

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
	{
		std::lock_guard<std::mutex> lock(m_mSocketThreadMutex);
		StoreFrame(m_pCameraSpaceCoordinates/*vertices*/, pCapture->pColorRGBX/*color*/);

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

	//View the depth, colour, status etc.
	m_viewer.start();
	ShowRawColour();
	ShowRawDepth();
	CalculateFPS();
	ShowStatus();
	if (m_viewer.finish()) {
		delete pCapture;
		exit(0);
	}
}

#define GREY_SCALE_DEPTH
// Show the raw depth buffer recieved (depth in mm mapped to greyscale)
void LiveScanClient::ShowRawDepth()
{
	//pCapture->pDepth is 16 bit samples representing mm distance. Range ~500-4500mm (https://docs.depthkit.tv/docs/kinect-for-windows-v2)
#ifdef GREY_SCALE_DEPTH
	//this is more accurate - though less pretty
	// simply show the depth as grey scale intensity that wraps around every 256mm!
	for (int i = 0; i < pCapture->nDepthFrameWidth * pCapture->nDepthFrameHeight; i++)
	{
		BYTE intensity = 0;
		UINT16 depth = pCapture->pDepth[i];
		intensity = static_cast<BYTE>(depth);

		m_pDepthRGBX[i].rgbRed = intensity;
		m_pDepthRGBX[i].rgbGreen = intensity;
		m_pDepthRGBX[i].rgbBlue = intensity;
	}
	// Draw the data
	m_viewer.render_colour(reinterpret_cast<uint8_t*>(m_pDepthRGBX), pCapture->nDepthFrameWidth, pCapture->nDepthFrameHeight, sizeof(RGB), TOP_LEFT, "raw depth");
#else
	cv::Mat depth_mm (pCapture->nDepthFrameHeight, pCapture->nDepthFrameWidth, CV_16UC1, pCapture->pDepth);
    cv::Mat downsampled;
	//colour mapping needs 8 bit data so we scale. Divide by 16 (4500/256~=18).
	// get 16 bit depth into 8 bit greyscale for opencv
	depth_mm.convertTo(downsampled, CV_8UC1, 1 / 18.0);
	
    cv::Mat img_color (pCapture->nDepthFrameHeight, pCapture->nDepthFrameWidth, CV_8UC3);
    cv::applyColorMap(downsampled, img_color, cv::COLORMAP_VIRIDIS);
   	// Draw the data
   	//re-use the (larger) colour buffer
   	//TODO optimise this! (we shouldn't need to do pixel level fetch from opencv Mat)
   	RGB *dp = m_pDepthRGBX;
   	for (int y=0; y < 424; y++) {
   		for (int x=0; x < 512; x++) {
			dp->rgbBlue = img_color.data[img_color.channels()*(img_color.cols*y + x) + 0];    
			dp->rgbGreen = img_color.data[img_color.channels()*(img_color.cols*y + x) + 1];
			dp->rgbRed = img_color.data[img_color.channels()*(img_color.cols*y + x) + 2];
   			dp++;
   		}
   	}
	m_viewer.render_colour((uint8_t *)m_pDepthRGBX, img_color.cols, img_color.rows, sizeof(RGB), TOP_LEFT);
#endif
}

void LiveScanClient::ShowRawColour()
{
    // Make sure we've received valid data
	if (pCapture->pColorRGBX)
    {
        // Draw the data
		m_viewer.render_colour(reinterpret_cast<uint8_t*>(pCapture->pColorRGBX), pCapture->nColorFrameWidth, pCapture->nColorFrameHeight, sizeof(RGB), TOP_RIGHT, "raw colour");
    }
}

// A simple text window to show client status
void LiveScanClient::ShowStatus() {
	memset (m_pDepthRGBX, 0, sizeof(*m_pDepthRGBX)*pCapture->nColorFrameWidth*pCapture->nColorFrameHeight);
	std::vector<cv::String> strings;
	if (m_bCalibrate) {
		strings.push_back("calibrating");
	} else {
		strings.push_back("not calibrating");
	}
	strings.push_back(pCapture->Identifier());
	strings.push_back (m_sServer);
	if (m_bConnected) {
		strings.push_back("connected");
	} else {
		strings.push_back("not connected");
	}
	std::stringstream stream;
	stream << std::fixed << std::setprecision(2) << m_fps;
	std::string fps = stream.str();
	strings.push_back ("FPS " + fps);
	cv::Mat mat (512,512,CV_8UC3);
	mat = cv::Scalar(0,0,0);
	cv::Point position(0,50);
   	// Draw the text
   	for (auto &s: strings) {
        //std::cout << s << std::endl;
        int face = cv::FONT_HERSHEY_DUPLEX;
        int scale = 1;
        int thickness = 1;
	   	//cv::putText (InputOutputArray img, const String &text, Point org, int fontFace, double fontScale, Scalar color, int thickness=1, int lineType=LINE_8, bool bottomLeftOrigin=false)
	   	cv::putText(mat, s, position, face, scale, cv::Scalar(255,0,0), thickness);
	   	int baseline;
	   	//cv::getTextSize (const String &text, int fontFace, double fontScale, int thickness, int *baseLine)
	   	cv::Size textSize = cv::getTextSize (s, face, scale, thickness, &baseline);
	   	position.y += textSize.height*1.2;
    }
   	
   	//re-use the (larger) colour buffer
   	//TODO optimise this! (we shouldn't need to do pixel level fetch from opencv Mat)
   	RGB *dp = m_pDepthRGBX;
   	for (int y=0; y < 424; y++) {
   		for (int x=0; x < 512; x++) {
			dp->rgbBlue = mat.data[mat.channels()*(mat.cols*y + x) + 0];    
			dp->rgbGreen = mat.data[mat.channels()*(mat.cols*y + x) + 1];
			dp->rgbRed = mat.data[mat.channels()*(mat.cols*y + x) + 2];
   			dp++;
   		}
   	}
	m_viewer.render_colour((uint8_t *)m_pDepthRGBX, mat.cols, mat.rows, sizeof(RGB), BOTTOM_RIGHT);
}

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
				SendFrame(points, colors);
		}
		//send last frame
		else if (received[i] == MSG_REQUEST_LAST_FRAME)
		{
			byteToSend = MSG_LAST_FRAME;
			m_pClientSocket->SendBytes(&byteToSend, 1);

			SendFrame(m_vLastFrameVertices, m_vLastFrameRGB);
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

void LiveScanClient::SendFrame(vector<Point3s> vertices, vector<RGB> RGB)
{
	//JW: RGB.size() = number of pixels in visible frame, i.e. width*height
	int size = RGB.size() * (3 + 3 * sizeof(short)) + sizeof(int);
	//JW size = total number of bytes to be written to buffer. Calculated as:
	// number of colour pixels * (3 bytes per RGB pixel + 3 shorts of vertex X,Y,Z as mm) + int (number of pixels)
	// buffer stucture:
	// [number of vertices]
	// for n in vertices {
	//	[RGB X Y Z]
	//}

	vector<char> buffer(size);
	char *ptr2 = (char*)vertices.data();
	int pos = 0;

	int nVertices = RGB.size();	
	//JW write first int = number of following vertices (image pixels)
	memcpy(buffer.data() + pos, &nVertices, sizeof(nVertices));
	pos += sizeof(nVertices);

	for (unsigned int i = 0; i < RGB.size(); i++)
	{
		//JW write 3 bytes of RGB
		buffer[pos++] = RGB[i].rgbRed;
		buffer[pos++] = RGB[i].rgbGreen;
		buffer[pos++] = RGB[i].rgbBlue;

		//JW write 3 shorts of depth (X,Y,Z)
		memcpy(buffer.data() + pos, ptr2, sizeof(short)* 3);
		ptr2 += sizeof(short) * 3;
		pos += sizeof(short) * 3;
	}
	
	// send empty bodies header (the original windows code could send kinect tracked body data)
	int nBodies = 0;
	size += sizeof(nBodies);
	buffer.resize(size);
	
	memcpy(buffer.data() + pos, &nBodies, sizeof(nBodies));
	pos += sizeof(nBodies);

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

// JW note sets m_vLastFrameVertices & m_vLastFrameRGB which are later sent to the server

void LiveScanClient::StoreFrame(Point3f *vertices, RGB *color)
{
	std::vector<Point3f> goodVertices;
	std::vector<RGB> goodColorPoints;

	unsigned int nVertices = pCapture->nDepthFrameWidth * pCapture->nDepthFrameHeight;

	for (unsigned int vertexIndex = 0; vertexIndex < nVertices; vertexIndex++)
	{
		if (vertices[vertexIndex].Z >= 0)
		{
			Point3f temp = vertices[vertexIndex];
			// since we _know_ that depth & colour are the same resolution and 1:1 mapping...
			RGB tempColor = color[vertexIndex];
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
	
	if (m_bFilter)
		filter(goodVertices, goodColorPoints, m_nFilterNeighbors, m_fFilterThreshold);
	
	//JW note this will convert float meters to short mm (server expects mm)
	vector<Point3s> goodVerticesShort(goodVertices.size());

	for (unsigned int i = 0; i < goodVertices.size(); i++)
	{
		goodVerticesShort[i] = goodVertices[i];
	}

	m_vLastFrameVertices = goodVerticesShort;
	m_vLastFrameRGB = goodColorPoints;
}

void LiveScanClient::CalculateFPS()
{
	auto now = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed = now - m_lastClockReading;
	m_fps = 1.f/elapsed.count();
	m_lastClockReading = now;
}

