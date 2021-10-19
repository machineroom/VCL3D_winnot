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
#include <stdlib.h>

FreenectCapture::FreenectCapture()
{
	pColorRGBX = (RGB*)malloc(640*480*sizeof(*pColorRGBX));	//32 bits per RGBX sample
	pDepth = (UINT16*)malloc(640*480*sizeof(*pDepth));	//16 bits per depth sample
	f_video_mutex = PTHREAD_MUTEX_INITIALIZER;
	f_video_cond = PTHREAD_COND_INITIALIZER;
	f_depth_mutex = PTHREAD_MUTEX_INITIALIZER;
	f_depth_cond = PTHREAD_COND_INITIALIZER;
	
}

FreenectCapture::~FreenectCapture()
{
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	printf ("depth\n");
	FreenectCapture *capture = (FreenectCapture *)freenect_get_user (dev);
	pthread_mutex_lock(&capture->f_depth_mutex);
	pthread_cond_signal(&capture->f_depth_cond);
	pthread_mutex_unlock(&capture->f_depth_mutex);
}

void video_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	printf ("video\n");
	FreenectCapture *capture = (FreenectCapture *)freenect_get_user (dev);
	pthread_mutex_lock(&capture->f_video_mutex);
	pthread_cond_signal(&capture->f_video_cond);
	pthread_mutex_unlock(&capture->f_video_mutex);
}

void *freenect_threadfunc(void *arg)
{
	FreenectCapture *capture = (FreenectCapture *)arg;
	freenect_set_depth_buffer(capture->f_dev, capture->pDepth);
	freenect_set_video_buffer(capture->f_dev, capture->pColorRGBX);
	
	freenect_set_depth_callback(capture->f_dev, depth_cb);
	freenect_set_video_callback(capture->f_dev, video_cb);
	// MEDIUM = 640*480. REGISTERED = depth pixels aligned to RGB pixels
	freenect_set_depth_mode(capture->f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
	freenect_set_video_mode(capture->f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));

	freenect_start_depth(capture->f_dev);
	freenect_start_video(capture->f_dev);
	//set Kinect LED red to say we're watching you
	freenect_set_led(capture->f_dev, LED_RED);
	
	while (freenect_process_events(capture->f_ctx) >= 0) {
	}
	return NULL;
}

bool FreenectCapture::Initialize()
{
	printf ("freenect init\n");

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return false;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	//only interested in the camera and motor for now - no audio (note camera also means LED)
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	/* TODO support other devices */
	if (freenect_open_device(f_ctx, &f_dev, 0/*device_number*/) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return false;
	}
	//set Kinect LED green to say we're awake
	freenect_set_led(f_dev, LED_GREEN);
	
	freenect_set_user(f_dev, this);
	int res;
	pthread_t freenect_thread;
	res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, this);
	if (res) {
		printf("pthread_create failed\n");
		freenect_shutdown(f_ctx);
		return false;
	}
	bInitialized = true;

	return bInitialized;
}

bool FreenectCapture::AcquireFrame()
{
	return true;
}

// TODO leaving these unimplemented for now since the realsense (our actual target) matches the RGB & depth resolutions so this mapping isn't required.
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

