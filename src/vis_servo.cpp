// vis_servo.cpp

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <raspicam/raspicam.h>

#include <string>
#include <cstring>
#include <stdio.h>
#include <iostream>
#include <cstdlib>

#include "util.h"
#include "vis_servo.h"

#include <ros/ros.h>


#define RES_W 320
#define RES_H 240

using namespace cv;

int main(int argc, char** argv)
{
	// Extract command line arguments
	int K = 3;
	std::string filename("P4_Color_2.jpg");
	for (int i=0; i<argc; i++)
	{
		// Parse string
		if (strncmp(argv[i], "K=", 2) == 0)
		{
			K = atoi(argv[i] + 2);
		}
		else if (strncmp(argv[i], "file=", 5) == 0)
		{
			filename = std::string(argv[i] + 5);
		}
		else
		{
			printf("Invalid argument: %s.\n", argv[i]);
		}
	}

	printf("K = %d.\n", K);
	std::string img_path = pkg_path(1);
	img_path += '/' + filename;
	printf("Importing image file %s.\n", filename.c_str());

	// define target BGR
	Vec3b target_bgr;
	target_bgr[0] = 89;	// blue
	target_bgr[1] = 67;	// green
	target_bgr[2] = 151; // red
	initialize(K, target_bgr);

	Mat img = imread(img_path, IMREAD_COLOR);
	namedWindow("image", WINDOW_AUTOSIZE);
	imshow("image", img);

	Mat centroids(3, K, CV_16U);
	Mat colors(1, K, CV_8UC3);
	Mat labels(img.total(), 1, CV_8U);
	int nearest_centroid = get_centroids(img, K, centroids, colors, labels);

	std::cout << centroids << std::endl;

	reconstruct(img.size(), centroids, colors, labels, nearest_centroid);
	
	cleanup();

	return 0;
}

int cam_poll()
{
	// default arguments
	int K = 3;
	Vec3b target_bgr;
	target_bgr[0] = 89;	// blue
	target_bgr[1] = 67;	// green
	target_bgr[2] = 151; // red
	std::string img_path = pkg_path(1) + "images/";

	// initialize cv 
	initialize(K, target_bgr);

	// initialize raspicam
	raspicam::RaspiCam cam;
	printf("Opening raspicam.\n");
	cam.open(false);	// don't start capturing yet
	if (!cam.isOpened())
	{
		printf("Failed to open raspicam.\n");
	}

	// configure camera
	cam.setWidth(RES_W);
	cam.setHeight(RES_H);
	cam.setVideoStabilization(true);
	cam.setHorizontalFlip(false);
	cam.setVerticalFlip(false);
	cam.setFormat(raspicam::RASPICAM_FORMAT_BGR);

	printf("Starting capture.\n");
	cam.startCapture();

	int mat_shape[2] = {RES_W, RES_H};

	int frame_id = 0;
	char* filename = new char[10];
	
	while (true)
	{
		// grab image
		cam.grab();

		// import into Mat object
		Mat frame(2, mat_shape, CV_8UC3, (void*) cam.getImageBufferData(), 0);

		// save image to file for testing purposes
		std::sprintf(filename, "frame%d.jpg", frame_id);
		imwrite((img_path + filename).c_str(), frame);
		
		// calculate centroids
		Mat centroids(3, K, CV_16U);
		Mat colors(1, K, CV_8UC3);
		Mat labels(frame.total(), 1, CV_8U);
		int nearest_centroid = get_centroids(frame, K, centroids, colors, labels);

		// XXX vectorize and publish nearest centroid location

		// XXX calculate target color area (ie if we are "close")

		// XXX if we've reached target, report it

		frame_id++;
	}

	delete[] filename;
	// release camera
	cam.release();
	// cleanup cv
	cleanup();
	return 0;
}