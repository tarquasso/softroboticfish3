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

#include "util.h"
#include "vis_servo.h"

#include <ros/ros.h>

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
	int nearest_centroid = get_centroids(&img, K, centroids, colors, labels);

	std::cout << centroids << std::endl;

	reconstruct(img.size(), centroids, colors, labels, nearest_centroid);
	
	cleanup();

	return 0;
}

void cam_poll()
{
	// default arguments
	int K = 3;
	Vec3b target_bgr;
	target_bgr[0] = 89;	// blue
	target_bgr[1] = 67;	// green
	target_bgr[2] = 151; // red

	// initialize cv 
	initialize(K, target_bgr);

	// initialize raspicam
	raspicam::RaspiCam cam;
	cam.open();
	if (!cam.isOpened())
	{
		printf("Failed to open raspicam.");
	}

	// configure camera
	cam.setWidth(320);
	cam.setHeight(240);
	cam.setVideoStabilization(false);
	cam.setHorizontalFlip(false);
	cam.setVerticalFlip(false);


	while (true)
	{
		// import last image

		// command new pic

		// process last image 
		// calculate centroids
		// calculate target color area (ie if we are "close")

		// if we've reached target, report it

		// wait until camera has finished taking image

	}

	// release camera
	cam.release();
	return;
}