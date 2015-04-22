// vis_servo.cpp

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <cstring>
#include <stdio.h>
#include <iostream>

#include "util.h"
#include "vis_servo.h"

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

	initialize(K);

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
