// vis_servo.cpp

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <stdio.h>

#include "util.h"
#include "vis_servo.h"

using namespace cv;

#define K 3

int main(int argc, char** argv)
{
	std::string img_path = pkg_path(1);
	img_path += "/P4_Color_2.jpg";
	printf("Importing image file at %s.\n", img_path.c_str());

	intialize();

	Mat img = imread(img_path, CV_LOAD_IMAGE_COLOR);
	Mat centroids(2, K, CV_32S);
	Mat colors(1, K, CV_8UC3);
	Mat labels(img.total(), 1, CV_8U);
	get_centroids(&img, K, centroids, colors, labels);

	//reconstruct(img.size(), centroids, colors, labels);

	return 0;
}
