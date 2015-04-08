// vis_servo.cpp

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <stdio.h>

#include "util.h"
#include "vis_servo.h"

using namespace cv;

int main(int argc, char** argv)
{
	std::string img_path = pkg_path(1);
	img_path += "/P4_Color_2.jpg";
	printf("Importing image file at %s.\n", img_path.c_str());

	Mat img = imread(img_path, CV_LOAD_IMAGE_COLOR);
	Mat centroids(2, K, CV_32S);
	Mat colors(1, K, CV_UC3);
	Mat labels(img.total(), 1, CV_8U);
	get_centroids(&img, 6, centroids, colors, lables)

	// Normalize and draw circles at centroids
	for (int k=0; k < K; k++)
	{
		Point center;
		int num_samples = colorCentroids.at<int32_t>(2,k);
		center.y = colorCentroids.at<int32_t>(0,k) / (float) num_samples;
		center.x = colorCentroids.at<int32_t>(1,k) / (float) num_samples;
		
		int thickness = 2;
		float radius = s.width/64.0;
		int linetype = 1;
		Vec3b color_v = colorTable.at<Vec3b>(k);
		uchar b = color_v[0];
		uchar g = color_v[1];
		uchar r = color_v[2];
		Scalar color(b,g,r);
		circle(r_img, center, radius, color, thickness, linetype);
	}

	namedWindow("reduced image", WINDOW_AUTOSIZE);
	imshow("reduced image", r_img);
	waitKey();

	return 0;
}
