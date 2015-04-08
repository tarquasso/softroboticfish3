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

	Mat centroids = Mat::zeros(3, K, CV_32S);
	Mat colors = Mat::zeros(1, K, CV_UC3);

	colorCentroids(img_path.c_str(), 6, )

	// Calculate color centroids and reconstruct reduced image
	Mat colorCentroids = Mat::zeros(3, K, CV_32S);

	Mat r_img(s.height, s.width, CV_8UC3);
	MatIterator_<Vec3b> dit, dend;
	MatIterator_<uchar> lit = labels.begin<uchar>();
	int i,j;
	i = j = 0;

	for (dit=r_img.begin<Vec3b>(), dend=r_img.end<Vec3b>(); dit != dend; ++dit, ++lit)
	{
		// Color pixel according to label
		*dit = colorTable.at<Vec3b>(*lit);
		
		// Count pixel coord in color centroid calculation
		colorCentroids.at<int32_t>(0,*lit) += i;
		colorCentroids.at<int32_t>(1,*lit) += j;
		colorCentroids.at<int32_t>(2,*lit) += 1;

		// Count current pixel coordinate
		j++;
		if (j == s.width) // wraparound when we get to end of row
		{
			j = 0;
			i++;
		}
	}

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
