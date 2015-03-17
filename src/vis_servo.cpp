// vis_servo.cpp

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <stdio.h>

#include "util.h"

using namespace cv;

int main(int argc, char** argv)
{
	std::string img_path = pkg_path(1);
	img_path += "/P4_Color_2.jpg";
	printf("Importing image file at %s.", img_path.c_str());

	Mat img = imread(img_path.c_str());
	if (img.data == NULL)
	{
		printf("Could not read input file.");
		return 1;
	}

	Size s = img.size();
	printf("Imported %d-channel image of size (%d, %d).", img.channels(), s.width, s.height);
	switch (img.depth())
	{
		case CV_32F:
			printf("Image depth is CV_32F.");
			break;
		case CV_8U:
			printf("Image depth is CV_8U.");
			break;
		default:
			printf("Unexpected image depth.");
			break;
	}

	//namedWindow("image", WINDOW_AUTOSIZE);
	//imshow("image", img);

	// Prepare LUT for conversion from uchar
	Mat lutFloat(1, 256, CV_32F);
	MatIterator_<float> it, end;
	int u = 0;
	for (it=lutFloat.begin<float>(), end=lutFloat.end<float>(); it != end; ++it)
	{
		*it = u/(256.0);
		u++;
	}
	
	// Convert to CV_32F pixel array for kmeans
	Mat px_array(img.total(), 3, CV_32F);
	printf("Converting to float vector.");
	LUT(img.reshape(1, img.total()), lutFloat, px_array);
	// Prepare other arguments for kmeans
	int K = 6;
	TermCriteria termCrit(TermCriteria::COUNT + TermCriteria::EPS, 50, 0.001);
	Mat labels(img.total(), 1, CV_8U);
	Mat centers(K, img.channels(), CV_32F);
	int flags = KMEANS_RANDOM_CENTERS;
	int attempts = 10;

	float clustering_coeff = kmeans(px_array, K, labels, termCrit, attempts, flags, centers);
	printf("Kmeans successful with clustering coefficient %f.", clustering_coeff );

	// prepare reconstruction table
	Mat colorTable(1, K, CV_8UC3);
	for (u=0; u<K; u++)
	{
		Vec3b color;
		color[0] = centers.at<float>(u, 0) * 256;
		color[1] = centers.at<float>(u, 1) * 256;
		color[2] = centers.at<float>(u, 2) * 256;
		colorTable.at<Vec3b>(u) = color;
	}
	
	if (!colorTable.isContinuous())
	{
		printf("LUT not continuous.");
	}

	switch (colorTable.depth())
	{
		case CV_8U:
			printf("Reconstruction table completed. %d channels with depth CV_8U. Total = %d.", colorTable.channels(), (int) colorTable.total());
			break;
		default:
			printf("Reconstruction table completed. %d channels with unrecognized depth. Total = %d.", colorTable.channels(), (int) colorTable.total());
			break;
	}

	std::cout << colorTable << std::endl;

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


	//namedWindow("reduced image", WINDOW_AUTOSIZE);
	//imshow("reduced image", r_img);
	waitKey();
	return 0;
}
