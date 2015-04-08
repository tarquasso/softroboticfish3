// vis_servo.h

#ifndef VIS_SERVO_H
#define VIS_SERVO_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>

using namespace cv;

// Look up table for color to float conversion
Mat lutFloat(1, 256, CV_32F);
bool initialized = false;

// Parameters for kmeans
TermCriteria termCrit(TermCriteria::COUNT + TermCriteria::EPS, 10, 0.01);
Mat labels(img.total(), 1, CV_8U);
Mat centers(K, img.channels(), CV_32F);
int flags = KMEANS_RANDOM_CENTERS;
int attempts = 10;

void initialize()
{
	MatIterator_<float> it, end;
	int u = 0;
	for (it=lutFloat.begin<float>(), end=lutFloat.end<float>(); it != end; ++it)
	{
		*it = u/(256.0);
		u++;
	}

	initialized = true;
}

void get_centroids(const char* img_file, int K, Mat & centroids, Mat & colors)
{
	if (!initialized)
	{
		printf("Must call initialize() first.\n");
		return;
	}

	Mat img = imread(img_file, CV_LOAD_IMAGE_COLOR);

	if (img.data == NULL)
	{
		printf("Could not read input file.\n");
		return 1;
	}

	Size s = img.size();
	printf("Imported %d-channel image of size (%d, %d).\n", img.channels(), s.width, s.height);
	switch (img.depth())
	{
		case CV_32F:
			printf("Image depth is CV_32F.\n");
			break;
		case CV_8U:
			printf("Image depth is CV_8U.\n");
			break;
		default:
			printf("Unexpected image depth.\n");
			break;
	}
	
	// Convert to CV_32F pixel array for kmeans
	Mat px_array(img.total(), 3, CV_32F);
	printf("Converting to float vector.");
	LUT(img.reshape(1, img.total()), lutFloat, px_array);

	// Do kmeans with global parameters
	float clustering_coeff = kmeans(px_array, K, labels, termCrit, attempts, flags, centers);
	printf("Kmeans successful with clustering coefficient %f.", clustering_coeff );

	// populate reconstruction table
	for (u=0; u<K; u++)
	{
		Vec3b color;
		color[0] = centers.at<float>(u, 0) * 256;
		color[1] = centers.at<float>(u, 1) * 256;
		color[2] = centers.at<float>(u, 2) * 256;
		colors.at<Vec3b>(u) = color;
	}
	
	if (!colors.isContinuous())
	{
		printf("LUT not continuous.");
	}

	switch (colors.depth())
	{
		case CV_8U:
			printf("Reconstruction table completed. %d channels with depth CV_8U. Total = %d.\n", colors.channels(), (int) colors.total());
			break;
		default:
			printf("Reconstruction table completed. %d channels with unrecognized depth. Total = %d.\n", colors.channels(), (int) colors.total());
			break;
	}

	std::cout << colors << std::endl;

	Mat r_img(s.height, s.width, CV_8UC3);
	MatIterator_<Vec3b> dit, dend;
	MatIterator_<uchar> lit = labels.begin<uchar>();
	int i,j;
	i = j = 0;

	for (dit=r_img.begin<Vec3b>(), dend=r_img.end<Vec3b>(); dit != dend; ++dit, ++lit)
	{
		// Color pixel according to label
		*dit = colors.at<Vec3b>(*lit);
		
		// Count pixel coord in color centroid calculation
		centroids.at<int32_t>(0,*lit) += i;
		centroids.at<int32_t>(1,*lit) += j;
		centroids.at<int32_t>(2,*lit) += 1;

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
		int num_samples = centroids.at<int32_t>(2,k);
		center.y = centroids.at<int32_t>(0,k) / (float) num_samples;
		center.x = centroids.at<int32_t>(1,k) / (float) num_samples;
		
		int thickness = 2;
		float radius = s.width/64.0;
		int linetype = 1;
		Vec3b color_v = colors.at<Vec3b>(k);
		uchar b = color_v[0];
		uchar g = color_v[1];
		uchar r = color_v[2];
		Scalar color(b,g,r);
		circle(r_img, center, radius, color, thickness, linetype);
	}

}



#endif