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

// Helper array for centroid calculation
long long * x_counts;
long long * y_counts;
long long * num_pixels;

// Parameters for kmeans
#define KMEANS_TERM_CRIT TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 0.01)
#define KMEANS_FLAGS KMEANS_RANDOM_CENTERS
#define KMEANS_ATTEMPTS 10

void initialize(int K)
{
	MatIterator_<float> it, end;
	int u = 0;
	for (it=lutFloat.begin<float>(), end=lutFloat.end<float>(); it != end; ++it)
	{
		*it = u/(256.0);
		u++;
	}

	x_counts = new long long[K];
	y_counts = new long long[K];
	num_pixels = new long long[K];

	initialized = true;
}

void cleanup()
{
	delete[] num_pixels;
	delete[] y_counts;
	delete[] x_counts;
}

void get_centroids(const Mat* img, int K, Mat & centroids, Mat & colors, Mat & labels)
/* 	Input:
	img: pointer to img matrix
	K: number of colors to use

	Output:
	centroids: 	Matrix of pixel centroids for each cluster
	colors: 	Actual color values of each cluster
	labels:		Labels mapping each pixel to a cluster
*/
{
	if (!initialized)
	{
		printf("Must call initialize() first.\n");
		return;
	}

	Size s = img->size();

	// Convert to CV_32F pixel array for kmeans
	Mat px_array(img->total(), 3, CV_32F);
	printf("Converting to float vector.");
	LUT(img->reshape(1, img->total()), lutFloat, px_array);

	// Do kmeans with global parameters
	Mat centers(K, img->channels(), CV_32F);
	float clustering_coeff = kmeans(px_array, K, labels, KMEANS_TERM_CRIT, KMEANS_ATTEMPTS, KMEANS_FLAGS, centers);
	printf("Kmeans successful with clustering coefficient %f.", clustering_coeff );

	/* Calculate centroids */
	for (int k=0; k<K; k++)		// re-zero helper arrays
	{
		x_counts[k] = 0;
		y_counts[k] = 0;
		num_pixels[k] = 0;
	}

	MatIterator_<uchar> lit, lend;
	int i, j;	// i = row, j = column index
	i = j = 0;

	for (lit=labels.begin<uchar>(), lend=labels.end<uchar>(); lit != lend; lit++)
	{
		// Count pixel coord in color centroid calculation
		x_counts[*lit] += j;
		y_counts[*lit] += i;
		num_pixels[*lit]++;

		// Increment current pixel coordinates
		j++;
		if (j == s.width) // wraparound when we get to end of row
		{
			j = 0;
			i++;
		}
	}

	// Convert centers (floats) to colors
	// populate color reconstruction table
	// and complete centroid calculation
	printf("centroid table dimensions (%d, %d).\n", centroids.size().height, centroids.size().width);
	for (int u=0; u<K; u++)
	{
		centroids.at<uint16_t>(0,u) = x_counts[u] / num_pixels[u];	// calculate average pixel column
		centroids.at<uint16_t>(1,u) = y_counts[u] / num_pixels[u];	// calculate average pixel row
		centroids.at<uint16_t>(2,u) = 1;

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

	delete[] num_pixels;
	delete[] y_counts;
	delete[] x_counts;

	return;
}

void reconstruct(Size s, Mat& centroids, Mat& colors, Mat& labels)
{
	Mat r_img(s.height, s.width, CV_8UC3);
	MatIterator_<Vec3b> dit, dend;
	MatIterator_<uchar> lit = labels.begin<uchar>();
	int i,j;
	i = j = 0;

	for (dit=r_img.begin<Vec3b>(), dend=r_img.end<Vec3b>(); dit != dend; ++dit, ++lit)
	{
		// Color pixel according to label
		*dit = colors.at<Vec3b>(*lit);
	}

	// Normalize and draw circles at centroids
	int K = centroids.size().width;
	printf("K  = %d.\n", K);
	for (int k=0; k < K; k++)
	{
		Point center;
		center.x = centroids.at<uint16_t>(0,k);
		center.y = centroids.at<uint16_t>(1,k);
		
		int thickness = 2;
		float radius = s.width/64.0;
		int linetype = 1;
		Vec3b color_v = colors.at<Vec3b>(k);
		uchar b = color_v[0];
		uchar g = color_v[1];
		uchar r = color_v[2];
		Scalar color(b,g,r);
		Scalar white(255, 255, 255);
		circle(r_img, center, radius*1.5, white, thickness, linetype);
		circle(r_img, center, radius, color, thickness, linetype);
	}

	namedWindow("reduced image", WINDOW_AUTOSIZE);
	imshow("reduced image", r_img);
	waitKey();

	return;
}

#endif