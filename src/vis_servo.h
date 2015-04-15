// vis_servo.h

#ifndef VIS_SERVO_H
#define VIS_SERVO_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>

using namespace cv;

// Look up table for color to float conversion
Mat lutFloatWeighted(1, 256, CV_32FC3);
bool initialized = false;

// Helper array for centroid calculation
long long * x_counts;
long long * y_counts;
long long * num_pixels;

// Parameters for kmeans
#define KMEANS_FLAGS KMEANS_RANDOM_CENTERS
#define KMEANS_ATTEMPTS 10
TermCriteria term_crit(TermCriteria::COUNT + TermCriteria::EPS, 10, 0.01);

#define FT_0_WT 5.0f
#define FT_1_WT 2.0f
#define FT_2_WT 1.0f

void initialize(int K)
{
	MatIterator_<Vec3f> it, end;
	int u = 0;
	Vec3f v;
	for (it=lutFloatWeighted.begin<Vec3f>(), end=lutFloatWeighted.end<Vec3f>(); it != end; ++it)
	{
		// Apply weightings
		v[0] = u * (FT_0_WT/256.0); // hue
		v[1] = u * (FT_1_WT/256.0); // saturation
		v[2] = u * (FT_2_WT/256.0); // value 
		*it = v;
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

void convert_to_feature_space(const Mat& src, Mat& out, Size s)
{
	cvtColor(src, out, CV_BGR2HSV, 3); 		// remap color space
	LUT(out, lutFloatWeighted, out);		// change to float and apply weights
	out = out.reshape(1, src.total());
	printf("out depht = %d.", out.depth());
}

void convert_from_feature_space(const Mat& src, Mat& out, int K)
{
	Mat img = src.reshape(3,K);

	// remove weighting factor
	MatIterator_<Vec3f> cit, cend; 
	for (cit=img.begin<Vec3f>(), cend=img.end<Vec3f>(); cit!=cend; ++cit)
	{
		Vec3f v_a = *cit;
		Vec3f v_b;
		v_b[0] = v_a[0]/FT_0_WT;
		v_b[1] = v_a[1]/FT_1_WT;
		v_b[2] = v_a[2]/FT_2_WT;
		*cit = v_b;
	}

	cvtColor(img, img, CV_HSV2BGR, 3); // remap color space
	out = img;
	for (int k=0; k<K; k++)
	{
		Vec3f c_f= img.at<Vec3f>(k);
		Vec3b c_b;
		c_b[0] = c_f[0] * 256;
		c_b[1] = c_f[1] * 256;
		c_b[2] = c_f[2] * 256;
		out.at<Vec3b>(k) = c_b;
	}
	
	return;
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
	printf("Image has dimensions (%d, %d).\n", s.height, s.width);

	// Convert to CV_32F pixel array for kmeans
	Mat px_array(img->total(), 3, CV_32F);
	convert_to_feature_space(*img, px_array, s);
	printf("Converting to float vector.\n");
	printf("Float array dimensions (%d, %d) with depth %d.\n", px_array.size().height, px_array.size().width, px_array.depth());

	// Do kmeans with global parameters
	Mat centers;
	//Mat centers(K, img->channels(), CV_32F);
	float clustering_coeff = kmeans(px_array, K, labels, term_crit, KMEANS_ATTEMPTS, KMEANS_FLAGS, centers);
	printf("Kmeans successful with clustering compactness %f.\n", clustering_coeff );

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

	// complete centroid calculation
	for (int u=0; u<K; u++)
	{
		centroids.at<uint16_t>(0,u) = x_counts[u] / num_pixels[u];	// calculate average pixel column
		centroids.at<uint16_t>(1,u) = y_counts[u] / num_pixels[u];	// calculate average pixel row
		centroids.at<uint16_t>(2,u) = 1;
	}

	// Convert centers (floats) to colors
	// populate color reconstruction table
	Size centers_s = centers.size();
	printf("centers dimensions (%d, %d) with depth %d.\n", centers_s.height, centers_s.width, centers.depth());
	convert_from_feature_space(centers, colors, K);
	printf("color depth %d.\n", colors.depth());

	if (!colors.isContinuous())
	{
		printf("LUT not continuous.");
	}

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