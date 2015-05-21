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
#define KMEANS_ATTEMPTS 3
TermCriteria term_crit(TermCriteria::COUNT + TermCriteria::EPS, 20, 1.0);

#define FT_0_WT 0.79f
#define FT_1_WT 0.18f
#define FT_2_WT 0.03f

// target color in feature space
Vec3f target_ftr;

// Function headers
void convert_to_feature_space(const Mat& src, Mat& out, Size s);
void convert_from_feature_space(const Mat& src, Mat& out, Size out_s);
void set_target_bgr(Vec3b bgr);


void initialize(int K, Vec3b bgr)
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

	set_target_bgr(bgr);

	initialized = true;
}

void set_target_bgr(Vec3b bgr)
{
	Mat target_bgr_m(1,1,CV_8UC3);
	target_bgr_m.at<Vec3b>(0) = bgr;
	target_bgr_m.reshape(1);
	// now convert to feature space
	Mat target_ftr_m(3,1,CV_32F);
	convert_to_feature_space(target_bgr_m, target_ftr_m, Size(3,1));
	target_ftr_m.reshape(3);
	target_ftr = target_ftr_m.at<Vec3f>(0);

	return;
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
}

void convert_from_feature_space(const Mat& src, Mat& out, Size out_s)
{
	Mat img(out_s, CV_8UC3);

	// remove weighting factor
	// and convert to CV_8UC3
	MatConstIterator_<float> cit = src.begin<float>();
	MatConstIterator_<float> cend = src.end<float>();
	MatIterator_<Vec3b> oit = img.begin<Vec3b>();

	while (cit!=cend)
	{
		// Populate temp vector
		Vec3b v;
		v[0] = *cit * (256.0/FT_0_WT);
		++cit;
		v[1] = *cit * (256.0/FT_1_WT);
		++cit;
		v[2] = *cit * (256.0/FT_2_WT);
		++cit;

		// Store in img
		*oit = v;
		++oit;		
	}

	if (oit != img.end<Vec3b>())
	{
		printf("Output iterator misaligned. ERRROR.");
	}

	cvtColor(img, out, CV_HSV2BGR, 3); // remap color space
	return;
}

void sort_colors(const Mat& colors, const Vec3f& t, Mat& colors_sorted)
{
	int K = colors.size().height;
	Mat distances = Mat(K, 1, CV_32F);
	
	// calculate distances
	for (int k=0; k<K; k++)
	{
		float d = 0;
		d += pow( t[0] - colors.at<float>(k,0), 2);
		d += pow( t[1] - colors.at<float>(k,1), 2);
		d += pow( t[2] - colors.at<float>(k,2), 2);
		distances.at<float>(k) = d;
	}

	// now sort
	int min_k;
	float min_d;
	for (int i=0; i<K; i++)
	{
		// find ith lowest distance
		min_k = K;
		min_d = -1.0; // XXX
		for (int j=0; j<K; j++)
		{
			// grab distance from table
			float d = distances.at<float>(j);
			if (d < 0)
			{
				// negative (invalid) entry, so skip
				continue;
			}
			else if (min_d < 0 || d < min_d)
			{
				min_d = d;
				min_k = j;
			}
		}
		// Now we should j -> ith nearest color
		if (min_k == K)
		{
			// we didn't find one for some reason
			printf("Error in sorting. Didn't find min k for %d-th closest color.", i+1);
		}
		else
		{
			// copy into sorted table
			colors_sorted.at<float>(i,0) = colors.at<float>(min_k,0);
			colors_sorted.at<float>(i,1) = colors.at<float>(min_k,1);
			colors_sorted.at<float>(i,2) = colors.at<float>(min_k,2);
			// wipe entry in distance table
			distances.at<float>(min_k) = -1.0;
		}
	}

	return;
}

int find_nearest_to_target(const Mat& centers, int K, const Vec3f t)
{
	// now sort
	int min_k;
	float min_d;
	// find ith lowest distance
	min_k = 0;
	min_d = -1.0; // XXX
	float d;

	// calculate distances
	for (int k=0; k<K; k++)
	{
		d  = pow( t[0] - centers.at<float>(k,0), 2);
		d += pow( t[1] - centers.at<float>(k,1), 2);
		d += pow( t[2] - centers.at<float>(k,2), 2);

		if (min_d < 0 || d < min_d)
		{
			min_d = d;
			min_k = k;
		}
	}

	return min_k;
}

int get_centroids(const Mat& img, int K, Mat & centroids, Mat & colors, Mat & labels)
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
		return 0;
	}

	Size s = img.size();

	// Convert to CV_32F pixel array for kmeans
	Mat px_array(img.total(), 3, CV_32F);
	convert_to_feature_space(img, px_array, s);
	// printf("Converting to float vector.\n");
	// printf("Float array dimensions (%d, %d) with depth %d.\n", px_array.size().height, px_array.size().width, px_array.depth());

	// Do kmeans with global parameters
	Mat centers;
	float clustering_coeff = kmeans(px_array, K, labels, term_crit, KMEANS_ATTEMPTS, KMEANS_FLAGS, centers);
	// printf("Kmeans successful with clustering compactness %f.\n", clustering_coeff );

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

	// XXX: We need to determine if our target object isn't in frame at all 
	// -- perhaps set max_distance threshold for maximum centroid distance from target color in feature space?

	/* populate color reconstruction table */
	int nearest_k = find_nearest_to_target(centers, K, target_ftr);
	// print_dim("centers", centers);
	convert_from_feature_space(centers, colors, Size(K,1));
	// print_dim("color table", colors);

	if (!colors.isContinuous())
	{
		printf("LUT not continuous.");
	}

	return nearest_k;
}

float calc_fill_share(const Mat& labels, int k)
{
	// calculate how much of color k we see in image
	int c = 0;
	int t = labels.total();
	MatConstIterator_<uint8_t> it, it_end;
	it = labels.begin<uint8_t>();
	it_end = labels.end<uint8_t>();
	for (; it!=it_end; ++it)
	{
		// if label matches
		if (*it == k)
		{
			// increment counter
			c++;
		}
	}

	return ((float) c) / t;
}

void reconstruct(const Size& s, const Mat& centroids, const Mat& colors, const Mat& labels, int nearest_k, Mat& out)
{
	out = Mat(s.height, s.width, CV_8UC3);
	MatIterator_<Vec3b> dit, dend;
	MatConstIterator_<uchar> lit = labels.begin<uchar>();
	int i,j;
	i = j = 0;

	for (dit=out.begin<Vec3b>(), dend=out.end<Vec3b>(); dit != dend; ++dit, ++lit)
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
		if (k == nearest_k)
		{
			white = Scalar(0,255,0);
		}
		circle(out, center, radius*1.5, white, thickness, linetype);
		circle(out, center, radius, color, thickness, linetype);
	}

	return;
}

#endif