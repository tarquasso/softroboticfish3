// vis_servo.cpp

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

#include <iostream>

using namespace cv;

float lutFloat[256];

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vis_servo");
	ros::NodeHandle nh;

	std::string img_path = ros::package::getPath("fish_raspi") + "/plot.png";

	Mat img = imread(img_path.c_str(), CV_LOAD_IMAGE_COLOR);
	if (img.data == NULL)
	{
		ROS_ERROR("Could not read input file.");
		return 1;
	}

	Size s = img.size();
	ROS_INFO("Imported %d-channel image of size (%d, %d).", img.channels(), s.width, s.height);
	switch (img.depth())
	{
		case CV_32F:
			ROS_INFO("Image depth is CV_32F.");
			break;
		case CV_8U:
			ROS_INFO("Image depth is CV_8U.");
			break;
		default:
			ROS_INFO("Unexpected image depth.");
			break;
	}

	namedWindow("image", WINDOW_AUTOSIZE);
	imshow("image", img);

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
	ROS_INFO("Converting to float vector.");
	LUT(img.reshape(1, img.total()), lutFloat, px_array);
	// Prepare other arguments for kmeans
	int K = 4;
	TermCriteria termCrit(TermCriteria::COUNT + TermCriteria::EPS, 50, 0.001);
	Mat labels(img.total(), 1, CV_8U);
	Mat centers(K, img.channels(), CV_32F);
	int flags = KMEANS_RANDOM_CENTERS;
	int attempts = 3;

	float clustering_coeff = kmeans(px_array, K, labels, termCrit, attempts, flags, centers);
	ROS_INFO("Kmeans successful with clustering coefficient %f.", clustering_coeff );

	// prepare reconstruction table
	Mat colorTable(1, K, CV_8UC3);
	for (u=0; u<K; u++)
	{
		Vec3b color;
		color[0] = centers.at<float>(u, 0) * 256;
		color[1] = centers.at<float>(u, 1) * 256;
		color[2] = centers.at<float>(u, 2) * 256;
		colorTable.at<Vec3b>(0, u) = color;
	}
	
	if (!colorTable.isContinuous())
	{
		ROS_ERROR("LUT not continuous.");
	}

	switch (colorTable.depth())
	{
		case CV_8U:
			ROS_INFO("Reconstruction table completed. %d channels with depth CV_8U. Total = %d.", colorTable.channels(), (int) colorTable.total());
			break;
		default:
			ROS_INFO("Reconstruction table completed. %d channels with unrecognized depth. Total = %d.", colorTable.channels(), (int) colorTable.total());
			break;
	}

	std::cout << colorTable << std::endl;

	Mat r_img(s.height, s.width, CV_8UC3);
	MatIterator_<Vec3b> dit, dend;
	MatIterator_<uchar> lit = labels.begin<uchar>();
	for (dit=r_img.begin<Vec3b>(), dend=r_img.end<Vec3b>(); dit != dend; ++dit, ++lit)
	{
		*dit = colorTable.at<Vec3b>(*lit);
	}

	namedWindow("reduced image", WINDOW_AUTOSIZE);
	imshow("reduced image", r_img);
	waitKey();
	return 0;
}