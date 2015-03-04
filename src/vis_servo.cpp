// vis_servo.cpp

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

using namespace cv;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vis_servo");
	ros::NodeHandle nh;

	std::string img_path = ros::package::getPath("fish_raspi") + "/plot.png";

	Mat img = imread(img_path.c_str());
	if (img.data == NULL)
	{
		ROS_ERROR("Could not read input file.");
		return 1;
	}

	Size s = img.size();
	ROS_INFO("Imported image of size (%d, %d).", s.width, s.height);

	namedWindow("image", WINDOW_AUTOSIZE);
	imshow("image", img);
	waitKey();

	return 0;
}