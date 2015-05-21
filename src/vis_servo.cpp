// vis_servo.cpp

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <raspicam/raspicam_cv.h>

#include <string>
#include <cstring>
#include <stdio.h>
#include <iostream>
#include <cstdlib>

#include "util.h"
#include "vis_servo.h"

#include "ros/ros.h"
#include "fishcode/VisOffset.h"
#include "fishcode/SetTargetColorBgr.h"

#define RES_W 320
#define RES_H 240

using namespace cv;

int cv_test(int argc, char** argv);
int cam_poll(int argc, char** argv);

// Ros service wrapper
bool set_target_bgr_cb(fishcode::SetTargetColorBgr::Request &req, fishcode::SetTargetColorBgr::Response &res)
{
	Vec3b bgr(req.b, req.g, req.r);
	set_target_bgr(bgr);
	res.success = true;
	return true;
}

int main(int argc, char** argv)
{
	cam_poll(argc, argv);
	//cv_test(argc, argv);

	return 0;
}

int cv_test(int argc, char** argv)
{
	// Extract command line arguments
	int K = 3;
	std::string filename("P4_Color_2.jpg");
	for (int i=0; i<argc; i++)
	{
		// Parse string
		if (strncmp(argv[i], "K=", 2) == 0)
		{
			K = atoi(argv[i] + 2);
		}
		else if (strncmp(argv[i], "file=", 5) == 0)
		{
			filename = std::string(argv[i] + 5);
		}
		else
		{
			printf("Invalid argument: %s.\n", argv[i]);
		}
	}

	printf("K = %d.\n", K);
	std::string img_path = pkg_path(1);
	img_path += '/' + filename;
	printf("Importing image file %s.\n", filename.c_str());

	// define target BGR
	Vec3b target_bgr;
	target_bgr[0] = 89;	// blue
	target_bgr[1] = 67;	// green
	target_bgr[2] = 151; // red
	initialize(K, target_bgr);

	Mat img = imread(img_path, IMREAD_COLOR);
	namedWindow("image", WINDOW_AUTOSIZE);
	imshow("image", img);

	Mat centroids(3, K, CV_16U);
	Mat colors(1, K, CV_8UC3);
	Mat labels(img.total(), 1, CV_8U);
	int nearest_centroid = get_centroids(img, K, centroids, colors, labels);

	std::cout << centroids << std::endl;

	Mat out;
	reconstruct(img.size(), centroids, colors, labels, nearest_centroid, out);
	
	cleanup();
	return 0;
}
int cam_poll(int argc, char** argv)
{
	// default arguments
	int K = 3;
	Vec3b target_bgr;
	target_bgr[0] = 89;	// blue
	target_bgr[1] = 67;	// green
	target_bgr[2] = 151; // red
	std::string img_path = pkg_path(1) + "/images/";

	// parse cmd line
	for (int i=1; i<argc; i++)
	{
		// Parse string
		if (strncmp(argv[i], "K=", 2) == 0)
		{
			K = atoi(argv[i] + 2);
		}
		else
		{
			printf("Invalid argument: %s.\n", argv[i]);
		}
	}

	// initialize cv 
	initialize(K, target_bgr);

	// initialize raspicam
	raspicam::RaspiCam_Cv cam;

	// configure camera
	cam.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, 1280);	// capture at full resolution
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, 980);
	cam.set(CV_CAP_PROP_BRIGHTNESS, 50);
	cam.set(CV_CAP_PROP_CONTRAST, 50);
	cam.set(CV_CAP_PROP_SATURATION, 50);
	cam.set(CV_CAP_PROP_GAIN, 50);

	printf("Opening raspicam.\n");
	cam.open();	// open and start capturing
	if (!cam.isOpened())
	{
		printf("Failed to open raspicam.\n");
		return 1;
	}

	// initialize ros
	ros::init(argc, argv, "vis_servo");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<fishcode::VisOffset>("vis_offset", 5);
	ros::ServiceServer set_bgr_srv = nh.advertiseService("set_target_bgr", set_target_bgr_cb);

	int mat_shape[2] = {RES_W, RES_H};

	int frame_id = 0;
	char* filename = new char[10];
	float fill_share;
	
	Mat frame_raw;
	Mat frame;

	while (ros::ok())
	{
		// grab image
		cam.grab();
		ros::Time now = ros::Time::now();

		// import into Mat object
		cam.retrieve(frame_raw);
		ROS_INFO("Frame captured.");
//		print_dim("Frame_Raw", frame_raw);
		resize(frame_raw, frame, Size(RES_W, RES_H));
//		print_dim("Frame Reduced", frame);

		// save image to file for testing purposes
//		std::sprintf(filename, "frame%d.jpg", frame_id);
//		imwrite((img_path + filename).c_str(), frame);
//		printf("Frame captured and saved in %s.\n",(img_path+filename).c_str());

		// calculate centroids
		ROS_INFO("Calculating centroids.");
		Mat centroids(3, K, CV_16U);
		Mat colors(1, K, CV_8UC3);
		Mat labels(frame.total(), 1, CV_8U);
		int nearest_centroid = get_centroids(frame, K, centroids, colors, labels);
		float fill_share = calc_fill_share(labels, nearest_centroid);

		float xoff = ((float) (centroids.at<uint8_t>(0,nearest_centroid) - (RES_W/2))) / RES_W;
		float yoff = ((float) (centroids.at<uint8_t>(1,nearest_centroid) - (RES_H/2))) / RES_H;

		// now publish
		fishcode::VisOffset vmsg;
		vmsg.timestamp = now;
		vmsg.xoff = yoff;	// account for camera mounting orientation here
		vmsg.yoff = xoff;
		vmsg.b = target_bgr[0];
		vmsg.g = target_bgr[1];
		vmsg.r = target_bgr[2];
		vmsg.fill_share = fill_share;

		pub.publish(vmsg);
		ROS_INFO("Vis offset to target is (%f, %f), fill share is %f", vmsg.xoff, vmsg.yoff, vmsg.fill_share);

		// reconstruct image
//		Mat out;
//		reconstruct(frame.size(), centroids, colors, labels, nearest_centroid, out);
//		std::sprintf(filename, "frame%d_reduced.jpg", frame_id);
//		imwrite((img_path + filename).c_str(), frame);
//		printf("Reduced frame saved in %s.\n",(img_path+filename).c_str());


		printf("\n");
		frame_id++;
	}

	delete[] filename;
	// release camera
	cam.release();
	// cleanup cv
	cleanup();
	return 0;
}
