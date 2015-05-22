// commander.cpp

#include <ros/ros.h>
#include <cstdio>
#include <string>

#include <boost/bind.hpp>

#include "commander.h"

using namespace fishcode;

int curr_swim_mode;
ros::Subscriber sub_SetSwimModeRqst;
ros::Subscriber sub_VisOffset;
ros::Publisher pub_SetSwimMode;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "commander");
	ros::NodeHandle nh;

	// Initialize
	curr_swim_mode = SWIM_MODE_IDLE;

	// Start subscribers
	sub_SetSwimModeRqst = nh.subscribe<SetSwimMode>("fishcode/swim_mode_rqst", 1, boost::bind(&cb_SetSwimModeRqst, _1));
	sub_VisOffset = nh.subscribe<VisOffset>("fishcode/vis_offset", 1, boost::bind(&cb_VisOffset, _1));
	pub_SetSwimMode = nh.advertise<SetSwimMode>("fishcode/swim_mode_set", 1);

	ROS_INFO("Commander started.");

	commandSwimMode(curr_swim_mode);
	ROS_INFO("Set swim mode to %d.", curr_swim_mode);

	// XXX Implement target color service client
	
	ros::spin();

	return 0;
}


void cb_SetSwimModeRqst(const SetSwimMode::ConstPtr& msg)
{
	ROS_INFO("Received SetSwimModeRqst to change to mode %d.", msg->mode);
	if (msg->mode == curr_swim_mode)
	{
		ROS_WARN("Requested mode is same as current mode. Seems like mBed and RaspPi do not agree on current swim mode.");
	}

	// XXX Probably should check other things too
	else if (msg->mode >= SWIM_MODE_COUNT || msg->mode < 0)
	{
		ROS_WARN("Requested mode is invalid.");
	}

	else
	{
		ROS_INFO("Transitioning to swim mode %d.", msg->mode);
		curr_swim_mode = msg->mode;

		commandSwimMode(curr_swim_mode);
	}

	return;
}

void cb_VisOffset(const VisOffset::ConstPtr& msg)
{
	return;
}

void commandSwimMode(int mode)
{
	SetSwimMode msg;
	msg.timestamp = ros::Time::now();
	msg.mode = mode;

	pub_SetSwimMode.publish(msg);
}