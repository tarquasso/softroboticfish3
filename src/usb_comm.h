// usb_comm.h

#include "ros/ros.h"
#include "fishcode/VisOffset.h"
#include "fishcode/SetSwimMode.h"

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using namespace boost::asio;

#define SYNC_BYTE 0x57
#define BAUD_RATE 57600
#define MAX_MSG_LENGTH 64

// XXX add more message definitions, ie status
#define MSG_ID_SET_SWIM_MODE 0x1
#define MSG_ID_VIS_TARGET 0x3

enum st_rx_t { ST_SYNC, ST_LEN, ST_RX };

class SerialComm
{
private:

	ros::NodeHandle _nh;

	void handle_msg(const char* buf, int msg_len);

	io_service* _io;
	serial_port* _serial;

	boost::mutex _tx_mtx;
	char msg_buf[MAX_MSG_LENGTH];

	ros::Publisher set_swim_mode_rqst_pub;
	ros::Subscriber set_swim_mode_sub;
	ros::Subscriber vis_offset_sub;

	void VisOffset_cb(const fishcode::VisOffset::ConstPtr& vis_msg);
	void SetSwimMode_cb(const fishcode::SetSwimMode::ConstPtr& mode_msg);


public:
	SerialComm(ros::NodeHandle& nh);
	~SerialComm();

	void spin();
};

