// usb_comm.h

#include "ros/ros.h"
#include "fishcode/VisOffset.h"
#include "fishcode/SetTargetColorBgr.h"

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
	void handle_msg(const char* buf, int msg_len);

	io_service* _io;
	serial_port* _serial;

	boost::mutex _tx_mtx;

	ros::Subscriber set_swim_mode_sub;
	ros::Subscriber vis_offset_sub;

public:
	SerialComm();
	~SerialComm();

	void spin();
};

