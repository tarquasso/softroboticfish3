// usb_comm.cpp

#include "usb_comm.h"
#include <cstdio>
#include <string>

int main(int argc, char** argv)
{

	return 0;
}

SerialComm::SerialComm()
{
	_nh = ros::NodeHandle("serial_comm");

	std::string port_name = "/dev/ttyUSB0";
	printf("Opening serial port at %s.\n", port_name.c_str());
	_io = new io_service();
	_serial = new serial_port(*_io);
	_serial->open(port_name);
	if (_serial->is_open())
	{
		printf("Success.\n");
	}
	else
	{
		printf("Failed to open port at %s.\n", port_name.c_str());
	}

	// configure port
	_serial->set_option(serial_port_base::baud_rate(BAUD_RATE));
	_serial->set_option(serial_port_base::character_size(8));
	_serial->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
	_serial->set_option(serial_port_base::parity(serial_port_base::parity::none));
	_serial->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));


	// initialize ROS publishers/subscribers
	set_swim_mode_rqst_pub = _nh.advertise<fishcode::SetSwimMode>("fishcode/swim_mode_rqst", 1);
	set_swim_mode_sub =  _nh.subscribe<fishcode::SetSwimMode>("fishcode/swim_mode_set", 1, boost::bind(&SerialComm::SetSwimMode_cb, this, _1));
	vis_offset_sub =  _nh.subscribe<fishcode::VisOffset>("fishcode/vis_offset", 1, boost::bind(&SerialComm::VisOffset_cb, this, _1));

}

SerialComm::~SerialComm()
{
	if (_serial->is_open())
	{
		_serial->close();
	}
	delete _serial;
	delete _io;
}

void SerialComm::spin()
{
	// initialize state variables
	enum st_rx_t state = ST_SYNC;
	int msg_len = 0;
	bool escaping = false;

	// initialize buffer
	char c;
	int rx_buf_in = 0;
	char rx_buf[MAX_MSG_LENGTH];

	while (ros::ok())
	{
		// read a byte
		_serial->read_some(buffer(&c, 1));

		// check for sync byte
        if (c == SYNC_BYTE)
        {
            if (state == ST_SYNC)   // if we're waiting for sync
            {
                state = ST_LEN;     // expect next byte to be length field
                escaping = false;
            }
            else if (escaping)      // if we got unexpected sync byte last time
            {
                escaping = false;	// let it through
            }
            else					// this sync byte is unexpected
            {
            	escaping = true;	// indicate to check for escaped byte next time
            	continue;			// don't further process this byte
            }
        }

        // if this isn't a sync byte, but the last one was
        else if (escaping)
        {
        	state = ST_LEN;			// the last byte was a legit SYNC
        	escaping = false;
        }
            
        // act based on state
        switch (state)
        {
            case (ST_LEN):
                msg_len = c;    // update length field
                rx_buf_in = 0;  // init buffer index
                state = ST_RX;
                break;
            
            case (ST_RX):
                rx_buf[rx_buf_in] = c; // store byte in buffer
                rx_buf_in++;            // increment _rx_buf_in
                if (rx_buf_in == msg_len) // if full message has been received
                {
                    // handle message
                    handle_msg(rx_buf, msg_len);
                    state = ST_SYNC;
                }
                break;

            default:
                // should never get here
            	printf("ERROR: Invalid SerialComm receive state.\n");
                state = ST_SYNC;
                break;
        }
	}

	return;
}

void SerialComm::handle_msg(const char* buf, int msg_len)
{
    int msg_id = buf[0];    // first byte
    switch (msg_id)
    {
        default:
        	// currently no supported message types
            printf("WARN: SerialComm recieved unrecognized message of type %d.\n", msg_id);
            break; // ignore for now
    }
}

void SerialComm::VisOffset_cb(const fishcode::VisOffset::ConstPtr& vis_msg)
{
	char msg_len = 25;

	_tx_mtx.lock(); // claim mutex to modify msg_buf

	msg_buf[0] = SYNC_BYTE;
	msg_buf[1] = msg_len;
	// XXX insert timestamp in bytes 2-9
	msg_buf[10] = vis_msg->b;
	msg_buf[11] = vis_msg->g;
	msg_buf[12] = vis_msg->r;
	uint32_t xoff = (uint32_t) vis_msg->xoff;
	msg_buf[13] = (xoff       ) & 0xFF;
	msg_buf[14] = (xoff >> 8  ) & 0xFF;
	msg_buf[15] = (xoff >> 16 ) & 0xFF;
	msg_buf[16] = (xoff >> 24 ) & 0xFF;
	uint32_t yoff = (uint32_t) vis_msg->yoff;
	msg_buf[17] = (yoff       ) & 0xFF;
	msg_buf[18] = (yoff >> 8  ) & 0xFF;
	msg_buf[19] = (yoff >> 16 ) & 0xFF;
	msg_buf[20] = (yoff >> 24 ) & 0xFF;
	uint32_t fill_share = (uint32_t) vis_msg->fill_share;
	msg_buf[21] = (fill_share       ) & 0xFF;
	msg_buf[22] = (fill_share >> 8  ) & 0xFF;
	msg_buf[23] = (fill_share >> 16 ) & 0xFF;
	msg_buf[24] = (fill_share >> 24 ) & 0xFF;

	// now send it over serial
	write(*_serial, buffer(msg_buf, msg_len));


	_tx_mtx.unlock(); // release it

	return;
}

void SerialComm::SetSwimMode_cb(const fishcode::SetSwimMode::ConstPtr& mode_msg)
{
	char msg_len = 11;

	_tx_mtx.lock(); // claim mutex to modify msg_buf

	msg_buf[0] = SYNC_BYTE;
	msg_buf[1] = msg_len;
	// XXX insert timestamp in bytes 2-9
	msg_buf[10] = mode_msg->mode;
	
	// now send it over serial
	write(*_serial, buffer(msg_buf, msg_len));

	_tx_mtx.unlock(); // release it

	return;
}