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
	std::string port_name = "/dev/ttyACM0";
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