# comm.py

import lcm
import serial
import struct

BAUD_RATE = 57600
SYNC_BYTE = 0x57

MSG_ID_SET_CMD_MODE 	= 0x1
MSG_ID_VIS_TARGET 		= 0x3

# serial port to mbed
mbed = None

def encode_payload(payload):
	# Adds msg meta-data (sync and length fields)
	# And escapes any accidental sync bytes
	# Inputs:
	# payload: data payload

	# first escape-check payload
	esc_payload = bytearray()
	for c in payload:
		esc_payload.append(c)
		# check if we need to escape
		if c is SYNC_BYTE:
			# append again
			esc_payload.append(c)

	# now get length of escaped payload
	l = len(esc_payload)

	tx_buf = bytearray()
	tx_buf.append(SYNC_BYTE)
	tx_buf.append(l)
	if l == SYNC_BYTE:		# escape length byte if necessary
		tx_buf.append(l) 
	tx_buf.extend(esc_payload)

	return tx_buf

def handle_msg(rx_buf, len):
	# Inputs:
	# rx_buf: bytearray containing message payload (id and data)
	# len: number of bytes
	
	msgid = rx_buf[0]

	# handle message

	return

def send_SetCmdMode(mode_msg):
	# Inputs:
	# cmd_mode: SetCmdMode msg

	if mbed is None:
		# can't do anything
		return


	msgid = MSG_ID_SET_CMD_MODE
	mode = mode_msg.mode

	# package message
	payload = struct.pack('<BB', msgid, mode)
	send_buf = encode_payload(payload)

	# send over serial
	mbed.write(send_buf)
	return

def send_VisTarget(vis_target_msg):
	# Inputs:
	# vis_target_msg

	if mbed is None:
		return

	msgid = MSG_ID_VIS_TARGET
	in_sight = vis_target_msg.in_sight
	x_off = vis_target_msg.x_off
	y_off = vis_target_msg.y_off
	dist = vis_target_msg.distance

	# package message
	payload = struct.pack('<BBfff')
	send_buf = encode_payload(payload)

	# send over serial
	mbed.write(send_buf)
	return