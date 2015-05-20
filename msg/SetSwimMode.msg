# SetSwimMode.msg
# Message from supervisor to change swim mode

# Timestamp of command
time timestamp

uint8 mode

uint8 SWIM_MODE_RC = 0
uint8 SWIM_MODE_VIS_SERVO = 1
uint8 SWIM_MODE_CIRCLE = 2
