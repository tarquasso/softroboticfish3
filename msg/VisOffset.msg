# VisOffset.msg
# Message describing results of visual offset

# Timestamp of measurement
time timestamp

# Target BGR
uint8 b
uint8 g
uint8 r

# Offset of target
float32 xoff  	#[-1,1]
float32 yoff  	#[-1,1]

# Perceived "size"
float32 fill_share	#[0,1] percentage of total resolution occupied by target color
