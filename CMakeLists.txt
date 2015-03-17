cmake_minimum_required(VERSION 2.8)
project(robofish)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( vis_servo
	src/vis_servo.cpp
	)
target_link_libraries( vis_servo
	${OpenCV_LIBS}
	)
