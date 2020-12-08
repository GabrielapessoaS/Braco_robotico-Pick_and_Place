cmake_minimum_required(VERSION 2.8.12)

PROJECT(tests)

find_package( OpenCV REQUIRED )

SET(OpenCV_DIR ~/opencv/)

add_subdirectory(inv_kinematics)

include_directories( ${OpenCV_INCLUDE_DIRS} )

MACRO(add_object name)
	ADD_EXECUTABLE(${name} ${name}.cpp)
	TARGET_LINK_LIBRARIES(${name} ${OpenCV_LIBS} pthread pigpio m rt libinv_kinematics)
	install(TARGETS ${name} DESTINATION bin)
ENDMACRO()

add_object(motion)
