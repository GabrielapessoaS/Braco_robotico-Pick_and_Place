cmake_minimum_required(VERSION 2.8.12)

PROJECT(tests)

find_package( OpenCV REQUIRED )

SET(OpenCV_DIR ~/opencv/)

include_directories( include ${OpenCV_INCLUDE_DIRS} )

file(GLOB SOURCES "src/*.cpp")

add_executable(motion ${SOURCES})
add_library(inv_kinematics "src/inv_kinematics.c")
add_library(buttons "src/buttons.c")
target_link_libraries(motion ${OpenCV_LIBS} pthread pigpio m rt inv_kinematics buttons)
install(TARGETS motion DESTINATION bin)
