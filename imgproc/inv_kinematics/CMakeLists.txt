cmake_minimum_required(VERSION 2.8.12)

PROJECT(libinv_kinematics)

add_library(inv_kinematics STATIC inv_kinematics.c)

install(TARGETS inv_kinematics DESTINATION lib)
install(FILES inv_kinematics.h DESTINATION include)
