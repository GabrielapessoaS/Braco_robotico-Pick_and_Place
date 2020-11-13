# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gabriel/opencv

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gabriel/opencv

# Include any dependencies generated for this target.
include CMakeFiles/hues.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hues.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hues.dir/flags.make

CMakeFiles/hues.dir/hues.cpp.o: CMakeFiles/hues.dir/flags.make
CMakeFiles/hues.dir/hues.cpp.o: hues.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gabriel/opencv/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hues.dir/hues.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hues.dir/hues.cpp.o -c /home/gabriel/opencv/hues.cpp

CMakeFiles/hues.dir/hues.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hues.dir/hues.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gabriel/opencv/hues.cpp > CMakeFiles/hues.dir/hues.cpp.i

CMakeFiles/hues.dir/hues.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hues.dir/hues.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gabriel/opencv/hues.cpp -o CMakeFiles/hues.dir/hues.cpp.s

# Object files for target hues
hues_OBJECTS = \
"CMakeFiles/hues.dir/hues.cpp.o"

# External object files for target hues
hues_EXTERNAL_OBJECTS =

hues: CMakeFiles/hues.dir/hues.cpp.o
hues: CMakeFiles/hues.dir/build.make
hues: /home/gabriel/.local/lib/libopencv_dnn.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_gapi.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_highgui.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_ml.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_objdetect.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_photo.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_stitching.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_video.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_videoio.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_imgcodecs.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_calib3d.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_features2d.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_flann.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_imgproc.so.4.5.0
hues: /home/gabriel/.local/lib/libopencv_core.so.4.5.0
hues: CMakeFiles/hues.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gabriel/opencv/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable hues"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hues.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hues.dir/build: hues

.PHONY : CMakeFiles/hues.dir/build

CMakeFiles/hues.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hues.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hues.dir/clean

CMakeFiles/hues.dir/depend:
	cd /home/gabriel/opencv && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gabriel/opencv /home/gabriel/opencv /home/gabriel/opencv /home/gabriel/opencv /home/gabriel/opencv/CMakeFiles/hues.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hues.dir/depend

