# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_SOURCE_DIR = /home/kevin/ros_ws/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kevin/ros_ws/catkin_ws/build

# Include any dependencies generated for this target.
include dso_ros/CMakeFiles/dso_live.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include dso_ros/CMakeFiles/dso_live.dir/compiler_depend.make

# Include the progress variables for this target.
include dso_ros/CMakeFiles/dso_live.dir/progress.make

# Include the compile flags for this target's objects.
include dso_ros/CMakeFiles/dso_live.dir/flags.make

dso_ros/CMakeFiles/dso_live.dir/src/main.cpp.o: dso_ros/CMakeFiles/dso_live.dir/flags.make
dso_ros/CMakeFiles/dso_live.dir/src/main.cpp.o: /home/kevin/ros_ws/catkin_ws/src/dso_ros/src/main.cpp
dso_ros/CMakeFiles/dso_live.dir/src/main.cpp.o: dso_ros/CMakeFiles/dso_live.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/ros_ws/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dso_ros/CMakeFiles/dso_live.dir/src/main.cpp.o"
	cd /home/kevin/ros_ws/catkin_ws/build/dso_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT dso_ros/CMakeFiles/dso_live.dir/src/main.cpp.o -MF CMakeFiles/dso_live.dir/src/main.cpp.o.d -o CMakeFiles/dso_live.dir/src/main.cpp.o -c /home/kevin/ros_ws/catkin_ws/src/dso_ros/src/main.cpp

dso_ros/CMakeFiles/dso_live.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dso_live.dir/src/main.cpp.i"
	cd /home/kevin/ros_ws/catkin_ws/build/dso_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kevin/ros_ws/catkin_ws/src/dso_ros/src/main.cpp > CMakeFiles/dso_live.dir/src/main.cpp.i

dso_ros/CMakeFiles/dso_live.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dso_live.dir/src/main.cpp.s"
	cd /home/kevin/ros_ws/catkin_ws/build/dso_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kevin/ros_ws/catkin_ws/src/dso_ros/src/main.cpp -o CMakeFiles/dso_live.dir/src/main.cpp.s

# Object files for target dso_live
dso_live_OBJECTS = \
"CMakeFiles/dso_live.dir/src/main.cpp.o"

# External object files for target dso_live
dso_live_EXTERNAL_OBJECTS =

/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: dso_ros/CMakeFiles/dso_live.dir/src/main.cpp.o
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: dso_ros/CMakeFiles/dso_live.dir/build.make
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /home/kevin/dso/build/lib/libdso.a
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_gapi.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_highgui.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_ml.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_objdetect.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_photo.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_stitching.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_video.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_videoio.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libroscpp.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libpangolin.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libroslib.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librospack.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libcv_bridge.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librosconsole.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librostime.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libcpp_common.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libroscpp.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libroslib.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librospack.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libcv_bridge.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librosconsole.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/librostime.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /opt/ros/melodic/lib/libcpp_common.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libGL.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libGLU.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libGLEW.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libSM.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libICE.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libX11.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libXext.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libpython3.6m.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libdc1394.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libavcodec.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libavformat.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libavutil.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libswscale.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/libOpenNI.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/libOpenNI2.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libpng.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libz.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libjpeg.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libtiff.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/lib/aarch64-linux-gnu/libIlmImf.so
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_dnn.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_imgcodecs.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_calib3d.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_features2d.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_flann.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_imgproc.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: /usr/local/lib/libopencv_core.so.4.5.1
/home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live: dso_ros/CMakeFiles/dso_live.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/ros_ws/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live"
	cd /home/kevin/ros_ws/catkin_ws/build/dso_ros && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dso_live.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dso_ros/CMakeFiles/dso_live.dir/build: /home/kevin/ros_ws/catkin_ws/devel/lib/dso_ros/dso_live
.PHONY : dso_ros/CMakeFiles/dso_live.dir/build

dso_ros/CMakeFiles/dso_live.dir/clean:
	cd /home/kevin/ros_ws/catkin_ws/build/dso_ros && $(CMAKE_COMMAND) -P CMakeFiles/dso_live.dir/cmake_clean.cmake
.PHONY : dso_ros/CMakeFiles/dso_live.dir/clean

dso_ros/CMakeFiles/dso_live.dir/depend:
	cd /home/kevin/ros_ws/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/ros_ws/catkin_ws/src /home/kevin/ros_ws/catkin_ws/src/dso_ros /home/kevin/ros_ws/catkin_ws/build /home/kevin/ros_ws/catkin_ws/build/dso_ros /home/kevin/ros_ws/catkin_ws/build/dso_ros/CMakeFiles/dso_live.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dso_ros/CMakeFiles/dso_live.dir/depend
