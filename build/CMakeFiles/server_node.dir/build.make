# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/akshat/flytbase_ws/src/image_cropping

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/akshat/flytbase_ws/src/image_cropping/build

# Include any dependencies generated for this target.
include CMakeFiles/server_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/server_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/server_node.dir/flags.make

CMakeFiles/server_node.dir/src/server_node.cpp.o: CMakeFiles/server_node.dir/flags.make
CMakeFiles/server_node.dir/src/server_node.cpp.o: ../src/server_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/akshat/flytbase_ws/src/image_cropping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/server_node.dir/src/server_node.cpp.o"
	/usr/bin/g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/server_node.dir/src/server_node.cpp.o -c /home/akshat/flytbase_ws/src/image_cropping/src/server_node.cpp

CMakeFiles/server_node.dir/src/server_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/server_node.dir/src/server_node.cpp.i"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/akshat/flytbase_ws/src/image_cropping/src/server_node.cpp > CMakeFiles/server_node.dir/src/server_node.cpp.i

CMakeFiles/server_node.dir/src/server_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/server_node.dir/src/server_node.cpp.s"
	/usr/bin/g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/akshat/flytbase_ws/src/image_cropping/src/server_node.cpp -o CMakeFiles/server_node.dir/src/server_node.cpp.s

CMakeFiles/server_node.dir/src/server_node.cpp.o.requires:

.PHONY : CMakeFiles/server_node.dir/src/server_node.cpp.o.requires

CMakeFiles/server_node.dir/src/server_node.cpp.o.provides: CMakeFiles/server_node.dir/src/server_node.cpp.o.requires
	$(MAKE) -f CMakeFiles/server_node.dir/build.make CMakeFiles/server_node.dir/src/server_node.cpp.o.provides.build
.PHONY : CMakeFiles/server_node.dir/src/server_node.cpp.o.provides

CMakeFiles/server_node.dir/src/server_node.cpp.o.provides.build: CMakeFiles/server_node.dir/src/server_node.cpp.o


# Object files for target server_node
server_node_OBJECTS = \
"CMakeFiles/server_node.dir/src/server_node.cpp.o"

# External object files for target server_node
server_node_EXTERNAL_OBJECTS =

devel/lib/image_cropping/server_node: CMakeFiles/server_node.dir/src/server_node.cpp.o
devel/lib/image_cropping/server_node: CMakeFiles/server_node.dir/build.make
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libcv_bridge.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libimage_transport.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libmessage_filters.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libclass_loader.so
devel/lib/image_cropping/server_node: /usr/lib/libPocoFoundation.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libroslib.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/librospack.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libroscpp.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/librosconsole.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libxmlrpcpp.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libroscpp_serialization.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/librostime.so
devel/lib/image_cropping/server_node: /opt/ros/melodic/lib/libcpp_common.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
devel/lib/image_cropping/server_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
devel/lib/image_cropping/server_node: CMakeFiles/server_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/akshat/flytbase_ws/src/image_cropping/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/image_cropping/server_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/server_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/server_node.dir/build: devel/lib/image_cropping/server_node

.PHONY : CMakeFiles/server_node.dir/build

CMakeFiles/server_node.dir/requires: CMakeFiles/server_node.dir/src/server_node.cpp.o.requires

.PHONY : CMakeFiles/server_node.dir/requires

CMakeFiles/server_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/server_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/server_node.dir/clean

CMakeFiles/server_node.dir/depend:
	cd /home/akshat/flytbase_ws/src/image_cropping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/akshat/flytbase_ws/src/image_cropping /home/akshat/flytbase_ws/src/image_cropping /home/akshat/flytbase_ws/src/image_cropping/build /home/akshat/flytbase_ws/src/image_cropping/build /home/akshat/flytbase_ws/src/image_cropping/build/CMakeFiles/server_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/server_node.dir/depend

