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
CMAKE_SOURCE_DIR = /home/dylan/3D/3d-Scanning/final/reconstruction

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dylan/3D/3d-Scanning/final/reconstruction/build

# Include any dependencies generated for this target.
include CMakeFiles/matching.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/matching.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/matching.dir/flags.make

CMakeFiles/matching.dir/matching.cpp.o: CMakeFiles/matching.dir/flags.make
CMakeFiles/matching.dir/matching.cpp.o: ../matching.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/matching.dir/matching.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matching.dir/matching.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/matching.cpp

CMakeFiles/matching.dir/matching.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matching.dir/matching.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/matching.cpp > CMakeFiles/matching.dir/matching.cpp.i

CMakeFiles/matching.dir/matching.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matching.dir/matching.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/matching.cpp -o CMakeFiles/matching.dir/matching.cpp.s

CMakeFiles/matching.dir/matching.cpp.o.requires:

.PHONY : CMakeFiles/matching.dir/matching.cpp.o.requires

CMakeFiles/matching.dir/matching.cpp.o.provides: CMakeFiles/matching.dir/matching.cpp.o.requires
	$(MAKE) -f CMakeFiles/matching.dir/build.make CMakeFiles/matching.dir/matching.cpp.o.provides.build
.PHONY : CMakeFiles/matching.dir/matching.cpp.o.provides

CMakeFiles/matching.dir/matching.cpp.o.provides.build: CMakeFiles/matching.dir/matching.cpp.o


CMakeFiles/matching.dir/orb.cpp.o: CMakeFiles/matching.dir/flags.make
CMakeFiles/matching.dir/orb.cpp.o: ../orb.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/matching.dir/orb.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matching.dir/orb.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/orb.cpp

CMakeFiles/matching.dir/orb.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matching.dir/orb.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/orb.cpp > CMakeFiles/matching.dir/orb.cpp.i

CMakeFiles/matching.dir/orb.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matching.dir/orb.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/orb.cpp -o CMakeFiles/matching.dir/orb.cpp.s

CMakeFiles/matching.dir/orb.cpp.o.requires:

.PHONY : CMakeFiles/matching.dir/orb.cpp.o.requires

CMakeFiles/matching.dir/orb.cpp.o.provides: CMakeFiles/matching.dir/orb.cpp.o.requires
	$(MAKE) -f CMakeFiles/matching.dir/build.make CMakeFiles/matching.dir/orb.cpp.o.provides.build
.PHONY : CMakeFiles/matching.dir/orb.cpp.o.provides

CMakeFiles/matching.dir/orb.cpp.o.provides.build: CMakeFiles/matching.dir/orb.cpp.o


CMakeFiles/matching.dir/undistort.cpp.o: CMakeFiles/matching.dir/flags.make
CMakeFiles/matching.dir/undistort.cpp.o: ../undistort.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/matching.dir/undistort.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matching.dir/undistort.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/undistort.cpp

CMakeFiles/matching.dir/undistort.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matching.dir/undistort.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/undistort.cpp > CMakeFiles/matching.dir/undistort.cpp.i

CMakeFiles/matching.dir/undistort.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matching.dir/undistort.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/undistort.cpp -o CMakeFiles/matching.dir/undistort.cpp.s

CMakeFiles/matching.dir/undistort.cpp.o.requires:

.PHONY : CMakeFiles/matching.dir/undistort.cpp.o.requires

CMakeFiles/matching.dir/undistort.cpp.o.provides: CMakeFiles/matching.dir/undistort.cpp.o.requires
	$(MAKE) -f CMakeFiles/matching.dir/build.make CMakeFiles/matching.dir/undistort.cpp.o.provides.build
.PHONY : CMakeFiles/matching.dir/undistort.cpp.o.provides

CMakeFiles/matching.dir/undistort.cpp.o.provides.build: CMakeFiles/matching.dir/undistort.cpp.o


CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o: CMakeFiles/matching.dir/flags.make
CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o: ../FindEssentialMatrix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/FindEssentialMatrix.cpp

CMakeFiles/matching.dir/FindEssentialMatrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matching.dir/FindEssentialMatrix.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/FindEssentialMatrix.cpp > CMakeFiles/matching.dir/FindEssentialMatrix.cpp.i

CMakeFiles/matching.dir/FindEssentialMatrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matching.dir/FindEssentialMatrix.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/FindEssentialMatrix.cpp -o CMakeFiles/matching.dir/FindEssentialMatrix.cpp.s

CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o.requires:

.PHONY : CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o.requires

CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o.provides: CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o.requires
	$(MAKE) -f CMakeFiles/matching.dir/build.make CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o.provides.build
.PHONY : CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o.provides

CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o.provides.build: CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o


CMakeFiles/matching.dir/RecoverRT.cpp.o: CMakeFiles/matching.dir/flags.make
CMakeFiles/matching.dir/RecoverRT.cpp.o: ../RecoverRT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/matching.dir/RecoverRT.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matching.dir/RecoverRT.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/RecoverRT.cpp

CMakeFiles/matching.dir/RecoverRT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matching.dir/RecoverRT.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/RecoverRT.cpp > CMakeFiles/matching.dir/RecoverRT.cpp.i

CMakeFiles/matching.dir/RecoverRT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matching.dir/RecoverRT.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/RecoverRT.cpp -o CMakeFiles/matching.dir/RecoverRT.cpp.s

CMakeFiles/matching.dir/RecoverRT.cpp.o.requires:

.PHONY : CMakeFiles/matching.dir/RecoverRT.cpp.o.requires

CMakeFiles/matching.dir/RecoverRT.cpp.o.provides: CMakeFiles/matching.dir/RecoverRT.cpp.o.requires
	$(MAKE) -f CMakeFiles/matching.dir/build.make CMakeFiles/matching.dir/RecoverRT.cpp.o.provides.build
.PHONY : CMakeFiles/matching.dir/RecoverRT.cpp.o.provides

CMakeFiles/matching.dir/RecoverRT.cpp.o.provides.build: CMakeFiles/matching.dir/RecoverRT.cpp.o


CMakeFiles/matching.dir/helper.cpp.o: CMakeFiles/matching.dir/flags.make
CMakeFiles/matching.dir/helper.cpp.o: ../helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/matching.dir/helper.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matching.dir/helper.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/helper.cpp

CMakeFiles/matching.dir/helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matching.dir/helper.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/helper.cpp > CMakeFiles/matching.dir/helper.cpp.i

CMakeFiles/matching.dir/helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matching.dir/helper.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/helper.cpp -o CMakeFiles/matching.dir/helper.cpp.s

CMakeFiles/matching.dir/helper.cpp.o.requires:

.PHONY : CMakeFiles/matching.dir/helper.cpp.o.requires

CMakeFiles/matching.dir/helper.cpp.o.provides: CMakeFiles/matching.dir/helper.cpp.o.requires
	$(MAKE) -f CMakeFiles/matching.dir/build.make CMakeFiles/matching.dir/helper.cpp.o.provides.build
.PHONY : CMakeFiles/matching.dir/helper.cpp.o.provides

CMakeFiles/matching.dir/helper.cpp.o.provides.build: CMakeFiles/matching.dir/helper.cpp.o


CMakeFiles/matching.dir/pointcloud.cpp.o: CMakeFiles/matching.dir/flags.make
CMakeFiles/matching.dir/pointcloud.cpp.o: ../pointcloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/matching.dir/pointcloud.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/matching.dir/pointcloud.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/pointcloud.cpp

CMakeFiles/matching.dir/pointcloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/matching.dir/pointcloud.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/pointcloud.cpp > CMakeFiles/matching.dir/pointcloud.cpp.i

CMakeFiles/matching.dir/pointcloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/matching.dir/pointcloud.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/pointcloud.cpp -o CMakeFiles/matching.dir/pointcloud.cpp.s

CMakeFiles/matching.dir/pointcloud.cpp.o.requires:

.PHONY : CMakeFiles/matching.dir/pointcloud.cpp.o.requires

CMakeFiles/matching.dir/pointcloud.cpp.o.provides: CMakeFiles/matching.dir/pointcloud.cpp.o.requires
	$(MAKE) -f CMakeFiles/matching.dir/build.make CMakeFiles/matching.dir/pointcloud.cpp.o.provides.build
.PHONY : CMakeFiles/matching.dir/pointcloud.cpp.o.provides

CMakeFiles/matching.dir/pointcloud.cpp.o.provides.build: CMakeFiles/matching.dir/pointcloud.cpp.o


# Object files for target matching
matching_OBJECTS = \
"CMakeFiles/matching.dir/matching.cpp.o" \
"CMakeFiles/matching.dir/orb.cpp.o" \
"CMakeFiles/matching.dir/undistort.cpp.o" \
"CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o" \
"CMakeFiles/matching.dir/RecoverRT.cpp.o" \
"CMakeFiles/matching.dir/helper.cpp.o" \
"CMakeFiles/matching.dir/pointcloud.cpp.o"

# External object files for target matching
matching_EXTERNAL_OBJECTS =

matching: CMakeFiles/matching.dir/matching.cpp.o
matching: CMakeFiles/matching.dir/orb.cpp.o
matching: CMakeFiles/matching.dir/undistort.cpp.o
matching: CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o
matching: CMakeFiles/matching.dir/RecoverRT.cpp.o
matching: CMakeFiles/matching.dir/helper.cpp.o
matching: CMakeFiles/matching.dir/pointcloud.cpp.o
matching: CMakeFiles/matching.dir/build.make
matching: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
matching: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
matching: CMakeFiles/matching.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable matching"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/matching.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/matching.dir/build: matching

.PHONY : CMakeFiles/matching.dir/build

CMakeFiles/matching.dir/requires: CMakeFiles/matching.dir/matching.cpp.o.requires
CMakeFiles/matching.dir/requires: CMakeFiles/matching.dir/orb.cpp.o.requires
CMakeFiles/matching.dir/requires: CMakeFiles/matching.dir/undistort.cpp.o.requires
CMakeFiles/matching.dir/requires: CMakeFiles/matching.dir/FindEssentialMatrix.cpp.o.requires
CMakeFiles/matching.dir/requires: CMakeFiles/matching.dir/RecoverRT.cpp.o.requires
CMakeFiles/matching.dir/requires: CMakeFiles/matching.dir/helper.cpp.o.requires
CMakeFiles/matching.dir/requires: CMakeFiles/matching.dir/pointcloud.cpp.o.requires

.PHONY : CMakeFiles/matching.dir/requires

CMakeFiles/matching.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/matching.dir/cmake_clean.cmake
.PHONY : CMakeFiles/matching.dir/clean

CMakeFiles/matching.dir/depend:
	cd /home/dylan/3D/3d-Scanning/final/reconstruction/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dylan/3D/3d-Scanning/final/reconstruction /home/dylan/3D/3d-Scanning/final/reconstruction /home/dylan/3D/3d-Scanning/final/reconstruction/build /home/dylan/3D/3d-Scanning/final/reconstruction/build /home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles/matching.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/matching.dir/depend
