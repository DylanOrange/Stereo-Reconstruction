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
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/main.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/main.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/main.cpp

CMakeFiles/main.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/main.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/main.cpp > CMakeFiles/main.dir/main.cpp.i

CMakeFiles/main.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/main.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/main.cpp -o CMakeFiles/main.dir/main.cpp.s

CMakeFiles/main.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/main.cpp.o.requires

CMakeFiles/main.dir/main.cpp.o.provides: CMakeFiles/main.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/main.cpp.o.provides

CMakeFiles/main.dir/main.cpp.o.provides.build: CMakeFiles/main.dir/main.cpp.o


CMakeFiles/main.dir/orb.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/orb.cpp.o: ../orb.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/main.dir/orb.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/orb.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/orb.cpp

CMakeFiles/main.dir/orb.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/orb.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/orb.cpp > CMakeFiles/main.dir/orb.cpp.i

CMakeFiles/main.dir/orb.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/orb.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/orb.cpp -o CMakeFiles/main.dir/orb.cpp.s

CMakeFiles/main.dir/orb.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/orb.cpp.o.requires

CMakeFiles/main.dir/orb.cpp.o.provides: CMakeFiles/main.dir/orb.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/orb.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/orb.cpp.o.provides

CMakeFiles/main.dir/orb.cpp.o.provides.build: CMakeFiles/main.dir/orb.cpp.o


CMakeFiles/main.dir/sift.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/sift.cpp.o: ../sift.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/sift.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/sift.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/sift.cpp

CMakeFiles/main.dir/sift.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/sift.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/sift.cpp > CMakeFiles/main.dir/sift.cpp.i

CMakeFiles/main.dir/sift.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/sift.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/sift.cpp -o CMakeFiles/main.dir/sift.cpp.s

CMakeFiles/main.dir/sift.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/sift.cpp.o.requires

CMakeFiles/main.dir/sift.cpp.o.provides: CMakeFiles/main.dir/sift.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/sift.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/sift.cpp.o.provides

CMakeFiles/main.dir/sift.cpp.o.provides.build: CMakeFiles/main.dir/sift.cpp.o


CMakeFiles/main.dir/undistort.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/undistort.cpp.o: ../undistort.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/main.dir/undistort.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/undistort.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/undistort.cpp

CMakeFiles/main.dir/undistort.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/undistort.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/undistort.cpp > CMakeFiles/main.dir/undistort.cpp.i

CMakeFiles/main.dir/undistort.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/undistort.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/undistort.cpp -o CMakeFiles/main.dir/undistort.cpp.s

CMakeFiles/main.dir/undistort.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/undistort.cpp.o.requires

CMakeFiles/main.dir/undistort.cpp.o.provides: CMakeFiles/main.dir/undistort.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/undistort.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/undistort.cpp.o.provides

CMakeFiles/main.dir/undistort.cpp.o.provides.build: CMakeFiles/main.dir/undistort.cpp.o


CMakeFiles/main.dir/FindEssentialMatrix.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/FindEssentialMatrix.cpp.o: ../FindEssentialMatrix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/main.dir/FindEssentialMatrix.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/FindEssentialMatrix.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/FindEssentialMatrix.cpp

CMakeFiles/main.dir/FindEssentialMatrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/FindEssentialMatrix.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/FindEssentialMatrix.cpp > CMakeFiles/main.dir/FindEssentialMatrix.cpp.i

CMakeFiles/main.dir/FindEssentialMatrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/FindEssentialMatrix.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/FindEssentialMatrix.cpp -o CMakeFiles/main.dir/FindEssentialMatrix.cpp.s

CMakeFiles/main.dir/FindEssentialMatrix.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/FindEssentialMatrix.cpp.o.requires

CMakeFiles/main.dir/FindEssentialMatrix.cpp.o.provides: CMakeFiles/main.dir/FindEssentialMatrix.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/FindEssentialMatrix.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/FindEssentialMatrix.cpp.o.provides

CMakeFiles/main.dir/FindEssentialMatrix.cpp.o.provides.build: CMakeFiles/main.dir/FindEssentialMatrix.cpp.o


CMakeFiles/main.dir/RecoverRT.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/RecoverRT.cpp.o: ../RecoverRT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/main.dir/RecoverRT.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/RecoverRT.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/RecoverRT.cpp

CMakeFiles/main.dir/RecoverRT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/RecoverRT.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/RecoverRT.cpp > CMakeFiles/main.dir/RecoverRT.cpp.i

CMakeFiles/main.dir/RecoverRT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/RecoverRT.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/RecoverRT.cpp -o CMakeFiles/main.dir/RecoverRT.cpp.s

CMakeFiles/main.dir/RecoverRT.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/RecoverRT.cpp.o.requires

CMakeFiles/main.dir/RecoverRT.cpp.o.provides: CMakeFiles/main.dir/RecoverRT.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/RecoverRT.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/RecoverRT.cpp.o.provides

CMakeFiles/main.dir/RecoverRT.cpp.o.provides.build: CMakeFiles/main.dir/RecoverRT.cpp.o


CMakeFiles/main.dir/helper.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/helper.cpp.o: ../helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/main.dir/helper.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/helper.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/helper.cpp

CMakeFiles/main.dir/helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/helper.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/helper.cpp > CMakeFiles/main.dir/helper.cpp.i

CMakeFiles/main.dir/helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/helper.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/helper.cpp -o CMakeFiles/main.dir/helper.cpp.s

CMakeFiles/main.dir/helper.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/helper.cpp.o.requires

CMakeFiles/main.dir/helper.cpp.o.provides: CMakeFiles/main.dir/helper.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/helper.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/helper.cpp.o.provides

CMakeFiles/main.dir/helper.cpp.o.provides.build: CMakeFiles/main.dir/helper.cpp.o


CMakeFiles/main.dir/Rectify_KITTI.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/Rectify_KITTI.cpp.o: ../Rectify_KITTI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/main.dir/Rectify_KITTI.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/Rectify_KITTI.cpp.o -c /home/dylan/3D/3d-Scanning/final/reconstruction/Rectify_KITTI.cpp

CMakeFiles/main.dir/Rectify_KITTI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/Rectify_KITTI.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dylan/3D/3d-Scanning/final/reconstruction/Rectify_KITTI.cpp > CMakeFiles/main.dir/Rectify_KITTI.cpp.i

CMakeFiles/main.dir/Rectify_KITTI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/Rectify_KITTI.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dylan/3D/3d-Scanning/final/reconstruction/Rectify_KITTI.cpp -o CMakeFiles/main.dir/Rectify_KITTI.cpp.s

CMakeFiles/main.dir/Rectify_KITTI.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/Rectify_KITTI.cpp.o.requires

CMakeFiles/main.dir/Rectify_KITTI.cpp.o.provides: CMakeFiles/main.dir/Rectify_KITTI.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/Rectify_KITTI.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/Rectify_KITTI.cpp.o.provides

CMakeFiles/main.dir/Rectify_KITTI.cpp.o.provides.build: CMakeFiles/main.dir/Rectify_KITTI.cpp.o


# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/main.cpp.o" \
"CMakeFiles/main.dir/orb.cpp.o" \
"CMakeFiles/main.dir/sift.cpp.o" \
"CMakeFiles/main.dir/undistort.cpp.o" \
"CMakeFiles/main.dir/FindEssentialMatrix.cpp.o" \
"CMakeFiles/main.dir/RecoverRT.cpp.o" \
"CMakeFiles/main.dir/helper.cpp.o" \
"CMakeFiles/main.dir/Rectify_KITTI.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

main: CMakeFiles/main.dir/main.cpp.o
main: CMakeFiles/main.dir/orb.cpp.o
main: CMakeFiles/main.dir/sift.cpp.o
main: CMakeFiles/main.dir/undistort.cpp.o
main: CMakeFiles/main.dir/FindEssentialMatrix.cpp.o
main: CMakeFiles/main.dir/RecoverRT.cpp.o
main: CMakeFiles/main.dir/helper.cpp.o
main: CMakeFiles/main.dir/Rectify_KITTI.cpp.o
main: CMakeFiles/main.dir/build.make
main: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
main: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/requires: CMakeFiles/main.dir/main.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/orb.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/sift.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/undistort.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/FindEssentialMatrix.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/RecoverRT.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/helper.cpp.o.requires
CMakeFiles/main.dir/requires: CMakeFiles/main.dir/Rectify_KITTI.cpp.o.requires

.PHONY : CMakeFiles/main.dir/requires

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/dylan/3D/3d-Scanning/final/reconstruction/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dylan/3D/3d-Scanning/final/reconstruction /home/dylan/3D/3d-Scanning/final/reconstruction /home/dylan/3D/3d-Scanning/final/reconstruction/build /home/dylan/3D/3d-Scanning/final/reconstruction/build /home/dylan/3D/3d-Scanning/final/reconstruction/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend
