# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/ubuntu/Desktop/IR

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Desktop/IR/build

# Include any dependencies generated for this target.
include CMakeFiles/blob.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/blob.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/blob.dir/flags.make

CMakeFiles/blob.dir/src/blob.cpp.o: CMakeFiles/blob.dir/flags.make
CMakeFiles/blob.dir/src/blob.cpp.o: ../src/blob.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/Desktop/IR/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blob.dir/src/blob.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/blob.dir/src/blob.cpp.o -c /home/ubuntu/Desktop/IR/src/blob.cpp

CMakeFiles/blob.dir/src/blob.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blob.dir/src/blob.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/Desktop/IR/src/blob.cpp > CMakeFiles/blob.dir/src/blob.cpp.i

CMakeFiles/blob.dir/src/blob.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blob.dir/src/blob.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/Desktop/IR/src/blob.cpp -o CMakeFiles/blob.dir/src/blob.cpp.s

CMakeFiles/blob.dir/src/blob.cpp.o.requires:
.PHONY : CMakeFiles/blob.dir/src/blob.cpp.o.requires

CMakeFiles/blob.dir/src/blob.cpp.o.provides: CMakeFiles/blob.dir/src/blob.cpp.o.requires
	$(MAKE) -f CMakeFiles/blob.dir/build.make CMakeFiles/blob.dir/src/blob.cpp.o.provides.build
.PHONY : CMakeFiles/blob.dir/src/blob.cpp.o.provides

CMakeFiles/blob.dir/src/blob.cpp.o.provides.build: CMakeFiles/blob.dir/src/blob.cpp.o

CMakeFiles/blob.dir/src/clipper.cpp.o: CMakeFiles/blob.dir/flags.make
CMakeFiles/blob.dir/src/clipper.cpp.o: ../src/clipper.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/Desktop/IR/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/blob.dir/src/clipper.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/blob.dir/src/clipper.cpp.o -c /home/ubuntu/Desktop/IR/src/clipper.cpp

CMakeFiles/blob.dir/src/clipper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/blob.dir/src/clipper.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/Desktop/IR/src/clipper.cpp > CMakeFiles/blob.dir/src/clipper.cpp.i

CMakeFiles/blob.dir/src/clipper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/blob.dir/src/clipper.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/Desktop/IR/src/clipper.cpp -o CMakeFiles/blob.dir/src/clipper.cpp.s

CMakeFiles/blob.dir/src/clipper.cpp.o.requires:
.PHONY : CMakeFiles/blob.dir/src/clipper.cpp.o.requires

CMakeFiles/blob.dir/src/clipper.cpp.o.provides: CMakeFiles/blob.dir/src/clipper.cpp.o.requires
	$(MAKE) -f CMakeFiles/blob.dir/build.make CMakeFiles/blob.dir/src/clipper.cpp.o.provides.build
.PHONY : CMakeFiles/blob.dir/src/clipper.cpp.o.provides

CMakeFiles/blob.dir/src/clipper.cpp.o.provides.build: CMakeFiles/blob.dir/src/clipper.cpp.o

# Object files for target blob
blob_OBJECTS = \
"CMakeFiles/blob.dir/src/blob.cpp.o" \
"CMakeFiles/blob.dir/src/clipper.cpp.o"

# External object files for target blob
blob_EXTERNAL_OBJECTS =

blob: CMakeFiles/blob.dir/src/blob.cpp.o
blob: CMakeFiles/blob.dir/src/clipper.cpp.o
blob: CMakeFiles/blob.dir/build.make
blob: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
blob: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
blob: CMakeFiles/blob.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable blob"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/blob.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/blob.dir/build: blob
.PHONY : CMakeFiles/blob.dir/build

CMakeFiles/blob.dir/requires: CMakeFiles/blob.dir/src/blob.cpp.o.requires
CMakeFiles/blob.dir/requires: CMakeFiles/blob.dir/src/clipper.cpp.o.requires
.PHONY : CMakeFiles/blob.dir/requires

CMakeFiles/blob.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/blob.dir/cmake_clean.cmake
.PHONY : CMakeFiles/blob.dir/clean

CMakeFiles/blob.dir/depend:
	cd /home/ubuntu/Desktop/IR/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Desktop/IR /home/ubuntu/Desktop/IR /home/ubuntu/Desktop/IR/build /home/ubuntu/Desktop/IR/build /home/ubuntu/Desktop/IR/build/CMakeFiles/blob.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/blob.dir/depend

