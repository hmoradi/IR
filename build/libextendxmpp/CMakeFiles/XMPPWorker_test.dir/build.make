# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/hessam/Desktop/HotSpotter/IR

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hessam/Desktop/HotSpotter/IR/build

# Include any dependencies generated for this target.
include libextendxmpp/CMakeFiles/XMPPWorker_test.dir/depend.make

# Include the progress variables for this target.
include libextendxmpp/CMakeFiles/XMPPWorker_test.dir/progress.make

# Include the compile flags for this target's objects.
include libextendxmpp/CMakeFiles/XMPPWorker_test.dir/flags.make

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o: libextendxmpp/CMakeFiles/XMPPWorker_test.dir/flags.make
libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o: ../libextendxmpp/XMPPWorker_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hessam/Desktop/HotSpotter/IR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o"
	cd /home/hessam/Desktop/HotSpotter/IR/build/libextendxmpp && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o -c /home/hessam/Desktop/HotSpotter/IR/libextendxmpp/XMPPWorker_test.cc

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.i"
	cd /home/hessam/Desktop/HotSpotter/IR/build/libextendxmpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hessam/Desktop/HotSpotter/IR/libextendxmpp/XMPPWorker_test.cc > CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.i

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.s"
	cd /home/hessam/Desktop/HotSpotter/IR/build/libextendxmpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hessam/Desktop/HotSpotter/IR/libextendxmpp/XMPPWorker_test.cc -o CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.s

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o.requires:

.PHONY : libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o.requires

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o.provides: libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o.requires
	$(MAKE) -f libextendxmpp/CMakeFiles/XMPPWorker_test.dir/build.make libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o.provides.build
.PHONY : libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o.provides

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o.provides.build: libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o


# Object files for target XMPPWorker_test
XMPPWorker_test_OBJECTS = \
"CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o"

# External object files for target XMPPWorker_test
XMPPWorker_test_EXTERNAL_OBJECTS =

libextendxmpp/XMPPWorker_test: libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o
libextendxmpp/XMPPWorker_test: libextendxmpp/CMakeFiles/XMPPWorker_test.dir/build.make
libextendxmpp/XMPPWorker_test: libextendxmpp/libextendxmpp.a
libextendxmpp/XMPPWorker_test: libextendxmpp/CMakeFiles/XMPPWorker_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hessam/Desktop/HotSpotter/IR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable XMPPWorker_test"
	cd /home/hessam/Desktop/HotSpotter/IR/build/libextendxmpp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/XMPPWorker_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libextendxmpp/CMakeFiles/XMPPWorker_test.dir/build: libextendxmpp/XMPPWorker_test

.PHONY : libextendxmpp/CMakeFiles/XMPPWorker_test.dir/build

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/requires: libextendxmpp/CMakeFiles/XMPPWorker_test.dir/XMPPWorker_test.cc.o.requires

.PHONY : libextendxmpp/CMakeFiles/XMPPWorker_test.dir/requires

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/clean:
	cd /home/hessam/Desktop/HotSpotter/IR/build/libextendxmpp && $(CMAKE_COMMAND) -P CMakeFiles/XMPPWorker_test.dir/cmake_clean.cmake
.PHONY : libextendxmpp/CMakeFiles/XMPPWorker_test.dir/clean

libextendxmpp/CMakeFiles/XMPPWorker_test.dir/depend:
	cd /home/hessam/Desktop/HotSpotter/IR/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hessam/Desktop/HotSpotter/IR /home/hessam/Desktop/HotSpotter/IR/libextendxmpp /home/hessam/Desktop/HotSpotter/IR/build /home/hessam/Desktop/HotSpotter/IR/build/libextendxmpp /home/hessam/Desktop/HotSpotter/IR/build/libextendxmpp/CMakeFiles/XMPPWorker_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libextendxmpp/CMakeFiles/XMPPWorker_test.dir/depend

