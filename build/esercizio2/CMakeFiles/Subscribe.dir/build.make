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
CMAKE_SOURCE_DIR = /home/robotica/robotica_ws/src/esercizio2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robotica/robotica_ws/build/esercizio2

# Include any dependencies generated for this target.
include CMakeFiles/Subscribe.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Subscribe.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Subscribe.dir/flags.make

CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o: CMakeFiles/Subscribe.dir/flags.make
CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o: /home/robotica/robotica_ws/src/esercizio2/Subscribe/Subscribe.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robotica/robotica_ws/build/esercizio2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o -c /home/robotica/robotica_ws/src/esercizio2/Subscribe/Subscribe.cpp

CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robotica/robotica_ws/src/esercizio2/Subscribe/Subscribe.cpp > CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.i

CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robotica/robotica_ws/src/esercizio2/Subscribe/Subscribe.cpp -o CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.s

CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o.requires:

.PHONY : CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o.requires

CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o.provides: CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o.requires
	$(MAKE) -f CMakeFiles/Subscribe.dir/build.make CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o.provides.build
.PHONY : CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o.provides

CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o.provides.build: CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o


# Object files for target Subscribe
Subscribe_OBJECTS = \
"CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o"

# External object files for target Subscribe
Subscribe_EXTERNAL_OBJECTS =

/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: CMakeFiles/Subscribe.dir/build.make
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /opt/ros/melodic/lib/libroscpp.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /opt/ros/melodic/lib/librosconsole.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /opt/ros/melodic/lib/librostime.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /opt/ros/melodic/lib/libcpp_common.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe: CMakeFiles/Subscribe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robotica/robotica_ws/build/esercizio2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Subscribe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Subscribe.dir/build: /home/robotica/robotica_ws/devel/.private/esercizio2/lib/esercizio2/Subscribe

.PHONY : CMakeFiles/Subscribe.dir/build

CMakeFiles/Subscribe.dir/requires: CMakeFiles/Subscribe.dir/Subscribe/Subscribe.cpp.o.requires

.PHONY : CMakeFiles/Subscribe.dir/requires

CMakeFiles/Subscribe.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Subscribe.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Subscribe.dir/clean

CMakeFiles/Subscribe.dir/depend:
	cd /home/robotica/robotica_ws/build/esercizio2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robotica/robotica_ws/src/esercizio2 /home/robotica/robotica_ws/src/esercizio2 /home/robotica/robotica_ws/build/esercizio2 /home/robotica/robotica_ws/build/esercizio2 /home/robotica/robotica_ws/build/esercizio2/CMakeFiles/Subscribe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Subscribe.dir/depend

