# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/pi/IMU/myimu

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/IMU/myimu/build

# Include any dependencies generated for this target.
include CMakeFiles/myimu_exec.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/myimu_exec.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myimu_exec.dir/flags.make

CMakeFiles/myimu_exec.dir/src/myimu.cpp.o: CMakeFiles/myimu_exec.dir/flags.make
CMakeFiles/myimu_exec.dir/src/myimu.cpp.o: ../src/myimu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/IMU/myimu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/myimu_exec.dir/src/myimu.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myimu_exec.dir/src/myimu.cpp.o -c /home/pi/IMU/myimu/src/myimu.cpp

CMakeFiles/myimu_exec.dir/src/myimu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myimu_exec.dir/src/myimu.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/IMU/myimu/src/myimu.cpp > CMakeFiles/myimu_exec.dir/src/myimu.cpp.i

CMakeFiles/myimu_exec.dir/src/myimu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myimu_exec.dir/src/myimu.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/IMU/myimu/src/myimu.cpp -o CMakeFiles/myimu_exec.dir/src/myimu.cpp.s

# Object files for target myimu_exec
myimu_exec_OBJECTS = \
"CMakeFiles/myimu_exec.dir/src/myimu.cpp.o"

# External object files for target myimu_exec
myimu_exec_EXTERNAL_OBJECTS =

myimu_exec: CMakeFiles/myimu_exec.dir/src/myimu.cpp.o
myimu_exec: CMakeFiles/myimu_exec.dir/build.make
myimu_exec: /usr/local/lib/libfreespace.so
myimu_exec: CMakeFiles/myimu_exec.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/IMU/myimu/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable myimu_exec"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myimu_exec.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myimu_exec.dir/build: myimu_exec

.PHONY : CMakeFiles/myimu_exec.dir/build

CMakeFiles/myimu_exec.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myimu_exec.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myimu_exec.dir/clean

CMakeFiles/myimu_exec.dir/depend:
	cd /home/pi/IMU/myimu/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/IMU/myimu /home/pi/IMU/myimu /home/pi/IMU/myimu/build /home/pi/IMU/myimu/build /home/pi/IMU/myimu/build/CMakeFiles/myimu_exec.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/myimu_exec.dir/depend

