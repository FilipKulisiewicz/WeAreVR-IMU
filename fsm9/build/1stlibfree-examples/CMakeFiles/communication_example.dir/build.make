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
CMAKE_SOURCE_DIR = /home/kuli/Desktop/fsm9/libfreespace-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kuli/Desktop/fsm9/build/libfree-examples

# Include any dependencies generated for this target.
include CMakeFiles/communication_example.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/communication_example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/communication_example.dir/flags.make

CMakeFiles/communication_example.dir/src/communication_example.c.o: CMakeFiles/communication_example.dir/flags.make
CMakeFiles/communication_example.dir/src/communication_example.c.o: /home/kuli/Desktop/fsm9/libfreespace-examples/src/communication_example.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/communication_example.dir/src/communication_example.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/communication_example.dir/src/communication_example.c.o   -c /home/kuli/Desktop/fsm9/libfreespace-examples/src/communication_example.c

CMakeFiles/communication_example.dir/src/communication_example.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/communication_example.dir/src/communication_example.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kuli/Desktop/fsm9/libfreespace-examples/src/communication_example.c > CMakeFiles/communication_example.dir/src/communication_example.c.i

CMakeFiles/communication_example.dir/src/communication_example.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/communication_example.dir/src/communication_example.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kuli/Desktop/fsm9/libfreespace-examples/src/communication_example.c -o CMakeFiles/communication_example.dir/src/communication_example.c.s

CMakeFiles/communication_example.dir/src/appControlHandler.c.o: CMakeFiles/communication_example.dir/flags.make
CMakeFiles/communication_example.dir/src/appControlHandler.c.o: /home/kuli/Desktop/fsm9/libfreespace-examples/src/appControlHandler.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/communication_example.dir/src/appControlHandler.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/communication_example.dir/src/appControlHandler.c.o   -c /home/kuli/Desktop/fsm9/libfreespace-examples/src/appControlHandler.c

CMakeFiles/communication_example.dir/src/appControlHandler.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/communication_example.dir/src/appControlHandler.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kuli/Desktop/fsm9/libfreespace-examples/src/appControlHandler.c > CMakeFiles/communication_example.dir/src/appControlHandler.c.i

CMakeFiles/communication_example.dir/src/appControlHandler.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/communication_example.dir/src/appControlHandler.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kuli/Desktop/fsm9/libfreespace-examples/src/appControlHandler.c -o CMakeFiles/communication_example.dir/src/appControlHandler.c.s

# Object files for target communication_example
communication_example_OBJECTS = \
"CMakeFiles/communication_example.dir/src/communication_example.c.o" \
"CMakeFiles/communication_example.dir/src/appControlHandler.c.o"

# External object files for target communication_example
communication_example_EXTERNAL_OBJECTS =

communication_example: CMakeFiles/communication_example.dir/src/communication_example.c.o
communication_example: CMakeFiles/communication_example.dir/src/appControlHandler.c.o
communication_example: CMakeFiles/communication_example.dir/build.make
communication_example: libfreespace/libfreespace.a
communication_example: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
communication_example: CMakeFiles/communication_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable communication_example"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/communication_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/communication_example.dir/build: communication_example

.PHONY : CMakeFiles/communication_example.dir/build

CMakeFiles/communication_example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/communication_example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/communication_example.dir/clean

CMakeFiles/communication_example.dir/depend:
	cd /home/kuli/Desktop/fsm9/build/libfree-examples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kuli/Desktop/fsm9/libfreespace-examples /home/kuli/Desktop/fsm9/libfreespace-examples /home/kuli/Desktop/fsm9/build/libfree-examples /home/kuli/Desktop/fsm9/build/libfree-examples /home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles/communication_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/communication_example.dir/depend

