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
include libfreespace/CMakeFiles/freespace.dir/depend.make

# Include the progress variables for this target.
include libfreespace/CMakeFiles/freespace.dir/progress.make

# Include the compile flags for this target's objects.
include libfreespace/CMakeFiles/freespace.dir/flags.make

libfreespace/gen_src/freespace_codecs.c: /home/kuli/Desktop/fsm9/libfreespace/common/messageCodeGenerator.py
libfreespace/gen_src/freespace_codecs.c: /home/kuli/Desktop/fsm9/libfreespace/common/setupMessages.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating libfreespace message code"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /home/kuli/miniconda3/bin/python /home/kuli/Desktop/fsm9/libfreespace/common/messageCodeGenerator.py -I /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/include/ -s /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/gen_src/ /home/kuli/Desktop/fsm9/libfreespace/common/setupMessages.py 

libfreespace/gen_src/freespace_printers.c: libfreespace/gen_src/freespace_codecs.c
	@$(CMAKE_COMMAND) -E touch_nocreate libfreespace/gen_src/freespace_printers.c

libfreespace/include/freespace/freespace_codecs.h: libfreespace/gen_src/freespace_codecs.c
	@$(CMAKE_COMMAND) -E touch_nocreate libfreespace/include/freespace/freespace_codecs.h

libfreespace/include/freespace/freespace_printers.h: libfreespace/gen_src/freespace_codecs.c
	@$(CMAKE_COMMAND) -E touch_nocreate libfreespace/include/freespace/freespace_printers.h

libfreespace/CMakeFiles/freespace.dir/common/freespace_deviceTable.c.o: libfreespace/CMakeFiles/freespace.dir/flags.make
libfreespace/CMakeFiles/freespace.dir/common/freespace_deviceTable.c.o: /home/kuli/Desktop/fsm9/libfreespace/common/freespace_deviceTable.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object libfreespace/CMakeFiles/freespace.dir/common/freespace_deviceTable.c.o"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freespace.dir/common/freespace_deviceTable.c.o   -c /home/kuli/Desktop/fsm9/libfreespace/common/freespace_deviceTable.c

libfreespace/CMakeFiles/freespace.dir/common/freespace_deviceTable.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freespace.dir/common/freespace_deviceTable.c.i"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kuli/Desktop/fsm9/libfreespace/common/freespace_deviceTable.c > CMakeFiles/freespace.dir/common/freespace_deviceTable.c.i

libfreespace/CMakeFiles/freespace.dir/common/freespace_deviceTable.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freespace.dir/common/freespace_deviceTable.c.s"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kuli/Desktop/fsm9/libfreespace/common/freespace_deviceTable.c -o CMakeFiles/freespace.dir/common/freespace_deviceTable.c.s

libfreespace/CMakeFiles/freespace.dir/common/freespace_util.c.o: libfreespace/CMakeFiles/freespace.dir/flags.make
libfreespace/CMakeFiles/freespace.dir/common/freespace_util.c.o: /home/kuli/Desktop/fsm9/libfreespace/common/freespace_util.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object libfreespace/CMakeFiles/freespace.dir/common/freespace_util.c.o"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freespace.dir/common/freespace_util.c.o   -c /home/kuli/Desktop/fsm9/libfreespace/common/freespace_util.c

libfreespace/CMakeFiles/freespace.dir/common/freespace_util.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freespace.dir/common/freespace_util.c.i"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kuli/Desktop/fsm9/libfreespace/common/freespace_util.c > CMakeFiles/freespace.dir/common/freespace_util.c.i

libfreespace/CMakeFiles/freespace.dir/common/freespace_util.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freespace.dir/common/freespace_util.c.s"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kuli/Desktop/fsm9/libfreespace/common/freespace_util.c -o CMakeFiles/freespace.dir/common/freespace_util.c.s

libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.o: libfreespace/CMakeFiles/freespace.dir/flags.make
libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.o: libfreespace/gen_src/freespace_codecs.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.o"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.o   -c /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/gen_src/freespace_codecs.c

libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.i"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/gen_src/freespace_codecs.c > CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.i

libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.s"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/gen_src/freespace_codecs.c -o CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.s

libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_printers.c.o: libfreespace/CMakeFiles/freespace.dir/flags.make
libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_printers.c.o: libfreespace/gen_src/freespace_printers.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_printers.c.o"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freespace.dir/gen_src/freespace_printers.c.o   -c /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/gen_src/freespace_printers.c

libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_printers.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freespace.dir/gen_src/freespace_printers.c.i"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/gen_src/freespace_printers.c > CMakeFiles/freespace.dir/gen_src/freespace_printers.c.i

libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_printers.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freespace.dir/gen_src/freespace_printers.c.s"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/gen_src/freespace_printers.c -o CMakeFiles/freespace.dir/gen_src/freespace_printers.c.s

libfreespace/CMakeFiles/freespace.dir/linux/freespace.c.o: libfreespace/CMakeFiles/freespace.dir/flags.make
libfreespace/CMakeFiles/freespace.dir/linux/freespace.c.o: /home/kuli/Desktop/fsm9/libfreespace/linux/freespace.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object libfreespace/CMakeFiles/freespace.dir/linux/freespace.c.o"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freespace.dir/linux/freespace.c.o   -c /home/kuli/Desktop/fsm9/libfreespace/linux/freespace.c

libfreespace/CMakeFiles/freespace.dir/linux/freespace.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freespace.dir/linux/freespace.c.i"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kuli/Desktop/fsm9/libfreespace/linux/freespace.c > CMakeFiles/freespace.dir/linux/freespace.c.i

libfreespace/CMakeFiles/freespace.dir/linux/freespace.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freespace.dir/linux/freespace.c.s"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kuli/Desktop/fsm9/libfreespace/linux/freespace.c -o CMakeFiles/freespace.dir/linux/freespace.c.s

libfreespace/CMakeFiles/freespace.dir/linux/linux_hotplug.c.o: libfreespace/CMakeFiles/freespace.dir/flags.make
libfreespace/CMakeFiles/freespace.dir/linux/linux_hotplug.c.o: /home/kuli/Desktop/fsm9/libfreespace/linux/linux_hotplug.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object libfreespace/CMakeFiles/freespace.dir/linux/linux_hotplug.c.o"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/freespace.dir/linux/linux_hotplug.c.o   -c /home/kuli/Desktop/fsm9/libfreespace/linux/linux_hotplug.c

libfreespace/CMakeFiles/freespace.dir/linux/linux_hotplug.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/freespace.dir/linux/linux_hotplug.c.i"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/kuli/Desktop/fsm9/libfreespace/linux/linux_hotplug.c > CMakeFiles/freespace.dir/linux/linux_hotplug.c.i

libfreespace/CMakeFiles/freespace.dir/linux/linux_hotplug.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/freespace.dir/linux/linux_hotplug.c.s"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/kuli/Desktop/fsm9/libfreespace/linux/linux_hotplug.c -o CMakeFiles/freespace.dir/linux/linux_hotplug.c.s

# Object files for target freespace
freespace_OBJECTS = \
"CMakeFiles/freespace.dir/common/freespace_deviceTable.c.o" \
"CMakeFiles/freespace.dir/common/freespace_util.c.o" \
"CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.o" \
"CMakeFiles/freespace.dir/gen_src/freespace_printers.c.o" \
"CMakeFiles/freespace.dir/linux/freespace.c.o" \
"CMakeFiles/freespace.dir/linux/linux_hotplug.c.o"

# External object files for target freespace
freespace_EXTERNAL_OBJECTS =

libfreespace/libfreespace.a: libfreespace/CMakeFiles/freespace.dir/common/freespace_deviceTable.c.o
libfreespace/libfreespace.a: libfreespace/CMakeFiles/freespace.dir/common/freespace_util.c.o
libfreespace/libfreespace.a: libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_codecs.c.o
libfreespace/libfreespace.a: libfreespace/CMakeFiles/freespace.dir/gen_src/freespace_printers.c.o
libfreespace/libfreespace.a: libfreespace/CMakeFiles/freespace.dir/linux/freespace.c.o
libfreespace/libfreespace.a: libfreespace/CMakeFiles/freespace.dir/linux/linux_hotplug.c.o
libfreespace/libfreespace.a: libfreespace/CMakeFiles/freespace.dir/build.make
libfreespace/libfreespace.a: libfreespace/CMakeFiles/freespace.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kuli/Desktop/fsm9/build/libfree-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking C static library libfreespace.a"
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && $(CMAKE_COMMAND) -P CMakeFiles/freespace.dir/cmake_clean_target.cmake
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/freespace.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libfreespace/CMakeFiles/freespace.dir/build: libfreespace/libfreespace.a

.PHONY : libfreespace/CMakeFiles/freespace.dir/build

libfreespace/CMakeFiles/freespace.dir/clean:
	cd /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace && $(CMAKE_COMMAND) -P CMakeFiles/freespace.dir/cmake_clean.cmake
.PHONY : libfreespace/CMakeFiles/freespace.dir/clean

libfreespace/CMakeFiles/freespace.dir/depend: libfreespace/gen_src/freespace_codecs.c
libfreespace/CMakeFiles/freespace.dir/depend: libfreespace/gen_src/freespace_printers.c
libfreespace/CMakeFiles/freespace.dir/depend: libfreespace/include/freespace/freespace_codecs.h
libfreespace/CMakeFiles/freespace.dir/depend: libfreespace/include/freespace/freespace_printers.h
	cd /home/kuli/Desktop/fsm9/build/libfree-examples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kuli/Desktop/fsm9/libfreespace-examples /home/kuli/Desktop/fsm9/libfreespace /home/kuli/Desktop/fsm9/build/libfree-examples /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace /home/kuli/Desktop/fsm9/build/libfree-examples/libfreespace/CMakeFiles/freespace.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libfreespace/CMakeFiles/freespace.dir/depend

