# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/shucheng/contact

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shucheng/contact/build/contact

# Include any dependencies generated for this target.
include serial/lib/CMakeFiles/serial.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include serial/lib/CMakeFiles/serial.dir/compiler_depend.make

# Include the progress variables for this target.
include serial/lib/CMakeFiles/serial.dir/progress.make

# Include the compile flags for this target's objects.
include serial/lib/CMakeFiles/serial.dir/flags.make

serial/lib/CMakeFiles/serial.dir/src/serial.cc.o: serial/lib/CMakeFiles/serial.dir/flags.make
serial/lib/CMakeFiles/serial.dir/src/serial.cc.o: ../../serial/lib/src/serial.cc
serial/lib/CMakeFiles/serial.dir/src/serial.cc.o: serial/lib/CMakeFiles/serial.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shucheng/contact/build/contact/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object serial/lib/CMakeFiles/serial.dir/src/serial.cc.o"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT serial/lib/CMakeFiles/serial.dir/src/serial.cc.o -MF CMakeFiles/serial.dir/src/serial.cc.o.d -o CMakeFiles/serial.dir/src/serial.cc.o -c /home/shucheng/contact/serial/lib/src/serial.cc

serial/lib/CMakeFiles/serial.dir/src/serial.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial.dir/src/serial.cc.i"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shucheng/contact/serial/lib/src/serial.cc > CMakeFiles/serial.dir/src/serial.cc.i

serial/lib/CMakeFiles/serial.dir/src/serial.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial.dir/src/serial.cc.s"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shucheng/contact/serial/lib/src/serial.cc -o CMakeFiles/serial.dir/src/serial.cc.s

serial/lib/CMakeFiles/serial.dir/src/impl/unix.cc.o: serial/lib/CMakeFiles/serial.dir/flags.make
serial/lib/CMakeFiles/serial.dir/src/impl/unix.cc.o: ../../serial/lib/src/impl/unix.cc
serial/lib/CMakeFiles/serial.dir/src/impl/unix.cc.o: serial/lib/CMakeFiles/serial.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shucheng/contact/build/contact/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object serial/lib/CMakeFiles/serial.dir/src/impl/unix.cc.o"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT serial/lib/CMakeFiles/serial.dir/src/impl/unix.cc.o -MF CMakeFiles/serial.dir/src/impl/unix.cc.o.d -o CMakeFiles/serial.dir/src/impl/unix.cc.o -c /home/shucheng/contact/serial/lib/src/impl/unix.cc

serial/lib/CMakeFiles/serial.dir/src/impl/unix.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial.dir/src/impl/unix.cc.i"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shucheng/contact/serial/lib/src/impl/unix.cc > CMakeFiles/serial.dir/src/impl/unix.cc.i

serial/lib/CMakeFiles/serial.dir/src/impl/unix.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial.dir/src/impl/unix.cc.s"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shucheng/contact/serial/lib/src/impl/unix.cc -o CMakeFiles/serial.dir/src/impl/unix.cc.s

serial/lib/CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o: serial/lib/CMakeFiles/serial.dir/flags.make
serial/lib/CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o: ../../serial/lib/src/impl/list_ports/list_ports_linux.cc
serial/lib/CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o: serial/lib/CMakeFiles/serial.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shucheng/contact/build/contact/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object serial/lib/CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT serial/lib/CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o -MF CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o.d -o CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o -c /home/shucheng/contact/serial/lib/src/impl/list_ports/list_ports_linux.cc

serial/lib/CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.i"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shucheng/contact/serial/lib/src/impl/list_ports/list_ports_linux.cc > CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.i

serial/lib/CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.s"
	cd /home/shucheng/contact/build/contact/serial/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shucheng/contact/serial/lib/src/impl/list_ports/list_ports_linux.cc -o CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.s

# Object files for target serial
serial_OBJECTS = \
"CMakeFiles/serial.dir/src/serial.cc.o" \
"CMakeFiles/serial.dir/src/impl/unix.cc.o" \
"CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o"

# External object files for target serial
serial_EXTERNAL_OBJECTS =

serial/lib/libserial.a: serial/lib/CMakeFiles/serial.dir/src/serial.cc.o
serial/lib/libserial.a: serial/lib/CMakeFiles/serial.dir/src/impl/unix.cc.o
serial/lib/libserial.a: serial/lib/CMakeFiles/serial.dir/src/impl/list_ports/list_ports_linux.cc.o
serial/lib/libserial.a: serial/lib/CMakeFiles/serial.dir/build.make
serial/lib/libserial.a: serial/lib/CMakeFiles/serial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shucheng/contact/build/contact/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libserial.a"
	cd /home/shucheng/contact/build/contact/serial/lib && $(CMAKE_COMMAND) -P CMakeFiles/serial.dir/cmake_clean_target.cmake
	cd /home/shucheng/contact/build/contact/serial/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
serial/lib/CMakeFiles/serial.dir/build: serial/lib/libserial.a
.PHONY : serial/lib/CMakeFiles/serial.dir/build

serial/lib/CMakeFiles/serial.dir/clean:
	cd /home/shucheng/contact/build/contact/serial/lib && $(CMAKE_COMMAND) -P CMakeFiles/serial.dir/cmake_clean.cmake
.PHONY : serial/lib/CMakeFiles/serial.dir/clean

serial/lib/CMakeFiles/serial.dir/depend:
	cd /home/shucheng/contact/build/contact && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shucheng/contact /home/shucheng/contact/serial/lib /home/shucheng/contact/build/contact /home/shucheng/contact/build/contact/serial/lib /home/shucheng/contact/build/contact/serial/lib/CMakeFiles/serial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial/lib/CMakeFiles/serial.dir/depend

