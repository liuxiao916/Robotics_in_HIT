# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liuxiao/code/SCARA

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liuxiao/code/SCARA/build

# Include any dependencies generated for this target.
include CMakeFiles/Scara.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Scara.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Scara.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Scara.dir/flags.make

CMakeFiles/Scara.dir/main.cpp.o: CMakeFiles/Scara.dir/flags.make
CMakeFiles/Scara.dir/main.cpp.o: ../main.cpp
CMakeFiles/Scara.dir/main.cpp.o: CMakeFiles/Scara.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuxiao/code/SCARA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Scara.dir/main.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Scara.dir/main.cpp.o -MF CMakeFiles/Scara.dir/main.cpp.o.d -o CMakeFiles/Scara.dir/main.cpp.o -c /home/liuxiao/code/SCARA/main.cpp

CMakeFiles/Scara.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Scara.dir/main.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuxiao/code/SCARA/main.cpp > CMakeFiles/Scara.dir/main.cpp.i

CMakeFiles/Scara.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Scara.dir/main.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuxiao/code/SCARA/main.cpp -o CMakeFiles/Scara.dir/main.cpp.s

# Object files for target Scara
Scara_OBJECTS = \
"CMakeFiles/Scara.dir/main.cpp.o"

# External object files for target Scara
Scara_EXTERNAL_OBJECTS =

Scara: CMakeFiles/Scara.dir/main.cpp.o
Scara: CMakeFiles/Scara.dir/build.make
Scara: CMakeFiles/Scara.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/liuxiao/code/SCARA/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Scara"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Scara.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Scara.dir/build: Scara
.PHONY : CMakeFiles/Scara.dir/build

CMakeFiles/Scara.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Scara.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Scara.dir/clean

CMakeFiles/Scara.dir/depend:
	cd /home/liuxiao/code/SCARA/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liuxiao/code/SCARA /home/liuxiao/code/SCARA /home/liuxiao/code/SCARA/build /home/liuxiao/code/SCARA/build /home/liuxiao/code/SCARA/build/CMakeFiles/Scara.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Scara.dir/depend

