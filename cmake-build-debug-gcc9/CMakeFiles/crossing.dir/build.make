# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /opt/clion-2019.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2019.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kris271c/Desktop/assignment

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kris271c/Desktop/assignment/cmake-build-debug-gcc9

# Include any dependencies generated for this target.
include CMakeFiles/crossing.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/crossing.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/crossing.dir/flags.make

CMakeFiles/crossing.dir/crossing.cpp.o: CMakeFiles/crossing.dir/flags.make
CMakeFiles/crossing.dir/crossing.cpp.o: ../crossing.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kris271c/Desktop/assignment/cmake-build-debug-gcc9/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/crossing.dir/crossing.cpp.o"
	/usr/bin/g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/crossing.dir/crossing.cpp.o -c /home/kris271c/Desktop/assignment/crossing.cpp

CMakeFiles/crossing.dir/crossing.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/crossing.dir/crossing.cpp.i"
	/usr/bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kris271c/Desktop/assignment/crossing.cpp > CMakeFiles/crossing.dir/crossing.cpp.i

CMakeFiles/crossing.dir/crossing.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/crossing.dir/crossing.cpp.s"
	/usr/bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kris271c/Desktop/assignment/crossing.cpp -o CMakeFiles/crossing.dir/crossing.cpp.s

# Object files for target crossing
crossing_OBJECTS = \
"CMakeFiles/crossing.dir/crossing.cpp.o"

# External object files for target crossing
crossing_EXTERNAL_OBJECTS =

crossing: CMakeFiles/crossing.dir/crossing.cpp.o
crossing: CMakeFiles/crossing.dir/build.make
crossing: CMakeFiles/crossing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kris271c/Desktop/assignment/cmake-build-debug-gcc9/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable crossing"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/crossing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/crossing.dir/build: crossing

.PHONY : CMakeFiles/crossing.dir/build

CMakeFiles/crossing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/crossing.dir/cmake_clean.cmake
.PHONY : CMakeFiles/crossing.dir/clean

CMakeFiles/crossing.dir/depend:
	cd /home/kris271c/Desktop/assignment/cmake-build-debug-gcc9 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kris271c/Desktop/assignment /home/kris271c/Desktop/assignment /home/kris271c/Desktop/assignment/cmake-build-debug-gcc9 /home/kris271c/Desktop/assignment/cmake-build-debug-gcc9 /home/kris271c/Desktop/assignment/cmake-build-debug-gcc9/CMakeFiles/crossing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/crossing.dir/depend

