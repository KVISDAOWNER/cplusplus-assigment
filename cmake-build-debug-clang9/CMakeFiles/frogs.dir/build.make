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
CMAKE_BINARY_DIR = /home/kris271c/Desktop/assignment/cmake-build-debug-clang9

# Include any dependencies generated for this target.
include CMakeFiles/frogs.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/frogs.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/frogs.dir/flags.make

CMakeFiles/frogs.dir/frogs.cpp.o: CMakeFiles/frogs.dir/flags.make
CMakeFiles/frogs.dir/frogs.cpp.o: ../frogs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kris271c/Desktop/assignment/cmake-build-debug-clang9/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/frogs.dir/frogs.cpp.o"
	/usr/bin/clang++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/frogs.dir/frogs.cpp.o -c /home/kris271c/Desktop/assignment/frogs.cpp

CMakeFiles/frogs.dir/frogs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/frogs.dir/frogs.cpp.i"
	/usr/bin/clang++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kris271c/Desktop/assignment/frogs.cpp > CMakeFiles/frogs.dir/frogs.cpp.i

CMakeFiles/frogs.dir/frogs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/frogs.dir/frogs.cpp.s"
	/usr/bin/clang++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kris271c/Desktop/assignment/frogs.cpp -o CMakeFiles/frogs.dir/frogs.cpp.s

CMakeFiles/frogs.dir/family.cpp.o: CMakeFiles/frogs.dir/flags.make
CMakeFiles/frogs.dir/family.cpp.o: ../family.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kris271c/Desktop/assignment/cmake-build-debug-clang9/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/frogs.dir/family.cpp.o"
	/usr/bin/clang++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/frogs.dir/family.cpp.o -c /home/kris271c/Desktop/assignment/family.cpp

CMakeFiles/frogs.dir/family.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/frogs.dir/family.cpp.i"
	/usr/bin/clang++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kris271c/Desktop/assignment/family.cpp > CMakeFiles/frogs.dir/family.cpp.i

CMakeFiles/frogs.dir/family.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/frogs.dir/family.cpp.s"
	/usr/bin/clang++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kris271c/Desktop/assignment/family.cpp -o CMakeFiles/frogs.dir/family.cpp.s

# Object files for target frogs
frogs_OBJECTS = \
"CMakeFiles/frogs.dir/frogs.cpp.o" \
"CMakeFiles/frogs.dir/family.cpp.o"

# External object files for target frogs
frogs_EXTERNAL_OBJECTS =

frogs: CMakeFiles/frogs.dir/frogs.cpp.o
frogs: CMakeFiles/frogs.dir/family.cpp.o
frogs: CMakeFiles/frogs.dir/build.make
frogs: CMakeFiles/frogs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kris271c/Desktop/assignment/cmake-build-debug-clang9/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable frogs"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/frogs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/frogs.dir/build: frogs

.PHONY : CMakeFiles/frogs.dir/build

CMakeFiles/frogs.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/frogs.dir/cmake_clean.cmake
.PHONY : CMakeFiles/frogs.dir/clean

CMakeFiles/frogs.dir/depend:
	cd /home/kris271c/Desktop/assignment/cmake-build-debug-clang9 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kris271c/Desktop/assignment /home/kris271c/Desktop/assignment /home/kris271c/Desktop/assignment/cmake-build-debug-clang9 /home/kris271c/Desktop/assignment/cmake-build-debug-clang9 /home/kris271c/Desktop/assignment/cmake-build-debug-clang9/CMakeFiles/frogs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/frogs.dir/depend

