# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hxy/raisim_workspace/legs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hxy/raisim_workspace/legs/build

# Include any dependencies generated for this target.
include CMakeFiles/leg_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/leg_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/leg_controller.dir/flags.make

CMakeFiles/leg_controller.dir/src/leg_controller.cpp.o: CMakeFiles/leg_controller.dir/flags.make
CMakeFiles/leg_controller.dir/src/leg_controller.cpp.o: ../src/leg_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hxy/raisim_workspace/legs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/leg_controller.dir/src/leg_controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/leg_controller.dir/src/leg_controller.cpp.o -c /home/hxy/raisim_workspace/legs/src/leg_controller.cpp

CMakeFiles/leg_controller.dir/src/leg_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/leg_controller.dir/src/leg_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hxy/raisim_workspace/legs/src/leg_controller.cpp > CMakeFiles/leg_controller.dir/src/leg_controller.cpp.i

CMakeFiles/leg_controller.dir/src/leg_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/leg_controller.dir/src/leg_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hxy/raisim_workspace/legs/src/leg_controller.cpp -o CMakeFiles/leg_controller.dir/src/leg_controller.cpp.s

# Object files for target leg_controller
leg_controller_OBJECTS = \
"CMakeFiles/leg_controller.dir/src/leg_controller.cpp.o"

# External object files for target leg_controller
leg_controller_EXTERNAL_OBJECTS =

libleg_controller.a: CMakeFiles/leg_controller.dir/src/leg_controller.cpp.o
libleg_controller.a: CMakeFiles/leg_controller.dir/build.make
libleg_controller.a: CMakeFiles/leg_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hxy/raisim_workspace/legs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libleg_controller.a"
	$(CMAKE_COMMAND) -P CMakeFiles/leg_controller.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/leg_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/leg_controller.dir/build: libleg_controller.a

.PHONY : CMakeFiles/leg_controller.dir/build

CMakeFiles/leg_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/leg_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/leg_controller.dir/clean

CMakeFiles/leg_controller.dir/depend:
	cd /home/hxy/raisim_workspace/legs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hxy/raisim_workspace/legs /home/hxy/raisim_workspace/legs /home/hxy/raisim_workspace/legs/build /home/hxy/raisim_workspace/legs/build /home/hxy/raisim_workspace/legs/build/CMakeFiles/leg_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/leg_controller.dir/depend

