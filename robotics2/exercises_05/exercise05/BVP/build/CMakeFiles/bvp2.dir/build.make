# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build

# Include any dependencies generated for this target.
include CMakeFiles/bvp2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bvp2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bvp2.dir/flags.make

CMakeFiles/bvp2.dir/SRC/bvp2.cc.o: CMakeFiles/bvp2.dir/flags.make
CMakeFiles/bvp2.dir/SRC/bvp2.cc.o: ../SRC/bvp2.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bvp2.dir/SRC/bvp2.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bvp2.dir/SRC/bvp2.cc.o -c /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/SRC/bvp2.cc

CMakeFiles/bvp2.dir/SRC/bvp2.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bvp2.dir/SRC/bvp2.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/SRC/bvp2.cc > CMakeFiles/bvp2.dir/SRC/bvp2.cc.i

CMakeFiles/bvp2.dir/SRC/bvp2.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bvp2.dir/SRC/bvp2.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/SRC/bvp2.cc -o CMakeFiles/bvp2.dir/SRC/bvp2.cc.s

CMakeFiles/bvp2.dir/SRC/bvp2.cc.o.requires:

.PHONY : CMakeFiles/bvp2.dir/SRC/bvp2.cc.o.requires

CMakeFiles/bvp2.dir/SRC/bvp2.cc.o.provides: CMakeFiles/bvp2.dir/SRC/bvp2.cc.o.requires
	$(MAKE) -f CMakeFiles/bvp2.dir/build.make CMakeFiles/bvp2.dir/SRC/bvp2.cc.o.provides.build
.PHONY : CMakeFiles/bvp2.dir/SRC/bvp2.cc.o.provides

CMakeFiles/bvp2.dir/SRC/bvp2.cc.o.provides.build: CMakeFiles/bvp2.dir/SRC/bvp2.cc.o


# Object files for target bvp2
bvp2_OBJECTS = \
"CMakeFiles/bvp2.dir/SRC/bvp2.cc.o"

# External object files for target bvp2
bvp2_EXTERNAL_OBJECTS =

libbvp2.so: CMakeFiles/bvp2.dir/SRC/bvp2.cc.o
libbvp2.so: CMakeFiles/bvp2.dir/build.make
libbvp2.so: CMakeFiles/bvp2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libbvp2.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bvp2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bvp2.dir/build: libbvp2.so

.PHONY : CMakeFiles/bvp2.dir/build

CMakeFiles/bvp2.dir/requires: CMakeFiles/bvp2.dir/SRC/bvp2.cc.o.requires

.PHONY : CMakeFiles/bvp2.dir/requires

CMakeFiles/bvp2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bvp2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bvp2.dir/clean

CMakeFiles/bvp2.dir/depend:
	cd /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build/CMakeFiles/bvp2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bvp2.dir/depend

