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
include CMakeFiles/bvp1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bvp1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bvp1.dir/flags.make

CMakeFiles/bvp1.dir/SRC/bvp1.cc.o: CMakeFiles/bvp1.dir/flags.make
CMakeFiles/bvp1.dir/SRC/bvp1.cc.o: ../SRC/bvp1.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bvp1.dir/SRC/bvp1.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bvp1.dir/SRC/bvp1.cc.o -c /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/SRC/bvp1.cc

CMakeFiles/bvp1.dir/SRC/bvp1.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bvp1.dir/SRC/bvp1.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/SRC/bvp1.cc > CMakeFiles/bvp1.dir/SRC/bvp1.cc.i

CMakeFiles/bvp1.dir/SRC/bvp1.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bvp1.dir/SRC/bvp1.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/SRC/bvp1.cc -o CMakeFiles/bvp1.dir/SRC/bvp1.cc.s

CMakeFiles/bvp1.dir/SRC/bvp1.cc.o.requires:

.PHONY : CMakeFiles/bvp1.dir/SRC/bvp1.cc.o.requires

CMakeFiles/bvp1.dir/SRC/bvp1.cc.o.provides: CMakeFiles/bvp1.dir/SRC/bvp1.cc.o.requires
	$(MAKE) -f CMakeFiles/bvp1.dir/build.make CMakeFiles/bvp1.dir/SRC/bvp1.cc.o.provides.build
.PHONY : CMakeFiles/bvp1.dir/SRC/bvp1.cc.o.provides

CMakeFiles/bvp1.dir/SRC/bvp1.cc.o.provides.build: CMakeFiles/bvp1.dir/SRC/bvp1.cc.o


# Object files for target bvp1
bvp1_OBJECTS = \
"CMakeFiles/bvp1.dir/SRC/bvp1.cc.o"

# External object files for target bvp1
bvp1_EXTERNAL_OBJECTS =

libbvp1.so: CMakeFiles/bvp1.dir/SRC/bvp1.cc.o
libbvp1.so: CMakeFiles/bvp1.dir/build.make
libbvp1.so: CMakeFiles/bvp1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libbvp1.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bvp1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bvp1.dir/build: libbvp1.so

.PHONY : CMakeFiles/bvp1.dir/build

CMakeFiles/bvp1.dir/requires: CMakeFiles/bvp1.dir/SRC/bvp1.cc.o.requires

.PHONY : CMakeFiles/bvp1.dir/requires

CMakeFiles/bvp1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bvp1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bvp1.dir/clean

CMakeFiles/bvp1.dir/depend:
	cd /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build /export/home/ROB/ROB007/robotics1/robotics2/exercises_05/exercise05/BVP/build/CMakeFiles/bvp1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bvp1.dir/depend
