# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/robo/robotics1/sheet05/exercise05/bouncing_ball

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robo/robotics1/sheet05/exercise05/bouncing_ball/build

# Utility rule file for COPY_bouncing_ball.lua.

# Include the progress variables for this target.
include CMakeFiles/COPY_bouncing_ball.lua.dir/progress.make

CMakeFiles/COPY_bouncing_ball.lua: bouncing_ball.lua


bouncing_ball.lua: ../cmake/copySrc.cmake.in
bouncing_ball.lua: ../config/bouncing_ball.lua
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/robo/robotics1/sheet05/exercise05/bouncing_ball/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Copying bouncing_ball.lua -> bouncing_ball.lua to binary tree"
	/usr/bin/cmake -P /home/robo/robotics1/sheet05/exercise05/bouncing_ball/build/CMakeFiles/bouncing_ball.lua.cmake

COPY_bouncing_ball.lua: CMakeFiles/COPY_bouncing_ball.lua
COPY_bouncing_ball.lua: bouncing_ball.lua
COPY_bouncing_ball.lua: CMakeFiles/COPY_bouncing_ball.lua.dir/build.make

.PHONY : COPY_bouncing_ball.lua

# Rule to build all files generated by this target.
CMakeFiles/COPY_bouncing_ball.lua.dir/build: COPY_bouncing_ball.lua

.PHONY : CMakeFiles/COPY_bouncing_ball.lua.dir/build

CMakeFiles/COPY_bouncing_ball.lua.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/COPY_bouncing_ball.lua.dir/cmake_clean.cmake
.PHONY : CMakeFiles/COPY_bouncing_ball.lua.dir/clean

CMakeFiles/COPY_bouncing_ball.lua.dir/depend:
	cd /home/robo/robotics1/sheet05/exercise05/bouncing_ball/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robo/robotics1/sheet05/exercise05/bouncing_ball /home/robo/robotics1/sheet05/exercise05/bouncing_ball /home/robo/robotics1/sheet05/exercise05/bouncing_ball/build /home/robo/robotics1/sheet05/exercise05/bouncing_ball/build /home/robo/robotics1/sheet05/exercise05/bouncing_ball/build/CMakeFiles/COPY_bouncing_ball.lua.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/COPY_bouncing_ball.lua.dir/depend
