# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /home/maxl/CLion-2019.1.3/clion-2019.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/maxl/CLion-2019.1.3/clion-2019.1.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/maxl/Documents/exercises_03/exercise03/icub2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maxl/Documents/exercises_03/exercise03/icub2/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/rbdl_model_analysis.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rbdl_model_analysis.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rbdl_model_analysis.dir/flags.make

CMakeFiles/rbdl_model_analysis.dir/main.cpp.o: CMakeFiles/rbdl_model_analysis.dir/flags.make
CMakeFiles/rbdl_model_analysis.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maxl/Documents/exercises_03/exercise03/icub2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rbdl_model_analysis.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rbdl_model_analysis.dir/main.cpp.o -c /home/maxl/Documents/exercises_03/exercise03/icub2/main.cpp

CMakeFiles/rbdl_model_analysis.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rbdl_model_analysis.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maxl/Documents/exercises_03/exercise03/icub2/main.cpp > CMakeFiles/rbdl_model_analysis.dir/main.cpp.i

CMakeFiles/rbdl_model_analysis.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rbdl_model_analysis.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maxl/Documents/exercises_03/exercise03/icub2/main.cpp -o CMakeFiles/rbdl_model_analysis.dir/main.cpp.s

# Object files for target rbdl_model_analysis
rbdl_model_analysis_OBJECTS = \
"CMakeFiles/rbdl_model_analysis.dir/main.cpp.o"

# External object files for target rbdl_model_analysis
rbdl_model_analysis_EXTERNAL_OBJECTS =

rbdl_model_analysis: CMakeFiles/rbdl_model_analysis.dir/main.cpp.o
rbdl_model_analysis: CMakeFiles/rbdl_model_analysis.dir/build.make
rbdl_model_analysis: /usr/local/lib/librbdl.so
rbdl_model_analysis: /usr/local/lib/librbdl_luamodel.so
rbdl_model_analysis: /usr/lib/x86_64-linux-gnu/liblua5.1.so
rbdl_model_analysis: /usr/lib/x86_64-linux-gnu/libm.so
rbdl_model_analysis: CMakeFiles/rbdl_model_analysis.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maxl/Documents/exercises_03/exercise03/icub2/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rbdl_model_analysis"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rbdl_model_analysis.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rbdl_model_analysis.dir/build: rbdl_model_analysis

.PHONY : CMakeFiles/rbdl_model_analysis.dir/build

CMakeFiles/rbdl_model_analysis.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rbdl_model_analysis.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rbdl_model_analysis.dir/clean

CMakeFiles/rbdl_model_analysis.dir/depend:
	cd /home/maxl/Documents/exercises_03/exercise03/icub2/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maxl/Documents/exercises_03/exercise03/icub2 /home/maxl/Documents/exercises_03/exercise03/icub2 /home/maxl/Documents/exercises_03/exercise03/icub2/cmake-build-debug /home/maxl/Documents/exercises_03/exercise03/icub2/cmake-build-debug /home/maxl/Documents/exercises_03/exercise03/icub2/cmake-build-debug/CMakeFiles/rbdl_model_analysis.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rbdl_model_analysis.dir/depend
