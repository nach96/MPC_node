# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = "/home/inazio/TFG/code/Nacho Code/MPC_node/non_ros"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/inazio/TFG/code/Nacho Code/MPC_node/build-non_ros-Desktop-Default"

# Include any dependencies generated for this target.
include CMakeFiles/myNLP.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/myNLP.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myNLP.dir/flags.make

CMakeFiles/myNLP.dir/MyNLP.cpp.o: CMakeFiles/myNLP.dir/flags.make
CMakeFiles/myNLP.dir/MyNLP.cpp.o: /home/inazio/TFG/code/Nacho\ Code/MPC_node/non_ros/MyNLP.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/inazio/TFG/code/Nacho Code/MPC_node/build-non_ros-Desktop-Default/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/myNLP.dir/MyNLP.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myNLP.dir/MyNLP.cpp.o -c "/home/inazio/TFG/code/Nacho Code/MPC_node/non_ros/MyNLP.cpp"

CMakeFiles/myNLP.dir/MyNLP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myNLP.dir/MyNLP.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/inazio/TFG/code/Nacho Code/MPC_node/non_ros/MyNLP.cpp" > CMakeFiles/myNLP.dir/MyNLP.cpp.i

CMakeFiles/myNLP.dir/MyNLP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myNLP.dir/MyNLP.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/inazio/TFG/code/Nacho Code/MPC_node/non_ros/MyNLP.cpp" -o CMakeFiles/myNLP.dir/MyNLP.cpp.s

CMakeFiles/myNLP.dir/MyNLP.cpp.o.requires:

.PHONY : CMakeFiles/myNLP.dir/MyNLP.cpp.o.requires

CMakeFiles/myNLP.dir/MyNLP.cpp.o.provides: CMakeFiles/myNLP.dir/MyNLP.cpp.o.requires
	$(MAKE) -f CMakeFiles/myNLP.dir/build.make CMakeFiles/myNLP.dir/MyNLP.cpp.o.provides.build
.PHONY : CMakeFiles/myNLP.dir/MyNLP.cpp.o.provides

CMakeFiles/myNLP.dir/MyNLP.cpp.o.provides.build: CMakeFiles/myNLP.dir/MyNLP.cpp.o


# Object files for target myNLP
myNLP_OBJECTS = \
"CMakeFiles/myNLP.dir/MyNLP.cpp.o"

# External object files for target myNLP
myNLP_EXTERNAL_OBJECTS =

libmyNLP.a: CMakeFiles/myNLP.dir/MyNLP.cpp.o
libmyNLP.a: CMakeFiles/myNLP.dir/build.make
libmyNLP.a: CMakeFiles/myNLP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/inazio/TFG/code/Nacho Code/MPC_node/build-non_ros-Desktop-Default/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmyNLP.a"
	$(CMAKE_COMMAND) -P CMakeFiles/myNLP.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myNLP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myNLP.dir/build: libmyNLP.a

.PHONY : CMakeFiles/myNLP.dir/build

CMakeFiles/myNLP.dir/requires: CMakeFiles/myNLP.dir/MyNLP.cpp.o.requires

.PHONY : CMakeFiles/myNLP.dir/requires

CMakeFiles/myNLP.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myNLP.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myNLP.dir/clean

CMakeFiles/myNLP.dir/depend:
	cd "/home/inazio/TFG/code/Nacho Code/MPC_node/build-non_ros-Desktop-Default" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/inazio/TFG/code/Nacho Code/MPC_node/non_ros" "/home/inazio/TFG/code/Nacho Code/MPC_node/non_ros" "/home/inazio/TFG/code/Nacho Code/MPC_node/build-non_ros-Desktop-Default" "/home/inazio/TFG/code/Nacho Code/MPC_node/build-non_ros-Desktop-Default" "/home/inazio/TFG/code/Nacho Code/MPC_node/build-non_ros-Desktop-Default/CMakeFiles/myNLP.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/myNLP.dir/depend
