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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/francesco/Desktop/computer_graphics_hw3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/francesco/Desktop/computer_graphics_hw3/build

# Include any dependencies generated for this target.
include CMakeFiles/pathtrace.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pathtrace.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pathtrace.dir/flags.make

CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o: CMakeFiles/pathtrace.dir/flags.make
CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o: ../src/pathtrace.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/francesco/Desktop/computer_graphics_hw3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o -c /Users/francesco/Desktop/computer_graphics_hw3/src/pathtrace.cpp

CMakeFiles/pathtrace.dir/src/pathtrace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pathtrace.dir/src/pathtrace.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/francesco/Desktop/computer_graphics_hw3/src/pathtrace.cpp > CMakeFiles/pathtrace.dir/src/pathtrace.cpp.i

CMakeFiles/pathtrace.dir/src/pathtrace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pathtrace.dir/src/pathtrace.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/francesco/Desktop/computer_graphics_hw3/src/pathtrace.cpp -o CMakeFiles/pathtrace.dir/src/pathtrace.cpp.s

CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o.requires:

.PHONY : CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o.requires

CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o.provides: CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o.requires
	$(MAKE) -f CMakeFiles/pathtrace.dir/build.make CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o.provides.build
.PHONY : CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o.provides

CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o.provides.build: CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o


# Object files for target pathtrace
pathtrace_OBJECTS = \
"CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o"

# External object files for target pathtrace
pathtrace_EXTERNAL_OBJECTS =

../bin/pathtrace: CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o
../bin/pathtrace: CMakeFiles/pathtrace.dir/build.make
../bin/pathtrace: CMakeFiles/pathtrace.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/francesco/Desktop/computer_graphics_hw3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/pathtrace"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pathtrace.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pathtrace.dir/build: ../bin/pathtrace

.PHONY : CMakeFiles/pathtrace.dir/build

CMakeFiles/pathtrace.dir/requires: CMakeFiles/pathtrace.dir/src/pathtrace.cpp.o.requires

.PHONY : CMakeFiles/pathtrace.dir/requires

CMakeFiles/pathtrace.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pathtrace.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pathtrace.dir/clean

CMakeFiles/pathtrace.dir/depend:
	cd /Users/francesco/Desktop/computer_graphics_hw3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/francesco/Desktop/computer_graphics_hw3 /Users/francesco/Desktop/computer_graphics_hw3 /Users/francesco/Desktop/computer_graphics_hw3/build /Users/francesco/Desktop/computer_graphics_hw3/build /Users/francesco/Desktop/computer_graphics_hw3/build/CMakeFiles/pathtrace.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pathtrace.dir/depend

