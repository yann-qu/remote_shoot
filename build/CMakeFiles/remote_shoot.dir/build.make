# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.17

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

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = D:\CMake\bin\cmake.exe

# The command to remove a file.
RM = D:\CMake\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\57315\Desktop\remote_shoot_test\remote_shoot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\build

# Include any dependencies generated for this target.
include CMakeFiles/remote_shoot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/remote_shoot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/remote_shoot.dir/flags.make

CMakeFiles/remote_shoot.dir/remote_shoot.cpp.obj: CMakeFiles/remote_shoot.dir/flags.make
CMakeFiles/remote_shoot.dir/remote_shoot.cpp.obj: ../remote_shoot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/remote_shoot.dir/remote_shoot.cpp.obj"
	D:\mingw64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\remote_shoot.dir\remote_shoot.cpp.obj -c C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\remote_shoot.cpp

CMakeFiles/remote_shoot.dir/remote_shoot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/remote_shoot.dir/remote_shoot.cpp.i"
	D:\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\remote_shoot.cpp > CMakeFiles\remote_shoot.dir\remote_shoot.cpp.i

CMakeFiles/remote_shoot.dir/remote_shoot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/remote_shoot.dir/remote_shoot.cpp.s"
	D:\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\remote_shoot.cpp -o CMakeFiles\remote_shoot.dir\remote_shoot.cpp.s

# Object files for target remote_shoot
remote_shoot_OBJECTS = \
"CMakeFiles/remote_shoot.dir/remote_shoot.cpp.obj"

# External object files for target remote_shoot
remote_shoot_EXTERNAL_OBJECTS =

libremote_shoot.a: CMakeFiles/remote_shoot.dir/remote_shoot.cpp.obj
libremote_shoot.a: CMakeFiles/remote_shoot.dir/build.make
libremote_shoot.a: CMakeFiles/remote_shoot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libremote_shoot.a"
	$(CMAKE_COMMAND) -P CMakeFiles\remote_shoot.dir\cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\remote_shoot.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/remote_shoot.dir/build: libremote_shoot.a

.PHONY : CMakeFiles/remote_shoot.dir/build

CMakeFiles/remote_shoot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\remote_shoot.dir\cmake_clean.cmake
.PHONY : CMakeFiles/remote_shoot.dir/clean

CMakeFiles/remote_shoot.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\57315\Desktop\remote_shoot_test\remote_shoot C:\Users\57315\Desktop\remote_shoot_test\remote_shoot C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\build C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\build C:\Users\57315\Desktop\remote_shoot_test\remote_shoot\build\CMakeFiles\remote_shoot.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/remote_shoot.dir/depend

