# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/build

# Include any dependencies generated for this target.
include CMakeFiles/Project.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Project.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Project.dir/flags.make

CMakeFiles/Project.dir/GripperMain.cpp.o: CMakeFiles/Project.dir/flags.make
CMakeFiles/Project.dir/GripperMain.cpp.o: ../GripperMain.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Project.dir/GripperMain.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Project.dir/GripperMain.cpp.o -c /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/GripperMain.cpp

CMakeFiles/Project.dir/GripperMain.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Project.dir/GripperMain.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/GripperMain.cpp > CMakeFiles/Project.dir/GripperMain.cpp.i

CMakeFiles/Project.dir/GripperMain.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Project.dir/GripperMain.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/GripperMain.cpp -o CMakeFiles/Project.dir/GripperMain.cpp.s

CMakeFiles/Project.dir/GripperMain.cpp.o.requires:
.PHONY : CMakeFiles/Project.dir/GripperMain.cpp.o.requires

CMakeFiles/Project.dir/GripperMain.cpp.o.provides: CMakeFiles/Project.dir/GripperMain.cpp.o.requires
	$(MAKE) -f CMakeFiles/Project.dir/build.make CMakeFiles/Project.dir/GripperMain.cpp.o.provides.build
.PHONY : CMakeFiles/Project.dir/GripperMain.cpp.o.provides

CMakeFiles/Project.dir/GripperMain.cpp.o.provides.build: CMakeFiles/Project.dir/GripperMain.cpp.o

# Object files for target Project
Project_OBJECTS = \
"CMakeFiles/Project.dir/GripperMain.cpp.o"

# External object files for target Project
Project_EXTERNAL_OBJECTS =

Project: CMakeFiles/Project.dir/GripperMain.cpp.o
Project: CMakeFiles/Project.dir/build.make
Project: /usr/lib/x86_64-linux-gnu/libQtOpenGL.so
Project: /usr/lib/x86_64-linux-gnu/libQtGui.so
Project: /usr/lib/x86_64-linux-gnu/libQtCore.so
Project: /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/SchunkGripper/build/libSchunkGripper.a
Project: CMakeFiles/Project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Project"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Project.dir/build: Project
.PHONY : CMakeFiles/Project.dir/build

CMakeFiles/Project.dir/requires: CMakeFiles/Project.dir/GripperMain.cpp.o.requires
.PHONY : CMakeFiles/Project.dir/requires

CMakeFiles/Project.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Project.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Project.dir/clean

CMakeFiles/Project.dir/depend:
	cd /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/build /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/build /home/castella/Documents/scl-manips-v2.git/tutorial/DomiBot/gripperStandalone/build/CMakeFiles/Project.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Project.dir/depend
