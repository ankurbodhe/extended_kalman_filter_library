# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/ankur/ankur_projects/extended_kalman_filter_library

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ankur/ankur_projects/extended_kalman_filter_library

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ankur/ankur_projects/extended_kalman_filter_library/CMakeFiles /home/ankur/ankur_projects/extended_kalman_filter_library/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ankur/ankur_projects/extended_kalman_filter_library/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named Extended_Kalman_Filter

# Build rule for target.
Extended_Kalman_Filter: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 Extended_Kalman_Filter
.PHONY : Extended_Kalman_Filter

# fast build rule for target.
Extended_Kalman_Filter/fast:
	$(MAKE) -f CMakeFiles/Extended_Kalman_Filter.dir/build.make CMakeFiles/Extended_Kalman_Filter.dir/build
.PHONY : Extended_Kalman_Filter/fast

src/file_parse_util.o: src/file_parse_util.cpp.o

.PHONY : src/file_parse_util.o

# target to build an object file
src/file_parse_util.cpp.o:
	$(MAKE) -f CMakeFiles/Extended_Kalman_Filter.dir/build.make CMakeFiles/Extended_Kalman_Filter.dir/src/file_parse_util.cpp.o
.PHONY : src/file_parse_util.cpp.o

src/file_parse_util.i: src/file_parse_util.cpp.i

.PHONY : src/file_parse_util.i

# target to preprocess a source file
src/file_parse_util.cpp.i:
	$(MAKE) -f CMakeFiles/Extended_Kalman_Filter.dir/build.make CMakeFiles/Extended_Kalman_Filter.dir/src/file_parse_util.cpp.i
.PHONY : src/file_parse_util.cpp.i

src/file_parse_util.s: src/file_parse_util.cpp.s

.PHONY : src/file_parse_util.s

# target to generate assembly for a file
src/file_parse_util.cpp.s:
	$(MAKE) -f CMakeFiles/Extended_Kalman_Filter.dir/build.make CMakeFiles/Extended_Kalman_Filter.dir/src/file_parse_util.cpp.s
.PHONY : src/file_parse_util.cpp.s

src/main.o: src/main.cpp.o

.PHONY : src/main.o

# target to build an object file
src/main.cpp.o:
	$(MAKE) -f CMakeFiles/Extended_Kalman_Filter.dir/build.make CMakeFiles/Extended_Kalman_Filter.dir/src/main.cpp.o
.PHONY : src/main.cpp.o

src/main.i: src/main.cpp.i

.PHONY : src/main.i

# target to preprocess a source file
src/main.cpp.i:
	$(MAKE) -f CMakeFiles/Extended_Kalman_Filter.dir/build.make CMakeFiles/Extended_Kalman_Filter.dir/src/main.cpp.i
.PHONY : src/main.cpp.i

src/main.s: src/main.cpp.s

.PHONY : src/main.s

# target to generate assembly for a file
src/main.cpp.s:
	$(MAKE) -f CMakeFiles/Extended_Kalman_Filter.dir/build.make CMakeFiles/Extended_Kalman_Filter.dir/src/main.cpp.s
.PHONY : src/main.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... Extended_Kalman_Filter"
	@echo "... src/file_parse_util.o"
	@echo "... src/file_parse_util.i"
	@echo "... src/file_parse_util.s"
	@echo "... src/main.o"
	@echo "... src/main.i"
	@echo "... src/main.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
