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
CMAKE_SOURCE_DIR = /home/jdflo/GIT_REPOS/art_skills

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jdflo/GIT_REPOS/art_skills/build

# Utility rule file for chebyshev_alphabet_test.valgrind.

# Include the progress variables for this target.
include cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/progress.make

cpp/CMakeFiles/chebyshev_alphabet_test.valgrind: cpp/chebyshev_alphabet_test
	cd /home/jdflo/GIT_REPOS/art_skills/build/cpp && valgrind --error-exitcode=1 /home/jdflo/GIT_REPOS/art_skills/build/cpp/chebyshev_alphabet_test

chebyshev_alphabet_test.valgrind: cpp/CMakeFiles/chebyshev_alphabet_test.valgrind
chebyshev_alphabet_test.valgrind: cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/build.make

.PHONY : chebyshev_alphabet_test.valgrind

# Rule to build all files generated by this target.
cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/build: chebyshev_alphabet_test.valgrind

.PHONY : cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/build

cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/clean:
	cd /home/jdflo/GIT_REPOS/art_skills/build/cpp && $(CMAKE_COMMAND) -P CMakeFiles/chebyshev_alphabet_test.valgrind.dir/cmake_clean.cmake
.PHONY : cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/clean

cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/depend:
	cd /home/jdflo/GIT_REPOS/art_skills/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jdflo/GIT_REPOS/art_skills /home/jdflo/GIT_REPOS/art_skills/cpp /home/jdflo/GIT_REPOS/art_skills/build /home/jdflo/GIT_REPOS/art_skills/build/cpp /home/jdflo/GIT_REPOS/art_skills/build/cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cpp/CMakeFiles/chebyshev_alphabet_test.valgrind.dir/depend

