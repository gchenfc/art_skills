# add_subdirectory(tests)

cmake_minimum_required(VERSION 3.10)
enable_testing()
project(planarnav CXX C)

add_compile_options(
	-faligned-new
)

set(CMAKE_CXX_STANDARD 17)

#####################
# Find Dependencies #
#####################

# Find GTSAM components.
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBS gtsam)

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# For unit tests and scripts.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam.
find_package(Boost 1.43 REQUIRED)
find_package(Boost COMPONENTS filesystem system thread program_options serialization regex timer REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

if(NOT (${Boost_VERSION} LESS 105600))
	message("Ignoring Boost restriction on optional lvalue assignment from rvalues")
	add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

# For cython wrapper.
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

#####################################
# Headers/Sources
#####################################

# Add this directory as include.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Glob all source/header files in SOURCE_SUBDIRS.
file(GLOB sources "*.cpp")
file(GLOB headers "*.h")

add_library(planarnav SHARED ${headers} ${sources})

target_link_libraries(planarnav ${GTSAM_LIBS})

target_include_directories(planarnav PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Install library and headers.
install(TARGETS planarnav LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
set_target_properties(planarnav PROPERTIES LINKER_LANGUAGE CXX)

#####################################
# Executables
#####################################

# add_executable(main main.cpp)
# target_link_libraries(main PUBLIC gtsam planarnav)
# target_include_directories(main PUBLIC
# 	${GTSAM_INCLUDE_DIR}
# 	${CMAKE_CURRENT_SOURCE_DIR}/include)

gtsamAddExamplesGlob("*.cpp" "" "planarnav")
add_subdirectory(tests)


