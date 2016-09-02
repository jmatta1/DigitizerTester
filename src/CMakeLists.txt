cmake_minimum_required (VERSION 2.6)
# This is the build file for ORCHID (Oak Ridge baCkground at HfIr Daq)
project (ORCHID)

# Add the CAEN digitizer library folders to the include stack
# This include directory may already be there but it never hurts to be certain
include_directories(/usr/include)
link_directories(/usr/lib)

# include the file where I define a couple of compiler flags, including the long
# lists of warning flags for the compiler
include(./CMakeFlagDefs.txt)

# force the existence of the BOOST library package
find_package(Boost COMPONENTS thread log log_setup REQUIRED)
#find the threading stuff
find_package(Threads REQUIRED)

# add our default g++ flags
add_definitions(-m64 -std=c++14 -D_FILE_OFFSET_BITS=64 -lpthread)

# a little hack to make Qt Creator a little happier when it runs cmake to get
# the project files

# check for each supported build type and add the appropriate g++ flags
if(BUILD_TYPE STREQUAL "Release")
    add_definitions(${GCC_OPT_FLAG})
elseif(BUILD_TYPE STREQUAL "Debug")
    add_definitions(${GCC_DEB_FLAG})
elseif(BUILD_TYPE STREQUAL "OptDebug")
    add_definitions(${GCC_DEB_FLAG})
    add_definitions(${GCC_OPT_FLAG})
elseif(BUILD_TYPE STREQUAL "Warn")
    add_definitions(${WARN_FLAGS})
elseif(BUILD_TYPE STREQUAL "Warn_Opt")
    add_definitions(${WARN_FLAGS})
    add_definitions(${WARN_OPT_FLAGS})
endif(BUILD_TYPE STREQUAL "Release")

# include the boost header dir in my system
include_directories( ${Boost_INCLUDE_DIR} )
# include the ncurses header dir
include_directories( ${CURSES_INCLUDE_DIR} )

# Grab all the source files
file(GLOB_RECURSE SRCS *.cpp)

# Grab all the header files
file(GLOB_RECURSE HDRS *.h)

# set things up so that files include as if they were at the root of the dir
# tree unless they are including a file within their own folder
include_directories(".")

# Set the executable and what it depends on
add_executable(digitizerTester ${SRCS} ${HDRS})

# make sure that the system knows to link the boost libraries
target_link_libraries( digitizerTester ${Boost_LIBRARIES})
target_link_libraries( digitizerTester ${CMAKE_THREAD_LIBS_INIT})
# now also set linking to the CAEN comm library
target_link_libraries( digitizerTester CAENComm)
