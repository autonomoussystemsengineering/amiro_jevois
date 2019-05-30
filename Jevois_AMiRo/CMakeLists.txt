cmake_minimum_required (VERSION 2.8)

string (REGEX MATCH "[^/]+$" PROJECT_NAME "${CMAKE_CURRENT_BINARY_DIR}")
message("CMAKE_CURRENT_LIST_DIR is ${CMAKE_CURRENT_LIST_DIR}")

#--------------------------------------------------------------------------------------
##########################Jevois Aruco Projekt#########################################
#--------------------------------------------------------------------------------------
project ("jevoisAruco")

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)

add_executable ("jevoisAruco"
  jevoisAruco_AmiroDist.cpp
)

# Set the project name to the name of the folder
#INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/AdditionalCMakeFiles/setProjectName.cmake")

# Get modules
#SET (BOOST_COMPONENTS regex date_time program_options system thread)
#INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
#INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/AdditionalCMakeFiles/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    #"$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_LIST_DIR}/includes"
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    #${Boost_INCLUDE_DIR}
)

# Define the source
SET (SOURCE main.cxx)

TARGET_LINK_LIBRARIES (
    "jevoisAruco"
    #${Boost_LIBRARIES}
)

ADD_DEFINITIONS(-std=c++0x)

# Additional compiler flags
# ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++0x)
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++0x" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++0x" )



#--------------------------------------------------------------------------------------
##########################Jevois Optical Flow Projekt##################################
#--------------------------------------------------------------------------------------


project ("jevoisOpticalFlow")

# Read the pkg-config modules
FIND_PACKAGE(PkgConfig)

add_executable ("jevoisOpticalFlow"
  jevoisOpticalFlow.cpp
)

# Set the project name to the name of the folder
#INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setProjectName.cmake")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/AdditionalCMakeFiles/setProjectName.cmake")

# Get modules
#SET (BOOST_COMPONENTS regex date_time program_options system thread)
#INCLUDE("$ENV{MUROX_CMAKE_MODULES}/findPackageBoost.cmake")

# Get the compiler specific includes and store them in GXX_INCLUDES
#INCLUDE("$ENV{MUROX_CMAKE_MODULES}/setCompilerIncludes.cmake")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/AdditionalCMakeFiles/setCompilerIncludes.cmake")

# Set the include pathe
include_directories(
    #"$ENV{MUROX_INCLUDE_DIRS}"                                      # Other includes like converters and stuff
    "${CMAKE_CURRENT_LIST_DIR}/includes"
    ${CMAKE_CURRENT_BINARY_DIR}                                     # add the binary tree to the search path for
    ${GXX_INCLUDES}
    #${Boost_INCLUDE_DIR}
)

# Define the source
SET (SOURCE main.cxx)

TARGET_LINK_LIBRARIES (
    "jevoisOpticalFlow"
    #${Boost_LIBRARIES}
)

ADD_DEFINITIONS(-std=c++0x)

# Additional compiler flags
# ADD_DEFINITIONS (-Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++0x)
SET ( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-deprecated-declarations -O0 -g -Wno-write-strings -std=c++0x" )
SET ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -Wno-deprecated-declarations -O2 -Wno-write-strings -std=c++0x" )


