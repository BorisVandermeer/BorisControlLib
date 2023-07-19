cmake_minimum_required(VERSION 3.21.0)
project(BorisControlLib VERSION 0.0.1)
SET(THIS_VERSION 0.0.1)

# Top Level Check
if( BorisControlLib_IS_TOP_LEVEL)
    SET(IS_TOP_DIR YES)
    SET(ENABLE_TEST YES)
else()
    SET(IS_TOP_DIR NO)
    SET(ENABLE_TEST NO)
endif()

# Build Config
if(IS_TOP_DIR)
    SET(CMAKE_BUILD_TYPE "Debug")
    SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
    SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
endif()

# Find Dependencies
if(NOT BORIS_MATH_FOUND)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
        execute_process(COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
    endif()
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty/BorisMathLib)
        execute_process(COMMAND git clone https://github.com/BorisVandermeer/BorisMathLib.git --depth=1
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
    endif()

    add_subdirectory(SecondParty/BorisMathLib)
endif()


# Build Libs
include_directories(${BORIS_MATH_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
aux_source_directory(controllers CONTROLLER_SRC_FILES)

add_subdirectory(BorisPNCCommon)

add_library(BORIS_CONTROL_LIB ${CONTROLLER_SRC_FILES})

if(IS_TOP_DIR)
    SET(BORIS_CONTROL_LIBRARIES BORIS_CONTROL_LIB)
    SET(BORIS_CONTROL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
else()
    SET(BORIS_CONTROL_VERSION THIS_VERSION PARENT_SCOPE)
    SET(BORIS_CONTROL_FOUND YES PARENT_SCOPE)
    SET(BORIS_CONTROL_LIBRARIES BORIS_CONTROL_LIB PARENT_SCOPE)
    SET(BORIS_CONTROLINCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
endif()

# Build Test
if(ENABLE_TEST) 
    add_subdirectory(simulator)
endif()