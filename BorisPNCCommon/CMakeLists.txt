cmake_minimum_required(VERSION 3.21.0)
project(BorisPNCCommon VERSION 0.0.1)
SET(THIS_VERSION 0.0.1)

if( BorisPNCCommon_IS_TOP_LEVEL)
    SET(IS_TOP_DIR YES)
    # SET(ENABLE_TEST YES)
else()
    SET(IS_TOP_DIR NO)
    # SET(ENABLE_TEST NO)
endif()

# <<<<< Find Dependencies <<<<< #
if(NOT BORIS_MATH_FOUND)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
        execute_process(COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
    endif()
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty/BorisMathLib)
        execute_process(COMMAND git clone git@github.com:BorisVandermeer/BorisMathLib.git --depth=1
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
    endif()

    add_subdirectory(SecondParty/BorisMathLib)
endif()
# >>>>> Find Dependencies >>>>> #


# Find SRCs
include_directories(.)
aux_source_directory(Trajectory SRC_FILES_P1)

set(SRC_FILES ${SRC_FILES_P1})

add_library(BORIS_PNC_COMMON_LIB ${SRC_FILES})

if(IS_TOP_DIR)
    SET(BORIS_PNC_COMMON_VERSION THIS_VERSION)
    SET(BORIS_PNC_COMMON_FOUND YES)
    SET(BORIS_PNC_COMMON_LIBRARIES BORIS_PNC_COMMON_LIB)
    SET(BORIS_PNC_COMMON_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
else()
    SET(BORIS_PNC_COMMON_VERSION THIS_VERSION PARENT_SCOPE)
    SET(BORIS_PNC_COMMON_FOUND YES PARENT_SCOPE)
    SET(BORIS_PNC_COMMON_LIBRARIES BORIS_MATH_LIB PARENT_SCOPE)
    SET(BORIS_PNC_COMMON_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
endif()
