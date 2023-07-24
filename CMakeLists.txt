cmake_minimum_required(VERSION 3.21.0)
project(BorisControlLib VERSION 0.0.3)

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
    SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
    SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
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

if(NOT BORIS_PNC_COMMON_FOUND)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
        execute_process(COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
    endif()
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty/BorisPNCCommon)
        execute_process(COMMAND git clone git@github.com:BorisVandermeer/BorisPNCCommon.git --depth=1
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/SecondParty)
    endif()

    add_subdirectory(SecondParty/BorisPNCCommon)
endif()
# >>>>> Find Dependencies >>>>> #

SET(BORIS_CONTROL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} )

# Build Libs
aux_source_directory(controllers CONTROLLER_SRC_FILES)

add_library(BORIS_CONTROL_LIB ${CONTROLLER_SRC_FILES})
target_link_libraries(BORIS_CONTROL_LIB ${BORIS_MATH_LIBRARIES})
target_include_directories(BORIS_CONTROL_LIB PUBLIC ${BORIS_MATH_INCLUDE_DIRS} ${BORIS_PNC_COMMON_INCLUDE_DIRS})
target_include_directories(BORIS_CONTROL_LIB PUBLIC ${BORIS_CONTROL_INCLUDE_DIRS})

if(IS_TOP_DIR)
    SET(BORIS_CONTROL_VERSION BorisControlLib_VERSION )
    SET(BORIS_CONTROL_FOUND YES )
    SET(BORIS_CONTROL_LIBRARIES BORIS_CONTROL_LIB)
    SET(BORIS_CONTROL_INCLUDE_DIRS ${BORIS_CONTROL_INCLUDE_DIRS})
else()
    SET(BORIS_CONTROL_VERSION BorisControlLib_VERSION PARENT_SCOPE)
    SET(BORIS_CONTROL_FOUND YES PARENT_SCOPE)
    SET(BORIS_CONTROL_LIBRARIES BORIS_CONTROL_LIB PARENT_SCOPE)
    SET(BORIS_CONTROL_INCLUDE_DIRS ${BORIS_CONTROL_INCLUDE_DIRS} PARENT_SCOPE)
endif()

# Build Test
if(ENABLE_TEST) 
    add_subdirectory(simulator)
endif()