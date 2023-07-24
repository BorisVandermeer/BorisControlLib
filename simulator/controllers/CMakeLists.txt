find_package(OpenCV REQUIRED)

SET(TEST_LINKED_LIBRARIES 
    ${OpenCV_LIBS} 
    ${BORIS_MATH_LIBRARIES} ${BORIS_PNC_COMMON_LIBRARIES}
    ${BORIS_CONTROL_LIBRARIES} 
    )

SET(TEST_INCLUDED_DIRS (
    ${OpenCV_INCLUDE_DIRS})
    ${BORIS_MATH_INCLUDE_DIRS} ${BORIS_PNC_COMMON_INCLUDE_DIRS}
    ${BORIS_CONTROL_INCLUDE_DIRS}
    )

include_directories(${TEST_INCLUDED_DIRS})

add_executable(PIDSimulator PIDSimulator.cpp)
target_link_libraries(PIDSimulator ${TEST_LINKED_LIBRARIES} )

add_executable(PPSimulator PPSimulator.cpp)
target_link_libraries(PPSimulator ${TEST_LINKED_LIBRARIES} )