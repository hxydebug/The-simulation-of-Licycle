cmake_minimum_required(VERSION 3.10)
project(raisim_examples LANGUAGES CXX)

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
include_directories (${QPOASES_INLUDE_DIRS})
include_directories (${EIGEN3_INCLUDE_DIRS})

include_directories(
    include
    third_party/alglib/src
)

file(GLOB_RECURSE alglib_files "${CMAKE_CURRENT_SOURCE_DIR}/third_party/alglib/src/**.cpp")
add_library(alglib_sdk ${alglib_files})

add_executable(nlctest
    nlctest.cpp
)
target_link_libraries(nlctest
    alglib_sdk
)

add_library(leg_controller	src/leg_controller.cpp)
add_library(bicycle_controller	src/bicycle_controller.cpp)
add_library(swing_leg_controller	src/swing_leg_controller.cpp)
add_library(stance_leg_controller	src/stance_leg_controller.cpp)
add_library(control	src/control.cpp)
add_library(gait_generator	src/gait_generator.cpp)
add_library(model	src/model.cpp)
add_library(opmath	src/opmath.cpp)

add_executable(unmanned_bicycle main.cpp)
target_link_libraries(unmanned_bicycle PUBLIC raisim::raisim pthread leg_controller bicycle_controller model opmath swing_leg_controller stance_leg_controller gait_generator control)
target_include_directories(unmanned_bicycle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(model PUBLIC raisim::raisim pthread opmath)
target_include_directories(model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(opmath PUBLIC pthread)
target_include_directories(opmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(leg_controller PUBLIC raisim::raisim pthread model opmath swing_leg_controller stance_leg_controller control)
target_include_directories(leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(bicycle_controller PUBLIC raisim::raisim pthread model opmath)
target_include_directories(bicycle_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(gait_generator PUBLIC raisim::raisim pthread model opmath)
target_include_directories(gait_generator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(stance_leg_controller PUBLIC raisim::raisim pthread model opmath qpOASES)
target_include_directories(stance_leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(swing_leg_controller PUBLIC raisim::raisim pthread model opmath control)
target_include_directories(swing_leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(control PUBLIC raisim::raisim pthread opmath)
target_include_directories(control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(qptest qptest.cpp)
target_link_libraries(qptest qpOASES)


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_BINARY_DIR})
