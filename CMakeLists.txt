cmake_minimum_required(VERSION 3.10)
project(ned2_validator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(orocos_kdl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(kdl_parser REQUIRED)

# Include directories
include_directories(
    ${orocos_kdl_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
)

# Main validator executable
add_executable(ned2_validator ned2_validator.cpp)

target_link_libraries(ned2_validator
    ${orocos_kdl_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${kdl_parser_LIBRARIES}
)

# YAML validator (optional - needs yaml-cpp)
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    add_executable(validate_yaml_poses validate_yaml_poses.cpp)
    target_link_libraries(validate_yaml_poses
        ${orocos_kdl_LIBRARIES}
        ${urdfdom_LIBRARIES}
        ${kdl_parser_LIBRARIES}
        yaml-cpp
    )
endif()

# Install
install(TARGETS ned2_validator DESTINATION bin)
