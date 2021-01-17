cmake_minimum_required(VERSION 3.0.0)
project(remote_shoot VERSION 0.1.0)

include(CTest)
enable_testing()

set(src
    ${CMAKE_CURRENT_SOURCE_DIR}/remote_shoot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/remote_shoot.cpp
)

add_library(remote_shoot STATIC 
    ${src}
)

find_package(glog REQUIRED)

target_link_libraries(remote_shoot glog::glog)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
