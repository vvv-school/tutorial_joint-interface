# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(tutorial_joint-interface)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

include_directories(${YARP_INCLUDE_DIRS})

# position
add_executable(${PROJECT_NAME}-position ${CMAKE_SOURCE_DIR}/src/position.cpp)
target_link_libraries(${PROJECT_NAME}-position ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-position DESTINATION bin)

# positiondirect
add_executable(${PROJECT_NAME}-positiondirect ${CMAKE_SOURCE_DIR}/src/positiondirect.cpp)
target_link_libraries(${PROJECT_NAME}-positiondirect ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-positiondirect DESTINATION bin)

# velocity
add_executable(${PROJECT_NAME}-velocity ${CMAKE_SOURCE_DIR}/src/velocity.cpp)
target_link_libraries(${PROJECT_NAME}-velocity ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-velocity DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
