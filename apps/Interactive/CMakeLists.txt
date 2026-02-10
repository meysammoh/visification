set(APP_NAME VisificationInteractive)
project (${APP_NAME})
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g -ggdb3 -std=c++11")

SET(CPACK_PACKAGE_VERSION_MAJOR "1")
SET(CPACK_PACKAGE_VERSION_MINOR "0")
SET(CPACK_PACKAGE_VERSION_PATCH "0")

SET(APP_FULL_NAME ${APP_NAME}.${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH} )

set(PROJ_SRC
  main.cpp
  VisiX.cpp
  State.cpp
  Config.cpp
  Graphics.cpp
  XVisifier.cpp
)
include_directories ("${PROJECT_SOURCE_DIR}/lib")
add_executable(${APP_FULL_NAME} ${PROJ_SRC} )
FILE(GLOB_RECURSE LibFiles "./*.hpp")
add_custom_target(Xheaders SOURCES ${LibFiles})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

target_link_libraries(${APP_FULL_NAME}
  VisificationLib.1.0.0
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
  boost_thread
)

INSTALL(TARGETS ${APP_FULL_NAME} DESTINATION /bin/${APP_NAME})

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Meysam Mohammadi and Ali Narenji") #required

INCLUDE(CPack)
