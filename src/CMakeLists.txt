cmake_minimum_required(VERSION 3.21)
project(TwistSolver)

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/../modules")

set(
	SOURCE_FILES
	"pluginMain.cpp"
	"TwistSolverNode.cpp"
	"TwistSolverNode.h"
)

find_package(Maya REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${MAYA_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
link_directories(${MAYA_LIBRARY_DIR})

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES} ${OPENGL_LIBRARIES})

MAYA_PLUGIN(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION ${MAYA_VERSION})