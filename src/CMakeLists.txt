cmake_minimum_required(VERSION 3.21)
project(TwistSolver)
include($ENV{DEVKIT_LOCATION}/cmake/pluginEntry.cmake)

set(
	SOURCE_FILES
	"pluginMain.cpp"
	"TwistSolverNode.h"
	"TwistSolverNode.cpp"
)

set(
	LIBRARIES
	OpenMaya
	OpenMayaAnim
	Foundation
)

build_plugin()