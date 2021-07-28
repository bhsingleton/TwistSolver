//
// File: pluginMain.cpp
//
// Author: Ben Singleton
//

#include "TwistSolverNode.h"

#include <maya/MFnPlugin.h>


MStatus initializePlugin(MObject obj) 
{ 

	MStatus   status;

	MFnPlugin plugin( obj, "Ben Singleton", "2017", "Any");
	status = plugin.registerNode("twistSolver", TwistSolver::id, TwistSolver::creator, TwistSolver::initialize);

	if (!status) 
	{

		status.perror("registerNode");
		return status;

	}

	return status;

}

MStatus uninitializePlugin(MObject obj)
{

	MStatus   status;

	MFnPlugin plugin(obj);
	status = plugin.deregisterNode(TwistSolver::id);

	if (!status)
	{

		status.perror("deregisterNode");
		return status;

	}

	return status;

}
