#ifndef _TWIST_SOLVER_NODE
#define _TWIST_SOLVER_NODE
#define _USE_MATH_DEFINES
#define RELATIVE_TOLERANCE	1e-03
#define ABSOLUTE_TOLERANCE	0.0
//
// File: TwistSolverNode.h
//
// Dependency Graph Node: twistSolver
//
// Author: Benjamin H. Singleton
//

#include <maya/MPxNode.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MObject.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MEulerRotation.h>
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MAngle.h>
#include <maya/MPointArray.h>
#include <maya/MVectorArray.h>
#include <maya/MDoubleArray.h>
#include <maya/MAnimControl.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnNurbsCurveData.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MRampAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MStatus.h>
#include <maya/MTypeId.h> 

#include <iostream>
#include <string>
#include <cmath>


enum class Axis
{

	X = 0,
	Y = 1,
	Z = 2

};


class TwistSolver : public MPxNode 
{

public:

										TwistSolver();
	virtual								~TwistSolver();

	virtual MStatus						compute(const MPlug& plug, MDataBlock& data);

	virtual	MPxNode::SchedulingType		schedulingType() const;

	static	MVector						getAxisVector(const MMatrix& matrix, const Axis axis, const bool flip, const bool normalize);
	static	MVector						getAxisVector(const Axis axis, const bool flip);

	static	MStatus						composeMatrix(const Axis forwardAxis, const MVector& forwardVector, const Axis upAxis, const MVector& upVector, const MPoint& pos, MMatrix& matrix);
	static	MStatus						composeMatrix(const Axis forwardAxis, const MVector& forwardVector, const Axis upAxis, const MVector& upVector, MMatrix& matrix);
	static	MMatrix						composeMatrix(const MVector& xAxis, const MVector& yAxis, const MVector& zAxis, const MPoint& position);

	static	MStatus						createCurveData(const MPoint& startPoint, const MVector& startVector, const MPoint& endPoint, const MVector& endVector, MObject &curveData);
	static	MMatrix						transportMatrix(const MObject& curve, const Axis upAxis, const bool upAxisFlip, const MMatrix& matrix, MStatus* status);
	static	MVector						projectVector(const MPoint& origin, const MVector& normal, const MVector& vector);
	virtual	MDoubleArray				distributeRoll(const double roll, const unsigned int segments, const bool reverse);

	static	double						degToRad(const double degrees);
	static	double						radToDeg(const double radians);

	static	bool						isClose(const double a, const double b);
	static	bool						isClose(const double a, const double b, const double relativeTolerance, const double absoluteTolerance);

	static  void*						creator();
	virtual	void						postConstructor();

	static  MStatus						initialize();
	virtual MStatus						initializeCurveRamp(MObject& node, MObject& attribute, const int index, const float position, const float value, const int interpolation);

public:

	static  MObject						forwardAxis;
	static  MObject						forwardAxisFlip;
	static  MObject						upAxis;
	static  MObject						upAxisFlip;
	static  MObject						startMatrix;
	static  MObject						startOffsetMatrix;
	static  MObject						endMatrix;
	static	MObject						endOffsetMatrix;
	static  MObject						inCurve;
	static	MObject						segments;
	static	MObject						inverse;
	static	MObject						reverse;
	static  MObject						falloff;
	static  MObject						falloffEnabled;

	static  MObject						twist;
	static  MObject						roll;
	static  MObject						debug;

	static	MString						inputCategory;
	static	MString						outputCategory;

	static	MTypeId						id;

};

#endif
