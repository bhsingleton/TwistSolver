#ifndef _TwistSolverNode
#define _TwistSolverNode
//
// File: TwistSolverNode.h
//
// Dependency Graph Node: TwistSolver
//
// Author: Benjamin H. Singleton
//

#include <maya/MPxNode.h>

#include <maya/MGlobal.h>
#include <maya/MStatus.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MObject.h>
#include <maya/MTypeId.h> 
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

#include <iostream>
#include <string>
#include <cmath>


class TwistSolver : public MPxNode 
{

public:

							TwistSolver();
	virtual					~TwistSolver();

	virtual MStatus			compute(const MPlug& plug, MDataBlock& data);

	static  void*			creator();
	virtual	void			postConstructor();


	static  MStatus			initialize();
	virtual MStatus			initializeCurveRamp(MObject &node, MObject &attribute, int index, float position, float value, int interpolation);

	static	MVector			getAxisVector(int axis, MMatrix matrix, bool normalize);
	static	MVector			getAxisVector(int axis);

	static	MStatus			composeMatrix(int forwardAxis, MVector forwardVector, int upAxis, MVector upVector, MPoint pos, MMatrix &matrix);
	static	MStatus			composeMatrix(int forwardAxis, MVector forwardVector, int upAxis, MVector upVector, MMatrix &matrix);
	static	MMatrix			composeMatrix(MVector x, MVector y, MVector z, MPoint p);

	static	MStatus			createRotationMatrix(int forwardAxis, MAngle angle, MMatrix &matrix);

	static	MStatus			createCurveData(MPoint startPoint, MVector startVector, MPoint endPoint, MVector endVector, MObject &curveData);
	static	MStatus			transportVector(MObject curve, MVector &transport, MVector &tangent, int samples);
	virtual	MDoubleArray	distributeRoll(double roll, unsigned int segments, bool reverse);

	static	double			degToRad(double degrees);
	static	double			radToDeg(double radians);

	static	bool			isClose(double a, double b);
	static	bool			isClose(double a, double b, double relativeTolerance, double absoluteTolerance);

public:

	static	MObject		operation;
	static  MObject		forwardAxis;
	static  MObject		upAxis;
	static  MObject		startMatrix;
	static	MObject		startOffsetAngle;
	static  MObject		endMatrix;
	static	MObject		endOffsetAngle;
	static  MObject		inputCurve;
	static  MObject		samples;
	static	MObject		segments;
	static	MObject		inverseTwist;
	static	MObject		reverseTwist;
	static  MObject		falloff;
	static  MObject		falloffEnabled;
	static	MObject		restMatrix; // Internal tracker attributes
	static	MObject		buffer; // Internal tracker attributes

	static  MObject		twist;
	static  MObject		roll;
	static  MObject		local;

	static	MTypeId		id;

};

#endif
