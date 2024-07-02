//
// File: TwistSolverNode.cpp
//
// Author: Benjamin H. Singleton
//

#include "TwistSolverNode.h"

MObject TwistSolver::forwardAxis;
MObject TwistSolver::forwardAxisFlip;
MObject TwistSolver::upAxis;
MObject TwistSolver::upAxisFlip;
MObject TwistSolver::startMatrix;
MObject TwistSolver::startOffsetMatrix;
MObject TwistSolver::endMatrix;
MObject TwistSolver::endOffsetMatrix;
MObject	TwistSolver::inCurve;
MObject	TwistSolver::segments;
MObject	TwistSolver::inverse;
MObject	TwistSolver::reverse;
MObject TwistSolver::falloff;
MObject TwistSolver::falloffEnabled;

MObject TwistSolver::twist;
MObject TwistSolver::roll;
MObject	TwistSolver::debug;

MString TwistSolver::inputCategory("Input");
MString TwistSolver::outputCategory("Output");

MTypeId TwistSolver::id(0x0013b1c1);


TwistSolver::TwistSolver() {}
TwistSolver::~TwistSolver() {}


MStatus TwistSolver::compute(const MPlug& plug, MDataBlock& data)
/**
This method should be overridden in user defined nodes.
Recompute the given output based on the nodes inputs.
The plug represents the data value that needs to be recomputed, and the data block holds the storage for all of the node's attributes.
The MDataBlock will provide smart handles for reading and writing this node's attribute values.
Only these values should be used when performing computations!

@param plug: Plug representing the attribute that needs to be recomputed.
@param data: Data block containing storage for the node's attributes.
@return: Return status.
*/
{
	
	MStatus status;
	
	// Check which attribute requires computing
	//
	MObject attribute = plug.attribute(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnAttribute fnAttribute(attribute, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	bool isOutput = fnAttribute.hasCategory(TwistSolver::outputCategory);

	if (isOutput)
	{
		
		// Get input data handles
		//
		MDataHandle forwardAxisHandle = data.inputValue(TwistSolver::forwardAxis, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle forwardAxisFlipHandle = data.inputValue(TwistSolver::forwardAxisFlip, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle upAxisHandle = data.inputValue(TwistSolver::upAxis, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle upAxisFlipHandle = data.inputValue(TwistSolver::upAxisFlip, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle startMatrixHandle = data.inputValue(TwistSolver::startMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle startOffsetMatrixHandle = data.inputValue(TwistSolver::startOffsetMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle endMatrixHandle = data.inputValue(TwistSolver::endMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle endOffsetMatrixHandle = data.inputValue(TwistSolver::endOffsetMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle inCurveHandle = data.inputValue(TwistSolver::inCurve, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle segmentsHandle = data.inputValue(TwistSolver::segments, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle inverseHandle = data.inputValue(TwistSolver::inverse, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle reverseHandle = data.inputValue(TwistSolver::reverse, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle falloffEnabledHandle = data.inputValue(TwistSolver::falloffEnabled, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get data values
		//
		Axis forwardAxis = Axis(forwardAxisHandle.asShort());
		bool forwardAxisFlip = forwardAxisFlipHandle.asBool();
		Axis upAxis = Axis(upAxisHandle.asShort());
		bool upAxisFlip = upAxisFlipHandle.asBool();
		short segments = segmentsHandle.asShort();
		bool inverse = inverseHandle.asBool();
		bool reverse = reverseHandle.asBool();
		bool falloffEnabled = falloffEnabledHandle.asBool();
		
		MMatrix startOffsetMatrix = startOffsetMatrixHandle.asMatrix();
		MMatrix startMatrix = startOffsetMatrix * startMatrixHandle.asMatrix();
		MMatrix endOffsetMatrix = endOffsetMatrixHandle.asMatrix();
		MMatrix endMatrix = endOffsetMatrix * endMatrixHandle.asMatrix();

		// Check if curve is valid
		//
		MObject curve = inCurveHandle.asNurbsCurve();
		bool hasCurve = !curve.isNull();

		if (!hasCurve)
		{

			// Build curve data from start/end matrices
			//
			MVector startVector = TwistSolver::getAxisVector(startMatrix, forwardAxis, forwardAxisFlip, true);
			MVector endVector = TwistSolver::getAxisVector(endMatrix, forwardAxis, forwardAxisFlip, true);

			MPoint startPoint = MPoint(startMatrix[3]);
			MPoint endPoint = MPoint(endMatrix[3]);

			status = TwistSolver::createCurveData(startPoint, startVector, endPoint, endVector, curve); 
			CHECK_MSTATUS_AND_RETURN_IT(status);

		}

		// Transport start matrix along curve
		//
		MMatrix transportMatrix = TwistSolver::transportMatrix(curve, upAxis, upAxisFlip, startMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Evaluate twist angle
		// 
		MVector initialUpVector = TwistSolver::getAxisVector(transportMatrix, upAxis, upAxisFlip, true);
		MVector forwardVector = TwistSolver::getAxisVector(endMatrix, forwardAxis, forwardAxisFlip, true);

		MVector startUpVector = TwistSolver::getAxisVector(endMatrix, upAxis, upAxisFlip, true);
		MVector endUpVector = TwistSolver::projectVector(MPoint::origin, forwardVector, initialUpVector).normal();
		
		double sign = ((endUpVector ^ startUpVector).normal() * forwardVector > 0.0) ? 1.0 : -1.0;
		double angle = startUpVector.angle(endUpVector) * sign;

		// Build output array
		//
		MArrayDataHandle twistHandle = data.outputArrayValue(TwistSolver::twist, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataBuilder builder(&data, TwistSolver::twist, segments, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		double roll = (inverse) ? -angle : angle;
		MDoubleArray twists = this->distributeRoll(roll, segments, reverse);

		MDataHandle element;
		MAngle twist;

		for (int i = 0; i < segments; i++) 
		{

			twist = MAngle(twists[i], MAngle::kRadians);

			element = builder.addElement(i, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			element.setMAngle(twist);
			element.setClean();

		}

		status = twistHandle.set(builder);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = twistHandle.setAllClean();
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Update data handle with array builder
		//
		MDataHandle rollHandle = data.outputValue(TwistSolver::roll, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		rollHandle.setMAngle(MAngle(roll, MAngle::kRadians));
		rollHandle.setClean();

		MDataHandle debugHandle = data.outputValue(TwistSolver::debug, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		debugHandle.setMObject(curve);
		debugHandle.setClean();

		// Mark plug as clean
		//
		status = data.setClean(plug);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;

	}
	else
	{

		return MS::kUnknownParameter;

	}

};


MPxNode::SchedulingType TwistSolver::schedulingType() const
/**
When overridden this method controls the degree of parallelism supported by the node during threaded evaluation.

@return: The scheduling type to be used for this node.
*/
{

	return MPxNode::SchedulingType::kParallel;

};


MStatus TwistSolver::createCurveData(const MPoint& startPoint, const MVector& startVector, const MPoint& endPoint, const MVector& endVector, MObject &curveData)
/**
Creates a curve data object that smoothly passes through the supplied points and forward vectors.

@param startPoint: The start of the curve.
@param startVector: The forward vector for the start point.
@param endPoint: The end of the curve.
@param endVector: The forward vector for the end point.
@param curveData: The passed data object to populate.
@return: Return status.
*/
{

	MStatus status;

	// Get aim length
	//
	MVector aimVector = endPoint - startPoint;
	double aimLength = aimVector.length();

	double distance = (aimLength > 1.0) ? aimLength : 1.0;

	// Calculate mid points
	//
	MPoint p0 = startPoint;
	MPoint p1 = startPoint + (startVector * (distance * 0.25));

	MPoint mid = startPoint + (startVector * (distance * 0.5));

	MPoint p2 = mid + (endVector * (distance * 0.25));
	MPoint p3 = mid + (endVector * (distance * 0.5));

	MPointArray controlVertices(4);
	controlVertices[0] = p0;
	controlVertices[1] = p1;
	controlVertices[2] = p2;
	controlVertices[3] = p3;

	// Define curve knots
	//
	double src[6] = { 0, 0, 0, 1, 1, 1 };
	MDoubleArray knots(src, 6);

	// Create curve data to sample from
	//
	MFnNurbsCurveData fnCurveData;
	curveData = fnCurveData.create();

	MFnNurbsCurve fnCurve;

	MObject curveGeom = fnCurve.create(controlVertices, knots, 3, MFnNurbsCurve::kOpen, false, true, curveData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;

};


MMatrix TwistSolver::transportMatrix(const MObject& curve, const Axis upAxis, const bool upAxisFlip, const MMatrix &matrix, MStatus* status)
/**
Transports a vector along a curve based on the number of parameter samples.

@param curve: The curve object.
@param transport: The passed vector to transport.
@param tangent: The up vector to assist the transportation.
@param samples: The number of points to sample along the curve.
@return: Return status.
*/
{

	// Calculate number of required samples
	//
	MFnNurbsCurve fnCurve(curve, status);
	CHECK_MSTATUS_AND_RETURN(*status, MMatrix::identity);

	unsigned int numCVs = fnCurve.numCVs(status);
	CHECK_MSTATUS_AND_RETURN(*status, MMatrix::identity);

	unsigned int samples = std::pow(numCVs, 2);

	// Transport matrix across sample range
	//
	MMatrix transportMatrix = MMatrix(matrix);
	MVector upVector = TwistSolver::getAxisVector(matrix, upAxis, upAxisFlip, true);

	double length = fnCurve.length();
	double step = static_cast<double>(1.0 / samples);

	MMatrix startMatrix, endMatrix;
	MPoint position;
	MVector forwardVector, rightVector;
	double distance, parameter;

	for (int i = 0; i < samples; i++)
	{

		// Find parameter from distance
		// Don't forget to round off distances or else the tangents will zero out!
		//
		distance = (step * static_cast<double>(i)) * length;

		if (distance <= 0.0)
		{

			distance += 1e-3;

		}
		else if (distance >= length)
		{

			distance -= 1e-3;

		}
		else;

		parameter = fnCurve.findParamFromLength(distance, status);
		CHECK_MSTATUS_AND_RETURN(*status, MMatrix::identity);

		// Get next tangent and transport vector
		//
		forwardVector = fnCurve.tangent(parameter, MSpace::kWorld, status).normal();
		CHECK_MSTATUS_AND_RETURN(*status, MMatrix::identity);

		rightVector = (forwardVector ^ upVector).normal();
		upVector = (rightVector ^ forwardVector).normal();

		// Multiply start matrix into curve space for first iteration
		//
		if (i == 0)
		{

			*status = fnCurve.getPointAtParam(parameter, position, MSpace::kWorld);
			CHECK_MSTATUS_AND_RETURN(*status, MMatrix::identity);

			startMatrix = TwistSolver::composeMatrix(forwardVector, upVector, rightVector, position);
			transportMatrix *= startMatrix.inverse();

		}

	}

	// Multiply start matrix out of curve space
	//
	endMatrix = TwistSolver::composeMatrix(forwardVector, upVector, rightVector, position);
	transportMatrix *= endMatrix;

	return transportMatrix;

};


MVector TwistSolver::projectVector(const MPoint& origin, const MVector& normal, const MVector& vector)
/**
Projects the supplied vector onto the specified normal.

@param origin: The position of the normal.
@param normal: The vector direction of the normal.
@param vector: The vector to project.
@return: The projected vector.
*/
{

	MVector unit = normal.normal();
	double dot = unit * vector;
	
	return origin + (vector - (unit * dot));

};


MStatus TwistSolver::composeMatrix(const Axis forwardAxis, const MVector& forwardVector, const Axis upAxis, const MVector& upVector, const MPoint& position, MMatrix& matrix)
/**
Composes a matrix the given forward/up vector and position.
Axis enumerator values can be supplied to designate the vectors.

@param forwardAxis: The forward axis to assign the forward vector to.
@param forwardVector: The forward vector of the matrix.
@param upAxis: The up axis to assign the up vector to.
@param upVector: Tangent vector to resolve the last remaining axis.
@param pos: The position of the transform matrix.
@param matrix: The passed matrix to populate.
@return: Return status.
*/
{

	MStatus status;

	// Declare axis vectors
	//
	MVector xAxis, yAxis, zAxis;

	switch (forwardAxis) 
	{

		case Axis::X:
		{

			// Assign forward vector
			//
			xAxis = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == Axis::X) 
			{

				return MS::kFailure;

			}
			else if (upAxis == Axis::Y) 
			{

				zAxis = (xAxis ^ upVector).normal();
				yAxis = (zAxis ^ xAxis).normal();

			}
			else if (upAxis == Axis::Z) 
			{

				yAxis = (upVector ^ xAxis).normal();
				zAxis = (xAxis ^ yAxis).normal();

			}

		}
		break;

		case Axis::Y:
		{

			// Assign forward vector
			//
			yAxis = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == Axis::X)
			{

				zAxis = (upVector ^ yAxis).normal();
				xAxis = (yAxis ^ zAxis).normal();

			}
			else if (upAxis == Axis::Y)
			{

				return MS::kFailure;

			}
			else if (upAxis == Axis::Z)
			{

				xAxis = (yAxis ^ upVector).normal();
				zAxis = (xAxis ^ yAxis).normal();

			}

		}
		break;

		case Axis::Z:
		{

			// Assign forward vector
			//
			zAxis = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == Axis::X) 
			{

				yAxis = (zAxis ^ upVector).normal();
				xAxis = (yAxis ^ zAxis).normal();

			}
			else if (upAxis == Axis::Y)
			{

				xAxis = (upVector ^ zAxis).normal();
				yAxis = (zAxis ^ xAxis).normal();

			}
			else if (upAxis == Axis::Z)
			{

				return MS::kFailure;

			}

		}
		break;

		default:
		{

			return MS::kFailure;

		}
		break;

	}

	// Return matrixAxis from axis vectors
	//
	matrix = TwistSolver::composeMatrix(xAxis, yAxis, zAxis, position);

	return MS::kSuccess;

};


MStatus TwistSolver::composeMatrix(const Axis forwardAxis, const MVector& forwardVector, const Axis upAxis, const MVector& upVector, MMatrix& matrix)
/**
Composes a matrix the given forward/up vector and position.
Axis enumerator values can be supplied to designate the vectors.

@param forwardAxis: The forward axis to assign the forward vector to.
@param forwardVector: The forward vector of the matrix.
@param upAxis: The up axis to assign the up vector to.
@param upVector: Tangent vector to resolve the last remaining axis.
@param matrix: The passed matrix to populate.
@return: Return status.
*/
{

	return TwistSolver::composeMatrix(forwardAxis, forwardVector, upAxis, upVector, MPoint::origin, matrix);

};


MMatrix TwistSolver::composeMatrix(const MVector& x, const MVector& y, const MVector& z, const MPoint& pos)
/**
Composes a matrix from the axis vectors and position.

@param x: The X axis vector.
@param y: The Y axis vector.
@param z: The Z axis vector.
@param p: The position.
@return: MMatrix
*/
{

	const double rows[4][4] = {
		{x.x, x.y, x.z, 0.0},
		{y.x, y.y, y.z, 0.0},
		{z.x, z.y, z.z, 0.0},
		{pos.x, pos.y, pos.z, 1.0}
	};

	return MMatrix(rows);

};


MDoubleArray TwistSolver::distributeRoll(const double roll, const unsigned int segments, const bool reverse)
/**
Breaks down the supplied roll value into individual twist values.

@param roll: The roll value.
@param segments: The number of segments to distribute.
@param reverse: Whether to reverse the twist values.
@return: MDoubleArray
*/
{

	MStatus status;

	// Get curve ramp attribute
	//
	MObject node = this->thisMObject();

	MPlug falloffEnabledPlug = MPlug(node, TwistSolver::falloffEnabled);
	bool falloffEnabled = falloffEnabledPlug.asBool();

	MRampAttribute falloffRamp(node, TwistSolver::falloff, &status);
	CHECK_MSTATUS(status);

	// Distribute roll values
	//
	MDoubleArray twists(segments);

	double fraction = roll / (static_cast<double>(segments) - 1.0);

	double increment = (reverse) ? -fraction : fraction;
	double accum = (reverse) ? roll : 0.0;

	float pos, val;

	for (unsigned int i = 0; i < segments; i++)
	{

		// Check if falloff is enabled
		//
		double twist;

		if (falloffEnabled)
		{

			pos = ((1.0f / (static_cast<float>(segments) - 1.0f)) * static_cast<float>(i));

			falloffRamp.getValueAtPosition(pos, val);
			twist = roll * static_cast<double>(val);

		}
		else
		{

			twist = accum;

		}

		twists[i] = twist;
		accum += fraction;

	}

	return twists;

};


MVector	TwistSolver::getAxisVector(const MMatrix& matrix, const Axis axis, const bool flip, const bool normalize)
/**
Returns the axis vector from the supplied matrix

@param matrix: The transform matrix to sample from.
@param axis: The axis vector to return.
@param flip: Flips the requested axis vector.
@param normalize: Normalizes the requested axis vector.
@return: MVector
*/
{

	// Get vector
	//
	double sign = flip ? -1.0 : 1.0;
	MVector axisVector = MVector::zero;

	switch (axis)
	{

		case Axis::X:
		{

			axisVector = MVector(matrix(0, 0), matrix(0, 1), matrix(0, 2)) * sign;

		}
		break;

		case Axis::Y:
		{

			axisVector = MVector(matrix(1, 0), matrix(1, 1), matrix(1, 2)) * sign;

		}
		break;

		case Axis::Z:
		{

			axisVector = MVector(matrix(2, 0), matrix(2, 1), matrix(2, 2)) * sign;

		}
		break;

		default:
		{

			axisVector = MVector::zero;

		}
		break;

	}

	// Check if vector should be normalized
	//
	if (normalize)
	{

		axisVector.normalize();

	}
	
	return axisVector;

};


MVector TwistSolver::getAxisVector(const Axis axis, const bool flip)
/**
Returns the axis vector for given enum field value.

@param axis: The axis vector to return.
@param flip: Flips the requested axis vector.
@return: MVector
*/
{

	return TwistSolver::getAxisVector(MMatrix::identity, axis, flip, false);

};


double TwistSolver::degToRad(const double degrees)
/**
Converts degrees to radians.

@param degrees: The angle in degrees.
@return: The equivalent radian.
*/
{

	return (degrees * (M_PI / 180.0));

};


double TwistSolver::radToDeg(const double radians)
/**
Converts radians to degrees.

@param radians: The angle in radians.
@return: The equivalent degree.
*/
{

	return (radians * (180.0 / M_PI));

};


bool TwistSolver::isClose(const double a, const double b)
/**
Checks if two floating point numbers are equivent within a threshold.

@param a: The first floating point number.
@param b: The second floating point number.
@return: Boolean
*/
{

	return TwistSolver::isClose(a, b, RELATIVE_TOLERANCE, ABSOLUTE_TOLERANCE);

};


bool TwistSolver::isClose(const double a, const double b, const double relativeTolerance, const double absoluteTolerance)
/**
Checks if two floating point numbers are equivent within a threshold.

@param a: The first floating point number.
@param b: The second floating point number.
@param relativeTolerance: The relative tolerance.
@param absoluteTolerance: The absolute tolerance.
@return: Boolean
*/
{
	
	return abs(a - b) <= fmax(relativeTolerance * fmax(abs(a), abs(b)), absoluteTolerance);

};


void* TwistSolver::creator() 
/**
This function is called by Maya when a new instance is requested.
See pluginMain.cpp for details.

@return: TwistSolver
*/
{

	return new TwistSolver();

};


void TwistSolver::postConstructor()
/**
Internally maya creates two objects when a user defined node is created, the internal MObject and the user derived object. 
The association between the these two objects is not made until after the MPxNode constructor is called. 
This implies that no MPxNode member function can be called from the MPxNode constructor. 
The postConstructor will get called immediately after the constructor when it is safe to call any MPxNode member function.

@return: void
*/
{

	// Modify curve ramp attribute
	//
	MObject node = this->thisMObject();

	TwistSolver::initializeCurveRamp(node, TwistSolver::falloff, 0, 0.0f, 0.0f, 1);
	TwistSolver::initializeCurveRamp(node, TwistSolver::falloff, 1, 1.0f, 1.0f, 1);

};


MStatus TwistSolver::initializeCurveRamp(MObject &node, MObject &attribute, const int index, const float position, const float value, const int interpolation)
/**
Adds a point to a curve ramp attribute.

@param node: The node to edit.
@param attribute: The attribute belonging to the node.
@param index: The index to insert a point at.
@param position: The x position for this point.
@param value: The value for this point.
@param interpolation: The interpolation for this point.
@return: Return status.
*/
{

	MStatus status;
	
	// Initialize MPlug from node and attribute
	//
	MPlug plug(node, attribute);

	// Get requested plug index
	//
	MPlug element = plug.elementByLogicalIndex(index, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Edit position plug
	//
	MPlug positionPlug = element.child(0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = positionPlug.setFloat(position);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Edit value plug
	//
	MPlug valuePlug = element.child(1, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = valuePlug.setFloat(value);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Edit interpolation plug
	//
	MPlug interpolationPlug = element.child(2, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = interpolationPlug.setInt(interpolation);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;

}


MStatus TwistSolver::initialize() 
/**
This function is called by Maya after a plugin has been loaded.
Use this function to define any static attributes.

@return: Return status.
*/
{

	MStatus	status;

	// Declare attribute function sets
	//
	MFnNumericAttribute fnNumericAttr;
	MFnMatrixAttribute fnMatrixAttr;
	MFnUnitAttribute fnUnitAttr;
	MFnTypedAttribute fnTypedAttr;
	MFnEnumAttribute fnEnumAttr;
	MRampAttribute fnRampAttr;

	// Input attributes:
	// ".forwardAxis" attribute
	//
	TwistSolver::forwardAxis = fnEnumAttr.create("forwardAxis", "fa", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("X", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("Y", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("Z", 2));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".forwardAxisFlip" attribute
	//
	TwistSolver::forwardAxisFlip = fnNumericAttr.create("forwardAxisFlip", "faf", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(TwistSolver::inputCategory));

	// ".upAxis" attribute
	//
	TwistSolver::upAxis = fnEnumAttr.create("upAxis", "ua", short(1), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("X", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("Y", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("Z", 2));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".upAxisFlip" attribute
	//
	TwistSolver::upAxisFlip = fnNumericAttr.create("upAxisFlip", "uaf", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(TwistSolver::inputCategory));

	// ".startMatrix" attribute
	//
	TwistSolver::startMatrix = fnMatrixAttr.create("startMatrix", "sm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(TwistSolver::inputCategory));

	// ".startOffsetMatrix" attribute
	//
	TwistSolver::startOffsetMatrix = fnMatrixAttr.create("startOffsetMatrix", "som", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(TwistSolver::inputCategory));

	// ".endMatrix" attribute
	//
	TwistSolver::endMatrix = fnMatrixAttr.create("endMatrix", "em", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(TwistSolver::inputCategory));

	// ".endOffsetAngle" attribute
	//
	TwistSolver::endOffsetMatrix = fnMatrixAttr.create("endOffsetMatrix", "eom", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(TwistSolver::inputCategory));

	// ".inCurve" attribute
	//
	TwistSolver::inCurve = fnTypedAttr.create("inCurve", "in", MFnData::kNurbsCurve, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.addToCategory(TwistSolver::inputCategory));

	// ".segments" attribute
	//
	TwistSolver::segments = fnNumericAttr.create("segments", "seg", MFnNumericData::kInt, 1, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(1));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(TwistSolver::inputCategory));

	// ".inverse" attribute
	//
	TwistSolver::inverse = fnEnumAttr.create("inverse", "inv", 0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("Off", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("On", 1));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".reverse" attribute
	//
	TwistSolver::reverse = fnEnumAttr.create("reverse", "rev", 0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("Off", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("On", 1));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".falloff" attribute
	// Any further initialization should be handled in the post constructor!
	//
	TwistSolver::falloff = fnRampAttr.createCurveRamp("falloff", "fo", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".falloffEnabled" attribute
	//
	TwistSolver::falloffEnabled = fnNumericAttr.create("falloffEnabled", "foe", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(TwistSolver::inputCategory));

	// Output attributes:
	// ".twist" attribute
	//
	TwistSolver::twist = fnUnitAttr.create("twist", "t", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.setArray(true));
	CHECK_MSTATUS(fnUnitAttr.setUsesArrayDataBuilder(true));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(TwistSolver::outputCategory));

	// ".roll" attribute
	//
	TwistSolver::roll = fnUnitAttr.create("roll", "r", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(TwistSolver::outputCategory));

	// ".local" attribute
	//
	TwistSolver::debug = fnTypedAttr.create("debug", "d", MFnData::kNurbsCurve, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.setWritable(false));
	CHECK_MSTATUS(fnTypedAttr.setStorable(false));
	CHECK_MSTATUS(fnTypedAttr.addToCategory(TwistSolver::outputCategory));

	// Add attributes
	//
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::forwardAxis));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::forwardAxisFlip));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::upAxis));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::upAxisFlip));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::startMatrix));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::startOffsetMatrix));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::endMatrix));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::endOffsetMatrix));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::inCurve));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::segments));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::inverse));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::reverse));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::falloff));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::falloffEnabled));

	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::debug));

	// Define attribute relationships
	//
	MObject attributes[2] = { TwistSolver::twist, TwistSolver::roll };

	for (MObject attribute : attributes)
	{

		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::forwardAxis, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::upAxis, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::startMatrix, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::startOffsetMatrix, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::endMatrix, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::endOffsetMatrix, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::inCurve, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::segments, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::inverse, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::reverse, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::falloff, attribute));
		CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::falloffEnabled, attribute));

	}

	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::startMatrix, TwistSolver::debug));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::endMatrix, TwistSolver::debug));

	return status;

}
