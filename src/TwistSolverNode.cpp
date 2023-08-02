//
// File: TwistSolverNode.cpp
//
// Dependency Graph Node: TwistSolver
//
// Author: Ben Singleton
//

#include "TwistSolverNode.h"

MObject TwistSolver::operation;
MObject TwistSolver::forwardAxis;
MObject TwistSolver::upAxis;
MObject TwistSolver::startMatrix;
MObject TwistSolver::startOffsetAngle;
MObject TwistSolver::endMatrix;
MObject TwistSolver::endOffsetAngle;
MObject	TwistSolver::inputCurve;
MObject	TwistSolver::samples;
MObject	TwistSolver::segments;
MObject	TwistSolver::inverseTwist;
MObject	TwistSolver::reverseTwist;
MObject TwistSolver::falloff;
MObject TwistSolver::falloffEnabled;
MObject TwistSolver::restMatrix;
MObject TwistSolver::buffer;

MObject TwistSolver::twist;
MObject TwistSolver::roll;
MObject	TwistSolver::local;

MTypeId TwistSolver::id(0x0013b1c1);

MString TwistSolver::inputCategory("Input");
MString TwistSolver::simulationCategory("Simulation");
MString TwistSolver::outputCategory("Output");


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

		MDataHandle upAxisHandle = data.inputValue(TwistSolver::upAxis, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle startMatrixHandle = data.inputValue(TwistSolver::startMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle startOffsetAngleHandle = data.inputValue(TwistSolver::startOffsetAngle, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle endMatrixHandle = data.inputValue(TwistSolver::endMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle endOffsetAngleHandle = data.inputValue(TwistSolver::endOffsetAngle, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle operationHandle = data.inputValue(TwistSolver::operation, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle inputCurveHandle = data.inputValue(TwistSolver::inputCurve, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle samplesHandle = data.inputValue(TwistSolver::samples, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle segmentsHandle = data.inputValue(TwistSolver::segments, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle inverseTwistHandle = data.inputValue(TwistSolver::inverseTwist, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle reverseTwistHandle = data.inputValue(TwistSolver::reverseTwist, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle falloffEnabledHandle = data.inputValue(TwistSolver::falloffEnabled, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle restMatrixHandle = data.inputValue(TwistSolver::restMatrix, &status); // Used for caching purposes
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle bufferHandle = data.inputValue(TwistSolver::buffer, &status); // Used for caching purposes
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get data values
		//
		MMatrix startMatrix = startMatrixHandle.asMatrix();
		MAngle startOffsetAngle = startOffsetAngleHandle.asAngle();

		MMatrix endMatrix = endMatrixHandle.asMatrix();
		MAngle endOffsetAngle = endOffsetAngleHandle.asAngle();

		Axis forwardAxis = Axis(forwardAxisHandle.asShort());
		Axis upAxis = Axis(upAxisHandle.asShort());
		Operation operation = Operation(operationHandle.asShort());

		int samples = samplesHandle.asShort();
		int segments = segmentsHandle.asShort();

		bool inverse = inverseTwistHandle.asBool();
		bool reverse = reverseTwistHandle.asBool();

		bool falloffEnabled = falloffEnabledHandle.asBool();

		MMatrix restMatrix = restMatrixHandle.asMatrix();
		double buffer = bufferHandle.asDouble();

		// Offset input matrices
		//
		MMatrix startOffsetMatrix, endOffsetMatrix; 
		
		status = TwistSolver::createRotationMatrix(forwardAxis, startOffsetAngle, startOffsetMatrix);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		startMatrix = startOffsetMatrix * startMatrix;

		status = TwistSolver::createRotationMatrix(forwardAxis, endOffsetAngle, endOffsetMatrix);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		endMatrix = endOffsetMatrix * endMatrix;

		// Extract twist vectors
		//
		MVector startVector = TwistSolver::getAxisVector(forwardAxis, startMatrix, true);
		MVector endVector = TwistSolver::getAxisVector(forwardAxis, endMatrix, true);
		MVector upVector = TwistSolver::getAxisVector(upAxis, startMatrix, true);

		MPoint startPoint = MPoint(startMatrix[3]);
		MPoint endPoint = MPoint(endMatrix[3]);

		// Check if ".inputCurve" has a connection
		//
		MObject node = this->thisMObject();
		MPlug inputCurvePlug(node, TwistSolver::inputCurve);

		bool isConnected = inputCurvePlug.isConnected(&status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MObject curve;

		if (isConnected) 
		{

			// Get MObject from data handle
			//
			curve = inputCurveHandle.asNurbsCurve();

		}
		else 
		{

			// Build curve data
			//
			status = TwistSolver::createCurveData(startPoint, startVector, endPoint, endVector, curve);
			CHECK_MSTATUS_AND_RETURN_IT(status);

		}

		// Transport up vector along curve
		//
		MVector forwardVector;

		status = TwistSolver::transportVector(curve, upVector, forwardVector, samples);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Build transformation matrix from vectors
		//
		MMatrix transportMatrix;
			
		status = TwistSolver::composeMatrix(forwardAxis, forwardVector, upAxis, upVector, endPoint, transportMatrix);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Check which operation to perform
		//
		double roll = 0.0;
		double negateIt = (inverse) ? -1.0 : 1.0;

		switch (operation) 
		{

		case Operation::Shortest:
			{

				// Put end matrix in transport matrix space
				// Remember we're trying to calculate the rotational difference between the transported start matrix!
				//
				MMatrix matrix = endMatrix * transportMatrix.inverse();

				// Initialize transformation matrix
				//
				MTransformationMatrix transform(matrix);

				MEulerRotation eulerRotation = transform.eulerRotation();
				roll = radToDeg(eulerRotation.x) * negateIt;

			}
			break;

		case Operation::NoFlip:
			{
				
				// Check if maya is playing
				//
				bool isPlaying = MAnimControl::isPlaying();

				if (isPlaying) 
				{

					// Multiply twist matrix by previous computed rest matrix
					//
					MMatrix matrix = endMatrix * transportMatrix.inverse();
					MMatrix temp = transportMatrix * restMatrix.inverse();

					// Initialize transformation matrix
					//
					MTransformationMatrix transform(temp);

					MEulerRotation eulerRot = transform.eulerRotation();
					roll = radToDeg(eulerRot.x) * negateIt;

					roll += buffer;

					// Store twist values
					//
					restMatrixHandle.setMMatrix(transportMatrix);
					restMatrixHandle.setClean();

					bufferHandle.setDouble(roll);
					bufferHandle.setClean();

				}
				else 
				{

					// Put end matrix in transport matrix space
					//
					MMatrix matrix = endMatrix * transportMatrix.inverse();

					// Initialize transformation matrix
					//
					MTransformationMatrix transform(transportMatrix);

					MEulerRotation eulerRot = transform.eulerRotation();
					roll = radToDeg(eulerRot.x) * negateIt;

					// Store twist values
					//
					restMatrixHandle.setMMatrix(transportMatrix);
					restMatrixHandle.setClean();

					bufferHandle.setDouble(roll);
					bufferHandle.setClean();

				}

			}
			break;

		}

		// Build output array values
		//
		MArrayDataHandle twistHandle = data.outputArrayValue(TwistSolver::twist, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataBuilder builder(&data, TwistSolver::twist, segments, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDoubleArray twists = this->distributeRoll(roll, segments, reverse);

		MDataHandle element;
		MAngle angle;

		for (int i = 0; i < segments; i++) 
		{

			// Create data handle
			//
			element = builder.addElement(i, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			// Initialize MAngle from double
			//
			angle = MAngle(twists[i], MAngle::kDegrees);

			element.setMAngle(angle);
			element.setClean();

		}

		status = twistHandle.set(builder);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = twistHandle.setAllClean();
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get output data handle and assign value
		//
		MDataHandle rollHandle = data.outputValue(TwistSolver::roll, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		rollHandle.setMAngle(MAngle(roll, MAngle::kDegrees));
		rollHandle.setClean();

		MDataHandle localHandle = data.outputValue(TwistSolver::local, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		localHandle.setMObject(curve);
		localHandle.setClean();

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

	// Evaluate vector length
	//
	double distance = startPoint.distanceTo(endPoint);
	double halfDistance = distance / 2.0;

	bool isClose = TwistSolver::isClose(0.0, distance);

	// Calculate mid points
	//
	MPoint p0 = startPoint;
	MPoint p3 = isClose ? MPoint(startPoint + endVector.normal()) : endPoint;

	MPoint p1 = (p0 + (startVector * halfDistance));
	MPoint p2 = (p3 + (-endVector * halfDistance));

	MPointArray controlVertices(4);
	controlVertices[0] = p0;
	controlVertices[1] = p1;
	controlVertices[2] = p2;
	controlVertices[3] = p3;

	// Define curve knots
	//
	double src[6] = {0, 0, 0, 1, 1, 1};
	MDoubleArray knots(src, 6);

	// Create curve data to sample from
	//
	MFnNurbsCurveData fnCurveData;
	curveData = fnCurveData.create();

	MFnNurbsCurve fnCurve;

	fnCurve.create(controlVertices, knots, 3, MFnNurbsCurve::kOpen, false, true, curveData, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;

};


MStatus TwistSolver::transportVector(const MObject& curve, MVector &transport, MVector &tangent, const int samples)
/**
Transports a vector along a curve based on the number of parameter samples.

@param curve: The curve object.
@param transport: The passed vector to transport.
@param tangent: The up vector to assist the transportation.
@param samples: The number of points to sample along the curve.
@return: Return status.
*/
{

	MStatus status;

	// Initialize function set
	//
	MFnNurbsCurve fnCurve(curve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Perform parallel frame transportation on up vector
	//
	double length = fnCurve.length();
	double step = (1.0 / static_cast<double>(samples));

	MVector cross1, cross2;
	double distance, parameter;

	for (int i = 0; i < samples; i++)
	{

		// Get u-parameter
		// Remember to compensate for the end caps or else the tangent will zero out!
		//
		distance = ((step * static_cast<double>(i)) * length);

		if (distance == 0.0)
		{

			distance += 1e-3;

		}
		else if (distance == length)
		{

			distance -= 1e-3;

		}
		else;

		parameter = fnCurve.findParamFromLength(distance);

		// Get tangent and flip if specified
		//
		tangent = fnCurve.tangent(parameter, MSpace::kTransform).normal();

		// Perform cross product
		//
		cross1 = transport ^ tangent;
		cross2 = tangent ^ cross1;

		transport = cross2.normal();

	}

	return MS::kSuccess;

};


MStatus TwistSolver::composeMatrix(const Axis forwardAxis, const MVector& forwardVector, const Axis upAxis, const MVector& upVector, const MPoint& pos, MMatrix& matrix)
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
	MVector x, y, z;

	switch (forwardAxis) 
	{

	case Axis::PosX:
	{

		// Assign forward vector
		//
		x = forwardVector;

		// Calculate remaining axises
		//
		if (upAxis == Axis::PosX || upAxis == Axis::NegX) 
		{

			return MS::kFailure;

		}
		else if (upAxis == Axis::PosY || upAxis == Axis::NegY) 
		{

			z = (x ^ upVector).normal();
			y = (z ^ x).normal();

		}
		else if (upAxis == Axis::PosZ || upAxis == Axis::NegZ) 
		{

			y = (upVector ^ x).normal();
			z = (x ^ y).normal();

		}

	}
	break;

	case Axis::NegX:
	{

		// Assign forward vector
		//
		x = forwardVector;

		// Calculate remaining axises
		//
		if (upAxis == Axis::PosX || upAxis == Axis::NegX)
		{

			return MS::kFailure;

		}
		else if (upAxis == Axis::PosY || upAxis == Axis::NegY)
		{

			z = (x ^ upVector).normal();
			y = (z ^ x).normal();

		}
		else if (upAxis == Axis::PosZ || upAxis == Axis::NegZ)
		{

			y = (upVector ^ x).normal();
			z = (x ^ y).normal();

		}

	}
	break;

	case Axis::PosY:
	{

		// Assign forward vector
		//
		y = forwardVector;

		// Calculate remaining axises
		//
		if (upAxis == Axis::PosX || upAxis == Axis::NegX)
		{

			z = (upVector ^ y).normal();
			x = (y ^ z).normal();

		}
		else if (upAxis == Axis::PosY || upAxis == Axis::NegY)
		{

			return MS::kFailure;

		}
		else if (upAxis == Axis::PosZ || upAxis == Axis::NegZ)
		{

			x = (y ^ upVector).normal();
			z = (x ^ y).normal();

		}

	}
	break;

	case Axis::NegY:
	{

		// Assign forward vector
		//
		y = forwardVector;

		// Calculate remaining axises
		//
		if (upAxis == Axis::PosX || upAxis == Axis::NegX)
		{

			z = (upVector ^ y).normal();
			x = (y ^ z).normal();

		}
		else if (upAxis == Axis::PosY || upAxis == Axis::NegY)
		{

			return MS::kFailure;

		}
		else if (upAxis == Axis::PosZ || upAxis == Axis::NegZ)
		{

			x = (y ^ upVector).normal();
			z = (x ^ y).normal();

		}

	}
	break;

	case Axis::PosZ:
	{

		// Assign forward vector
		//
		z = forwardVector;

		// Calculate remaining axises
		//
		if (upAxis == Axis::PosX || upAxis == Axis::NegX) 
		{

			y = (z ^ upVector).normal();
			x = (y ^ z).normal();

		}
		else if (upAxis == Axis::PosY || upAxis == Axis::NegY)
		{

			x = (upVector ^ z).normal();
			y = (z ^ x).normal();

		}
		else if (upAxis == Axis::PosZ || upAxis == Axis::NegZ)
		{

			return MS::kFailure;

		}

	}
	break;

	case Axis::NegZ:
	{

		// Assign forward vector
		//
		z = forwardVector;

		// Calculate remaining axises
		//
		if (upAxis == Axis::PosX || upAxis == Axis::NegX)
		{

			y = (z ^ upVector).normal();
			x = (y ^ z).normal();

		}
		else if (upAxis == Axis::PosY || upAxis == Axis::NegY)
		{

			x = (upVector ^ z).normal();
			y = (z ^ x).normal();

		}
		else if (upAxis == Axis::PosZ || upAxis == Axis::NegZ)
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

	// Return matrix from axis vectors
	//
	matrix = TwistSolver::composeMatrix(x, y, z, pos);

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


MStatus TwistSolver::createRotationMatrix(const Axis forwardAxis, const MAngle angle, MMatrix &matrix)
/**
Creates a rotation matrix from the given forward axis and angle.

@param forwardAxis: The forward axis to rotate from.
@param angle: The angle of rotation.
@param matrix: The passed matrix to populate.
@return: Return status.
*/
{

	double radian = angle.asRadians();

	switch (forwardAxis)
	{

	case Axis::PosX: case Axis::NegX:

		matrix = MEulerRotation(radian, 0.0, 0.0).asMatrix();
		break;

	case Axis::PosY: case Axis::NegY:

		matrix = MEulerRotation(0.0, radian, 0.0).asMatrix();
		break;

	case Axis::PosZ: case Axis::NegZ:

		matrix = MEulerRotation(0.0, 0.0, radian).asMatrix();
		break;

	default:

		return MS::kFailure;

	}

	return MS::kSuccess;

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


MVector	TwistSolver::getAxisVector(const Axis axis, const MMatrix& matrix, const bool normalize)
/**
Returns the axis vector from the supplied matrix

@param axis: The field value to query.
@param matrix: The matrix to query.
@param normalize: Normalizes the returned axis vector.
@return: MVector
*/
{

	// Get vector
	//
	MVector axisVector;

	switch (axis)
	{

	case Axis::PosX:

		axisVector = MVector(matrix[0]);
		break;

	case Axis::NegX:

		axisVector = -MVector(matrix[0]);
		break;

	case Axis::PosY:

		axisVector = MVector(matrix[1]);
		break;

	case Axis::NegY:

		axisVector = -MVector(matrix[1]);
		break;

	case Axis::PosZ:

		axisVector = MVector(matrix[2]);
		break;

	case Axis::NegZ:

		axisVector = -MVector(matrix[2]);
		break;

	default:

		axisVector = MVector::zero;
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


MVector TwistSolver::getAxisVector(const Axis axis)
/**
Returns the axis vector for given enum field value.

@param axis: The field value to query.
@return: MVector
*/
{

	return TwistSolver::getAxisVector(axis, MMatrix::identity, false);

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

	initializeCurveRamp(node, TwistSolver::falloff, 0, 0.0f, 0.0f, 1);
	initializeCurveRamp(node, TwistSolver::falloff, 1, 1.0f, 1.0f, 1);

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
	// ".operation" attribute
	//
	TwistSolver::operation = fnEnumAttr.create("operation", "operation", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("Shortest", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("No Flip", 1));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".forwardAxis" attribute
	//
	TwistSolver::forwardAxis = fnEnumAttr.create("forwardAxis", "forwardAxis", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("PosX", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("NegX", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("PosY", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("NegY", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("PosZ", 4));
	CHECK_MSTATUS(fnEnumAttr.addField("NegZ", 5));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".upAxis" attribute
	//
	TwistSolver::upAxis = fnEnumAttr.create("upAxis", "upAxis", short(2), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("PosX", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("NegX", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("PosY", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("NegY", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("PosZ", 4));
	CHECK_MSTATUS(fnEnumAttr.addField("NegZ", 5));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".startMatrix" attribute
	//
	TwistSolver::startMatrix = fnMatrixAttr.create("startMatrix", "startMatrix", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(TwistSolver::inputCategory));

	// ".startOffsetAngle" attribute
	//
	TwistSolver::startOffsetAngle = fnUnitAttr.create("startOffsetAngle", "startOffsetAngle", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(TwistSolver::inputCategory));

	// ".endMatrix" attribute
	//
	TwistSolver::endMatrix = fnMatrixAttr.create("endMatrix", "endMatrix", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(TwistSolver::inputCategory));

	// ".endOffsetAngle" attribute
	//
	TwistSolver::endOffsetAngle = fnUnitAttr.create("endOffsetAngle", "endOffsetAngle", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(TwistSolver::inputCategory));

	// ".inputCurve" attribute
	//
	TwistSolver::inputCurve = fnTypedAttr.create("inputCurve", "inputCurve", MFnData::kNurbsCurve, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.addToCategory(TwistSolver::inputCategory));

	// ".segments" attribute
	//
	TwistSolver::segments = fnNumericAttr.create("segments", "segments", MFnNumericData::kInt, 1, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(1));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(TwistSolver::inputCategory));

	// ".samples" attribute
	//
	TwistSolver::samples = fnNumericAttr.create("samples", "samples", MFnNumericData::kInt, 3, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(3));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(TwistSolver::inputCategory));

	// ".inverseTwist" attribute
	//
	TwistSolver::inverseTwist = fnEnumAttr.create("inverseTwist", "inverseTwist", 0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("Off", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("On", 1));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".reverseTwist" attribute
	//
	TwistSolver::reverseTwist = fnEnumAttr.create("reverseTwist", "reverseTwist", 0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("Off", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("On", 1));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(TwistSolver::inputCategory));

	// ".falloff" attribute
	// Any further initialization should be handled in the post constructor!
	//
	TwistSolver::falloff = fnRampAttr.createCurveRamp("falloff", "falloff", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".falloffEnabled" attribute
	//
	TwistSolver::falloffEnabled = fnNumericAttr.create("falloffEnabled", "falloffEnabled", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(TwistSolver::inputCategory));

	// ".restMatrix" tracking attribute
	//
	TwistSolver::restMatrix = fnMatrixAttr.create("restMatrix", "restMatrix", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setHidden(true));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(TwistSolver::simulationCategory));

	// ".buffer" tracking attribute
	//
	TwistSolver::buffer = fnNumericAttr.create("buffer", "buffer", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setHidden(true));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(TwistSolver::simulationCategory));

	// Output attributes:
	// ".twist" attribute
	//
	TwistSolver::twist = fnUnitAttr.create("twist", "twist", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.setArray(true));
	CHECK_MSTATUS(fnUnitAttr.setUsesArrayDataBuilder(true));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(TwistSolver::outputCategory));

	// ".roll" attribute
	//
	TwistSolver::roll = fnUnitAttr.create("roll", "roll", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(TwistSolver::outputCategory));

	// ".local" attribute
	//
	TwistSolver::local = fnTypedAttr.create("local", "local", MFnData::kNurbsCurve, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.setWritable(false));
	CHECK_MSTATUS(fnTypedAttr.setStorable(false));
	CHECK_MSTATUS(fnTypedAttr.addToCategory(TwistSolver::outputCategory));

	// Add attributes
	//
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::operation));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::forwardAxis));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::upAxis));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::startMatrix));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::startOffsetAngle));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::endMatrix));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::endOffsetAngle));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::inputCurve));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::segments));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::samples));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::inverseTwist));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::reverseTwist));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::falloff));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::falloffEnabled));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::restMatrix)); // DO NOT INCLUDE IN ATTRIBUTE AFFECTS
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::buffer)); // DO NOT INCLUDE IN ATTRIBUTE AFFECTS

	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::addAttribute(TwistSolver::local));

	// Define attribute relationships
	//
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::operation, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::forwardAxis, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::upAxis, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::startMatrix, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::startOffsetAngle, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::endMatrix, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::endOffsetAngle, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::inputCurve, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::segments, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::samples, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::inverseTwist, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::reverseTwist, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::falloff, TwistSolver::twist));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::falloffEnabled, TwistSolver::twist));

	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::operation, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::forwardAxis, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::upAxis, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::startMatrix, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::startOffsetAngle, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::endMatrix, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::endOffsetAngle, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::inputCurve, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::segments, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::samples, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::inverseTwist, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::reverseTwist, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::falloff, TwistSolver::roll));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::falloffEnabled, TwistSolver::roll));

	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::startMatrix, TwistSolver::local));
	CHECK_MSTATUS(TwistSolver::attributeAffects(TwistSolver::endMatrix, TwistSolver::local));

	return status;

}
