#include "rigidbody.h"
using namespace Thealmighty;

RigidBody::RigidBody()
{
	qDebug() << "init a rb";
}

void RigidBody::CalcDerivedData()
{
	orientation.normalize();

	// Calculate the transform matrix for the body.
	_calculateTransformMatrix(transformMatrix, position, orientation);

	// Calculate the inertiaTensor in world space.
	_transformInertiaTensor(inverseInertiaTensorWorld,
		orientation,
		inverseInertiaTensor,
		transformMatrix);

}

void RigidBody::Integrate(float duration)
{
	if (!isAwake)
		return;
	lastFrameAcceleration = acceleration;
	lastFrameAcceleration += forceAccum * inverseMass;
	QVector3D angularAcceleration = _Mat3ProductVec3(inverseInertiaTensorWorld, torqueAccum);
	velocity += lastFrameAcceleration * duration;


	rotation += angularAcceleration * duration;
	velocity *= pow(linearDamping, duration);
	rotation *= pow(angularDamping, duration);
	position += velocity * duration;
	_QuaAddScaledVec3(orientation, rotation, duration);
	CalcDerivedData();
	ClearAccumulators();

	if (canSleep)
	{
		float currentMotion = _scalarProductVec3(velocity, velocity) + _scalarProductVec3(rotation, rotation);
		float bias = pow(0.5, duration);
		motion = bias * motion + (1 - bias)*currentMotion;
		if (motion < sleepEpsilon)
			SetAwake(false);
		else if (motion > 10 * sleepEpsilon)
			motion = 10 * sleepEpsilon;

	}
}

void RigidBody::SetMass(const float mass)
{
	assert(mass != 0);
	RigidBody::inverseMass = 1.0 / mass;
}

float RigidBody::GetMass() const
{
	if (inverseMass == 0)
	{
		return MAX_FLOAT;
	}
	else
	{
		return (float)1.0 / inverseMass;
	}
}

void RigidBody::SetInverseMass(const float im)
{
	inverseMass = im;
}

float RigidBody::GetInverseMass() const
{
	return inverseMass;
}

bool RigidBody::HasFiniteMass() const
{
	return inverseMass >= 0.0f;
}

void RigidBody::SetInertiaTensor(const QMatrix3x3 &inertiaTensor)
{
	inverseInertiaTensor = _mat3Inverse(inertiaTensor);
}

QMatrix3x3 RigidBody::GetInertiaTensor() const
{
	return _mat3Inverse(inverseInertiaTensor);
}

void RigidBody::GetInertiaTensor(QMatrix3x3 *inertiaTensor) const
{
	inertiaTensor = &_mat3Inverse(inverseInertiaTensor);
}

QMatrix3x3 RigidBody::GetInertiaTensorWorld() const
{
	return _mat3Inverse(inverseInertiaTensorWorld);
}

void RigidBody::GetInertiaTensorWorld(QMatrix3x3 *inertiaTensor) const
{
	inertiaTensor = &_mat3Inverse(inverseInertiaTensorWorld);
}

void RigidBody::SetInverseInertiaTensor(const QMatrix3x3 &inverseInertiaTensor)
{
	RigidBody::inverseInertiaTensor = inverseInertiaTensor;
}

QMatrix3x3 RigidBody::GetInverseInertiaTensor() const
{
	return inverseInertiaTensor;
}

void RigidBody::GetInverseInertiaTensor(QMatrix3x3 *inverseInertiaTensor) const
{
	*inverseInertiaTensor = RigidBody::inverseInertiaTensor;
}

QMatrix3x3 RigidBody::GetInverseInertiaTensorWorld() const
{
	return inverseInertiaTensorWorld;
}

void RigidBody::GetInverseInertiaTensorWorld(QMatrix3x3 *inverseInertiaTensor) const
{
	*inverseInertiaTensor = inverseInertiaTensorWorld;
}

void RigidBody::SetDamping(const float linearDamping, const float angularDamping)
{
	RigidBody::linearDamping = linearDamping;
	RigidBody::angularDamping = angularDamping;
}

void RigidBody::SetLinearDamping(const float linearDamping)
{
	RigidBody::linearDamping = linearDamping;
}

float RigidBody::GetLinearDamping() const
{
	return linearDamping;
}

void RigidBody::SetAngularDamping(const float angularDamping)
{
	RigidBody::angularDamping = angularDamping;
}

float RigidBody::GetAngularDamping() const
{
	return angularDamping;
}

void RigidBody::SetPosition(const QVector3D &position)
{
	RigidBody::position = position;
}

QVector3D RigidBody::GetPosition() const
{
	return position;
}

void RigidBody::GetPosition(QVector3D *position) const
{
	*position = RigidBody::position;
}

void RigidBody::SetOrientation(const QQuaternion &orientation)
{
	RigidBody::orientation = orientation;
	RigidBody::orientation.normalize();
}

void RigidBody::SetOrientation(const float r, const float i,
	const float j, const float k)
{
	orientation.setScalar(r);
	orientation.setX(i);
	orientation.setY(j);
	orientation.setZ(k);
	orientation.normalize();
}

QQuaternion RigidBody::GetOrientation() const
{
	return orientation;
}

void  RigidBody::GetOrientation(QQuaternion *orientation) const
{
	*orientation = RigidBody::orientation;
}

//»ñÈ¡Ðý×ª¾ØÕó
QMatrix3x3 RigidBody::GetOrientation()
{
	QMatrix3x3 matrix;
	matrix.data()[0] = transformMatrix.data()[0];
	matrix.data()[1] = transformMatrix.data()[1];
	matrix.data()[2] = transformMatrix.data()[2];

	matrix.data()[3] = transformMatrix.data()[4];
	matrix.data()[4] = transformMatrix.data()[5];
	matrix.data()[5] = transformMatrix.data()[6];

	matrix.data()[6] = transformMatrix.data()[8];
	matrix.data()[7] = transformMatrix.data()[9];
	matrix.data()[8] = transformMatrix.data()[10];

	return matrix;

}

QMatrix4x4 RigidBody::GetGLTransform() const
{
	QMatrix4x4 matrix;
	matrix.data()[0] = (float)transformMatrix.data()[0];
	matrix.data()[1] = (float)transformMatrix.data()[4];
	matrix.data()[2] = (float)transformMatrix.data()[8];
	matrix.data()[3] = 0;

	matrix.data()[4] = (float)transformMatrix.data()[1];
	matrix.data()[5] = (float)transformMatrix.data()[5];
	matrix.data()[6] = (float)transformMatrix.data()[9];
	matrix.data()[7] = 0;

	matrix.data()[8] = (float)transformMatrix.data()[2];
	matrix.data()[9] = (float)transformMatrix.data()[6];
	matrix.data()[10] = (float)transformMatrix.data()[10];
	matrix.data()[11] = 0;

	matrix.data()[12] = (float)transformMatrix.data()[3];
	matrix.data()[13] = (float)transformMatrix.data()[7];
	matrix.data()[14] = (float)transformMatrix.data()[11];
	matrix.data()[15] = 1;

	return matrix;
}

QMatrix3x4 RigidBody::GetTransform() const
{
	return transformMatrix;
}

void RigidBody::SetTransform(const QMatrix3x4& mat)
{
	transformMatrix = mat;

}

QVector3D RigidBody::GetPointInLocalSpace(const QVector3D &point) const
{
	return _Mat4ProductVec3Inverse(transformMatrix, point);
}

QVector3D RigidBody::GetPointInWorldSpace(const QVector3D &point) const
{
	return _Mat4ProductVec3(transformMatrix, point);
}

QVector3D RigidBody::GetDirectionInLocalSpace(const QVector3D &direction) const
{
	return _transformInverseDirection(transformMatrix,direction);
}

QVector3D RigidBody::GetDirectionInWorldSpace(const QVector3D &direction) const
{
	return _transformDirection(transformMatrix, direction);
}

QMatrix4x4 RigidBody::GetModelMatrix()
{
	return modelMatrix;
}

void RigidBody::SetVelocity(const QVector3D &velocity)
{
	RigidBody::velocity = velocity;
}

QVector3D RigidBody::GetVelocity() const
{
	return velocity;
}

void RigidBody::AddVelocity(const QVector3D &deltaVelocity)
{
	velocity += deltaVelocity;
}

void RigidBody::SetRotation(const QVector3D &rotation)
{
	RigidBody::rotation = rotation;
}

QVector3D RigidBody::GetRotation() const
{
	return rotation;
}

void RigidBody::AddRotation(const QVector3D &deltaRotation)
{
	rotation += deltaRotation;
}

void RigidBody::SetAwake(const bool awake)
{
	if (awake) {
		isAwake = true;

		// Add a bit of motion to avoid it falling asleep immediately.
		motion = sleepEpsilon * 2.0f;
	}
	else {
		isAwake = false;
		velocity = QVector3D(0, 0, 0);
		rotation = QVector3D(0, 0, 0);
	}
}

void RigidBody::SetCanSleep(const bool canSleep)
{
	RigidBody::canSleep = canSleep;

	if (!canSleep && !isAwake) SetAwake();
}

QVector3D RigidBody::GetLastFrameAcceleration() const
{
	return lastFrameAcceleration;
}

void RigidBody::GetLastFrameAcceleration(QVector3D *acceleration) const
{
	*acceleration = lastFrameAcceleration;
}

void RigidBody::ClearAccumulators()
{
	forceAccum = QVector3D(0, 0, 0);
	torqueAccum = QVector3D(0, 0, 0);
}

void RigidBody::AddForce(const QVector3D &force)
{
	forceAccum += force;
	isAwake = true;
}

void RigidBody::AddForceAtBodyPoint(const QVector3D &force,
	const QVector3D &point)
{
	// Convert to coordinates relative to center of mass.
	QVector3D pt = GetPointInWorldSpace(point);
	AddForceAtPoint(force, pt);
}

void RigidBody::AddForceAtPoint(const QVector3D &force,
	const QVector3D &point)
{
	// Convert to coordinates relative to center of mass.
	QVector3D pt = point;
	pt -= position;

	forceAccum += force;
	torqueAccum += QVector3D::crossProduct(pt, force);

	isAwake = true;
}

void RigidBody::AddTorque(const QVector3D &torque)
{
	torqueAccum += torque;
	isAwake = true;
}

void RigidBody::SetAcceleration(const QVector3D &acceleration)
{
	RigidBody::acceleration = acceleration;
}

QVector3D RigidBody::GetAcceleration() const
{
	return acceleration;
}

