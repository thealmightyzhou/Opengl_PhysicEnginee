#include "contacts.h"
using namespace Thealmighty;

//设置碰撞的双方数据
void Contact::SetBodyData(RigidBody* one, RigidBody *two, float friction, float restitution)
{
	Contact::rb[0] = one;
	Contact::rb[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;

}

//将双方的awake都设置为激活状态
void Contact::MatchAwakeState()
{
	if (!rb[1])
		return;
	bool body0awake = rb[0]->GetAwake();
	bool body1awake = rb[1]->GetAwake();

	if (body0awake ^ body1awake)
	{
		if (body0awake)
			rb[1]->SetAwake();
		else
			rb[0]->SetAwake();
	}
}

//交换两个刚体，并翻转碰撞法线
void Contact::SwapBodies()
{
	contactNormal *= -1;
	RigidBody *temp = rb[0];
	rb[0] = rb[1];
	rb[1] = temp;
}

//根据碰撞法线构造基向量存入contactToWorld矩阵，列向量分别为z（碰撞法线），新x，新y向量
inline void	Contact::CalcContactBasis()
{
	QVector3D contactTangent[2];

	if (abs(contactNormal.x()) > abs(contactNormal.y()))
	{
		const float s = 1.0f / sqrt(contactNormal.z()*contactNormal.z() + contactNormal.x()*contactNormal.x());
		//计算切线
		contactTangent[0].setX(contactNormal.z()*s);
		contactTangent[0].setY(0);
		contactTangent[0].setZ(-contactNormal.x()*s);

		contactTangent[1].setX(contactNormal.y()*contactTangent[0].x());
		contactTangent[1].setY(contactNormal.z()*contactTangent[0].x() -
			contactNormal.x()*contactTangent[0].z());
		contactTangent[1].setZ(-contactNormal.y()*contactTangent[0].x());
	}
	else
	{
		const float s = 1.0f / sqrt(contactNormal.z()*contactNormal.z() +
			contactNormal.y()*contactNormal.y());
		contactTangent[0].setX(0);
		contactTangent[0].setY(-contactNormal.z()*s);
		contactTangent[0].setZ(contactNormal.y()*s);

		contactTangent[1].setX(contactNormal.y()*contactTangent[0].z() - contactNormal.z()*contactTangent[0].y());
		contactTangent[1].setY(-contactNormal.x()*contactTangent[0].z());
		contactTangent[1].setZ(contactNormal.x()*contactTangent[0].y());
	}
	_Mat3x3AssignByVec3(contactToWorld, contactNormal, contactTangent[0], contactTangent[1]);
}

//通过获取刚体的加速度和速度计算碰撞点的本地速度
QVector3D Contact::CalcLocalVelocity(unsigned bodyIndex, float duration)
{
	RigidBody *thisBody = rb[bodyIndex];
	// Work out the velocity of the contact point.

	//获取计算碰撞点的速度
	QVector3D velocity = _Vec3CrossProductVec3(thisBody->GetRotation(), relativeContactPosition[bodyIndex]);
	velocity += thisBody->GetVelocity();	

	// Turn the velocity into contact-coordinates.
	//速度转换到碰撞基向量
	QVector3D contactVelocity = _Mat3TransposeTransformVec3(contactToWorld, velocity);

	// Calculate the ammount of velocity that is due to forces without
	// reactions.
	//获取加速度加上的速度
	QVector3D accVelocity = thisBody->GetLastFrameAcceleration() * duration;

	// Calculate the velocity in contact-coordinates.
	//同样转换到碰撞点基向量空间
	accVelocity = _Mat3TransposeTransformVec3(contactToWorld, accVelocity);

	// We ignore any component of acceleration in the contact normal
	// direction, we are only interested in planar acceleration
	//忽略x轴加速度
	accVelocity.setX(0);	

	// Add the planar velocities - if there's enough friction they will
	// be removed during velocity resolution
	contactVelocity += accVelocity;

	// And return it
	return contactVelocity;
}

//计算变化的速度
void Contact::CalcDesiredDeltaVelocity(float duration)
{
	const static float velocityLimit = 0.25f;

	// Calculate the acceleration induced velocity accumulated this frame
	float velocityFromAcc = 0;

	if (rb[0]->GetAwake())
	{
		//刚体的加速度在碰撞方向上的分量
		velocityFromAcc += _scalarProductVec3(rb[0]->GetLastFrameAcceleration(), contactNormal)*duration;
	}

	if (rb[1] && rb[1]->GetAwake())
	{
		velocityFromAcc -= _scalarProductVec3(rb[1]->GetLastFrameAcceleration(), contactNormal)*duration;
	}

	// If the velocity is very slow, limit the restitution
	float thisRestitution = restitution;
	if (abs(contactVelocity.x()) < velocityLimit)
	{
		thisRestitution = (float)0.0f;
	}

	// Combine the bounce velocity with the removed
	// acceleration velocity.
	desiredDeltaVelocity = -contactVelocity.x() - thisRestitution * (contactVelocity.x() - velocityFromAcc);
}

//计算内部数据
void Contact::CalcInternals(float duration)
{
	// Check if the first object is NULL, and swap if it is.
	if (!rb[0])
		SwapBodies();
	assert(rb[0]);

	// Calculate an set of axis at the contact point.
	CalcContactBasis();

	// Store the relative position of the contact relative to each body
	//存储碰撞点的相对位置
	relativeContactPosition[0] = contactPoint - rb[0]->GetPosition();
	if (rb[1]) {
		relativeContactPosition[1] = contactPoint - rb[1]->GetPosition();
	}

	// Find the relative velocity of the bodies at the contact point.
	contactVelocity = CalcLocalVelocity(0, duration);
	if (rb[1]) {
		contactVelocity -= CalcLocalVelocity(1, duration);
	}

	// Calculate the desired change in velocity for resolution
	CalcDesiredDeltaVelocity(duration);
}

void Contact::ApplyVelocityChange(QVector3D velocityChange[2],
	QVector3D rotationChange[2])
{
	// Get hold of the inverse mass and inverse inertia tensor, both in
	// world coordinates.
	QMatrix3x3 inverseInertiaTensor[2];
	rb[0]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
	if (rb[1])
		rb[1]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

	// We will calculate the impulse for each contact axis
	QVector3D impulseContact;

	if (friction == 0.0f)
	{
		// Use the short format for frictionless contacts
		//计算无摩擦的冲量
		impulseContact = CalcFrictionlessImpulse(inverseInertiaTensor);
	}
	else
	{
		// Otherwise we may have impulses that aren't in the direction of the
		// contact, so we need the more complex version.
		impulseContact = CalcFrictionImpulse(inverseInertiaTensor);
	}

	// Convert impulse to world coordinates
	QVector3D impulse = _Mat3ProductVec3(contactToWorld, impulseContact);

	// Split in the impulse into linear and rotational components
	QVector3D impulsiveTorque = _Vec3CrossProductVec3(relativeContactPosition[0], impulse);
	rotationChange[0] = _Mat3ProductVec3(inverseInertiaTensor[0], impulsiveTorque);
	_Clear(velocityChange[0]);
	velocityChange[0] += impulse * rb[0]->GetInverseMass();

	// Apply the changes
	rb[0]->AddVelocity(velocityChange[0]);
	rb[0]->AddRotation(rotationChange[0]);

	if (rb[1])
	{
		// Work out body one's linear and angular changes
		QVector3D impulsiveTorque = _Vec3CrossProductVec3(impulse, relativeContactPosition[1]);

		rotationChange[1] = _Mat3ProductVec3(inverseInertiaTensor[1], impulsiveTorque);
		_Clear(velocityChange[1]);
		velocityChange[1] += impulse * -rb[1]->GetInverseMass();

		// And apply them.
		rb[1]->AddVelocity(velocityChange[1]);
		rb[1]->AddRotation(rotationChange[1]);
	}
}

//计算无摩擦碰撞冲量
inline
QVector3D Contact::CalcFrictionlessImpulse(QMatrix3x3 * inverseInertiaTensor)
{
	QVector3D impulseContact;

	// Build a vector that shows the change in velocity in
	// world space for a unit impulse in the direction of the contact
	// normal.
	QVector3D deltaVelWorld = _Vec3CrossProductVec3(relativeContactPosition[0], contactNormal);
	deltaVelWorld = _Mat3ProductVec3(inverseInertiaTensor[0], deltaVelWorld);
	deltaVelWorld = _Vec3CrossProductVec3(deltaVelWorld, relativeContactPosition[0]);

	// Work out the change in velocity in contact coordiantes.
	float deltaVelocity = _scalarProductVec3(deltaVelWorld, contactNormal);

	// Add the linear component of velocity change
	deltaVelocity += rb[0]->GetInverseMass();

	// Check if we need to the second body's data
	if (rb[1])
	{
		// Go through the same transformation sequence again
		QVector3D deltaVelWorld = _Vec3CrossProductVec3(relativeContactPosition[1], contactNormal);
		deltaVelWorld = _Mat3ProductVec3(inverseInertiaTensor[1], deltaVelWorld);
		deltaVelWorld = _Vec3CrossProductVec3(deltaVelWorld, relativeContactPosition[1]);

		// Add the change in velocity due to rotation
		deltaVelocity += _scalarProductVec3(deltaVelWorld, contactNormal);

		// Add the change in velocity due to linear motion
		deltaVelocity += rb[1]->GetInverseMass();
	}

	// Calculate the required size of the impulse
	impulseContact.setX(desiredDeltaVelocity / deltaVelocity);
	impulseContact.setY(0);
	impulseContact.setZ(0);
	return impulseContact;
}

//计算有摩擦碰撞的冲量
inline
QVector3D Contact::CalcFrictionImpulse(QMatrix3x3 * inverseInertiaTensor)
{
	QVector3D impulseContact;
	float inverseMass = rb[0]->GetInverseMass();

	// The equivalent of a cross product in matrices is multiplication
	// by a skew symmetric matrix - we build the matrix for converting
	// between linear and angular quantities.
	QMatrix3x3 impulseToTorque;
	_Mat3SkewSymmetric(impulseToTorque, relativeContactPosition[0]);

	// Build the matrix to convert contact impulse to change in velocity
	// in world coordinates.
	QMatrix3x3 deltaVelWorld = impulseToTorque;
	deltaVelWorld = _Mat3ProductMat3(deltaVelWorld, inverseInertiaTensor[0]);
	deltaVelWorld = _Mat3ProductMat3(deltaVelWorld, impulseToTorque);
	deltaVelWorld *= -1;

	// Check if we need to add body two's data
	if (rb[1])
	{
		// Set the cross product matrix
		_Mat3SkewSymmetric(impulseToTorque, relativeContactPosition[1]);

		// Calculate the velocity change matrix
		QMatrix3x3 deltaVelWorld2 = impulseToTorque;
		deltaVelWorld2 = _Mat3ProductMat3(deltaVelWorld2, inverseInertiaTensor[1]);
		deltaVelWorld2 = _Mat3ProductMat3(deltaVelWorld2, impulseToTorque);
		deltaVelWorld2 *= -1;

		// Add to the total delta velocity.
		deltaVelWorld += deltaVelWorld2;

		// Add to the inverse mass
		inverseMass += rb[1]->GetInverseMass();
	}

	// Do a change of basis to convert into contact coordinates.
	QMatrix3x3 deltaVelocity = _Mat3Transpose(contactToWorld);
	deltaVelocity = _Mat3ProductMat3(deltaVelocity, deltaVelWorld);
	deltaVelocity = _Mat3ProductMat3(deltaVelocity, contactToWorld);

	// Add in the linear velocity change
	deltaVelocity.data()[0] += inverseMass;
	deltaVelocity.data()[4] += inverseMass;
	deltaVelocity.data()[8] += inverseMass;

	// Invert to get the impulse needed per unit velocity
	QMatrix3x3 impulseMatrix = _mat3Inverse(deltaVelocity);

	// Find the target velocities to kill
	QVector3D velKill(desiredDeltaVelocity, -contactVelocity.y(), -contactVelocity.z());

	// Find the impulse to kill target velocities
	impulseContact = _Mat3ProductVec3(impulseMatrix, velKill);

	// Check for exceeding friction
	float planarImpulse = sqrt(
		impulseContact.y()*impulseContact.y() +
		impulseContact.z()*impulseContact.z()
	);
	if (planarImpulse > impulseContact.x() * friction)
	{
		// We need to use dynamic friction
		impulseContact.setY(impulseContact.y() / planarImpulse);
		impulseContact.setZ(impulseContact.z() / planarImpulse);
		impulseContact.setX(deltaVelocity.data()[0] +
			deltaVelocity.data()[1] * friction*impulseContact.y() +
			deltaVelocity.data()[2] * friction*impulseContact.z());
		impulseContact.setX(desiredDeltaVelocity / impulseContact.x());
		impulseContact.setY(impulseContact.y()*friction * impulseContact.x());
		impulseContact.setZ(impulseContact.z()*friction * impulseContact.x());
	}
	return impulseContact;
}

//位置变化
void Contact::ApplyPositionChange(QVector3D linearChange[2],
	QVector3D angularChange[2],
	float penetration)
{
	const float angularLimit = 0.2f;
	float angularMove[2];
	float linearMove[2];

	float totalInertia = 0;
	float linearInertia[2];
	float angularInertia[2];

	// We need to work out the inertia of each object in the direction
	// of the contact normal, due to angular inertia only.
	for (unsigned i = 0; i < 2; i++)
		if (rb[i])
		{
			QMatrix3x3 inverseInertiaTensor;
			rb[i]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);

			// Use the same procedure as for calculating frictionless
			// velocity change to work out the angular inertia.
			QVector3D angularInertiaWorld = _Vec3CrossProductVec3(relativeContactPosition[i], contactNormal);
			angularInertiaWorld = _Mat3ProductVec3(inverseInertiaTensor, angularInertiaWorld);
			angularInertiaWorld = _Vec3CrossProductVec3(angularInertiaWorld, relativeContactPosition[i]);
			angularInertia[i] = _scalarProductVec3(angularInertiaWorld, contactNormal);

			// The linear component is simply the inverse mass
			linearInertia[i] = rb[i]->GetInverseMass();

			// Keep track of the total inertia from all components
			totalInertia += linearInertia[i] + angularInertia[i];

			// We break the loop here so that the totalInertia value is
			// completely calculated (by both iterations) before
			// continuing.
		}

	// Loop through again calculating and applying the changes
	for (unsigned i = 0; i < 2; i++)
		if (rb[i])
		{
			// The linear and angular movements required are in proportion to
			// the two inverse inertias.
			float sign = (i == 0) ? 1 : -1;
			angularMove[i] =
				sign * penetration * (angularInertia[i] / totalInertia);
			linearMove[i] =
				sign * penetration * (linearInertia[i] / totalInertia);

			// To avoid angular projections that are too great (when mass is large
			// but inertia tensor is small) limit the angular move.
			QVector3D projection = relativeContactPosition[i];

			projection += contactNormal * _scalarProductVec3(-relativeContactPosition[i], contactNormal);

			// Use the small angle approximation for the sine of the angle (i.e.
			// the magnitude would be sine(angularLimit) * projection.magnitude
			// but we approximate sine(angularLimit) to angularLimit).
			float maxMagnitude = angularLimit * projection.length();

			if (angularMove[i] < -maxMagnitude)
			{
				float totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = -maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}
			else if (angularMove[i] > maxMagnitude)
			{
				float totalMove = angularMove[i] + linearMove[i];
				angularMove[i] = maxMagnitude;
				linearMove[i] = totalMove - angularMove[i];
			}

			// We have the linear amount of movement required by turning
			// the rigid body (in angularMove[i]). We now need to
			// calculate the desired rotation to achieve that.
			if (angularMove[i] == 0)
			{
				// Easy case - no angular movement means no rotation.
				_Clear(angularChange[i]);
			}
			else
			{
				// Work out the direction we'd like to rotate in.
				QVector3D targetAngularDirection = _Vec3CrossProductVec3(relativeContactPosition[i], contactNormal);

				QMatrix3x3 inverseInertiaTensor;
				rb[i]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);

				// Work out the direction we'd need to rotate to achieve that
				angularChange[i] = _Mat3ProductVec3(inverseInertiaTensor, targetAngularDirection)*(angularMove[i] / angularInertia[i]);
			}

			// Velocity change is easier - it is just the linear movement
			// along the contact normal.
			linearChange[i] = contactNormal * linearMove[i];

			// Now we can start to apply the values we've calculated.
			// Apply the linear movement
			QVector3D pos;
			rb[i]->GetPosition(&pos);
			pos += contactNormal * linearMove[i];
			rb[i]->SetPosition(pos);

			// And the change in orientation
			QQuaternion q;
			rb[i]->GetOrientation(&q);
			_QuaAddScaledVec3(q, angularChange[i], 1.0f);
			rb[i]->SetOrientation(q);

			// We need to calculate the derived data for any body that is
			// asleep, so that the changes are reflected in the object's
			// data. Otherwise the resolution will not change the position
			// of the object, and the next collision detection round will
			// have the same penetration.
			if (!rb[i]->GetAwake()) 
				rb[i]->CalcDerivedData();
		}
}

ContactResolver::ContactResolver(unsigned iterations,
	float velocityEpsilon,
	float positionEpsilon)
{
	SetIterations(iterations, iterations);
	SetEpsilon(velocityEpsilon, positionEpsilon);
}

ContactResolver::ContactResolver(unsigned velocityIterations,
	unsigned positionIterations,
	float velocityEpsilon,
	float positionEpsilon)
{
	SetIterations(velocityIterations);
	SetEpsilon(velocityEpsilon, positionEpsilon);
}

void ContactResolver::SetIterations(unsigned iterations)
{
	SetIterations(iterations, iterations);
}

void ContactResolver::SetIterations(unsigned velocityIterations,
	unsigned positionIterations)
{
	ContactResolver::velocityIterations = velocityIterations;
	ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::SetEpsilon(float velocityEpsilon,
	float positionEpsilon)
{
	ContactResolver::velocityEpsilon = velocityEpsilon;
	ContactResolver::positionEpsilon = positionEpsilon;
}

void ContactResolver::ResolveContacts(Contact *contacts,
	unsigned numContacts,
	float duration)
{
	// Make sure we have something to do.
	if (numContacts == 0) return;
	if (!IsValid()) return;

	// Prepare the contacts for processing
	PrepareContacts(contacts, numContacts, duration);

	// Resolve the interpenetration problems with the contacts.
	AdjustPositions(contacts, numContacts, duration);

	// Resolve the velocity problems with the contacts.
	AdjustVelocities(contacts, numContacts, duration);
}

void ContactResolver::PrepareContacts(Contact* contacts,
	unsigned numContacts,
	float duration)
{
	// Generate contact velocity and axis information.
	Contact* lastContact = contacts + numContacts;
	for (Contact* contact = contacts; contact < lastContact; contact++)
	{
		// Calculate the internal contact data (inertia, basis, etc).
		contact->CalcInternals(duration);
	}
}

void ContactResolver::AdjustVelocities(Contact *c,
	unsigned numContacts,
	float duration)
{
	QVector3D velocityChange[2], rotationChange[2];
	QVector3D deltaVel;

	// iteratively handle impacts in order of severity.
	velocityIterationsUsed = 0;
	while (velocityIterationsUsed < velocityIterations)
	{
		// Find contact with maximum magnitude of probable velocity change.
		float max = velocityEpsilon;
		unsigned index = numContacts;
		for (unsigned i = 0; i < numContacts; i++)
		{
			if (c[i].desiredDeltaVelocity > max)
			{
				max = c[i].desiredDeltaVelocity;
				index = i;
			}
		}
		if (index == numContacts) break;

		// Match the awake state at the contact
		c[index].MatchAwakeState();

		// Do the resolution on the contact that came out top.
		c[index].ApplyVelocityChange(velocityChange, rotationChange);

		// With the change in velocity of the two bodies, the update of
		// contact velocities means that some of the relative closing
		// velocities need recomputing.
		for (unsigned i = 0; i < numContacts; i++)
		{
			// Check each body in the contact
			for (unsigned b = 0; b < 2; b++) if (c[i].rb[b])
			{
				// Check for a match with each body in the newly
				// resolved contact
				for (unsigned d = 0; d < 2; d++)
				{
					if (c[i].rb[b] == c[index].rb[d])
					{
						deltaVel = velocityChange[d] +
							_Vec3CrossProductVec3(rotationChange[d], c[i].relativeContactPosition[b]);

						// The sign of the change is negative if we're dealing
						// with the second body in a contact.
						c[i].contactVelocity +=
							_Mat3TransposeTransformVec3(c[i].contactToWorld, deltaVel) * (b ? -1 : 1);
						c[i].CalcDesiredDeltaVelocity(duration);
					}
				}
			}
		}
		velocityIterationsUsed++;
	}
}

void ContactResolver::AdjustPositions(Contact *c,
	unsigned numContacts,
	float duration)
{
	unsigned i, index;
	QVector3D linearChange[2], angularChange[2];
	float max;
	QVector3D deltaPosition;

	// iteratively resolve interpenetrations in order of severity.
	positionIterationsUsed = 0;
	while (positionIterationsUsed < positionIterations)
	{
		// Find biggest penetration
		max = positionEpsilon;
		index = numContacts;
		for (i = 0; i < numContacts; i++)
		{
			if (c[i].penetration > max)
			{
				max = c[i].penetration;
				index = i;
			}
		}
		if (index == numContacts) 
			break;

		// Match the awake state at the contact
		c[index].MatchAwakeState();

		// Resolve the penetration.
		c[index].ApplyPositionChange(
			linearChange,
			angularChange,
			max);

		// Again this action may have changed the penetration of other
		// bodies, so we update contacts.
		for (i = 0; i < numContacts; i++)
		{
			// Check each body in the contact
			for (unsigned b = 0; b < 2; b++) if (c[i].rb[b])
			{
				// Check for a match with each body in the newly
				// resolved contact
				for (unsigned d = 0; d < 2; d++)
				{
					if (c[i].rb[b] == c[index].rb[d])
					{
						deltaPosition = linearChange[d] +
							_Vec3CrossProductVec3(angularChange[d], c[i].relativeContactPosition[b]);

						// The sign of the change is positive if we're
						// dealing with the second body in a contact
						// and negative otherwise (because we're
						// subtracting the resolution)..
						c[i].penetration +=
							_scalarProductVec3(deltaPosition, c[i].contactNormal) * (b ? 1 : -1);
					}
				}
			}
		}
		positionIterationsUsed++;
	}
}
