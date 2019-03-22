#pragma once
#ifndef CONTACTS_H
#define CONTACTS_H
#include "rigidbody.h"
#include "assert.h"
#include "core.h"
namespace Thealmighty
{
	class ContactResolver;
	class RigidBody;

	class Contact
	{
		friend class ContactResolver;
	public:
		RigidBody* rb[2];
		float friction;
		float restitution;
		QVector3D contactPoint;
		QVector3D contactNormal;
		float penetration;
		void SetBodyData(RigidBody *one, RigidBody *two, float friction, float restitution);

	protected:
		QMatrix3x3 contactToWorld;
		QVector3D contactVelocity;
		float desiredDeltaVelocity;
		QVector3D relativeContactPosition[2];
	protected:
		void CalcInternals(float duration);
		void SwapBodies();
		void MatchAwakeState();
		void CalcDesiredDeltaVelocity(float duration);
		QVector3D CalcLocalVelocity(unsigned bodyIndex, float duration);
		void CalcContactBasis();
		void ApplyImpulse(const QVector3D &impulse, RigidBody* body, QVector3D *velocityChange, QVector3D *rotationChange);
		void ApplyVelocityChange(QVector3D velocityChange[2], QVector3D rotationChange[2]);
		void ApplyPositionChange(QVector3D linearChange[2], QVector3D angularChange[2], float penetration);
		QVector3D CalcFrictionlessImpulse(QMatrix3x3 *inverseInertiaTensor);
		QVector3D CalcFrictionImpulse(QMatrix3x3 *inverseInertiaTensor);

	};
	class ContactResolver 
	{
	protected:
		unsigned velocityIterations;
		unsigned positionIterations;
		
		float velocityEpsilon;
		float positionEpsilon;

	public:
		unsigned velocityIterationsUsed;
		unsigned positionIterationsUsed;

	private:
		bool validSettings;

	public:
		ContactResolver(unsigned iterations, float velocityEpsilon = 0.01f,float positionEpsilon = 0.01f);
		ContactResolver(unsigned velocityIterations,unsigned positionIterations,float velocityEpsilon = 0.01f, float positionEpsilon = 0.01f);
		bool IsValid()
		{
			return (velocityIterations > 0) &&
				(positionIterations > 0) &&
				(positionEpsilon >= 0.0f) &&
				(positionEpsilon >= 0.0f);
		}
		void SetIterations(unsigned velocityIterations,
			unsigned positionIterations);
		void SetIterations(unsigned iterations);
		void SetEpsilon(float velocityEpsilon,
			float positionEpsilon);
		void ResolveContacts(Contact *contactArray,
			unsigned numContacts,
			float duration);

	protected:
		void PrepareContacts(Contact *contactArray, unsigned numContacts,float duration);
		void AdjustVelocities(Contact *contactArray,
			unsigned numContacts,
			float duration);
		void AdjustPositions(Contact *contacts,
			unsigned numContacts,
			float duration);

	};

	class ContactGenerator
	{
	public:
		virtual unsigned AddContact(Contact *contact, unsigned limit) const = 0;
	};
}
#endif // !CONTACTS_H
