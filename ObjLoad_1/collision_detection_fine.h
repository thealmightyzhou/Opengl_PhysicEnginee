#pragma once
#ifndef COLLISION_DETECTION_FINE_H
#define COLLISION_DETECTION_FINE_H
#include "contacts.h"
#include "rigidbody.h"
#include "core.h"
namespace Thealmighty
{
	class IntersectionTests;
	class CollisionDetector;
	class RigidBody;
	//class Contact;
	class CollisionPrimitive
	{
	public:
		friend class IntersectionTests;
		friend class CollisionDetector;

		CollisionPrimitive(RigidBody *rb) { 
			this->rb = rb;
		}

		RigidBody* rb;
		QMatrix3x4 offset;
		void CalcInternals();
		QVector3D GetAxis(unsigned index) const
		{
			return _Mat3x4GetAxisVector(transform, index);
		}
		const QMatrix3x4& GetTransform() const
		{
			return transform;
		}

	protected:
		QMatrix3x4 transform;
	};

	class CollisionSphere : public CollisionPrimitive
	{
	public:
		CollisionSphere(RigidBody *r) :CollisionPrimitive(r)
		{
			rb = r;
		}
		float radius;
	};

	class CollisionPlane
	{
	public:
		QVector3D direction;
		float offset;
	};

	class CollisionBox : public CollisionPrimitive
	{
	public:
		CollisionBox(RigidBody *r):CollisionPrimitive(r)
		{
			rb = r;
		}
		QVector3D halfSize;
	};

	//Ç±ÔÚµÄÅö×²
	class IntersectionTests
	{
	public:
		static bool SphereAndHalfSpace(const CollisionSphere &sphere,const CollisionPlane &plane);
		static bool SphereAndSphere(const CollisionSphere &one,const CollisionSphere &two);
		static bool BoxAndBox(const CollisionBox &one,const CollisionBox &two);
		static bool BoxAndHalfSpace(const CollisionBox &box,const CollisionPlane &plane);

	};

	struct CollisionData
	{
		Contact *contactArray;
		Contact *contacts;
		int contactsLeft;
		unsigned contactCount;
		float friction;
		float restitution;
		float tolerance;
		bool HasMoreContacts()
		{
			return contactsLeft > 0;
		}
		void Reset(unsigned maxContacts)
		{
			contactsLeft = maxContacts;
			contactCount = 0;
			contacts = contactArray;
		}

		void AddContacts(unsigned count)
		{
			// Reduce the number of contacts remaining, add number used
			contactsLeft -= count;
			contactCount += count;

			// Move the array forward
			//((Contact*)contacts) += count;
			contacts += count;
		}
	};

	class CollisionDetector
	{
	public:

		static unsigned SphereAndHalfSpace(const CollisionSphere &sphere,const CollisionPlane &plane,CollisionData *data);
		static unsigned SphereAndTruePlane(const CollisionSphere &sphere,const CollisionPlane &plane,CollisionData *data);
		static unsigned SphereAndSphere(const CollisionSphere &one,const CollisionSphere &two,CollisionData *data);
		static unsigned BoxAndHalfSpace(const CollisionBox &box,const CollisionPlane &plane,CollisionData *data);
		static unsigned BoxAndBox(const CollisionBox &one,const CollisionBox &two,CollisionData *data);
		static unsigned BoxAndPoint(const CollisionBox &box,const QVector3D &point,CollisionData *data);
		static unsigned BoxAndSphere(const CollisionBox &box,const CollisionSphere &sphere,CollisionData *data);
	};
}
#endif // !COLLISION_DETECTION_FINE_H
