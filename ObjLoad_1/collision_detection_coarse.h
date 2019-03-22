#pragma once
#ifndef COLLISION_DETECTION_COARSE_H
#define COLLISION_DETECTION_COARSE_H
#include "core.h"
#include <iostream>
#include "rigidbody.h"
const double PI = 3.1415926535;
namespace Thealmighty
{
	//°üÎ§Çò
	struct BoundingSphere
	{
		QVector3D center;
		float radius;
	public:
		BoundingSphere(const QVector3D& c, float r);
		BoundingSphere(const BoundingSphere& s1, const BoundingSphere& s2);
		bool Overlaps(const BoundingSphere* other) const;
		float GetGrowth(const BoundingSphere& other) const;
		float GetSize() const
		{
			return 1.333333f*PI*radius*radius*radius;
		}

	};
	struct BoundingBox
	{
		QVector3D center;
		QVector3D halfSize;
	};

	struct PotentialContact
	{
		RigidBody* body[2];

	};

	template<class BoundingVolumeClass>
	class BVHNode
	{
	public:
		BVHNode *child[2];
		BoundingVolumeClass volume;
		RigidBody *rb;
		BVHNode *parent;
		//parent,volume,rb
		BVHNode(BVHNode* p, const BoundingVolumeClass &volume, RigidBody *rb = null) :
			parent(p), 
			this->volume(volume), 
			this->rb(rb)
		{
			child[0] = child[1] = NULL;
		}
		
		bool IsLeaf()
		{
			return rb != NULL;
		}
		unsigned GetPotentialContacts(PotentialContact* contacts, unsigned limit) const;
		void Insert(RigidBody* rb, const BoundingVolumeClass &volume);
		~BVHNode();
	protected:
		bool Overlaps(const BVHNode<BoundingVolumeClass> *other) const;

		unsigned GetPotentialContactsWith(const BVHNode<BoundingVolumeClass>*other, PotentialContact*contacts, unsigned limit) const;
		void ReCalcBoundingVolume(bool recurse = true);

	};


}
#endif // !COLLISION_DETECTION_COARSE_H
