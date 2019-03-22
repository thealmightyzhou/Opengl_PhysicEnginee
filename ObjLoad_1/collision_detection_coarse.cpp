#include "collision_detection_coarse.h"
using namespace Thealmighty;

BoundingSphere::BoundingSphere(const QVector3D& c, float r)
{
	center = c;
	radius = r;
}

//�ϲ�����Ϊһ��
BoundingSphere::BoundingSphere(const BoundingSphere& s1, const BoundingSphere& s2)
{
	QVector3D offset = s2.center - s1.center;
	float distance = offset.length();
	float radiusDiff = s2.radius - s1.radius;

	if (radiusDiff*radiusDiff >= distance*distance)
	{
		if (s1.radius > s2.radius)
		{
			center = s1.center;
			radius = s1.radius;
		}
		else
		{
			center = s2.center;
			radius = s2.radius;
		}
	}
	else
	{
		radius = (distance + s1.radius + s2.radius)*0.5f;
		center = s1.center;
		if (distance > 0)
		{
			center += offset * ((radius - s1.radius) / distance);
		}
	}
}

//�ж������Ƿ��ཻ
bool BoundingSphere::Overlaps(const BoundingSphere* other) const
{
	float distanceSquared = (center - other->center).length();
	return (distanceSquared*distanceSquared) < (radius + other->radius)*(radius + other->radius);
}

//����������1�ϲ����������������İ뾶ƽ����
float BoundingSphere::GetGrowth(const BoundingSphere& other) const
{
	BoundingSphere newSphere(*this, other);
	return newSphere.radius*newSphere.radius - radius * radius;
}

//�ж������Ƿ��ཻ
template<class BoundingVolumeClass>
bool BVHNode<BoundingVolumeClass>::Overlaps(const BVHNode<BoundingVolumeClass> *other) const
{
	return volume->Overlaps(other->volume);
}

//����һ���½ڵ�
template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::Insert(RigidBody* newrb, const BoundingVolumeClass &newvolume)
{
	//������ڵ�ΪҶ�ӽڵ㣬���������ӽڵ㣬���ڵ�����Ϊ���ڵ㣬�����ڵ�洢����ɾ������Ϊ֦
	if (IsLeaf())
	{
		child[0] = new BVHNode<BoundingVolumeClass>(this, volume, rb);
		child[1] = new BVHNode<BoundingVolumeClass>(this, newvolume, newrb);
		this->rb = NULL;
		ReCalcBoundingVolume();
	}
	//�����ΪҶ�ӽڵ㣬�Ƚ��������ӷֱ�ϲ���İ뾶��ֵ��С���ϲ��������뾶������С
	//�ݹ����֦���ӽڵ� 
	else
	{
		if (child[0]->volume.GetGrowth(newvolume) < child[1]->volume.GetGrowth(newvolume))
		{
			child[0]->Insert(newrb, newvolume);
		}
		else
		{
			child[1]->Insert(newrb, newvolume);
		}
	}
}

template<class BoundingVolumeClass>
BVHNode<BoundingVolumeClass>::~BVHNode()
{
	//����и��ڵ㣨���Ǹ��ڵ㣩
	if (parent)
	{
		//�ֵܽڵ�
		BVHNode<BoundingVolumeClass> *sibling;
		//���ڵ��Ǹ��ڵ�������
		if (parent->child[0] == this)
			sibling = parent->child[1];
		else
			sibling = parent->child[0];

		//�����ڵ�����ݸ�ֵΪ�ֵܽڵ������
		parent->volume = sibling->volume;
		parent->rb = sibling->rb;
		parent->child[0] = sibling->child[0];
		parent->child[1] = sibling->child[1];

		//ɾ���ֵܽڵ�
		sibling->parent = NULL;
		sibling->rb = NULL;
		sibling->child[0] = NULL;
		sibling->child[1] = NULL;
		delete sibling;

		//��ʱ�ֵܽڵ����˸��ڵ㣬���ϸ���
		parent->ReCalcBoundingVolume();

		//ͬʱɾ�����ӽڵ㣬���ڵ㲻��ʹ��֮�󣬶��ӽڵ�Ҳû��Ҫʹ����
		if (child[0])
		{
			child[0]->parent = NULL;
			delete child[0];
		}
		if (child[1])
		{
			child[1]->parent = NULL;
			delete child[1];
		}
	}
}

//����volume�������ݸ���
template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::ReCalcBoundingVolume(bool recurse)
{
	if (IsLeaf())
		return;
	//�ϲ��������ݸ�������֦����
	volume = BoundingVolumeClass(child[0]->volume, child[1]->volume);
	if (parent)
		parent->ReCalcBoundingVolume(true);
}

//
template<class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::GetPotentialContacts(PotentialContact* contacts, unsigned limit) const
{
	if (IsLeaf() || limit == 0)
		return 0;
	return child[0]->GetPotentialContactsWith(child[1], contacts, limit);
}


template<class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::GetPotentialContactsWith(const BVHNode<BoundingVolumeClass> *other,
	PotentialContact* contacts, unsigned limit) const
{
	//���ֱཻ�ӷ���0 ���߼������Ʋ���
	if (!Overlaps(other) || limit == 0)
		return 0;

	//�������Ҷ�ӽڵ㲢���ཻ(֮ǰ�Ѿ��жϹ����ཻ)����contacts��Ϣ����1
	if (IsLeaf() && other->IsLeaf())
	{
		contacts->body[0] = rb;
		contacts->body[1] = other->rb;
		return 1;
	}
	//����Ҷ������֦ ���߶���֦ȡsize��ļ�����
	if (other->IsLeaf() || (!IsLeaf() && volume->GetSize() >= other->volume->GetSize()))
	{
		//����x(1��0)����������������ཻ���
		unsigned cnt = child[0]->GetPotentialContactsWith(other, contacts, limit);
		//δ���������� 
		if (limit > cnt)
		{
			//cntΪ1����鵽�ཻ��������
			return cnt + child[1]->GetPotentialContactsWith(other, contacts + cnt, limit - cnt);
		}
		else
		{
			return cnt;
		}

	}
	//������Ҷ�ӣ�����Ҷ��
	else
	{
		unsigned cnt = GetPotentialContactsWith(other->children[0], contacts, limit);
		if (limit > cnt)
		{
			return cnt+ GetPotentialContactsWith(other->children[1], contacts + count, limit - count);
		}
		else
		{
			return cnt;
		}
	}
}