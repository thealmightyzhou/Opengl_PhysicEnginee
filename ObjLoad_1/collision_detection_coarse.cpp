#include "collision_detection_coarse.h"
using namespace Thealmighty;

BoundingSphere::BoundingSphere(const QVector3D& c, float r)
{
	center = c;
	radius = r;
}

//合并两球为一球
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

//判断两球是否相交
bool BoundingSphere::Overlaps(const BoundingSphere* other) const
{
	float distanceSquared = (center - other->center).length();
	return (distanceSquared*distanceSquared) < (radius + other->radius)*(radius + other->radius);
}

//将本球与球1合并，返回新球与旧球的半径平方差
float BoundingSphere::GetGrowth(const BoundingSphere& other) const
{
	BoundingSphere newSphere(*this, other);
	return newSphere.radius*newSphere.radius - radius * radius;
}

//判断两球是否相交
template<class BoundingVolumeClass>
bool BVHNode<BoundingVolumeClass>::Overlaps(const BVHNode<BoundingVolumeClass> *other) const
{
	return volume->Overlaps(other->volume);
}

//插入一个新节点
template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::Insert(RigidBody* newrb, const BoundingVolumeClass &newvolume)
{
	//如果本节点为叶子节点，生成两个子节点，父节点设置为本节点，将本节点存储数据删除，作为枝
	if (IsLeaf())
	{
		child[0] = new BVHNode<BoundingVolumeClass>(this, volume, rb);
		child[1] = new BVHNode<BoundingVolumeClass>(this, newvolume, newrb);
		this->rb = NULL;
		ReCalcBoundingVolume();
	}
	//如果不为叶子节点，比较两个儿子分别合并后的半径差值大小，合并后的新球半径尽可能小
	//递归遍历枝下子节点 
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
	//如果有父节点（不是根节点）
	if (parent)
	{
		//兄弟节点
		BVHNode<BoundingVolumeClass> *sibling;
		//本节点是父节点的左儿子
		if (parent->child[0] == this)
			sibling = parent->child[1];
		else
			sibling = parent->child[0];

		//将父节点的数据赋值为兄弟节点的数据
		parent->volume = sibling->volume;
		parent->rb = sibling->rb;
		parent->child[0] = sibling->child[0];
		parent->child[1] = sibling->child[1];

		//删除兄弟节点
		sibling->parent = NULL;
		sibling->rb = NULL;
		sibling->child[0] = NULL;
		sibling->child[1] = NULL;
		delete sibling;

		//此时兄弟节点变成了父节点，向上更新
		parent->ReCalcBoundingVolume();

		//同时删除儿子节点，父节点不被使用之后，儿子节点也没必要使用了
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

//更新volume数据上溯更新
template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::ReCalcBoundingVolume(bool recurse)
{
	if (IsLeaf())
		return;
	//合并两球并上溯更新这条枝到根
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
	//不相交直接返回0 或者计数限制不够
	if (!Overlaps(other) || limit == 0)
		return 0;

	//如果都是叶子节点并且相交(之前已经判断过不相交)保存contacts信息返回1
	if (IsLeaf() && other->IsLeaf())
	{
		contacts->body[0] = rb;
		contacts->body[1] = other->rb;
		return 1;
	}
	//他是叶子我是枝 或者都是枝取size大的检查儿子
	if (other->IsLeaf() || (!IsLeaf() && volume->GetSize() >= other->volume->GetSize()))
	{
		//计数x(1或0)，我左儿子与他的相交检查
		unsigned cnt = child[0]->GetPotentialContactsWith(other, contacts, limit);
		//未到计数限制 
		if (limit > cnt)
		{
			//cnt为1（检查到相交）：返回
			return cnt + child[1]->GetPotentialContactsWith(other, contacts + cnt, limit - cnt);
		}
		else
		{
			return cnt;
		}

	}
	//他不是叶子，我是叶子
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