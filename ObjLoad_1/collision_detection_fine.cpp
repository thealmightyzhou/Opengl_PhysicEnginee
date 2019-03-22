#include "collision_detection_fine.h"
using namespace Thealmighty;

//box������ָ�������ϵ�ͶӰ��
static inline float _TransformToAxis(const CollisionBox &box, const QVector3D &axis)
{
	return
		box.halfSize.x() * abs(_scalarProductVec3(axis, box.GetAxis(0))) + 
		box.halfSize.y() * abs(_scalarProductVec3(axis, box.GetAxis(1))) +
		box.halfSize.z() * abs(_scalarProductVec3(axis, box.GetAxis(2)));
}

//��������box��ĳһ�����ϵ�ͶӰ�Ƿ��ཻ
static inline bool OverlapOnAxis(const CollisionBox &one, const CollisionBox &two, const QVector3D &axis, const QVector3D &toCentre)
{
	float oneProject = _TransformToAxis(one, axis);
	float twoProject = _TransformToAxis(two, axis);

	//����box���ľ����������ϵ�ͶӰ
	float distance = abs(_scalarProductVec3(toCentre, axis));

	//�ж��Ƿ��ཻ
	return (distance < oneProject + twoProject);
}

//��������box��ĳһ�����ϵ��ཻ��ȣ�����0Ϊ�ཻ��ȣ�С��0Ϊ���ཻ
static inline float PenetrationOnAxis(const CollisionBox &one, const CollisionBox &two, const QVector3D &axis, const QVector3D &toCentre)
{
	float oneProject = _TransformToAxis(one, axis);
	float twoProject = _TransformToAxis(two, axis);

	float distance = abs(_scalarProductVec3(toCentre, axis));

	//�����ཻ���
	return oneProject + twoProject - distance;
}

//����box��ĳһ�����Ͻ��в��ԣ����������С�ཻ��Ⱥ�������ص��ô�smallestcase�����Сindex
static inline bool TryAxis(
	const CollisionBox &one,
	const CollisionBox &two,
	QVector3D axis,
	const QVector3D& toCentre,
	unsigned index,
	float& smallestPenetration,
	unsigned &smallestCase
)
{
	// Make sure we have a normalized axis, and don't check almost parallel axes
	if (axis.lengthSquared() < 0.0001f) 
		return true;
	axis.normalize();

	float penetration = PenetrationOnAxis(one, two, axis, toCentre);

	if (penetration < 0) 
		return false;
	if (penetration < smallestPenetration) {
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

//��������ռ��model����
void CollisionPrimitive::CalcInternals()
{
	transform = _Mat3x4ProductMat3x4(rb->GetTransform(), offset);
}

//�����ƽ���Ƿ��ཻ
bool IntersectionTests::SphereAndHalfSpace(
	const CollisionSphere &sphere,
	const CollisionPlane &plane)
{
	//����ƽ�淨�ߵ�ԭ��ľ���
	float ballDistance = _scalarProductVec3(plane.direction, sphere.GetAxis(3)) - sphere.radius;

	return ballDistance <= plane.offset;
}

//�������Ƿ��ཻ
bool IntersectionTests::SphereAndSphere(
	const CollisionSphere &one,
	const CollisionSphere &two)
{
	QVector3D midline = one.GetAxis(3) - two.GetAxis(3);

	return midline.lengthSquared() <
		(one.radius + two.radius)*(one.radius + two.radius);
}

#define TEST_OVERLAP(axis) OverlapOnAxis(one, two, (axis), toCentre)

//����box���ཻ����
bool IntersectionTests::BoxAndBox(
	const CollisionBox &one,
	const CollisionBox &two
)
{
	//����box֮�����������
	QVector3D toCentre = two.GetAxis(3) - one.GetAxis(3);

	return (
		//һ����15�������Ͻ����ཻ����
		TEST_OVERLAP(one.GetAxis(0)) &&
		TEST_OVERLAP(one.GetAxis(1)) &&
		TEST_OVERLAP(one.GetAxis(2)) &&

		TEST_OVERLAP(two.GetAxis(0)) &&
		TEST_OVERLAP(two.GetAxis(1)) &&
		TEST_OVERLAP(two.GetAxis(2)) &&

		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(0), two.GetAxis(0))) &&
		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(0), two.GetAxis(1))) &&
		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(0), two.GetAxis(2))) &&
		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(1), two.GetAxis(0))) &&
		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(1), two.GetAxis(1))) &&
		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(1), two.GetAxis(2))) &&
		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(2), two.GetAxis(0))) &&
		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(2), two.GetAxis(1))) &&
		TEST_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(2), two.GetAxis(2)))
		);
}

#undef TEST_OVERLAP

//box�Ͱ�ƽ���Ǳ����ײ���
bool IntersectionTests::BoxAndHalfSpace(
	const CollisionBox &box,
	const CollisionPlane &plane
)
{
	// Work out the projected radius of the box onto the plane direction
	float projectedRadius = _TransformToAxis(box, plane.direction);

	// Work out how far the box is from the origin
	float boxDistance = _scalarProductVec3(plane.direction, box.GetAxis(3)) - projectedRadius;

	// Check for the intersection
	return boxDistance <= plane.offset;
}

//�ж�����ƽ�����ײ���������ݣ���ײ���ߣ���ײ�㣬�ཻ��ȣ�trueplane����������
unsigned CollisionDetector::SphereAndTruePlane(
	const CollisionSphere &sphere,
	const CollisionPlane &plane,
	CollisionData *data
)
{
	// Make sure we have contacts
	if (data->contactsLeft <= 0) return 0;

	// Cache the sphere position
	QVector3D position = sphere.GetAxis(3);

	// Find the distance from the plane
	float centreDistance = _scalarProductVec3(plane.direction, position) - plane.offset;

	// Check if we're within radius
	if (centreDistance*centreDistance > sphere.radius*sphere.radius)
	{
		return 0;
	}

	// Check which side of the plane we're on
	QVector3D normal = plane.direction;
	float penetration = -centreDistance;
	if (centreDistance < 0)
	{
		normal *= -1;
		penetration = -penetration;
	}
	penetration += sphere.radius;

	// Create the contact - it has a normal in the plane direction.
	Contact* contact = data->contacts;
	contact->contactNormal = normal;
	contact->penetration = penetration;
	contact->contactPoint = position - plane.direction * centreDistance;
	contact->SetBodyData(sphere.rb, NULL,
		data->friction, data->restitution);

	data->AddContacts(1);
	return 1;
}

//�ж������ƽ�����ײ����������
unsigned CollisionDetector::SphereAndHalfSpace(
	const CollisionSphere &sphere,
	const CollisionPlane &plane,
	CollisionData *data
)
{
	// Make sure we have contacts
	if (data->contactsLeft <= 0) return 0;

	// Cache the sphere position
	QVector3D position = sphere.GetAxis(3);

	// Find the distance from the plane
	float ballDistance = _scalarProductVec3(plane.direction, position) - sphere.radius - plane.offset;

	if (ballDistance >= 0) return 0;

	// Create the contact - it has a normal in the plane direction.
	Contact* contact = data->contacts;
	contact->contactNormal = plane.direction;
	contact->penetration = -ballDistance;
	contact->contactPoint =
		position - plane.direction * (ballDistance + sphere.radius);
	contact->SetBodyData(sphere.rb, NULL,
		data->friction, data->restitution);

	data->AddContacts(1);
	return 1;
}

//������֮����ײ��������
unsigned CollisionDetector::SphereAndSphere(
	const CollisionSphere &one,
	const CollisionSphere &two,
	CollisionData *data
)
{
	// Make sure we have contacts
	if (data->contactsLeft <= 0) return 0;

	// Cache the sphere positions
	QVector3D positionOne = one.GetAxis(3);
	QVector3D positionTwo = two.GetAxis(3);

	// Find the vector between the objects
	QVector3D midline = positionOne - positionTwo;
	float size = midline.length();

	// See if it is large enough.
	if (size <= 0.0f || size >= one.radius + two.radius)
	{
		return 0;
	}

	// We manually create the normal, because we have the
	// size to hand.
	QVector3D normal = midline * (((float)1.0) / size);

	Contact* contact = data->contacts;
	contact->contactNormal = normal;
	contact->contactPoint = positionOne + midline * (float)0.5;
	contact->penetration = (one.radius + two.radius - size);
	contact->SetBodyData(one.rb, two.rb,
		data->friction, data->restitution);

	data->AddContacts(1);
	return 1;
}

//box֮��ĵ�����ײ���Ѿ�ȷ����������Դ������ײ����
void FillPointFaceBoxBox(
	const CollisionBox &one,//�ṩ��
	const CollisionBox &two,//�ṩ��
	const QVector3D &toCentre,
	CollisionData *data,
	unsigned best,//�����ཻ��ǳ�������淢����ײ
	float pen
)
{
	// This method is called when we know that a vertex from
	// box two is in contact with box one.

	Contact* contact = data->contacts;

	// We know which axis the collision is on (i.e. best),
	// but we need to work out which of the two faces on
	// this axis.
	QVector3D normal = one.GetAxis(best);
	if (_scalarProductVec3(one.GetAxis(best), toCentre) > 0)
	{
		normal = normal * -1.0f;
	}

	// Work out which vertex of box two we're colliding with.
	// Using toCentre doesn't work!
	//��ײ��Ϊbox2��һ��
	QVector3D vertex = two.halfSize;
	if (_scalarProductVec3(two.GetAxis(0), normal) < 0)
		vertex.setX(-vertex.x());

	if (_scalarProductVec3(two.GetAxis(1), normal) < 0)
		vertex.setY(-vertex.y());
	if (_scalarProductVec3(two.GetAxis(2), normal) < 0)
		vertex.setZ(-vertex.z());

	// Create the contact data
	contact->contactNormal = normal;
	contact->penetration = pen;
	contact->contactPoint = _Mat4ProductVec3(two.GetTransform(), vertex);
	contact->SetBodyData(one.rb, two.rb,
		data->friction, data->restitution);
}

//box��box֮�䷢������ײʱͨ����֪��ײ�߼�����ײ��
static inline QVector3D ContactPoint(
	const QVector3D &pOne,//box1��
	const QVector3D &dOne,//box1��
	float oneSize,//box1��Ӧ�᳤
	const QVector3D &pTwo,//box2��
	const QVector3D &dTwo,//box2��
	float twoSize,//box2��Ӧ�᳤

	// If this is true, and the contact point is outside
	// the edge (in the case of an edge-face contact) then
	// we use one's midpoint, otherwise we use two's.
	bool useOne)
{
	QVector3D toSt, cOne, cTwo;
	float dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	float denom, mua, mub;

	smOne = dOne.lengthSquared();
	smTwo = dTwo.lengthSquared();
	dpOneTwo = _scalarProductVec3(dTwo, dOne);

	toSt = pOne - pTwo;
	dpStaOne = _scalarProductVec3(dOne, toSt);
	dpStaTwo = _scalarProductVec3(dTwo, toSt);

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	// Zero denominator indicates parrallel lines
	if (abs(denom) < 0.0001f) {
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	// If either of the edges has the nearest point out
	// of bounds, then the edges aren't crossed, we have
	// an edge-face contact. Our point is on the edge, which
	// we know from the useOne parameter.
	if (mua > oneSize ||
		mua < -oneSize ||
		mub > twoSize ||
		mub < -twoSize)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5 + cTwo * 0.5;
	}
}

#define CHECK_OVERLAP(axis, index) \
    if (!TryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;

unsigned CollisionDetector::BoxAndBox(
	const CollisionBox &one,
	const CollisionBox &two,
	CollisionData *data
)
{
	//if (!IntersectionTests::boxAndBox(one, two)) return 0;

	// Find the vector between the two centres
	QVector3D toCentre = two.GetAxis(3) - one.GetAxis(3);

	// We start assuming there is no contact
	float pen = MAX_FLOAT;
	unsigned best = 0xffffff;

	// Now we check each axes, returning if it gives us
	// a separating axis, and keeping track of the axis with
	// the smallest penetration otherwise.
	//��15�������Ͻ��в����ҳ���С�ཻ���ֵ
	CHECK_OVERLAP(one.GetAxis(0), 0);
	CHECK_OVERLAP(one.GetAxis(1), 1);
	CHECK_OVERLAP(one.GetAxis(2), 2);

	CHECK_OVERLAP(two.GetAxis(0), 3);
	CHECK_OVERLAP(two.GetAxis(1), 4);
	CHECK_OVERLAP(two.GetAxis(2), 5);

	//��¼����ѵ����ᣬ�Է���������ƽ�е���ײ
	unsigned bestSingleAxis = best;

	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(0), two.GetAxis(0)), 6);
	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(0), two.GetAxis(1)), 7);
	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(0), two.GetAxis(2)), 8);
	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(1), two.GetAxis(0)), 9);
	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(1), two.GetAxis(1)), 10);
	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(1), two.GetAxis(2)), 11);	
	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(2), two.GetAxis(0)), 12);
	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(2), two.GetAxis(1)), 13);
	CHECK_OVERLAP(_Vec3CrossProductVec3(one.GetAxis(2), two.GetAxis(2)), 14);

	// Make sure we've got a result.
	assert(best != 0xffffff);

	// We now know there's a collision, and we know which
	// of the axes gave the smallest penetration. We now
	// can deal with it in different ways depending on
	// the case.
	//��ʱΪbox2�ĵ���ײ��box1������
	if (best < 3)
	{
		// We've got a vertex of box two on a face of box one.
		FillPointFaceBoxBox(one, two, toCentre, data, best, pen);
		data->AddContacts(1);
		return 1;
	}
	//��ʱΪbox1�ĵ���ײ��box2������
	else if (best < 6)
	{
		// We've got a vertex of box one on a face of box two.
		// We use the same algorithm as above, but swap around
		// one and two (and therefore also the vector between their
		// centres).
		FillPointFaceBoxBox(two, one, toCentre*-1.0f, data, best - 3, pen);
		data->AddContacts(1);
		return 1;
	}
	else
	{
		// We've got an edge-edge contact. Find out which axes
		best -= 6;
		unsigned oneAxisIndex = best / 3; 
		unsigned twoAxisIndex = best % 3; 
		QVector3D oneAxis = one.GetAxis(oneAxisIndex);//box1�Ķ�Ӧ���򵫲�ȷ��������������
		QVector3D twoAxis = two.GetAxis(twoAxisIndex);//box2�Ķ�Ӧ����
		QVector3D axis = _Vec3CrossProductVec3(oneAxis, twoAxis);//��ǳ�ཻ������Ϊbox1���box2������
		axis.normalize();

		// ȷ�������box1ָ��box2
		if (_scalarProductVec3(axis, toCentre) > 0)
			axis = axis * -1.0f;

		// We have the axes, but not the edges: each axis has 4 edges parallel
		// to it, we need to find which of the 4 for each object. We do
		// that by finding the point in the centre of the edge. We know
		// its component in the direction of the box's collision axis is zero
		// (its a mid-point) and we determine which of the extremes in each
		// of the other axes is closest.
		QVector3D ptOnOneEdge = one.halfSize;
		QVector3D ptOnTwoEdge = two.halfSize;
		//ȷ��������box�����������ڷ�����ײ
		for (unsigned i = 0; i < 3; i++)
		{
			if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
			else if (_scalarProductVec3(one.GetAxis(i), axis) > 0)
				ptOnOneEdge[i] = -ptOnOneEdge[i];

			if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
			else if (_scalarProductVec3(two.GetAxis(i), axis) < 0) 
				ptOnTwoEdge[i] = -ptOnTwoEdge[i];
		}

		// Move them into world coordinates (they are already oriented
		// correctly, since they have been derived from the axes).
		ptOnOneEdge = _Mat4ProductVec3(one.transform, ptOnOneEdge);
		ptOnTwoEdge = _Mat4ProductVec3(two.transform, ptOnTwoEdge);

		// So we have a point and a direction for the colliding edges.
		// We need to find out point of closest approach of the two
		// line-segments.
		QVector3D vertex = ContactPoint(
			ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
			ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
			bestSingleAxis > 2
		);

		// We can fill the contact.
		Contact* contact = data->contacts;

		contact->penetration = pen;
		contact->contactNormal = axis;
		contact->contactPoint = vertex;
		contact->SetBodyData(one.rb, two.rb,
			data->friction, data->restitution);
		data->AddContacts(1);
		return 1;
	}
	return 0;
}
#undef CHECK_OVERLAP

//box��͵㣨������壩����ײ��Ⲣ��������
unsigned CollisionDetector::BoxAndPoint(
	const CollisionBox &box,
	const QVector3D &point,
	CollisionData *data
)
{
	// �����������������ת��box��������
	QVector3D relPt = _Mat4ProductVec3Inverse(box.transform, point);

	QVector3D normal;

	// ��xyz��������Ѱ�ҵ���С�ཻ��ȵ��Ǹ���
	float min_depth = box.halfSize.x() - abs(relPt.x());
	if (min_depth < 0) return 0;
	normal = box.GetAxis(0) * ((relPt.x() < 0) ? -1 : 1);

	float depth = box.halfSize.y() - abs(relPt.y());
	if (depth < 0) return 0;
	else if (depth < min_depth)
	{
		min_depth = depth;
		normal = box.GetAxis(1) * ((relPt.y() < 0) ? -1 : 1);
	}

	depth = box.halfSize.z() - abs(relPt.z());
	if (depth < 0) return 0;
	else if (depth < min_depth)
	{
		min_depth = depth;
		normal = box.GetAxis(2) * ((relPt.z() < 0) ? -1 : 1);
	}

	// Compile the contact
	Contact* contact = data->contacts;
	contact->contactNormal = normal;
	contact->contactPoint = point;
	contact->penetration = min_depth;

	//������������������ϵĵ�������Ϊnull
	// Note that we don't know what rigid body the point
	// belongs to, so we just use NULL. Where this is called
	// this value can be left, or filled in.
	contact->SetBodyData(box.rb, NULL,
		data->friction, data->restitution);

	data->AddContacts(1);
	return 1;
}

//box�������ײ��⣬��ײ����box�߽��ϣ���ֱbox��
unsigned CollisionDetector::BoxAndSphere(
	const CollisionBox &box,
	const CollisionSphere &sphere,
	CollisionData *data
)
{
	// Transform the centre of the sphere into box coordinates
	QVector3D centre = sphere.GetAxis(3);
	QVector3D relCentre = _Mat4ProductVec3Inverse(box.transform, centre);//��ͬ����������

	//�����ཻ��������ǰ���˳�
	if (abs(relCentre.x()) - sphere.radius > box.halfSize.x() ||
		abs(relCentre.y()) - sphere.radius > box.halfSize.y() ||
		abs(relCentre.z()) - sphere.radius > box.halfSize.z())
	{
		return 0;
	}

	QVector3D closestPt(0, 0, 0);
	float dist;

	// �����ཻ�㣺Բ��xyz���β���
	dist = relCentre.x();
	if (dist > box.halfSize.x()) dist = box.halfSize.x();
	if (dist < -box.halfSize.x()) dist = -box.halfSize.x();
	closestPt.setX(dist);

	dist = relCentre.y();
	if (dist > box.halfSize.y()) dist = box.halfSize.y();
	if (dist < -box.halfSize.y()) dist = -box.halfSize.y();
	closestPt.setY(dist);

	dist = relCentre.z();
	if (dist > box.halfSize.z()) dist = box.halfSize.z();
	if (dist < -box.halfSize.z()) dist = -box.halfSize.z();
	closestPt.setZ(dist);

	// Check we're in contact
	dist = (closestPt - relCentre).lengthSquared();
	if (dist > sphere.radius * sphere.radius) return 0;

	// Compile the contact
	QVector3D closestPtWorld = _Mat4ProductVec3(box.transform, closestPt);

	Contact* contact = data->contacts;
	contact->contactNormal = (closestPtWorld - centre);
	contact->contactNormal.normalize();
	contact->contactPoint = closestPtWorld;
	contact->penetration = sphere.radius - sqrt(dist);
	contact->SetBodyData(box.rb, sphere.rb,
		data->friction, data->restitution);

	data->AddContacts(1);
	return 1;
}

//box�Ͱ�ƽ����ײ��⣬���μ��8������
unsigned CollisionDetector::BoxAndHalfSpace(
	const CollisionBox &box,
	const CollisionPlane &plane,
	CollisionData *data
)
{
	// Make sure we have contacts
	if (data->contactsLeft <= 0) return 0;

	// Check for intersection
	if (!IntersectionTests::BoxAndHalfSpace(box, plane))
	{
		return 0;
	}

	// We have an intersection, so find the intersection points. We can make
	// do with only checking vertices. If the box is resting on a plane
	// or on an edge, it will be reported as four or two contact points.

	// Go through each combination of + and - for each half-size
	static float mults[8][3] = { {1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
							   {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1} };

	Contact* contact = data->contacts;
	unsigned contactsUsed = 0;
	for (unsigned i = 0; i < 8; i++) {

		// Calculate the position of each vertex
		QVector3D vertexPos(mults[i][0], mults[i][1], mults[i][2]);
		//vertexPos��box��������
		Vec3ComponentProductUpdate(vertexPos, box.halfSize);
		vertexPos = _Mat4ProductVec3(box.transform, vertexPos);

		// Calculate the distance from the plane
		float vertexDistance = _scalarProductVec3(vertexPos, plane.direction);

		// Compare this to the plane's distance
		if (vertexDistance <= plane.offset)
		{
			// Create the contact data.

			// The contact point is halfway between the vertex and the
			// plane - we multiply the direction by half the separation
			// distance and add the vertex location.
			contact->contactPoint = plane.direction;
			contact->contactPoint *= (vertexDistance - plane.offset);
			contact->contactPoint += vertexPos;
			contact->contactNormal = plane.direction;
			contact->penetration = plane.offset - vertexDistance;

			// Write the appropriate data
			contact->SetBodyData(box.rb, NULL,
				data->friction, data->restitution);

			// Move onto the next contact
			contact++;
			contactsUsed++;
			if (contactsUsed == (unsigned)data->contactsLeft) return contactsUsed;
		}
	}

	data->AddContacts(contactsUsed);
	return contactsUsed;
}
