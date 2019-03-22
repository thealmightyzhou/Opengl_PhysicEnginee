#include "forcegenerator.h"
using namespace Thealmighty;
void ForceRegistry::UpdateForce(float duration)
{
	Registry::iterator it = registrations.begin();
	for (; it != registrations.end(); it++)
	{
		it->fg->UpdateForce(it->body, duration);
	}
}

void ForceRegistry::Add(RigidBody* body, ForceGenerator *fg)
{
	ForceRegistration fr;
	fr.body = body;
	fr.fg = fg;
	registrations.push_back(fr);
}

Gravity::Gravity(const QVector3D &g) :gravity(g) {}

void Gravity::UpdateForce(RigidBody* body, float duration)
{
	if (!body->HasFiniteMass())
		return;

	body->AddForce(gravity * body->GetMass());
}

Spring::Spring(const QVector3D &_localConnectionPt,
	RigidBody* _other,
	const QVector3D &_otherConnectionPt,
	float _springConstant,
	float _restLength) :connectionPoint(_localConnectionPt), 
	other(_other),
	otherConnectPoint(_otherConnectionPt),
	springConstant(_springConstant)
{}

void Spring::UpdateForce(RigidBody* body, float duration)
{
	QVector3D localPos = body->GetPointInWorldSpace(connectionPoint);
	QVector3D otherPos = other->GetPointInWorldSpace(otherConnectPoint);

	QVector3D force = localPos - otherPos;

	float magnitude = force.length();
	magnitude = abs(magnitude - restLength);
	magnitude *= springConstant;

	force.normalize();
	force *= -magnitude;
	body->AddForceAtPoint(force, localPos);
}

