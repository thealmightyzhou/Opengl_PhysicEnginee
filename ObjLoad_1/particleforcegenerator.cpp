#include "particleforcegenerator.h"
using namespace Thealmighty;

void ParticleForceRegistry::Add(Particle* p, ParticleForceGenerator* fg)
{
	ParticleForceRegistry::ParticleForceRegistration registration;
	registration.particle = p;
	registration.fg = fg;

	registrations.push_back(registration);
}
void ParticleForceRegistry::Remove(Particle* p, ParticleForceGenerator* fg)
{
	std::vector<ParticleForceRegistration>::iterator it;
	for (it = registrations.begin(); it != registrations.end(); it++)
	{
		if (it->particle == p && it->fg == fg)
		{
			registrations.erase(it);
		}
	}
}
void ParticleForceRegistry::Clear()
{
	registrations.clear();
}
void ParticleForceRegistry::UpdateForces(float duration)
{
	std::vector<ParticleForceRegistration>::iterator it;
	for (it = registrations.begin(); it != registrations.end(); it++)
	{
		it->fg->UpdateForce(it->particle, duration);
	}
}

void ParticleGravity::UpdateForce(Particle* p, float duration)
{
	if (!p->HasFiniteMass())
		return;
	p->AddForce(gravity * p->GetMass());
}

void ParticleDrag::UpdateForce(Particle *p, float duration)
{
	QVector3D force;
	force = p->GetVelocity();
	float dragCoeff = force.length();
	dragCoeff = k1 * dragCoeff + k2 * dragCoeff*dragCoeff;
	force.normalize();
	force *= -dragCoeff;
	p->AddForce(force);
}

void ParticleSpring::UpdateForce(Particle *p, float duration)
{
	QVector3D force;
	force = p->GetPosition();
	force -= other->GetPosition();
	float length = force.length();
	length = abs(length - restLength);
	length *= springConstant;

	force.normalize();
	force *= -length;
	p->AddForce(force);

}
