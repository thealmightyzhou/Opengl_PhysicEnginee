#include "particle.h"
using namespace Thealmighty;

void Particle::SetMass(float m)
{
	if (m <= 0)
		m = 0.000000001;
	mass = m;
	if (!HasFiniteMass())//��������100000000��Ϊ�����
		inverse_mass = 0.0;
	else
		inverse_mass = 1.0 / mass;
}
void Particle::SetDamping(float d)
{
	if (d < 0.0)
		d = 0.0;
	else
		damping = d;
}

//����ʱ����
//��������
void Particle::Integrate(float duration)
{
	CalcExtraForce();
	CalcTotalForce();
	assert(duration > 0.0);
	position += duration * velocity;
	acceleration = totalForce * inverse_mass;
	velocity += acceleration * duration;
}


void Particle::ActiveGravity(bool flag)
{ 
	active_gravity = flag;
	if (flag)
		gravity = 10.0;
	else
		gravity = 0.0;
}
void Particle::ActiveDamping(bool flag)
{ 
	active_damping = flag; 
	if (flag)
		damping = 1.0;
	else
		damping = 0.0;
}
void Particle::CalcExtraForce()
{
	dampingForce = -damping * velocity.length() * velocity.length() * velocity.normalized(); 
	gravityForce = QVector3D(0, -1, 0) * mass * gravity;
}
void Particle::CalcTotalForce()
{
	totalForce = force + dampingForce + gravityForce;
}

bool Particle::HasFiniteMass()
{
	return !mass > 100000000.0;
}