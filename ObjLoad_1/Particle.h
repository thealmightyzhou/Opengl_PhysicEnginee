#pragma once
#ifndef PARTICLE_H
#define PARTICLE_H
#include "core.h"
namespace Thealmighty
{
	class Particle:QObject
	{
		Q_OBJECT
	private:
		QVector3D position;
		QVector3D velocity;
		QVector3D acceleration;
		QVector3D force;
		QVector3D totalForce;
		float mass = 1.0;
		float inverse_mass = 1.0;
		bool active_gravity = false;
		float gravity = 10.0f;
		bool active_damping = false;
		float damping = 0.0;
		QVector3D dampingForce;
		QVector3D gravityForce;
		void CalcExtraForce();
		void CalcTotalForce();

	public:
		void SetPosition(QVector3D pos) { position = pos; }
		QVector3D GetPosition() { return position; }
		void ActiveGravity(bool flag);
		void ActiveDamping(bool flag);
		void SetMass(float m);
		float GetMass() { return mass; }
		void SetGravity(float g) { gravity = g; }
		void SetDamping(float d);
		void Integrate(float duration);
		void AddForce(QVector3D f) { force += f; }
		void SetForce(QVector3D f) { force = f; }
		QVector3D GetVelocity() { return velocity; }
		bool HasFiniteMass();
	};
}
#endif // !PARTICLE_H
