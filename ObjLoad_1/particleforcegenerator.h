#pragma once
#ifndef PARTICLE_FORCE_GENERATOR_H
#define PARTICLE_FORCE_GENERATOR_H
#include "particle.h"
#include "core.h"
namespace Thealmighty
{
	class ParticleForceGenerator
	{
	public:
		virtual void UpdateForce(Particle* particle, float duration) = 0;
	};

	class ParticleForceRegistry
	{
	protected:
		struct ParticleForceRegistration//粒子与力发生器的关系
		{
			Particle* particle;
			ParticleForceGenerator *fg;
		};
		typedef std::vector<ParticleForceRegistration> Registry;//注册表
		Registry registrations;
	public:
		void Add(Particle* p, ParticleForceGenerator* fg);
		void Remove(Particle* p, ParticleForceGenerator* fg);
		void Clear();
		void UpdateForces(float duration);

	};

	class ParticleGravity :public ParticleForceGenerator
	{
		QVector3D gravity;
	public:
		ParticleGravity(const QVector3D& g) { gravity = g; }
		virtual void UpdateForce(Particle* p, float duration);

	};

	class ParticleDrag :public ParticleForceGenerator
	{
		float k1, k2;
	public:
		ParticleDrag(float k1, float k2) { this->k1 = k1; this->k2 = k2; }
		virtual void UpdateForce(Particle *p, float duration);

	};

	class ParticleSpring :public ParticleForceGenerator
	{
		Particle *other;
		float springConstant;
		float restLength;
	public:
		ParticleSpring(Particle *other, float springConstant, float restLegth)
		{
			this->other = other;
			this->springConstant = springConstant;
			this->restLength = restLegth;
		}
		virtual void UpdateForce(Particle *p, float duration);

	};

}
#endif // !PARTICLE_FORCE_GENERATOR_H
