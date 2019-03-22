#pragma once
#ifndef FORCE_GENERATOR_H
#define FORCE_GENERATOR_H
#include "rigidbody.h"
#include "core.h"
namespace Thealmighty
{
	class ForceGenerator
	{
	public:
		virtual void UpdateForce(RigidBody* body, float duration) = 0;
	};

	class Gravity :public ForceGenerator
	{
		QVector3D gravity;
	public:
		Gravity(const QVector3D &g);
		virtual void UpdateForce(RigidBody* body, float duration);
	};

	class Spring :public ForceGenerator
	{
		QVector3D connectionPoint;
		QVector3D otherConnectPoint;
		RigidBody *other;
		float springConstant;
		float restLength;
	public:
		Spring(const QVector3D &_localConnectionPt, RigidBody* _other, const QVector3D &_otherConnectionPt, float _springConstant, float _restLength);
		virtual void UpdateForce(RigidBody* body, float duration);
	};
	//class Explosion
	//class Aero
	class ForceRegistry
	{
	protected:
		struct ForceRegistration
		{
			RigidBody *body;
			ForceGenerator *fg;
		};

		typedef std::vector<ForceRegistration> Registry;
		Registry registrations;

	public:
		void Add(RigidBody* body, ForceGenerator *fg);
		void Remove(RigidBody *body, ForceGenerator *fg);
		void Clear();
		void UpdateForce(float duration);

	};
}


#endif // !FORCE_GENETATOR_H
