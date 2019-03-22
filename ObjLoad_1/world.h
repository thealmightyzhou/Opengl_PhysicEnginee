#pragma once
#ifndef  WORLD_H
#define WORLD_H
#include "rigidbody.h"
namespace Thealmighty
{
	class World
	{
		bool calculateIterations;
		struct BodyRegistration
		{
			RigidBody *body;
			BodyRegistration * next;
		};

		BodyRegistration *firstBody;

	public:
		World(unsigned iterations = 0);
		~World();
		void RunPhysics(float duration);
		void StartFrame();
	};
}


#endif // ! WORLD_H


