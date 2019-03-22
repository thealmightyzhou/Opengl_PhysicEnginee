#include "world.h"
using namespace Thealmighty;

World::World(unsigned iterations)
:firstBody(NULL)
{
	calculateIterations = (iterations == 0);
}

World::~World()
{
}

void World::StartFrame()
{
	BodyRegistration *reg = firstBody;
	while (reg)
	{
		reg->body->ClearAccumulators();
		reg->body->CalcDerivedData();

		reg = reg->next;
	}
}

void World::RunPhysics(float duration)
{
	// First apply the force generators
	//registry.updateForces(duration);

	// Then integrate the objects
	BodyRegistration *reg = firstBody;
	while (reg)
	{
		reg->body->Integrate(duration);

		reg = reg->next;
	}
}