#include "object.h"
using namespace Thealmighty;
void Object::Render(QOpenGLExtraFunctions *f, const QMatrix4x4 &pMatrix, const QMatrix4x4 &vMatrix)
{
	renderer->Render(f, pMatrix, vMatrix, modelMatrix, id, mode);
}

void Object::Init(QString file)
{
	renderer->Init(file);
	rb = new RigidBody();

	rb->SetMass(1.0f);
	rb->SetAwake(true);
	rb->SetDamping(1, 1);
	Gravity *g = new Gravity(QVector3D(0, -30, 0));

	registry.Add(rb, g);

	cBox = new CollisionBox(this->rb);
	cBox->halfSize = QVector3D(5, 5, 5);


}

void Object::UpdateRB(float duration)
{
	rb->ClearAccumulators();
	registry.UpdateForce(duration);

	rb->Integrate(duration);

	modelMatrix = rb->GetGLTransform();
}
