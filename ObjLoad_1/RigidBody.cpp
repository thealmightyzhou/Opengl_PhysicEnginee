#include "rigidbody.h"
using namespace Thealmighty;
void RigidBody::CalcModelMatrix()
{
	modelMatrix = QMatrix4x4();
	modelMatrix.rotate(orientation);
}
QMatrix4x4 RigidBody::GetModelMatrix()
{
	return modelMatrix;
}