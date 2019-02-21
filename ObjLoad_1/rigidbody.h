#pragma once
#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include "core.h"
#include "ObjLoader.h"
namespace Thealmighty
{
	class RigidBody:QObject
	{
		Q_OBJECT
	public:
		float inverseMass;
		QVector3D position;
		QQuaternion orientation;
		QVector3D velocity;
		QVector3D rotation;
		QMatrix4x4 modelMatrix;
		void CalcModelMatrix();
		QMatrix4x4 GetModelMatrix();

		
	};
}
#endif // !RIGIDBODY_H
