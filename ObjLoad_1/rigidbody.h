#pragma once
#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include "core.h"
namespace Thealmighty
{
	class RigidBody
	{
	public:
		RigidBody();
		float inverseMass;
		QMatrix3x3 inverseInertiaTensor;
		QMatrix3x3 inverseInertiaTensorWorld;

		float linearDamping;
		float angularDamping;

		float motion;
		bool isAwake;
		bool canSleep;

		QVector3D position;
		QQuaternion orientation;
		QVector3D velocity;
		QVector3D rotation;
		QMatrix4x4 modelMatrix;
		QMatrix3x4 transformMatrix;
		QMatrix4x4 GetModelMatrix();
		QVector3D forceAccum;
		QVector3D torqueAccum;
		QVector3D acceleration;
		QVector3D lastFrameAcceleration;
		void Integrate(float duration);
		void SetMass(const float mass);
		float GetMass() const;
		void SetInverseMass(const float mass);
		float GetInverseMass() const;
		bool HasFiniteMass() const;
		void SetInertiaTensor(const QMatrix3x3 &inertiaTensor);
		void GetInertiaTensor(QMatrix3x3 *inertiaTensor) const;
		QMatrix3x3 GetInertiaTensor() const;
		void GetInertiaTensorWorld(QMatrix3x3 *inertiaTensor) const;
		QMatrix3x3 GetInertiaTensorWorld() const;
		void SetInverseInertiaTensor(const QMatrix3x3 &inverseInertiaTensor);
		void GetInverseInertiaTensor(QMatrix3x3 *inverseInertiaTensor) const;
		QMatrix3x3 GetInverseInertiaTensor() const;
		void GetInverseInertiaTensorWorld(QMatrix3x3 *inverseInertiaTensor) const;
		QMatrix3x3 GetInverseInertiaTensorWorld() const;
		void SetDamping(const float linearDamping, const float angularDamping);
		void SetLinearDamping(const float linearDamping);
		float GetLinearDamping() const;
		void SetAngularDamping(const float angularDamping);
		float GetAngularDamping() const;
		void SetPosition(const QVector3D &position);
		void SetPosition(const float x, const float y, const float z);
		void GetPosition(QVector3D *position) const;
		QVector3D GetPosition() const;
		void SetOrientation(const QQuaternion &orientation);
		void SetOrientation(const float r, const float i,const float j, const float k);
		void GetOrientation(QQuaternion *orientation) const;
		QQuaternion GetOrientation() const;
		void GetOrientation(QMatrix3x3 *matrix) const;
		QVector3D GetPointInLocalSpace(const QVector3D &point) const;
		QVector3D GetPointInWorldSpace(const QVector3D &point) const;
		QVector3D GetDirectionInLocalSpace(const QVector3D &direction) const;
		QVector3D GetDirectionInWorldSpace(const QVector3D &direction) const;
		void SetVelocity(const QVector3D &velocity);
		QVector3D GetVelocity() const;
		void AddVelocity(const QVector3D &deltaVelocity);
		void SetRotation(const QVector3D &rotation);
		QVector3D GetRotation() const;
		void AddRotation(const QVector3D &deltaRotation);
		bool GetAwake() const { return isAwake; }
		void SetAwake(const bool awake = true);
		bool GetCanSleep() const { return canSleep; }
		void SetCanSleep(const bool canSleep = true);
		void GetLastFrameAcceleration(QVector3D *linearAcceleration) const;
		QVector3D GetLastFrameAcceleration() const;
		void AddForceAtPoint(const QVector3D &force, const QVector3D &point);
		void AddForceAtBodyPoint(const QVector3D &force, const QVector3D &point);
		void AddTorque(const QVector3D &torque);
		void SetAcceleration(const QVector3D &acceleration);
		QVector3D GetAcceleration() const;
		void CalcDerivedData();
		void AddForce(const QVector3D& force);
		void ClearAccumulators();
		QMatrix3x3 GetOrientation();
		QMatrix3x4 GetTransform() const;
		QMatrix4x4 GetGLTransform() const;
		QVector3D GetForceAccum() const { return forceAccum; }
		void SetTransform(const QMatrix3x4& mat);

	};
}
#endif // !RIGIDBODY_H
