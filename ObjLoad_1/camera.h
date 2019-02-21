#pragma once
#ifndef CAMERA_H
#define CAMERA_H
#include "core.h"
namespace Thealmighty
{
	enum Direction
	{
		up,
		down,
		left,
		right,
		forward,
		back,
	};

	class MyCamera
	{
	public:
		/* 构造函数和析构函数 */
		MyCamera();
		~MyCamera() {};

		void SetCamera(QVector3D eye, QVector3D look, QVector3D up);
		void SetShape(float viewAngle, float aspect, float Near, float Far);
		void SetModelViewMatrix();
		void Translation(float speed);
		QMatrix4x4 GetViewMatrix();
		QMatrix4x4 GetProjectionMatrix();
		void KeyEvent(int id,bool press);
		float GetPitch();
		float GetYaw();
		void SetPitch(float a);
		void SetYaw(float a);
		void Update();


	private:
		/* 摄像机属性 */
		QVector3D location, forward, side, up;
		float pitchAngle, yawAngle;
		float viewAngle, aspect, nearDist, farDist;
		QMatrix4x4 viewMatrix, projectionMatrix;
		bool keyArr[100];
	};
}
#endif // !CAMERA_H
