#include "camera.h"
using namespace Thealmighty;

MyCamera::MyCamera() 
{
	for (int i = 0; i < sizeof(keyArr) /sizeof(bool); i++)
	{
		keyArr[i] = false;
	}
	yawAngle = 0.0;
	pitchAngle = 0.0;

}

void MyCamera::SetCamera(QVector3D eye,QVector3D look, QVector3D up1)
{
	/* 构造向量 */
	up = up1;
	up.normalize();

	location = eye;

	forward = look;
	forward.normalize();

	side = QVector3D::crossProduct(forward, up);
	side.normalize();
	SetModelViewMatrix();
}

/* 计算变换后的视点矩阵*/
void MyCamera::SetModelViewMatrix()
{
	viewMatrix = QMatrix4x4();//需要清零
	viewMatrix.lookAt(location, location + forward, up);

}

float MyCamera::GetPitch()
{
	return pitchAngle;
}
float MyCamera::GetYaw()
{
	return yawAngle;
}
void MyCamera::SetPitch(float a)
{
	pitchAngle = a;
}
void MyCamera::SetYaw(float a)
{
	yawAngle = a;
}


/* 摄像机初始化*/
void MyCamera::SetShape(float viewAngle, float aspect, float Near, float Far)
{
	projectionMatrix.perspective(viewAngle, aspect, Near, Far);
}

QMatrix4x4 MyCamera::GetViewMatrix()
{
	return viewMatrix;
}

QMatrix4x4 MyCamera::GetProjectionMatrix()
{
	return projectionMatrix;
}
void MyCamera::KeyEvent(int id,bool press)
{
	keyArr[id] = press;
}
void MyCamera::Translation(float speed)
{
	if (keyArr[Direction::up])
		location += up * speed;
	if (keyArr[Direction::down])
		location -= up * speed;
	if (keyArr[Direction::left])
		location -= side * speed;
	if (keyArr[Direction::right])
		location += side * speed;
	if (keyArr[Direction::forward])
		location += forward * speed;
	if (keyArr[Direction::back])
		location -= forward * speed;

}
void MyCamera::Update()
{
	forward.setX(-sin(qDegreesToRadians(yawAngle))*cos(qDegreesToRadians(pitchAngle)));
	forward.setY(sin(qDegreesToRadians(pitchAngle)));
	forward.setZ(-cos(qDegreesToRadians(yawAngle))*cos(qDegreesToRadians(pitchAngle)));
	forward.normalize();

	side.setX(cos(qDegreesToRadians(yawAngle)));
	side.setY(0);
	side.setZ(-sin(qDegreesToRadians(yawAngle)));
	side.normalize();

	SetModelViewMatrix();

}