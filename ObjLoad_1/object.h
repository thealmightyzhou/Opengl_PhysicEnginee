#pragma once
#ifndef OBJECT_H
#define OBJECT_H
#include "core.h"
#include "GenericRender.h"
#include "collision_detection_fine.h"
#include "log.h"
#include "forcegenerator.h"
namespace Thealmighty
{
	//����object model�����renderer��Ⱦ
	class Object
	{
	public:
		Object(GenericRender *r,GLuint _id):renderer(r),id(_id){ };
		void Render(QOpenGLExtraFunctions *f, const QMatrix4x4 &pMatrix, const QMatrix4x4 &vMatrix);
		void Init(QString file);
		void Translate(QVector3D dir) { modelMatrix.translate(dir); };
		void Scale(QVector3D s) { modelMatrix.scale(s); };
		void Rotate(QVector3D r,float angle) { modelMatrix.rotate(angle,r); };
		void Rotate(QQuaternion q) { modelMatrix.rotate(q); };
		QMatrix4x4 GetModelMatrix() { return modelMatrix; };
		//���������model����͸����3x4transform����
		void SetModelMatrix(QMatrix4x4 m)
		{
			modelMatrix = m; 
			QMatrix3x4 temp;
			_Mat4x4ToMat3x4(m, temp);
			rb->SetTransform(temp);
		};
		GLuint id = 0;
		QString name = "default";
		void SetName(QString n) { name = n; }
		GLenum mode = GL_RENDER;
		void OnSelectMode()
		{
			mode = GL_SELECT;
		}
		void OnRenderMode()
		{
			mode = GL_RENDER;
		}
		RigidBody* rb;
		CollisionBox* cBox;

		ForceRegistry registry;
		void UpdateRB(float duration);
	private:
		QMatrix4x4 modelMatrix;
		GenericRender *renderer;
		QString objFile;
	};
}
#endif // !OBJECT_H

