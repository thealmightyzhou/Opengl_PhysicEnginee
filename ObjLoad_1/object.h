#pragma once
#ifndef OBJECT_H
#define OBJECT_H
#include "core.h"
#include "GenericRender.h"
namespace Thealmighty
{
	//¹ÜÀíobject model¾ØÕóºÍrendereräÖÈ¾
	class Object
	{
	public:
		Object(GenericRender *r) { renderer = r; };
		void Render(QOpenGLExtraFunctions *f,const QMatrix4x4 &pMatrix,const QMatrix4x4 &vMatrix)
		{
			renderer->Render(f, pMatrix, vMatrix, modelMatrix);
		}
		void Init(QString file)
		{
			renderer->Init(file);
		}
		void Translate(QVector3D dir) { modelMatrix.translate(dir); };
		void Scale(QVector3D s) { modelMatrix.scale(s); };
		void Rotate(QVector3D r,float angle) { modelMatrix.rotate(angle,r); };
		QMatrix4x4 GetModelMatrix() { return modelMatrix; };
		void SetModelMatrix(QMatrix4x4 m) { modelMatrix = m; };
	private:
		QMatrix4x4 modelMatrix;
		GenericRender *renderer;
		QString objFile;
	};
}
#endif // !OBJECT_H

