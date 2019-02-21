#pragma once
#ifndef WIDGET_H
#define WIDGET_H

#include "core.h"
#include "ObjLoader.h"
#include "camera.h"
#include "genericrender.h"
#include "object.h"
namespace Thealmighty
{
	class MyWidget : public QGLWidget
	{
		Q_OBJECT
	public:
		 MyWidget(QWidget *parent = 0,bool fs = false);
		~MyWidget();
		float GetDeltaTime() { return deltaTime; }
		int GetFps() { return framePerSec; }
		void AddObjToScene(QString file);
		void AddObjToScene(Object *obj);
		void AddObjToScene(Object *obj,QMatrix4x4 modelMat);

	private:
		ObjLoader _objLoader;
		QVector<float> _vertPoints;
		QVector3D _cameraLocation, _lightLocation;

		bool fullscreen;

		QOpenGLShaderProgram _program;
		GLuint VBO, VAO, EBO;
		QMatrix4x4 pMatrix, vMatrix, mMatrix;

		QTime time;
		float newTime=0;
		float lastTime=0;
		int framePerSec=0;
		int frame=0;
		float timeSum=0;
		float deltaTime;
		std::vector<QString> fileList;
		std::vector<Object*> objList;


	protected:
		QOpenGLExtraFunctions *f;
		void initializeGL();
		void resizeGL(int w, int h);
		void paintGL();
		void keyPressEvent(QKeyEvent *e);
		void keyReleaseEvent(QKeyEvent *event);
		void mousePressEvent(QMouseEvent *event);
		void mouseMoveEvent(QMouseEvent *event);
		void BindRenderers();

		void HandleTime();

		GLfloat lightAmbient[4] = { 0.5, 0.5, 0.5, 1.0 };
		GLfloat lightDiffuse[4] = { 1.0, 1.0, 1.0, 1.0 };
		GLfloat lightPosition[4] = { 0.0, 0.0, 2.0, 1.0 };

		QPoint lastPos;
		MyCamera* camera;

		const int width=800, height=600;


	};
}

#endif // !WIDGET_H
