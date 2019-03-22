#pragma once
#ifndef WIDGET_H
#define WIDGET_H

#include "core.h"
#include "ObjLoader.h"
#include "camera.h"
#include "genericrender.h"
#include "object.h"
#include "world.h"
#include "contacts.h"
#include "collision_detection_fine.h"
namespace Thealmighty
{
	//场景在此搭建
	class RBMgr
	{
	protected:
		const static unsigned maxContacts = 256;
		Contact contacts[maxContacts];
		CollisionData cData;
		ContactResolver resolver;
		void GenerateContacts(std::vector<Object*> objs);
		void UpdateObjects(float duration,std::vector<Object*> objs, QOpenGLExtraFunctions *f, const QMatrix4x4 &pMatrix, const QMatrix4x4 &vMatrix);
	public:
		RBMgr();
		void Update(float duration,std::vector<Object*> objs, QOpenGLExtraFunctions *f, const QMatrix4x4 &pMatrix, const QMatrix4x4 &vMatrix);
	};

	class MyWidget : public QGLWidget
	{
		Q_OBJECT
	public:
		 MyWidget(QWidget *parent = 0,bool fs = false);
		~MyWidget();
		float GetDeltaTime() { return deltaTime; }
		int GetFps() { return framePerSec; }
		void AddObjToScene(QString file,QString name = "default");
		void AddObjToScene(QString file, QString name, QVector3D pos,QQuaternion q);
		void Reset();

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
		float deltaTime=0;
		std::vector<QString> fileList;
		std::vector<Object*> objList;
		std::vector<QString> nameList;
		std::vector<QVector3D> posList;
		std::vector<QQuaternion> rotateList;


		bool pickingObjs = false;
		World* world;

	protected:
		QOpenGLExtraFunctions *f;
		void initializeGL();
		void resizeGL(int w, int h);
		void paintGL();
		void keyPressEvent(QKeyEvent *e);
		void keyReleaseEvent(QKeyEvent *event);
		void mousePressEvent(QMouseEvent *event);
		void mouseMoveEvent(QMouseEvent *event);
		void mouseReleaseEvent(QMouseEvent *event);
		void BindRenderers();
		Object* PickObj(GLint x, GLint y);

		void HandleTime();

		GLfloat lightAmbient[4] = { 0.5, 0.5, 0.5, 1.0 };
		GLfloat lightDiffuse[4] = { 1.0, 1.0, 1.0, 1.0 };
		GLfloat lightPosition[4] = { 0.0, 0.0, 2.0, 1.0 };

		QPoint lastPos;
		MyCamera* camera;

		const int width=800, height=600;

		RBMgr* scene;

	};
}

#endif // !WIDGET_H
