#include "Widget.h"
using namespace Thealmighty;
MyWidget::MyWidget(QWidget *parent, bool fs) :QGLWidget(parent)
{
	qDebug() << "widget struct";

	fullscreen = fs;
	setMouseTracking(true);
	//setCursor(Qt::BlankCursor);
	time.start();
	world = new World();
	scene = new RBMgr();

	
}
MyWidget::~MyWidget()
{
	qDebug() << "destroy";
}

void MyWidget::AddObjToScene(QString file, QString name)
{
	fileList.push_back(file);
	nameList.push_back(name);
	QVector3D temp;
	posList.push_back(temp);
	QQuaternion q;
	rotateList.push_back(q);
}
void MyWidget::AddObjToScene(QString file, QString name, QVector3D pos, QQuaternion q)
{
	fileList.push_back(file);
	nameList.push_back(name);
	posList.push_back(pos);
	rotateList.push_back(q);

}

//将obj与renderer绑定生成obj
void MyWidget::BindRenderers()
{
	for (int i = 0; i < fileList.size(); i++)
	{
		GenericRender* renderer = new GenericRender();
		Object* obj = new Object(renderer, i + 100);
		obj->Init(fileList[i]);
		if (i < nameList.size())
			obj->SetName(nameList[i]);
		if (i < posList.size())
			obj->rb->SetPosition(posList[i]);
		if (i < rotateList.size())
			obj->rb->SetOrientation(rotateList[i]);
		objList.push_back(obj);
	}
}
void MyWidget::initializeGL()
{
	if (fullscreen)
	{
		showFullScreen();
	}
	else
	{
		showNormal();
		setGeometry(0, 0, width, height);
	}
	qDebug() << "filecount:" << fileList.size();

	BindRenderers();
	//objList[0]->Translate(QVector3D(0, 20, 0));

	f = QOpenGLContext::currentContext()->extraFunctions();

	camera = new MyCamera();
	camera->SetCamera(QVector3D(20.0, 0.0, 150.0), QVector3D(0.0, 0.0, -1.0), QVector3D(0.0, 1.0, 0.0));
}

void MyWidget::resizeGL(int w, int h)
{
	if (h == 0)
	{
		h = 1;
	}
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	camera->SetShape(45.0, (float)w / (float)h, 0.1, 10000.0);
}

void MyWidget::paintGL()
{
	time.start();//记录当前时间
	HandleTime();//计算fps,deltaTime

	camera->Translation(0.5);//相机移动
	camera->SetModelViewMatrix();//更新view矩阵

	glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
	f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (pickingObjs)
	{
		Object* pobj = PickObj(lastPos.x(), lastPos.y());
		if (pobj != nullptr)
			qDebug() << pobj->name;
	}
	//Log logger;
	//for (int i = 0; i < objList.size(); i++)
	//{
	//	objList[i]->UpdateRB(GetDeltaTime());
	//	//logger.LogFile(*objList[i]->rb);
	//	objList[i]->Render(f, camera->GetProjectionMatrix(), camera->GetViewMatrix());
	//}

	scene->Update(GetDeltaTime(), objList,f, camera->GetProjectionMatrix(), camera->GetViewMatrix());

	update();
}

void MyWidget::keyPressEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_F2:
		fullscreen = !fullscreen;
		if (fullscreen)
		{
			showFullScreen();
		}
		else
		{
			showNormal();
			setGeometry(0, 0, 640, 480);
		}
		updateGL();
		break;
	case Qt::Key_Escape:
		close();
		break;
	case Qt::Key_A:
		camera->KeyEvent(Direction::left, true);
		break;
	case Qt::Key_D:
		camera->KeyEvent(Direction::right, true);
		break;
	case Qt::Key_W:
		camera->KeyEvent(Direction::forward, true);
		break;
	case Qt::Key_S:
		camera->KeyEvent(Direction::back, true);
		break;
	case Qt::Key_Space:
		Reset();
		break;


	}

}

void MyWidget::keyReleaseEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_A:
		camera->KeyEvent(Direction::left, false);
		break;
	case Qt::Key_D:
		camera->KeyEvent(Direction::right, false);
		break;
	case Qt::Key_W:
		camera->KeyEvent(Direction::forward, false);
		break;
	case Qt::Key_S:
		camera->KeyEvent(Direction::back, false);
		break;


	}
}

void MyWidget::mousePressEvent(QMouseEvent *event)
{
	lastPos = event->pos();
	pickingObjs = true;
}

void MyWidget::mouseMoveEvent(QMouseEvent *event)
{
	float sensityX = 0.1;
	float sensityY = 0.1;
	float dx = event->x() - width / 2;
	float dy = event->y() - height / 2;
	dx *= sensityX;
	dy *= sensityY;

	camera->SetYaw(camera->GetYaw() - dx);
	if (camera->GetYaw() < 0.0f)
		camera->SetYaw(camera->GetYaw() + 360.0f);
	if (camera->GetYaw() > 360.0f)
		camera->SetYaw(camera->GetYaw() - 360.0f);

	camera->SetPitch(camera->GetPitch() - dy);
	if (camera->GetPitch() > 89.0)
		camera->SetPitch(89.0);
	if (camera->GetPitch() < -89.0)
		camera->SetPitch(-89.0);

	camera->Update();
	QCursor::setPos(QPoint(width / 2, height / 2));
}

void MyWidget::mouseReleaseEvent(QMouseEvent *event)
{
	pickingObjs = false;
}

void MyWidget::HandleTime()
{
	newTime = time.msecsSinceStartOfDay();
	deltaTime = 1.0*(newTime - lastTime) / 1000.0;
	if (deltaTime > 1.0f)
		deltaTime = 0.0f;
	timeSum += deltaTime;
	frame++;

	if (timeSum >= 1.0)
	{
		framePerSec = frame;
		frame = 0;
		//qDebug() << "fps: " << framePerSec;
		timeSum = 0.0;

		Log logger;
		for (int i = 0; i < objList.size(); i++)
		{
			logger.LogFile(*objList[i]->rb);
		}
	}

	lastTime = newTime;
}

Object* MyWidget::PickObj(GLint x, GLint y)
{
	QMatrix4x4 tempMat;

	GLuint selectBuff[32] = { 0 };
	GLint hits, viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glSelectBuffer(64, selectBuff);

	glRenderMode(GL_SELECT);

	glInitNames();
	glPushName(0);

	glMatrixMode(GL_PROJECTION);

	glPushMatrix();
	glLoadIdentity();

	float m[16];
	glGetFloatv(GL_PROJECTION_MATRIX, m);

	gluPickMatrix(x, viewport[3] - y, 2, 2, viewport);
	glGetFloatv(GL_PROJECTION_MATRIX, m);

	glOrtho(-10, 10, -10, 10, -camera->farDist, 0);
	glGetFloatv(GL_PROJECTION_MATRIX, m);
	for (int i = 0; i < sizeof(m) / sizeof(m[0]); i++)
	{
		tempMat.data()[i] = m[i];
	}

	for (int i = 0; i < objList.size(); i++)
	{
		objList[i]->OnSelectMode();
		objList[i]->Render(f, tempMat, camera->GetViewMatrix());
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glGetFloatv(GL_PROJECTION_MATRIX, m);
	hits = glRenderMode(GL_RENDER);
	qDebug() << "hits:" << hits;


	for (int i = 0; i < objList.size(); i++)
	{
		objList[i]->OnRenderMode();
	}

	if (hits > 0)
	{
		qDebug() << "id: " << selectBuff[3];
		for (int i = 0; i < objList.size(); i++)
		{
			if (selectBuff[3] == objList[i]->id)
				return objList[i];
		}
	}
	else
		return nullptr;

	return nullptr;

}

void MyWidget::Reset()
{
	for (int i = 0; i < posList.size(); i++)
	{
		objList[i]->rb->SetPosition(posList[i]);
		if (i < rotateList.size())
			objList[i]->rb->SetOrientation(rotateList[i]);

	}
}

RBMgr::RBMgr():resolver(maxContacts*8)
{
	cData.contactArray = contacts;
}

void RBMgr::Update(float duration, std::vector<Object*> objs, QOpenGLExtraFunctions *f, const QMatrix4x4 &pMatrix, const QMatrix4x4 &vMatrix)
{
	UpdateObjects(duration, objs,f, pMatrix, vMatrix);
	GenerateContacts(objs);
	resolver.ResolveContacts(cData.contactArray, cData.contactCount, duration);
}

void RBMgr::UpdateObjects(float duration,std::vector<Object*> objs,QOpenGLExtraFunctions *f, const QMatrix4x4 &pMatrix, const QMatrix4x4 &vMatrix)
{
	Log logger;
	for (int i = 0; i < objs.size(); i++)
	{
		logger.LogFile(*objs[i]->rb);

		objs[i]->UpdateRB(duration);
		objs[i]->Render(f, pMatrix, vMatrix);
		objs[i]->cBox->CalcInternals();
	}
}

void RBMgr::GenerateContacts(std::vector<Object*> objs)
{
	CollisionPlane plane;
	plane.direction = QVector3D(0, 1, 0);
	plane.offset = -100;

	cData.Reset(maxContacts);
	cData.friction = 0.9f;
	cData.restitution = 0.1f;
	cData.tolerance = 0.1f;

	for (int i = 0; i < objs.size(); i++)
	{
		if (!cData.HasMoreContacts())
			return;
		CollisionDetector::BoxAndHalfSpace(*objs[i]->cBox, plane, &cData);

	}

	CollisionDetector::BoxAndBox(*objs[0]->cBox, *objs[1]->cBox, &cData);
}