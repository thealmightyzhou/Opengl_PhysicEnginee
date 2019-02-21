#include "Widget.h"
using namespace Thealmighty;
MyWidget::MyWidget(QWidget *parent, bool fs) :QGLWidget(parent)
{
	qDebug() << "widget struct";

	fullscreen = fs;
	setMouseTracking(true);
	setCursor(Qt::BlankCursor);
	time.start();

}
MyWidget::~MyWidget()
{
	qDebug() << "destroy";
}

void MyWidget::AddObjToScene(QString file)
{
	fileList.push_back(file);
}
void MyWidget::AddObjToScene(Object* obj)
{
	objList.push_back(obj);
}
void MyWidget::AddObjToScene(Object *obj, QMatrix4x4 model)
{
	obj->SetModelMatrix(model);
	objList.push_back(obj);
}
//将obj与renderer绑定生成obj
void MyWidget::BindRenderers()
{
	for (int i = 0; i < fileList.size(); i++)
	{
		GenericRender* renderer = new GenericRender();
		Object* obj = new Object(renderer);
		obj->Init(fileList[i]);
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
	qDebug() <<"filecount:"<< fileList.size();

	BindRenderers();
	objList[0]->Translate(QVector3D(0, 20, 0));


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

	for (int i = 0; i < objList.size(); i++)
	{
		objList[i]->Render(f, camera->GetProjectionMatrix(), camera->GetViewMatrix());
	}
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
}
void MyWidget::mouseMoveEvent(QMouseEvent *event)
{
	float sensityX = 0.1;
	float sensityY = 0.1;
	float dx = event->x() - width/2;
	float dy = event->y() - height/2;
	dx *= sensityX;
	dy *= sensityY;

	camera->SetYaw(camera->GetYaw()- dx );
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

void MyWidget::HandleTime()
{
	newTime = time.msecsSinceStartOfDay();
	deltaTime = 1.0*(newTime - lastTime) / 1000.0;

	timeSum += deltaTime;
	frame++;

	if (timeSum >= 1.0)
	{
		framePerSec = frame;
		frame = 0;
		qDebug() << "fps: " << framePerSec;
		timeSum = 0.0;
	}
	
	lastTime = newTime;
}


