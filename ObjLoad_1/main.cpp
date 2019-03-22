#include "Widget.h"
#include <QtWidgets/QApplication>
#include <qmessagebox.h>
#include "log.h"
using namespace Thealmighty;

int main(int argc, char *argv[])
{
	Log logger;
	logger.Clear("Logs/rbLog.txt");

	bool fs = false;
	QApplication a(argc, argv);
	switch (QMessageBox::information(
		0,
		"isFullScreen",
		"do you want to run in fullscreen mode?",
		QMessageBox::Yes,
		QMessageBox::No|QMessageBox::Default
	))
	{
	case QMessageBox::Yes:
		fs = true;
		break;
	case QMessageBox::No:
		fs = false;
		break;

	}
	qDebug() << fs;

	//qDebug()<<static_cast<int>(QMessageBox::information(0, "asd", "test", QMessageBox::Yes, QMessageBox::No)) ;
	QVector3D vec(1, 2, 3);
	qDebug() << vec.length() << " " << vec.lengthSquared();

	Thealmighty::MyWidget w(0,fs);

	//w.AddObjToScene("Resources/box.obj");
	//w.AddObjToScene("Resources/stickman.obj");

	w.AddObjToScene("Resources/box.obj", "Box", QVector3D(20, 0, 0),QQuaternion::fromAxisAndAngle(QVector3D(1,1,1),0));
	w.AddObjToScene("Resources/box.obj", "Box", QVector3D(20, 20, 0), QQuaternion::fromAxisAndAngle(QVector3D(1, 1, -1), 45));
	w.AddObjToScene("Resources/MM.obj","Miku", QVector3D(0, 0, 0), QQuaternion::fromAxisAndAngle(QVector3D(1,1,1),30));
	//w.AddObjToScene("Resources/sphere.obj", "Sphere");

	w.show();
	return a.exec();


}
