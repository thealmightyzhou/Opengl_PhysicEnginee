#include "Widget.h"
#include <QtWidgets/QApplication>
#include <qmessagebox.h>
int main(int argc, char *argv[])
{
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

	Thealmighty::MyWidget w(0,fs);

	//w.AddObjToScene("Resources/box.obj");
	//w.AddObjToScene("Resources/stickman.obj");
	w.AddObjToScene("Resources/MM.obj");//MikuModel
	
	w.show();
	return a.exec();
}
