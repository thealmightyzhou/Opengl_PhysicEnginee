#include "GenericRender.h"
#include "ObjLoader.h"
using namespace Thealmighty;
void GenericRender::Init(QString filename)
{

	qDebug()<< _program.addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, "vsrc.vert");
	qDebug() << _program.addCacheableShaderFromSourceFile(QOpenGLShader::Fragment, "fsrc.frag");
	if (_program.link())
	{
		qDebug() << "link success!";
	}
	else
	{
		qDebug() << "link failed";
	}
	ObjLoader objModelLoader;
	objModelLoader.Load(filename, _vertPoints);
	qDebug() << _vertPoints.count();
}

void GenericRender::Render(QOpenGLExtraFunctions *f,const QMatrix4x4 &pMatrix,const QMatrix4x4 &vMatrix,const QMatrix4x4& mMatrix)
{
	f->glEnable(GL_DEPTH_TEST);
	_program.bind();

	_program.setUniformValue("uPMatrix", pMatrix);
	_program.setUniformValue("uVMatrix", vMatrix);
	_program.setUniformValue("uMMatrix", mMatrix);

	GLuint VBO, VAO;
	
	f->glGenVertexArrays(1, &VAO);

	f->glGenBuffers(1, &VBO);
	f->glBindBuffer(GL_ARRAY_BUFFER, VBO);

	f->glBufferData(GL_ARRAY_BUFFER, _vertPoints.size() * sizeof(float), &_vertPoints[0], GL_STATIC_DRAW);
	f->glBindVertexArray(VAO);


	f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GL_FLOAT), (GLvoid*)0);
	f->glEnableVertexAttribArray(0);

	f->glDrawArrays(GL_TRIANGLES, 0, _vertPoints.count()/3);//以三角面绘制
	//f->glDrawArrays(GL_POINTS, 0, _vertPoints.count());//以点绘制

	f->glBindVertexArray(0);
	f->glBindBuffer(GL_ARRAY_BUFFER, 0);


	_program.release();
	f->glDisable(GL_DEPTH_TEST);
}
//void GenericRender::LoadGLTextures()
//{
//	QImage buf, tex;
//
//	if (!buf.load("Resources/img.jpg"))//路径必须用'/'或'\\',根目录在C:\Users\Administrator\source\repos\QtOpengl\Opengltest1\Opengltest1
//	{
//		qWarning("Could not read image file, using single - color instead.");
//		QImage dummy(128, 128, QImage::Format::Format_ARGB32);
//		dummy.fill(Qt::green);
//		buf = dummy;
//	}
//	tex = QGLWidget::convertToGLFormat(buf);
//	glGenTextures(3, &texture[0]);
//	glBindTexture(GL_TEXTURE_2D, texture[0]);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//	glTexImage2D(GL_TEXTURE_2D, 0, 3, tex.width(), tex.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.bits());
//
//	glBindTexture(GL_TEXTURE_2D, texture[1]);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//	glTexImage2D(GL_TEXTURE_2D, 0, 3, tex.width(), tex.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.bits());
//
//	glBindTexture(GL_TEXTURE_2D, texture[2]);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
//}
void GenericRender::AddTexture(QString file)
{
	if (textureFiles.size() >= MAX_TEXTURE_NUM)
		return;
	textureFiles.push_back(file);

}
void GenericRender::LoadGLTextures()
{
	QImage buf, tex;
	std::vector<QString>::iterator it=textureFiles.begin();
	int i = 0;
	for (; it < textureFiles.end(); it++)
	{
		if (!buf.load(*it))
		{
			qWarning("Could not read image file, using single - color instead.");
			QImage dummy(128, 128, QImage::Format::Format_ARGB32);
			dummy.fill(Qt::gray);
			buf = dummy;
		}
		tex = QGLWidget::convertToGLFormat(buf);
		glGenTextures(1, &texture[i]);
		glBindTexture(GL_TEXTURE_2D, texture[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, tex.width(), tex.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.bits());
		i++;
	}
	
	
}