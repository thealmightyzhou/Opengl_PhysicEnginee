#ifndef  GENERICRENDER_H
#define GENERICRENDER_H
#include "core.h"
#define MAX_TEXTURE_NUM 10
namespace Thealmighty
{
	class GenericRender:public QObject
	{
		Q_OBJECT
	public:
		void AddTexture(QString file);
		void LoadGLTextures();
		void Init(QString filename);
		void Render(QOpenGLExtraFunctions *f,const QMatrix4x4 &pMatrix, const QMatrix4x4 &vMatrix, const QMatrix4x4& mMatrix);
	private:
		QOpenGLShaderProgram _program;
		QVector<float> _vertPoints;
		GLuint texture[MAX_TEXTURE_NUM];
		std::vector<QString> textureFiles;
	};

}

#endif // ! GENERICRENDER_H
