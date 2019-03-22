#pragma once
#ifndef LOG_H
#define LOG_H
#include <QFile>
#include <QTextStream>
#include "rigidbody.h"
const QString line = "\r\n";
namespace Thealmighty
{
	/*
	*log日志但是加上 性能会受到很大影响
	*/
	class Log
	{
	public:
		void Clear(QString str)
		{
			QFile file(str);
			file.open(QFile::WriteOnly | QFile::Truncate);
			file.close();
		}
		void LogLine(QString str)
		{
			qDebug() << str;
		}
		void LogFile(QString txtMsg)
		{
			QFile file("Logs/logMsg.txt");
			if (file.open(QIODevice::WriteOnly | QIODevice::Append))
			{
				QTextStream textStream(&file);
				textStream << txtMsg << endl;
			}
			else
				qDebug() << "log file open fail!";
			file.close();
		}
		void LogFile(QString filename,QString txtMsg)
		{
			QFile file("Logs/" + filename);
		
			if (file.open(QIODevice::WriteOnly | QIODevice::Append))
			{
				QTextStream textStream(&file);
				textStream << txtMsg << endl;
			}
			else
				qDebug() << "log file open fail!";
			file.close();
		}
		 QString ToString(float val)
		{
			return QString("%1").arg(val);
		}
		 QString ToString(QVector3D vec)
		{
			return ("(" + ToString(vec.x()) + "," + ToString(vec.y()) + "," + ToString(vec.z())+ ")");
		}
		 QString ToString(QQuaternion q)
		{
			QString str;
			str.append("(");
			str.append(ToString(q.x())+",");
			str.append(ToString(q.y()) + ",");
			str.append(ToString(q.z()) + ",");
			str.append(ToString(q.scalar()) + ")");
			return str;
		}
		 QString ToString(QMatrix4x4 mat)
		{
			QString str;
			str.append("(");
			str.append(ToString(mat.data()[0]));
			for (int i = 1; i < 16; i++)
			{
				str.append("," + ToString(mat.data()[i]));
			}
			str.append(")");
			return str;
		}
		 QString ToString(QMatrix3x4 mat)
		{
			QString str;
			str.append("(");
			for (int i = 0; i < 12; i++)
			{
				str.append(ToString(mat.data()[i]) + ",");
			}
			str.append(")");
			return str;
		}
		 void LogFile(const RigidBody& rb)
		{
			QString msg;
			msg.append("mass: " + ToString(rb.GetMass())+line);
			msg.append("inverseMass: " + ToString(rb.GetInverseMass())+ line);
			msg.append("position: " + ToString(rb.GetPosition())+line);
			msg.append("orientation: " + ToString(rb.GetOrientation())+line);
			msg.append("acceleration: " + ToString(rb.GetAcceleration()) + line);
			msg.append("GLmodelMat: " + ToString(rb.GetGLTransform()) + line);
			msg.append("transformMat: " + ToString(rb.GetTransform()) + line);
			msg.append("forceAccum: " + ToString(rb.GetForceAccum()) + line);
			msg.append(line);
			LogFile("rbLog.txt", msg);
		}

	};	
}
#endif // !LOG_H
