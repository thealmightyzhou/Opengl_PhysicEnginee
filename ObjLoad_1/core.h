#pragma once

#ifndef CORE_H
#define CORE_H
#define MAX_FLOAT 8388600

#include <QOpenGLShaderProgram>
#include <QOpenGLExtraFunctions>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QString>
#include <QVector>
#include <QFile>
#include <tuple>
#include <QDebug>
#include <windows.h> 

#include <GL/gl.h>
#include <GL/glu.h>
#include <QtWidgets/QWidget>
#include <QtOpenGL/qgl.h>

#include <QKeyEvent>
#include <qimage.h>
#include <QImageReader>

#include <QtOpenGl>

#include <iostream>
#include <vector>

#include<QObject>
#include<QVector3D>
#include<QVector4D>
#include<QMatrix4x4>
#include<QtMath>

namespace Thealmighty
{
	extern float sleepEpsilon;
	float GetSleepEpsilon();
	void SetSleepEpsilon(float s);
	static inline QVector3D _Vec3CrossProductVec3(QVector3D v1, QVector3D v2)
	{
		return QVector3D(v1.y()*v2.z() - v1.z() * v2.y(),
			v1.z()*v2.x() - v1.x() * v2.z(),
			v1.x()*v2.y() - v1.y() * v2.x());
	}
	QVector3D _transformDirection(const QMatrix3x4 mat, const QVector3D &vector);
	QVector3D _transformInverseDirection(const QMatrix3x4 mat, const QVector3D &vector);
	//matirx3x4 * vector3 用于将本地position转换到世界
	static inline QVector3D _Mat4ProductVec3(const QMatrix3x4& mat, const QVector3D vec)
	{
		return QVector3D(mat.data()[0] * vec.x() + mat.data()[1] * vec.y() + mat.data()[2] * vec.z() + mat.data()[3],
			mat.data()[4] * vec.x() + mat.data()[5] * vec.y() + mat.data()[6] * vec.z() + mat.data()[7],
			mat.data()[8] * vec.x() + mat.data()[9] * vec.y() + mat.data()[10] * vec.z() + mat.data()[11]);
	}
	//用于将世界position转换到本地
	static inline QVector3D _Mat4ProductVec3Inverse(const QMatrix3x4 &mat, const QVector3D &vector)
	{

		QVector3D tmp = vector;
		tmp.setX(tmp.x() - mat.data()[3]);
		tmp.setY(tmp.y() - mat.data()[7]);
		tmp.setZ(tmp.z() - mat.data()[11]);
		return QVector3D(
			tmp.x() * mat.data()[0] +
			tmp.y() * mat.data()[4] +
			tmp.z() * mat.data()[8],

			tmp.x() * mat.data()[1] +
			tmp.y() * mat.data()[5] +
			tmp.z() * mat.data()[9],

			tmp.x() * mat.data()[2] +
			tmp.y() * mat.data()[6] +
			tmp.z() * mat.data()[10]
		);
	}
	//矩阵乘向量 三维
	static inline QVector3D _Mat3ProductVec3(QMatrix3x3 mat, QVector3D vec)
	{
		return QVector3D(mat.data()[0] * vec.x() + mat.data()[1] * vec.y() + mat.data()[2] * vec.z(),
			mat.data()[3] * vec.x() + mat.data()[4] * vec.y() + mat.data()[5] * vec.z(),
			mat.data()[6] * vec.x() + mat.data()[7] * vec.y() + mat.data()[8] * vec.z());
	}
	//四元数加上缩放后的三维向量
	static inline void _QuaAddScaledVec3(QQuaternion &q, QVector3D vec, float scale)
	{
		QQuaternion temp = QQuaternion(0, vec.x()*scale, vec.y()*scale, vec.z()*scale);
		temp *= q;
		q.setX(q.x() + temp.x()*0.5);
		q.setY(q.y() + temp.y()*0.5);
		q.setZ(q.z() + temp.z()*0.5);
		q.setScalar(q.scalar() + temp.scalar()*0.5);
	}
	//三维向量标量积
	static inline float _scalarProductVec3(QVector3D vec1, QVector3D vec2)
	{
		return vec1.x()*vec2.x() + vec1.y()*vec2.y() + vec1.z()*vec2.z();
	}
	//3x3逆矩阵
	static inline QMatrix3x3 _mat3Inverse(const QMatrix3x3& m)
	{
		float t4 = m.data()[0] * m.data()[4];
		float t6 = m.data()[0] * m.data()[5];
		float t8 = m.data()[1] * m.data()[3];
		float t10 = m.data()[2] * m.data()[3];
		float t12 = m.data()[1] * m.data()[6];
		float t14 = m.data()[2] * m.data()[6];

		// Calculate the determinant
		float t16 = (t4*m.data()[8] - t6 * m.data()[7] - t8 * m.data()[8] +
			t10 * m.data()[7] + t12 * m.data()[5] - t14 * m.data()[4]);

		QMatrix3x3 tempMat;
		// Make sure the determinant is non-zero.
		if (t16 == (float)0.0f) return tempMat;
		float t17 = 1 / t16;

		tempMat.data()[0] = (m.data()[4] * m.data()[8] - m.data()[5] * m.data()[7])*t17;
		tempMat.data()[1] = -(m.data()[1] * m.data()[8] - m.data()[2] * m.data()[7])*t17;
		tempMat.data()[2] = (m.data()[1] * m.data()[5] - m.data()[2] * m.data()[4])*t17;
		tempMat.data()[3] = -(m.data()[3] * m.data()[8] - m.data()[5] * m.data()[6])*t17;
		tempMat.data()[4] = (m.data()[0] * m.data()[8] - t14)*t17;
		tempMat.data()[5] = -(t6 - t10)*t17;
		tempMat.data()[6] = (m.data()[3] * m.data()[7] - m.data()[4] * m.data()[6])*t17;
		tempMat.data()[7] = -(m.data()[0] * m.data()[7] - t12)*t17;
		tempMat.data()[8] = (t4 - t8)*t17;
		return tempMat;
	}
	//转换惯性张量从自身坐标系到世界坐标系 matrix3x3 保存在iitWorld中
	//rotmat<-->model矩阵(3x4)，只用3x3
	static inline void _transformInertiaTensor(QMatrix3x3 &iitWorld,
		const QQuaternion &q,
		const QMatrix3x3 &iitBody,
		const QMatrix3x4 &rotmat)
	{
		float t4 = rotmat.data()[0] * iitBody.data()[0] +
			rotmat.data()[1] * iitBody.data()[3] +
			rotmat.data()[2] * iitBody.data()[6];
		float t9 = rotmat.data()[0] * iitBody.data()[1] +
			rotmat.data()[1] * iitBody.data()[4] +
			rotmat.data()[2] * iitBody.data()[7];
		float t14 = rotmat.data()[0] * iitBody.data()[2] +
			rotmat.data()[1] * iitBody.data()[5] +
			rotmat.data()[2] * iitBody.data()[8];
		float t28 = rotmat.data()[4] * iitBody.data()[0] +
			rotmat.data()[5] * iitBody.data()[3] +
			rotmat.data()[6] * iitBody.data()[6];
		float t33 = rotmat.data()[4] * iitBody.data()[1] +
			rotmat.data()[5] * iitBody.data()[4] +
			rotmat.data()[6] * iitBody.data()[7];
		float t38 = rotmat.data()[4] * iitBody.data()[2] +
			rotmat.data()[5] * iitBody.data()[5] +
			rotmat.data()[6] * iitBody.data()[8];
		float t52 = rotmat.data()[8] * iitBody.data()[0] +
			rotmat.data()[9] * iitBody.data()[3] +
			rotmat.data()[10] * iitBody.data()[6];
		float t57 = rotmat.data()[8] * iitBody.data()[1] +
			rotmat.data()[9] * iitBody.data()[4] +
			rotmat.data()[10] * iitBody.data()[7];
		float t62 = rotmat.data()[8] * iitBody.data()[2] +
			rotmat.data()[9] * iitBody.data()[5] +
			rotmat.data()[10] * iitBody.data()[8];

		iitWorld.data()[0] = t4 * rotmat.data()[0] +
			t9 * rotmat.data()[1] +
			t14 * rotmat.data()[2];
		iitWorld.data()[1] = t4 * rotmat.data()[4] +
			t9 * rotmat.data()[5] +
			t14 * rotmat.data()[6];
		iitWorld.data()[2] = t4 * rotmat.data()[8] +
			t9 * rotmat.data()[9] +
			t14 * rotmat.data()[10];
		iitWorld.data()[3] = t28 * rotmat.data()[0] +
			t33 * rotmat.data()[1] +
			t38 * rotmat.data()[2];
		iitWorld.data()[4] = t28 * rotmat.data()[4] +
			t33 * rotmat.data()[5] +
			t38 * rotmat.data()[6];
		iitWorld.data()[5] = t28 * rotmat.data()[8] +
			t33 * rotmat.data()[9] +
			t38 * rotmat.data()[10];
		iitWorld.data()[6] = t52 * rotmat.data()[0] +
			t57 * rotmat.data()[1] +
			t62 * rotmat.data()[2];
		iitWorld.data()[7] = t52 * rotmat.data()[4] +
			t57 * rotmat.data()[5] +
			t62 * rotmat.data()[6];
		iitWorld.data()[8] = t52 * rotmat.data()[8] +
			t57 * rotmat.data()[9] +
			t62 * rotmat.data()[10];
	}

	//计算transformMatrix,最后一列直接放位置数据，前3x3放旋转矩阵
	static inline void _calculateTransformMatrix(QMatrix3x4 &transformMatrix,
		const QVector3D &position,
		const QQuaternion &orientation)
	{
		transformMatrix.data()[0] = 1 - 2 * orientation.y()*orientation.y() -
			2 * orientation.z()*orientation.z();
		transformMatrix.data()[1] = 2 * orientation.x()*orientation.y() -
			2 * orientation.scalar()*orientation.z();
		transformMatrix.data()[2] = 2 * orientation.x()*orientation.z() +
			2 * orientation.scalar()*orientation.y();
		transformMatrix.data()[3] = position.x();

		transformMatrix.data()[4] = 2 * orientation.x()*orientation.y() +
			2 * orientation.scalar()*orientation.z();
		transformMatrix.data()[5] = 1 - 2 * orientation.x()*orientation.x() -
			2 * orientation.z()*orientation.z();
		transformMatrix.data()[6] = 2 * orientation.y()*orientation.z() -
			2 * orientation.scalar()*orientation.x();
		transformMatrix.data()[7] = position.y();

		transformMatrix.data()[8] = 2 * orientation.x()*orientation.z() -
			2 * orientation.scalar()*orientation.y();
		transformMatrix.data()[9] = 2 * orientation.y()*orientation.z() +
			2 * orientation.scalar()*orientation.x();
		transformMatrix.data()[10] = 1 - 2 * orientation.x()*orientation.x() -
			2 * orientation.y()*orientation.y();
		transformMatrix.data()[11] = position.z();
	}

	static inline QVector3D _Mat3TransposeTransformVec3(QMatrix3x3 mat, const QVector3D& vector)
	{
		return QVector3D(
			vector.x() * mat.data()[0] + vector.y() * mat.data()[3] + vector.z() * mat.data()[6],
			vector.x() * mat.data()[1] + vector.y() * mat.data()[4] + vector.z() * mat.data()[7],
			vector.x() * mat.data()[2] + vector.y() * mat.data()[5] + vector.z() * mat.data()[8]
		);
	}

	static inline void _Clear(QVector3D& v)
	{
		v.setX(0.0f);
		v.setY(0.0f);
		v.setZ(0.0f);
	}

	static inline void _Mat3SkewSymmetric(QMatrix3x3 &mat,const QVector3D &vector)
	{
		mat.data()[0] = mat.data()[4] = mat.data()[8] = 0;
		mat.data()[1] = -vector.z();
		mat.data()[2] = vector.y();
		mat.data()[3] = vector.z();
		mat.data()[5] = -vector.x();
		mat.data()[6] = -vector.y();
		mat.data()[7] = vector.x();
	}

	static inline QMatrix3x3 _Mat3ProductMat3(const QMatrix3x3& mat1, const QMatrix3x3 &mat2)
	{
		float t1;
		float t2;
		float t3;
		QMatrix3x3 mat;
		t1 = mat1.data()[0] * mat2.data()[0] + mat1.data()[1] * mat2.data()[3] + mat1.data()[2] * mat2.data()[6];
		t2 = mat1.data()[0] * mat2.data()[1] + mat1.data()[1] * mat2.data()[4] + mat1.data()[2] * mat2.data()[7];
		t3 = mat1.data()[0] * mat2.data()[2] + mat1.data()[1] * mat2.data()[5] + mat1.data()[2] * mat2.data()[8];
		mat.data()[0] = t1;
		mat.data()[1] = t2;
		mat.data()[2] = t3;


		t1 = mat1.data()[3] * mat2.data()[0] + mat1.data()[4] * mat2.data()[3] + mat1.data()[5] * mat2.data()[6];
		t2 = mat1.data()[3] * mat2.data()[1] + mat1.data()[4] * mat2.data()[4] + mat1.data()[5] * mat2.data()[7];
		t3 = mat1.data()[3] * mat2.data()[2] + mat1.data()[4] * mat2.data()[5] + mat1.data()[5] * mat2.data()[8];
		mat.data()[3] = t1;
		mat.data()[4] = t2;
		mat.data()[5] = t3;

		t1 = mat1.data()[6] * mat2.data()[0] + mat1.data()[7] * mat2.data()[3] + mat1.data()[8] * mat2.data()[6];
		t2 = mat1.data()[6] * mat2.data()[1] + mat1.data()[7] * mat2.data()[4] + mat1.data()[8] * mat2.data()[7];
		t3 = mat1.data()[6] * mat2.data()[2] + mat1.data()[7] * mat2.data()[5] + mat1.data()[8] * mat2.data()[8];
		mat.data()[6] = t1;
		mat.data()[7] = t2;
		mat.data()[8] = t3;
		return mat;

	}

	static inline QMatrix3x3 _Mat3Transpose(const QMatrix3x3& mat)
	{
		QMatrix3x3 res;
		res.data()[0] = mat.data()[0];
		res.data()[1] = mat.data()[3];
		res.data()[2] = mat.data()[6];
		res.data()[3] = mat.data()[1];
		res.data()[4] = mat.data()[4];
		res.data()[5] = mat.data()[7];
		res.data()[6] = mat.data()[2];
		res.data()[7] = mat.data()[5];
		res.data()[8] = mat.data()[8];
		return res;
	}

	static inline void _Mat3x3AssignByVec3(QMatrix3x3& mat, const QVector3D& v1, const QVector3D& v2, const QVector3D& v3)
	{
		mat.data()[0] = v1.x();
		mat.data()[1] = v2.x();
		mat.data()[2] = v3.x();
		mat.data()[3] = v1.y();
		mat.data()[4] = v2.y();
		mat.data()[5] = v3.y();
		mat.data()[6] = v1.z();
		mat.data()[7] = v2.z();
		mat.data()[8] = v3.z();
	}

	static inline QMatrix3x4 _Mat3x4ProductMat3x4(const QMatrix3x4& mat1, const QMatrix3x4& mat2)
	{
		QMatrix3x4 result;
		result.data()[0] = (mat2.data()[0] * mat1.data()[0]) + (mat2.data()[4] * mat1.data()[1]) + (mat2.data()[8] * mat1.data()[2]);
		result.data()[4] = (mat2.data()[0] * mat1.data()[4]) + (mat2.data()[4] * mat1.data()[5]) + (mat2.data()[8] * mat1.data()[6]);
		result.data()[8] = (mat2.data()[0] * mat1.data()[8]) + (mat2.data()[4] * mat1.data()[9]) + (mat2.data()[8] * mat1.data()[10]);

		result.data()[1] = (mat2.data()[1] * mat1.data()[0]) + (mat2.data()[5] * mat1.data()[1]) + (mat2.data()[9] * mat1.data()[2]);
		result.data()[5] = (mat2.data()[1] * mat1.data()[4]) + (mat2.data()[5] * mat1.data()[5]) + (mat2.data()[9] * mat1.data()[6]);
		result.data()[9] = (mat2.data()[1] * mat1.data()[8]) + (mat2.data()[5] * mat1.data()[9]) + (mat2.data()[9] * mat1.data()[10]);

		result.data()[2] = (mat2.data()[2] * mat1.data()[0]) + (mat2.data()[6] * mat1.data()[1]) + (mat2.data()[10] * mat1.data()[2]);
		result.data()[6] = (mat2.data()[2] * mat1.data()[4]) + (mat2.data()[6] * mat1.data()[5]) + (mat2.data()[10] * mat1.data()[6]);
		result.data()[10] = (mat2.data()[2] * mat1.data()[8]) + (mat2.data()[6] * mat1.data()[9]) + (mat2.data()[10] * mat1.data()[10]);

		result.data()[3] = (mat2.data()[3] * mat1.data()[0]) + (mat2.data()[7] * mat1.data()[1]) + (mat2.data()[11] * mat1.data()[2]) + mat1.data()[3];
		result.data()[7] = (mat2.data()[3] * mat1.data()[4]) + (mat2.data()[7] * mat1.data()[5]) + (mat2.data()[11] * mat1.data()[6]) + mat1.data()[7];
		result.data()[11] = (mat2.data()[3] * mat1.data()[8]) + (mat2.data()[7] * mat1.data()[9]) + (mat2.data()[11] * mat1.data()[10]) + mat1.data()[11];

		return result;
	}

	static inline void Vec3ComponentProductUpdate(QVector3D& vec1, const QVector3D vec2)
	{
		vec1.setX(vec1.x()*vec2.x());
		vec1.setY(vec1.y()*vec2.y());
		vec1.setZ(vec1.z()*vec2.z());
	}

	//获得Mat3x4某一列的元素，一般前三列存物体轴向，最后一列存物体位置
	static inline QVector3D _Mat3x4GetAxisVector(QMatrix3x4 mat,unsigned int i)
	{
		return QVector3D(mat.data()[i], mat.data()[i + 4], mat.data()[i + 8]);
	}

	//qmat 4x4矩阵转换为cyclone的3x4矩阵
	static inline void _Mat4x4ToMat3x4(const QMatrix4x4& mat4x4,QMatrix3x4& mat3x4)
	{
		mat3x4.data()[0] = mat4x4.data()[0];
		mat3x4.data()[1] = mat4x4.data()[1];
		mat3x4.data()[2] = mat4x4.data()[2];
		mat3x4.data()[3] = mat4x4.data()[12];
		mat3x4.data()[4] = mat4x4.data()[4];
		mat3x4.data()[5] = mat4x4.data()[5];
		mat3x4.data()[6] = mat4x4.data()[6];
		mat3x4.data()[7] = mat4x4.data()[13];
		mat3x4.data()[8] = mat4x4.data()[8];
		mat3x4.data()[9] = mat4x4.data()[9];
		mat3x4.data()[10] = mat4x4.data()[10];
		mat3x4.data()[11] = mat4x4.data()[14];
	}

	//cyclone的3x4矩阵转换为4x4矩阵
	static inline void _Mat3x4ToMat4x4(const QMatrix3x4& mat3x4, QMatrix4x4 &mat4x4)
	{
		mat4x4.data()[0] = mat3x4.data()[0];
		mat4x4.data()[1] = mat3x4.data()[1];
		mat4x4.data()[2] = mat3x4.data()[2];
		mat4x4.data()[3] = 0;
		mat4x4.data()[4] = mat3x4.data()[4];
		mat4x4.data()[5] = mat3x4.data()[5];
		mat4x4.data()[6] = mat3x4.data()[6];
		mat4x4.data()[7] = 0;
		mat4x4.data()[8] = mat3x4.data()[8];
		mat4x4.data()[9] = mat3x4.data()[9];
		mat4x4.data()[10] = mat3x4.data()[10];
		mat4x4.data()[11] = 0;
		mat4x4.data()[12] = mat3x4.data()[3];
		mat4x4.data()[13] = mat3x4.data()[7];
		mat4x4.data()[14] = mat3x4.data()[11];
		mat4x4.data()[15] = 1;

	}

}	

#endif // !CORE_H