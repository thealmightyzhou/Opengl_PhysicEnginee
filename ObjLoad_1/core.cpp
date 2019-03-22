#include "core.h"
using namespace Thealmighty;

float Thealmighty::sleepEpsilon = ((float)0.3);
void Thealmighty::SetSleepEpsilon(float value)
{
	Thealmighty::sleepEpsilon = value;
}
float Thealmighty::GetSleepEpsilon()
{
	return Thealmighty::sleepEpsilon;
}
QVector3D Thealmighty::_transformDirection(const QMatrix3x4 mat, const QVector3D &vector)
{
	return QVector3D(
		vector.x() *  mat.data()[0] +
		vector.y() *  mat.data()[1] +
		vector.z() *  mat.data()[2],

		vector.x() *  mat.data()[4] +
		vector.y() *  mat.data()[5] +
		vector.z() *  mat.data()[6],

		vector.x() *  mat.data()[8] +
		vector.y() *  mat.data()[9] +
		vector.z() *  mat.data()[10]
	);
}

QVector3D Thealmighty::_transformInverseDirection(const QMatrix3x4 mat, const QVector3D &vector)
{
	return QVector3D(
		vector.x() * mat.data()[0] +
		vector.y() * mat.data()[4] +
		vector.z() * mat.data()[8],

		vector.x() * mat.data()[1] +
		vector.y() * mat.data()[5] +
		vector.z() * mat.data()[9],

		vector.x() * mat.data()[2] +
		vector.y() * mat.data()[6] +
		vector.z() * mat.data()[10]
	);
}
