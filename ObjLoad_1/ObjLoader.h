#pragma once
#ifndef _OBJLOADER_H
#define _OBJLOADER_H
#include "core.h"
namespace Thealmighty
{
	class ObjLoader
	{
	public:
		bool Load(QString fileName, std::vector<float> &vPoints);

	};
}
#endif // !_OBJLOADER_H
