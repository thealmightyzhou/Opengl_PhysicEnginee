#include "ObjLoader.h"
using namespace Thealmighty;

bool ObjLoader::Load(QString fileName, std::vector<float> &vPoints)
{
	if (fileName.mid(fileName.lastIndexOf('.')) != ".obj"&&fileName.mid(fileName.lastIndexOf('.')) != ".OBJ")
	{
		qDebug() << "file is not a obj file!";
		return false;
	}
	QFile objfile(fileName);
	if (!objfile.open(QIODevice::ReadOnly))
	{
		qDebug() << "open" << fileName << "failed";
		return false;
	}
	else
	{
		qDebug() << "open" << fileName << "success!";
	}
	std::vector<float> vertextPoints, texturePoints;
	std::vector<std::tuple< int, int>> facesIndexs;

	while (!objfile.atEnd())
	{
		QByteArray lineData = objfile.readLine();

		lineData = lineData.remove(lineData.count() - 2, 2);

		if (lineData == "")
			continue;
		QList<QByteArray> strValues = lineData.split(' ');
		strValues.removeAll("");
		QString dataType = strValues.takeFirst();
		if (dataType == "v")
		{
			for(int i=0;i<strValues.count();i++)
			{
				if (strValues[i] != "")
					vertextPoints.push_back( strValues[i].toFloat() );
			}

		}
		//else if (dataType == "vt")
		//{
		//	std::transform(strValues.begin(), strValues.end(), std::back_inserter(texturePoints), [](QByteArray &str) {
		//		return str.toFloat();
		//	});

		//}

		else if (dataType == "f")
		{
			if (strValues.size() == 4)
			{
				strValues.push_back(strValues.at(0));
				strValues.push_back(strValues.at(2));

			}
			std::transform(strValues.begin(), strValues.end(), std::back_inserter(facesIndexs), [](QByteArray &str) {
				QList<QByteArray> intStr = str.split('/');

				return std::make_tuple(intStr.first().toInt(), intStr.last().toInt());

			});

		}

	}
	if (vertextPoints.size() != 0)
	{
		qDebug() <<"vertpoints: "<< vertextPoints.size();
	}
	else
	{
		qDebug() << "none vert points";
		return false;
	}

	
	if (facesIndexs.size() != 0)
	{
		qDebug() << "facepoints: "<<facesIndexs.size();
	}
	else
	{
		qDebug() << "none faces";
		return false;
	}

	for (auto &verFaceInfo:facesIndexs)
	{
		int vIndex = std::get<0>(verFaceInfo);

		int vPointSizes = vertextPoints.size() / 3;
		//将顶点坐标放入
		vPoints.push_back(vertextPoints.at(vIndex * 3 - 3));
		vPoints.push_back(vertextPoints.at(vIndex * 3 - 2));
		vPoints.push_back(vertextPoints.at(vIndex * 3 - 1));
	}
	vertextPoints.clear();
	facesIndexs.clear();

	objfile.close();
	return true;
}