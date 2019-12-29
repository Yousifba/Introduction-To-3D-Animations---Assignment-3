#include "Movable.h"
#include <iostream>

Movable::Movable()
{
	T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}

Eigen::Matrix4f Movable::MakeTrans()
{
	return T.matrix();
}

Eigen::Transform<float, 3, Eigen::Affine>& Movable::getTrans()
{
	return T;
}

void Movable::Move(Eigen::Vector4f new_coords)
{
	Eigen::Matrix4f prev_transformation = T.matrix();
	T.matrix().col(3) = new_coords;
}

void Movable::MyTranslate(Eigen::Vector3f amt, int axis)
{
	switch (axis) {
		//T = Translation x Rotation x Scale
	case CAMERA_AXIS: 
	{
		Eigen::Matrix4f prev_transformation = T.matrix();
		float tmp[] = { 0, 0, 0, 0,
						0, 0, 0, 0,
						0, 0, 0, 0,
						amt.x(), amt.y(), amt.z(), 0 };
		Eigen::Matrix4f translation_matrix = Eigen::Map<Eigen::Matrix4f>(tmp);
		T = translation_matrix + prev_transformation;

		break;
	}
	case OBJECT_AXIS:
		T.translate(amt);

		break;
	}
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	T.rotate(Eigen::AngleAxisf(angle, rotAxis));
}

void Movable::RotateAround(Eigen::Vector3f point, Eigen::Vector3f rotAxis, float angle)
{
	/*Eigen::Vector3f delta = Eigen::Vector3f(point.x() - T.matrix().col(3).x(),
		point.y() - T.matrix().col(3).y(),
		point.z() - T.matrix().col(3).z());*/
	T.translate(point);
	T.rotate(Eigen::AngleAxisf(angle, rotAxis));
	T.translate(Eigen::Vector3f(-point));
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	T.scale(amt);
}
