#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>

class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	Eigen::Transform<float,3,Eigen::Affine>& getTrans();

	void Move(Eigen::Vector4f coords);
	void MyTranslate(Eigen::Vector3f amt, int axis);
	void MyRotate(Eigen::Vector3f rotAxis,float angle);
	void RotateAround(Eigen::Vector3f point, Eigen::Vector3f rotAxis, float angle);
	void MyScale(Eigen::Vector3f amt);
	enum Axis
	{
		CAMERA_AXIS = 0,
		OBJECT_AXIS = 1
	};
private:
	Eigen::Transform<float,3,Eigen::Affine> T;
};

