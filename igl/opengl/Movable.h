#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>

class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	Eigen::Matrix4d MakeTransD();
	void MyTranslate(Eigen::Vector3f amt);
	void MyTranslate(Eigen::Vector3f amt, bool preRotation);
	void MyRotate(Eigen::Vector3f rotAxis, float angle);
	void MyRotate(Eigen::Vector3f rotAxis, float angle, bool preTranslation);
	//---------Shaked---------//
	void MyRotateX(float angle);
	void MyRotateY(float angle);
	void ResetMovable();
	//------------------------//
	void MyScale(Eigen::Vector3f amt);
	void ScaleInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt);
	void TranslateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt, bool preRotation);
	void RotateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f rotAxis, float angle, bool preRotation);
	void SetCenterOfRotation(Eigen::Vector3f amt);
	Eigen::Matrix3f getRotation();
	Eigen::Vector3f GetCenterOfRotation();
private:
	Eigen::Transform<float, 3, Eigen::Affine> Tout;
	Eigen::Matrix3f Rx; // Euler Angle - X
	Eigen::Matrix3f Ry; // Euler Angle - Y
	Eigen::Transform<float, 3, Eigen::Affine> Tin;
};