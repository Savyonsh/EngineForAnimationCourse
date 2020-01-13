#include "Movable.h"
#include <iostream>

Movable::Movable()
{
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Rx = Eigen::Matrix3f::Identity();
	Ry = Eigen::Matrix3f::Identity();
}

Eigen::Matrix4f Movable::MakeTrans()
{
	Eigen::Matrix4f Rx4f = Eigen::Matrix4f::Identity();
	Rx4f.topLeftCorner(3, 3) = Rx;
	Eigen::Matrix4f Ry4f = Eigen::Matrix4f::Identity();
	Ry4f.topLeftCorner(3, 3) = Ry;
	return Tout.matrix() * Ry4f * Rx4f * Tin.matrix();
}

Eigen::Matrix4d Movable::MakeTransD() {
	return MakeTrans().cast<double>();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	MyTranslate(amt, true);
}

void Movable::MyTranslate(Eigen::Vector3f amt, bool preRotation) {
	if (preRotation) {
		Tout.pretranslate(amt);
	}
	else {
		Tout.translate(amt);
	}
}

void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle) {
	MyRotate(rotAxis, angle, false);
}

// NOT SUPPORTED
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle, bool preTranslation)
{
	if (preTranslation) {
		Tout.prerotate(Eigen::AngleAxisf(angle, rotAxis));
	}
	else {
		Tout.rotate(Eigen::AngleAxisf(angle, rotAxis));
	}
}

void Movable::MyRotateX(float angle) {
	Rx = Rx * Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX()).matrix();
}

void Movable::MyRotateY(float angle) {
	Ry = Ry * Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()).matrix();
}

// NOT SUPPORTED
void Movable::MyScale(Eigen::Vector3f amt)
{
	Tin.scale(amt);
}

// NOT SUPPORTED
void Movable::ScaleInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt) {
	MyScale(mat.block<3, 3>(0, 0).transpose() * amt);
}

// NOT SUPPORTED
void Movable::RotateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f rotAxis, float angle, bool preRotation) {
	MyRotate(mat.block<3, 3>(0, 0).transpose() * rotAxis, angle, preRotation);
}

void Movable::TranslateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt, bool preRotation) {
	MyTranslate(mat.block<3, 3>(0, 0).transpose() * amt, preRotation);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt) {
	Tin.translate(-amt);
	Tout.translate(amt);
}

Eigen::Vector3f Movable::GetCenterOfRotation() {
	return -Tin.translation();
}

Eigen::Matrix3f Movable::getRotation() {
	return MakeTrans().block<3, 3>(0, 0);
}

void Movable::ResetMovable() {
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Rx = Eigen::Matrix3f::Identity();
	Ry = Eigen::Matrix3f::Identity();
}