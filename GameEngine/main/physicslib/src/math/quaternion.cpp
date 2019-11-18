#include <iostream>
#include <math.h>
#include "math/quaternion.hpp"
#include "math/vector3.hpp"
namespace physicslib
{
	Quaternion::Quaternion() {};


	Quaternion::Quaternion(double r, physicslib::Vector3 vector)
	{
		m_r = r;
		m_i = vector.getX();
		m_j = vector.getY();
		m_k = vector.getZ();
	}

	Quaternion::Quaternion(double r, double i, double j, double k)
	{
		m_r = r;
		m_i = i;
		m_j = j;
		m_k = k;
	}


	Quaternion Quaternion::operator-() const
	{
		return Quaternion(-m_r, -m_i, -m_j, -m_k);
	}

	Quaternion& Quaternion::operator *=(double scalar)
	{
		m_r *= scalar;
		m_i *= scalar;
		m_j *= scalar;
		m_k *= scalar;
		return *this;
	}

	Quaternion& Quaternion::operator *=(const Quaternion& anotherQuaternion)
	{	// this might be bugged ??? xD
		double r = m_r * anotherQuaternion.getR() - m_i * anotherQuaternion.getI() - m_j * anotherQuaternion.getJ() - m_k * anotherQuaternion.getK();
		double i = m_r * anotherQuaternion.getI() + m_i * anotherQuaternion.getR() + m_j * anotherQuaternion.getK() - m_k * anotherQuaternion.getJ();
		double j = m_r * anotherQuaternion.getJ() + m_j * anotherQuaternion.getR() + m_k * anotherQuaternion.getI() - m_i * anotherQuaternion.getK();
		double k = m_r * anotherQuaternion.getK() + m_k * anotherQuaternion.getR() + m_i * anotherQuaternion.getJ() - m_j * anotherQuaternion.getI();
		this->m_r = r;
		this->m_i = i;
		this->m_j = j;
		this->m_k = k;
		return *this;
	}

	Quaternion Quaternion::operator*(double scalar) const
	{
		return Quaternion(m_r * scalar, m_i * scalar, m_j * scalar, m_k * scalar);
	}

	Quaternion Quaternion::operator*(const Quaternion& anotherQuaternion) const
	{
		double r = m_r * anotherQuaternion.getR() - m_i * anotherQuaternion.getI() - m_j * anotherQuaternion.getJ() - m_k * anotherQuaternion.getK();
		double i = m_r * anotherQuaternion.getI() + m_i * anotherQuaternion.getR() + m_j * anotherQuaternion.getK() - m_k * anotherQuaternion.getJ();
		double j = m_r * anotherQuaternion.getJ() + m_j * anotherQuaternion.getR() + m_k * anotherQuaternion.getI() - m_i * anotherQuaternion.getK();
		double k = m_r * anotherQuaternion.getK() + m_k * anotherQuaternion.getR() + m_i * anotherQuaternion.getJ() - m_j * anotherQuaternion.getI();
		return Quaternion(r, i, j, k);
	}

	void Quaternion::doRotation(Vector3 vector)
	{
		Quaternion q(0, vector.getX(), vector.getY(), vector.getZ());
		//q = (&this)*q;
		//do something else ? 
	}

	void Quaternion::updateAngularVelocity(Vector3 vector, double frametime)
	{

	}

	double Quaternion::getNorm() const
	{
		return pow(getSquaredNorm(), 0.5);
	}

	double Quaternion::getSquaredNorm() const
	{
		return m_r * m_r + m_i * m_i + m_j * m_j + m_k * m_k;
	}

	Quaternion Quaternion::getNormalizedQuaternion() const
	{
		double squaredNorm = getSquaredNorm();
		if (squaredNorm == 0)
		{
			return Quaternion(1, 0, 0, 0);
		}
		//else
		squaredNorm = 1.0 / pow(squaredNorm, 0.5);
		return Quaternion(m_r * squaredNorm, m_i * squaredNorm, m_j * squaredNorm, m_k * squaredNorm);
	}


	std::string Quaternion::toString() const
	{
		return("Quaternion(r = " + std::to_string(m_r) + 
			" ; i = " + std::to_string(m_i) + 
			" ; j = " + std::to_string(m_j) + 
			" ; k = " + std::to_string(m_k) + ")");
	}


}
