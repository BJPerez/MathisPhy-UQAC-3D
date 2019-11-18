#include <iostream>
#include <math.h>
#include "math/quaternion.hpp"
#include "math/vector3.hpp"
namespace physicslib
{
	Quaternion::Quaternion()
		: m_r(1.)
		, m_i(0.)
		, m_j(0.)
		, m_k(0.)
	{
	}

	Quaternion::Quaternion(double r, double i, double j, double k)
		: m_r(r)
		, m_i(i)
		, m_j(j)
		, m_k(k)
	{
	}

	Quaternion::Quaternion(double r, physicslib::Vector3 vector)
		: m_r(r)
		, m_i(vector.getX())
		, m_j(vector.getY())
		, m_k(vector.getZ())
	{
	}

	// ----------------------------------
	// Quaternion mathematical operations
	// ----------------------------------
	Vector3 Vector3::operator-() const
	{
		return Vector3(-m_x, -m_y, -m_z);
	}


	Quaternion Quaternion::operator-() const
	{
		return Quaternion(-m_r, -m_i, -m_j, -m_k);
	}

	Quaternion& Quaternion::operator*=(const Quaternion& anotherQuaternion)
	{
		m_r = (m_r * anotherQuaternion.m_r) - (m_i * anotherQuaternion.m_i) - (m_j * anotherQuaternion.m_j) - (m_k * anotherQuaternion.m_k);
		m_i = (m_r * anotherQuaternion.m_i) + (anotherQuaternion.m_r * m_i) + (m_j * anotherQuaternion.m_k) - (m_k * anotherQuaternion.m_j);
		m_j = (m_r * anotherQuaternion.m_j) + (anotherQuaternion.m_r * m_j) + (m_k * anotherQuaternion.m_i) - (m_i * anotherQuaternion.m_k);
		m_k = (m_r * anotherQuaternion.m_k) + (anotherQuaternion.m_r * m_k) + (m_i * anotherQuaternion.m_j) - (m_j * anotherQuaternion.m_i);

		return *this;
	}

	Quaternion Quaternion::operator*(const Quaternion& anotherQuaternion) const
	{
		Quaternion newQuaternion(*this);
		newQuaternion *= anotherQuaternion;

		return newQuaternion;
	}

	Quaternion& Quaternion::operator+=(const Quaternion& anotherQuaternion)
	{
		m_r += anotherQuaternion.m_r;
		m_i += anotherQuaternion.m_i;
		m_j += anotherQuaternion.m_j;
		m_k += anotherQuaternion.m_k;

		return *this;
	}

	Quaternion Quaternion::operator+(const Quaternion& anotherQuaternion) const
	{
		Quaternion newQuaternion(*this);
		newQuaternion += anotherQuaternion;

		return newQuaternion;
	}

	Quaternion& Quaternion::operator*=(double scalar)
	{
		m_r *= scalar;
		m_i *= scalar;
		m_j *= scalar;
		m_k *= scalar;

		return *this;
	}

	Quaternion operator*(const Quaternion& quaternion, double scalar)
	{
		Quaternion newQuaternion(quaternion);
		newQuaternion *= scalar;

		return newQuaternion;
	}

	Quaternion operator*(double scalar, const Quaternion& quaternion)
	{
		return quaternion * scalar;
	}

	double Quaternion::ScalarProduct(Quaternion const& anotherQuaternion) const
	{
		return (m_r * anotherQuaternion.m_r) + (m_i * anotherQuaternion.m_i) + (m_j * anotherQuaternion.m_j) + (m_k * anotherQuaternion.m_k);
	}

	void Quaternion::rotate(Vector3 vector)
	{
		Quaternion q(0., vector);
		(*this) *= q;
	}

	void Quaternion::updateAngularVelocity(Vector3 vector, double frameTime)
	{
		Quaternion omega(0., vector);
		(*this) += frameTime / 2. * omega * (*this);
	}

	double Quaternion::getNorm() const
	{
		return sqrt(getSquaredNorm());
	}

	double Quaternion::getSquaredNorm() const
	{
		return m_r * m_r + m_i * m_i + m_j * m_j + m_k * m_k;
	}

	void Quaternion::normalize()
	{
		double squaredNorm = getSquaredNorm();
		if (squaredNorm == 0)
		{
			(*this) = Quaternion(1., 0., 0., 0.);
			return;
		}
		
		squaredNorm = 1. / sqrt(squaredNorm);
		(*this) *= squaredNorm;
	}

	Quaternion Quaternion::getNormalizedQuaternion() const
	{
		Quaternion newQuaternion(*this);
		newQuaternion.normalize();

		return newQuaternion;
	}

	std::string Quaternion::toString() const
	{
		return("Quaternion(r = " + std::to_string(m_r) + 
			" ; i = " + std::to_string(m_i) + 
			" ; j = " + std::to_string(m_j) + 
			" ; k = " + std::to_string(m_k) + ")");
	}
}