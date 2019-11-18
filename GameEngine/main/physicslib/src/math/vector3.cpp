#include <iostream>
#include <math.h>
#include "math/vector3.hpp"
namespace physicslib
{
	Vector3::Vector3()
		: m_x(0.)
		, m_y(0.)
		, m_z(0.)
	{
	}

	Vector3::Vector3(double x, double y, double z)
		: m_x(x)
		, m_y(y)
		, m_z(z)
	{
	}

	// ------------------------------
	// Vector mathematical operations
	// ------------------------------
	Vector3 Vector3::operator-() const
	{
		return Vector3(-m_x, -m_y, -m_z);
	}

	Vector3& Vector3::operator+=(const Vector3& anotherVector) 
	{
		m_x += anotherVector.m_x;
		m_y += anotherVector.m_y;
		m_z += anotherVector.m_z;
		return *this;
	}

	Vector3 Vector3::operator+(const Vector3& anotherVector) const
	{
		Vector3 newVector(*this);
		newVector += anotherVector;

		return newVector;
	}

	Vector3& Vector3::operator-=(const Vector3& anotherVector)
	{
		m_x -= anotherVector.m_x;
		m_y -= anotherVector.m_y;
		m_z -= anotherVector.m_z;
		return *this;
	}

	Vector3 Vector3::operator-(const Vector3& anotherVector) const
	{
		Vector3 newVector(*this);
		newVector -= anotherVector;

		return newVector;
	}

	double Vector3::operator*(const Vector3& anotherVector) const
	{
		return m_x * anotherVector.m_x + m_y * anotherVector.m_y + m_z * anotherVector.m_z;
	}

	Vector3 Vector3::operator^(const Vector3& anotherVector) const
	{
		Vector3 newVector(
			(m_y * anotherVector.m_z) - (m_z * anotherVector.m_y),
			(m_z * anotherVector.m_x) - (m_x * anotherVector.m_z),
			(m_x * anotherVector.m_y) - (m_y * anotherVector.m_x)
		);

		return newVector;
	}

	// -------------------------------------
	// Vector/scalar mathematical operations
	// -------------------------------------
	Vector3& Vector3::operator*=(double scalar)
	{
		m_x *= scalar;
		m_y *= scalar;
		m_z *= scalar;
		return *this;
	}

	Vector3 operator*(const Vector3& vector, double scalar)
	{
		Vector3 newVector(vector);
		newVector *= scalar;

		return newVector;
	}

	Vector3 operator*(double scalar, const Vector3& vector)
	{
		return vector * scalar;
	}

	Vector3& Vector3::operator/=(double scalar)
	{
		m_x /= scalar;
		m_y /= scalar;
		m_z /= scalar;
		return *this;
	}

	Vector3 operator/(const Vector3& vector, double scalar)
	{
		Vector3 newVector(vector);
		newVector /= scalar;

		return newVector;
	}

	Vector3 operator/(double scalar, const Vector3& vector)
	{
		return vector / scalar;
	}

	// ----------------------------------
	// Functional equivalent to operators
	// ----------------------------------
	Vector3 Vector3::VectorAddition(const Vector3& anotherVector) const
	{
		return (*this) + anotherVector;
	}

	Vector3 Vector3::VectorSubtraction(const Vector3& anotherVector) const
	{
		return (*this) - anotherVector;
	}

	Vector3 Vector3::ScalarMultiplication(double scalar) const
	{
		return (*this) * scalar;
	}

	Vector3 Vector3::ScalarDivision(double scalar) const
	{
		return (*this) / scalar;
	}

	double Vector3::ScalarProduct(const Vector3& anotherVector) const
	{
		return (*this) * anotherVector;
	}

	Vector3 Vector3::CrossProduct(const Vector3& anotherVector) const
	{
		return (*this) ^ anotherVector;
	}

	Vector3 Vector3::ComponentProduct(const Vector3& anotherVector) const
	{
		Vector3 newVector(
			m_x * anotherVector.m_x,
			m_y * anotherVector.m_y,
			m_z * anotherVector.m_z
		);

		return newVector;
	}

	double Vector3::getNorm() const
	{
		return pow(getSquaredNorm(), 0.5);
	}

	double Vector3::getSquaredNorm() const
	{
		return (m_x * m_x + m_y * m_y + m_z * m_z);
	}

	void Vector3::normalize()
	{
		double norm = getNorm();
		if (norm != 0)
		{
			(*this) /= norm;
		}
	}

	Vector3 Vector3::getNormalizedVector() const
	{
		Vector3 newVector(*this);
		newVector.normalize();

		return newVector;
	}

	std::string Vector3::toString() const
	{
		return("Vector3(x = " + std::to_string(m_x) + " ; y = " + std::to_string(m_y) + " ; z = " + std::to_string(m_z) + ")");
	}

	
}

