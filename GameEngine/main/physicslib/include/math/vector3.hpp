#pragma once
#include <string>

namespace physicslib
{
	class Vector3
	{
	private :
		double m_x = 0;
		double m_y = 0;
		double m_z = 0;
	public :
		Vector3();
		Vector3(double x, double y, double z);
		Vector3(Vector3 const& anotherVector);

		Vector3& operator+=(const Vector3& anotherVector);
		Vector3& operator-=(const Vector3& anotherVector);
		Vector3 operator+(const Vector3& anotherVector) const;
		Vector3 operator-(const Vector3& anotherVector) const;
		Vector3 operator*(double scalar) const;
		double operator*(const Vector3& anotherVector) const;
		Vector3 operator/(double scalar) const;
		Vector3 operator^(const Vector3& anotherVector) const;
		Vector3 operator-() const;

		Vector3 VectorAddition(Vector3 const& anotherVectorv) const;
		Vector3 VectorSubtraction(Vector3 const& anotherVector) const;
		Vector3 ScalarDivision(double scalar) const;
		Vector3 ScalarMultiplication(double scalar) const;
		Vector3 ComponentProduct(Vector3 const& anotherVector) const;
		double ScalarProduct(Vector3 const& anotherVector)const;
		Vector3 CrossProduct(Vector3 const& anotherVector) const;

		std::string toString() const;

		double getX() const { return m_x; };
		double getY() const { return m_y; };
		double getZ() const { return m_z; };
		double getNorm() const;
		double getSquaredNorm() const;
		Vector3 getNormalizedVector() const;

		void setX(double newX) { m_x = newX; };
		void setY(double newY) { m_y = newY; };
		void setZ(double newZ) { m_z = newZ; };
	};
}