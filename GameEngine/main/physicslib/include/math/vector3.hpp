#pragma once

#include <string>

namespace physicslib
{
	class Matrix3;

	class Vector3
	{
	public :
		/**
		 * Default constructor
		 * Create a null vector3 (0, 0, 0)
		 */
		Vector3();

		/**
		 * Constructor
		 * Create a vector from 3 scalars
		 */
		Vector3(double x, double y, double z);

		/**
		 * Default copy constructor
		 */
		Vector3(Vector3 const& anotherVector) = default;

		/**
		 * Virtual destructor
		 */
		virtual ~Vector3() = default;

		/**
		 * Default assignment operator
		 */
		Vector3& operator=(const Vector3& anotherVector) = default;

		// Vector mathematical operations
		Vector3 operator-() const;
		Vector3& operator+=(const Vector3& anotherVector);
		Vector3 operator+(const Vector3& anotherVector) const;
		Vector3& operator-=(const Vector3& anotherVector);
		Vector3 operator-(const Vector3& anotherVector) const;
		double operator*(const Vector3& anotherVector) const;
		Vector3 operator^(const Vector3& anotherVector) const;

		// Vector/scalar mathematical operations
		Vector3& operator*=(double scalar);
		Vector3& operator/=(double scalar);

		// Functional equivalent to operators
		Vector3 VectorAddition(const Vector3& anotherVectorv) const;
		Vector3 VectorSubtraction(const Vector3& anotherVector) const;
		Vector3 ScalarMultiplication(double scalar) const;
		Vector3 ScalarDivision(double scalar) const;
		double ScalarProduct(const Vector3& anotherVector)const;
		Vector3 CrossProduct(const Vector3& anotherVector) const;
		Vector3 ComponentProduct(const Vector3& anotherVector) const;

		/**
		 * Get the norm of the vector
		 */
		double getNorm() const;

		/**
		 * Get the squared norm of the vector
		 */
		double getSquaredNorm() const;

		/**
		 * Normalizes the vector
		 * The vector is the same direction but with a norm equal to 1
		 */
		void normalize();

		/**
		 * Return normalized vector in a new vector object
		 */
		Vector3 getNormalizedVector() const;

		/**
		 * Return a new vector in world coordinates
		 */
		Vector3 localToWorld(const Matrix3& transformMatrix) const;

		/**
		 * Return a new vector in local coordinates
		 */
		Vector3 worldToLocal(const Matrix3& transformMatrix) const;

		// Getters
		double getX() const { return m_x; };
		double getY() const { return m_y; };
		double getZ() const { return m_z; };

		// Setters
		void setX(double newX) { m_x = newX; };
		void setY(double newY) { m_y = newY; };
		void setZ(double newZ) { m_z = newZ; };

		/**
		 * Return the string representation of the vector
		 */
		std::string toString() const;

	private:
		double m_x = 0;
		double m_y = 0;
		double m_z = 0;
	};

	// Vector/scalar mathematical operations
	Vector3 operator*(const Vector3& vector, double scalar);
	Vector3 operator*(double scalar, const Vector3& vector);
	Vector3 operator/(const Vector3& vector, double scalar);
	Vector3 operator/(double scalar, const Vector3& vector);
}