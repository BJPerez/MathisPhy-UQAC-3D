#pragma once
#include <string>
#include "math/vector3.hpp"

namespace physicslib
{
	class Quaternion
	{
	public:
		/**
		 * Default constructor
		 * Create a quaternion representing no rotation
		 */
		Quaternion();

		/**
		 * Constructor
		 * Create a quaternion from 4 scalars
		 */
		Quaternion(double r, double i, double j, double k);

		/**
		 * Constructor
		 * Create a quaternion from a scalar and a vector3
		 */
		Quaternion(double r, Vector3 vector);

		/**
		 * Default copy constructor
		 */
		Quaternion(Quaternion const& anotherQuaternion) = default;

		/**
		 * Virtual destructor
		 */
		virtual ~Quaternion() = default;

		/**
		 * Default assignment operator
		 */
		Quaternion& operator=(const Quaternion& anotherQuaternion) = default;

		// Quaternion mathematical operations
		Quaternion operator-() const;
		Quaternion& operator*=(const Quaternion& anotherQuaternion);
		Quaternion operator*(const Quaternion& anotherQuaternion) const;
		Quaternion& operator+=(const Quaternion& anotherQuaternion);
		Quaternion operator+(const Quaternion& anotherQuaternion) const;
		Quaternion& operator*=(double scalar);

		double ScalarProduct(Quaternion const& anotherQuaternion) const;

		/**
		 * Do a rotation around the axis represented by vector
		 */
		void rotate(Vector3 vector);

		/**
		 * Update the orientation quaternion by the angular velocity
		 */
		void updateOrientation(Vector3 vector, double frameTime);

		/**
		 * Get the norm of the quaternion
		 */
		double getNorm() const;

		/**
		 * Get the squared norm of the quaternion
		 */
		double getSquaredNorm() const;

		/**
		 * Normalizes the quaternion
		 */
		void normalize();

		/**
		 * Return normalized quaternion in a new quaternion object
		 */
		Quaternion getNormalizedQuaternion() const;

		// Getters
		double getR() const { return m_r; };
		double getI() const { return m_i; };
		double getJ() const { return m_j; };
		double getK() const { return m_k; };

		// Setters
		void setR(double newR) { m_r = newR; };
		void setI(double newI) { m_i = newI; };
		void setJ(double newJ) { m_j = newJ; };
		void setK(double newK) { m_k = newK; };

		/**
		 * Return the string representation of the quaternion
		 */
		std::string toString() const;

	private:
		double m_r = 0;
		double m_i = 0;
		double m_j = 0;
		double m_k = 0;
	};

	// Quaternion/scalar mathematical operations
	Quaternion operator*(const Quaternion& quaternion, double scalar);
	Quaternion operator*(double scalar, const Quaternion& quaternion);
}