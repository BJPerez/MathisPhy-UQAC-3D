#pragma once
#include <string>
#include "../include/vector3.hpp"

namespace physicslib
{
	class Quaternion
	{
	private:
		double m_r = 0;
		double m_i = 0;
		double m_j = 0;
		double m_k = 0;
	public:
		Quaternion();
		Quaternion(double r, Vector3 vector);
		Quaternion(double r, double i, double j, double k);
		//Quaternion(Quaternion const& anotherQuaternion);

		Quaternion operator-() const;
		Quaternion& operator *=(double scalar);
		Quaternion& operator *=(const Quaternion& anotherQuaternion);
		Quaternion operator*(double scalar) const;
		Quaternion operator*(const Quaternion& anotherQuaternion) const;

		void doRotation(Vector3 vector);
		void updateAngularVelocity(Vector3 vector, double frametime);
		double getNorm() const;
		double getSquaredNorm() const;
		Quaternion getNormalizedQuaternion() const;

		double getR() const { return m_r; };
		double getI() const { return m_i; };
		double getJ() const { return m_j; };
		double getK() const { return m_k; };

		void setR(double newR) { m_r = newR; };
		void setI(double newI) { m_i = newI; };
		void setJ(double newJ) { m_j = newJ; };
		void setK(double newK) { m_k = newK; };

		std::string toString() const;
	};
}