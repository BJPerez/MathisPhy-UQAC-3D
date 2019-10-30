#pragma once

#include <valarray>
#include <initializer_list>

namespace physicslib
{
	class Matrix3
	{
	public:
		explicit Matrix3(double fillNumber = 0.);
		Matrix3(const std::initializer_list<double>& initializerList);
		Matrix3(const Matrix3& anotherMatrix) = default;
		virtual ~Matrix3() = default;

		Matrix3& operator=(const Matrix3& anotherMatrix) = default;

		// Matrix operations
		double getDeterminant() const;
		void reverse();
		Matrix3 getReverseMatrix() const;

		// Matrix mathematical operations
		Matrix3& operator+=(const Matrix3& anotherMatrix);
		Matrix3 operator+(const Matrix3& anotherMatrix);
		Matrix3& operator-=(const Matrix3& anotherMatrix);
		Matrix3 operator-(const Matrix3& anotherMatrix);
		Matrix3& operator*=(const Matrix3& anotherMatrix);
		Matrix3 operator*(const Matrix3& anotherMatrix);

		// Matrix/scalar operations
		Matrix3& operator+=(const double scalar);
		Matrix3& operator-=(const double scalar);
		Matrix3& operator*=(const double scalar);
		Matrix3& operator/=(const double scalar);

		// Getters/Setters
		double& operator()(const unsigned int row, const unsigned int column);
		const double& operator()(const unsigned int row, const unsigned int column) const;
	private:
		std::valarray<double> m_data;
	};

	// Matrix/scalar operations
	Matrix3 operator+(const Matrix3& matrix, const double scalar);
	Matrix3 operator+(const double scalar, const Matrix3& matrix);
	Matrix3 operator-(const Matrix3& matrix, const double scalar);
	Matrix3 operator-(const double scalar, const Matrix3& matrix);
	Matrix3 operator*(const Matrix3& matrix, const double scalar);
	Matrix3 operator*(const double scalar, const Matrix3& matrix);
	Matrix3 operator/(const Matrix3& matrix, const double scalar);
	Matrix3 operator/(const double scalar, const Matrix3& matrix);
}