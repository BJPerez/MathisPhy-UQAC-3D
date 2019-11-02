#pragma once

#include <valarray>
#include <initializer_list>

namespace physicslib
{
	class Matrix3
	{
	public:
		/*
		 * Default constructor
		 * Create the identity matrix 3x3
		 */
		Matrix3();

		/*
		 * Create a matrix 3x3 filled with the number `fillNumber`
		 */
		explicit Matrix3(double fillNumber);

		/*
		 * Create a matrix 3x3 with an initializer list 
		 * `Matrix3 mat { 0, 1, 2, 3, 4, 5, 6, 7, 8 }`
		 */
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

		std::size_t getSize();
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