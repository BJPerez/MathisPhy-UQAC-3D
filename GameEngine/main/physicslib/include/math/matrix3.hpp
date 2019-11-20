#pragma once

#include <valarray>
#include <initializer_list>
#include "math/quaternion.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class Matrix3
	{
	public:
		/**
		 * Default constructor
		 * Create the identity matrix 3x3
		 */
		Matrix3();

		/**
		 * Create a matrix 3x3 filled with the number `fillNumber`
		 */
		explicit Matrix3(double fillNumber);

		/**
		 * Create a matrix 3x3 with an initializer list 
		 * `Matrix3 mat { 0, 1, 2, 3, 4, 5, 6, 7, 8 }`
		 */
		Matrix3(const std::initializer_list<double>& initializerList);

		/**
		 * Create a matrix 3x3 from a quaternion
		 */
		Matrix3(const Quaternion& quaternion);

		/**
		 * Default copy constructor
		 */
		Matrix3(const Matrix3& anotherMatrix) = default;

		/**
		 * Virtual destructor
		 */
		virtual ~Matrix3() = default;

		/**
		 * Default assignment operator
		 */
		Matrix3& operator=(const Matrix3& anotherMatrix) = default;

		/**
		 * Get the determinent of the matrix
		 */
		double getDeterminant() const;

		/**
		 * Reverses the matrix
		 */
		void reverse();

		/**
		 * Return reversed matrix in a new matrix object
		 */
		Matrix3 getReverseMatrix() const;

		/**
		 * Transposes the matrix
		 */
		void transpose();

		/**
		 * Return transposed matrix in a new matrix object
		 */
		Matrix3 getTransposedMatrix() const;

		// Matrix mathematical operations
		Matrix3 operator-() const;
		Matrix3& operator+=(const Matrix3& anotherMatrix);
		Matrix3 operator+(const Matrix3& anotherMatrix) const;
		Matrix3& operator-=(const Matrix3& anotherMatrix);
		Matrix3 operator-(const Matrix3& anotherMatrix) const;
		Matrix3& operator*=(const Matrix3& anotherMatrix);
		Matrix3 operator*(const Matrix3& anotherMatrix) const;

		// Matrix/vector operations
		Vector3 operator*(const Vector3& vector) const;

		// Matrix/scalar operations
		Matrix3& operator+=(const double scalar);
		Matrix3& operator-=(const double scalar);
		Matrix3& operator*=(const double scalar);
		Matrix3& operator/=(const double scalar);

		/**
		 * Setter
		 * `mat(i, j) = 3;`
		 */
		double& operator()(const std::size_t row, const std::size_t column);

		/**
		 * Getter
		 * `double n = mat(i, j);`
		 */
		const double& operator()(const std::size_t row, const std::size_t column) const;

		/**
		 * Get the size of the matrix
		 */
		constexpr std::size_t getSize() const;
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