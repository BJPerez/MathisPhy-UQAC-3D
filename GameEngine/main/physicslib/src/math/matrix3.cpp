#include "math/matrix3.hpp"

#include <iostream>
#include <algorithm>

namespace physicslib
{
	Matrix3::Matrix3()
		: m_data({
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		})
	{
	}

	Matrix3::Matrix3(double fillNumber)
	{
		m_data.fill(fillNumber);
	}

	Matrix3::Matrix3(const std::initializer_list<double>& initializerList)
	{
		std::copy(initializerList.begin(), initializerList.begin() + 9, m_data.begin());
	}

	Matrix3::Matrix3(const Quaternion& quaternion)
		: m_data({
			1 - (2 * quaternion.getJ() * quaternion.getJ() + 2 * quaternion.getK() * quaternion.getK()),
			2 * quaternion.getI() * quaternion.getJ() + 2 * quaternion.getK() * quaternion.getR(),
			2 * quaternion.getI() * quaternion.getK() - 2 * quaternion.getJ() * quaternion.getR(),
			2 * quaternion.getI() * quaternion.getJ() - 2 * quaternion.getK() * quaternion.getR(),
			1 - (2 * quaternion.getI() * quaternion.getI() + 2 * quaternion.getK() * quaternion.getK()),
			2 * quaternion.getJ() * quaternion.getK() + 2 * quaternion.getI() * quaternion.getR(),
			2 * quaternion.getI() * quaternion.getK() + 2 * quaternion.getJ() * quaternion.getR(),
			2 * quaternion.getJ() * quaternion.getK() - 2 * quaternion.getI() * quaternion.getR(),
			1 - (2 * quaternion.getI() * quaternion.getI() + 2 * quaternion.getJ() * quaternion.getJ()),
		})
	{
	}

	// -----------------
	// Matrix operations
	// -----------------
	double Matrix3::getDeterminant() const
	{
		return (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 2)
			 + (*this)(1, 0) * (*this)(2, 1) * (*this)(0, 2)
			 + (*this)(2, 0) * (*this)(0, 1) * (*this)(1, 2)
			 - (*this)(0, 0) * (*this)(2, 1) * (*this)(1, 2)
			 - (*this)(2, 0) * (*this)(1, 1) * (*this)(0, 2)
			 - (*this)(1, 0) * (*this)(0, 1) * (*this)(2, 2);
	}

	void Matrix3::reverse()
	{
		double determinant = getDeterminant();
		if (determinant == 0.)
		{
			return;
		}

		/*if (determinant == 1. || determinant == -1.)
		{
			transpose();
			return;
		}*/

		*this = {
			(*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1),
			(*this)(0, 2) * (*this)(2, 1) - (*this)(0, 1) * (*this)(2, 2),
			(*this)(0, 1) * (*this)(1, 2) - (*this)(0, 2) * (*this)(1, 1),

			(*this)(1, 2) * (*this)(2, 0) - (*this)(1, 0) * (*this)(2, 2),
			(*this)(0, 0) * (*this)(2, 2) - (*this)(0, 2) * (*this)(2, 0),
			(*this)(0, 2) * (*this)(1, 0) - (*this)(0, 0) * (*this)(1, 2),

			(*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0),
			(*this)(0, 1) * (*this)(2, 0) - (*this)(0, 0) * (*this)(2, 1),
			(*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0),
		};

		*this /= determinant;
	}

	Matrix3 Matrix3::getReverseMatrix() const
	{
		Matrix3 newMatrix(*this);
		newMatrix.reverse();

		return newMatrix;
	}

	void Matrix3::transpose()
	{
		*this = {
			(*this)(0, 0), (*this)(1, 0), (*this)(2, 0),
			(*this)(0, 1), (*this)(1, 1), (*this)(2, 1),
			(*this)(0, 2), (*this)(1, 2), (*this)(2, 2),
		};
	}

	Matrix3 Matrix3::getTransposedMatrix() const
	{
		Matrix3 newMatrix(*this);
		newMatrix.transpose();

		return newMatrix;
	}

	// ------------------------------
	// Matrix mathematical operations
	// ------------------------------
	Matrix3 Matrix3::operator-() const
	{
		Matrix3 newMatrix(*this);
		newMatrix *= -1.;

		return newMatrix;
	}

	Matrix3& Matrix3::operator+=(const Matrix3& anotherMatrix)
	{
		// Term-term addition
		std::transform(m_data.begin(), m_data.end(), anotherMatrix.m_data.begin(), m_data.begin(), std::plus<double>());

		return *this;
	}

	Matrix3 Matrix3::operator+(const Matrix3& anotherMatrix) const
	{
		Matrix3 newMatrix(*this);
		newMatrix += anotherMatrix;

		return newMatrix;
	}

	Matrix3& Matrix3::operator-=(const Matrix3& anotherMatrix)
	{
		// Term-term substraction
		std::transform(m_data.begin(), m_data.end(), anotherMatrix.m_data.begin(), m_data.begin(), std::minus<double>());

		return *this;
	}

	Matrix3 Matrix3::operator-(const Matrix3& anotherMatrix) const
	{
		Matrix3 newMatrix(*this);
		newMatrix -= anotherMatrix;

		return newMatrix;
	}

	Matrix3& Matrix3::operator*=(const Matrix3& anotherMatrix)
	{
		*this = {
			(*this)(0, 0) * anotherMatrix(0, 0) + (*this)(0, 1) * anotherMatrix(1, 0) + (*this)(0, 2) * anotherMatrix(2, 0),
			(*this)(0, 0) * anotherMatrix(0, 1) + (*this)(0, 1) * anotherMatrix(1, 1) + (*this)(0, 2) * anotherMatrix(2, 1),
			(*this)(0, 0) * anotherMatrix(0, 2) + (*this)(0, 1) * anotherMatrix(1, 2) + (*this)(0, 2) * anotherMatrix(2, 2),

			(*this)(1, 0) * anotherMatrix(0, 0) + (*this)(1, 1) * anotherMatrix(1, 0) + (*this)(1, 2) * anotherMatrix(2, 0),
			(*this)(1, 0) * anotherMatrix(0, 1) + (*this)(1, 1) * anotherMatrix(1, 1) + (*this)(1, 2) * anotherMatrix(2, 1),
			(*this)(1, 0) * anotherMatrix(0, 2) + (*this)(1, 1) * anotherMatrix(1, 2) + (*this)(1, 2) * anotherMatrix(2, 2),

			(*this)(2, 0) * anotherMatrix(0, 0) + (*this)(2, 1) * anotherMatrix(1, 0) + (*this)(2, 2) * anotherMatrix(2, 0),
			(*this)(2, 0) * anotherMatrix(0, 1) + (*this)(2, 1) * anotherMatrix(1, 1) + (*this)(2, 2) * anotherMatrix(2, 1),
			(*this)(2, 0) * anotherMatrix(0, 2) + (*this)(2, 1) * anotherMatrix(1, 2) + (*this)(2, 2) * anotherMatrix(2, 2),
		};

		return *this;
	}

	Matrix3 Matrix3::operator*(const Matrix3& anotherMatrix) const
	{
		Matrix3 newMatrix(*this);
		newMatrix *= anotherMatrix;

		return newMatrix;
	}

	// ------------------------
	// Matrix/vector operations
	// ------------------------
	Vector3 Matrix3::operator*(const Vector3& vector) const
	{
		Vector3 newVector(
			m_data[0] * vector.getX() + m_data[1] * vector.getY() + m_data[2] * vector.getZ(),
			m_data[3] * vector.getX() + m_data[4] * vector.getY() + m_data[5] * vector.getZ(),
			m_data[6] * vector.getX() + m_data[7] * vector.getY() + m_data[8] * vector.getZ()
		);

		return newVector;
	}

	// ------------------------
	// Matrix/scalar operations
	// ------------------------
	Matrix3& Matrix3::operator+=(const double scalar)
	{
		// m_data[i] += scalar
		std::transform(m_data.begin(), m_data.end(), m_data.begin(), [scalar](const double n) { return n + scalar; });

		return *this;
	}

	Matrix3& Matrix3::operator-=(const double scalar)
	{
		// m_data[i] -= scalar
		std::transform(m_data.begin(), m_data.end(), m_data.begin(), [scalar](const double n) { return n - scalar; });

		return *this;
	}

	Matrix3& Matrix3::operator*=(const double scalar)
	{
		// m_data[i] *= scalar
		std::transform(m_data.begin(), m_data.end(), m_data.begin(), [scalar](const double n) { return n * scalar; });

		return *this;
	}

	Matrix3& Matrix3::operator/=(const double scalar)
	{
		// m_data[i] /= scalar
		std::transform(m_data.begin(), m_data.end(), m_data.begin(), [scalar](const double n) { return n / scalar; });

		return *this;
	}

	Matrix3 operator+(const Matrix3& matrix, const double scalar)
	{
		Matrix3 newMatrix(matrix);
		newMatrix += scalar;

		return newMatrix;
	}

	Matrix3 operator+(const double scalar, const Matrix3& matrix)
	{
		return matrix + scalar;
	}

	Matrix3 operator-(const Matrix3& matrix, const double scalar)
	{
		Matrix3 newMatrix(matrix);
		newMatrix -= scalar;

		return newMatrix;
	}

	Matrix3 operator-(const double scalar, const Matrix3& matrix)
	{
		return matrix - scalar;
	}

	Matrix3 operator*(const Matrix3& matrix, const double scalar)
	{
		Matrix3 newMatrix(matrix);
		newMatrix *= scalar;

		return newMatrix;
	}

	Matrix3 operator*(const double scalar, const Matrix3& matrix)
	{
		return matrix * scalar;
	}

	Matrix3 operator/(const Matrix3& matrix, const double scalar)
	{
		Matrix3 newMatrix(matrix);
		newMatrix /= scalar;

		return newMatrix;
	}

	Matrix3 operator/(const double scalar, const Matrix3& matrix)
	{
		return matrix / scalar;
	}

	// ---------------
	// Getters/Setters
	// ---------------
	double& Matrix3::operator()(const std::size_t row, const std::size_t column)
	{
		return m_data[3 * row + column];
	}

	const double& Matrix3::operator()(const std::size_t row, const std::size_t column) const
	{
		return m_data[3 * row + column];
	}

	constexpr std::size_t Matrix3::getSize() const
	{
		return 3;
	}

	std::string Matrix3::toString() const
	{
		std::string str = "{ ";
		for (double n : m_data)
		{
			str += std::to_string(n);
			str += " ";
		}
		str += "}";

		return str;
	}
}