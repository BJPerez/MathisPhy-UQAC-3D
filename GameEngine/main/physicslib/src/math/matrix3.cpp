#include "math/matrix3.hpp"

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
		: m_data(fillNumber, 9)
	{
	}

	Matrix3::Matrix3(const std::initializer_list<double>& initializerList)
		: m_data(initializerList)
	{
	}

	// -----------------
	// Matrix operations
	// -----------------
	double Matrix3::getDeterminant() const
	{
		return (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 2) + (*this)(1, 0) * (*this)(2, 1) * (*this)(0, 2) + (*this)(2, 0) * (*this)(0, 1) * (*this)(1, 2)
			 - (*this)(0, 0) * (*this)(2, 1) * (*this)(1, 2) - (*this)(2, 0) * (*this)(1, 1) * (*this)(0, 2) - (*this)(1, 0) * (*this)(0, 1) * (*this)(2, 2);
	}

	void Matrix3::reverse()
	{
		double determinant = getDeterminant();
		if (determinant == 0.)
		{
			return;
		}

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

	// ------------------------------
	// Matrix mathematical operations
	// ------------------------------
	Matrix3 Matrix3::operator-() const
	{
		Matrix3 newMatrix(*this);
		newMatrix *= 1.;

		return newMatrix;
	}

	Matrix3& Matrix3::operator+=(const Matrix3& anotherMatrix)
	{
		m_data += anotherMatrix.m_data;

		return *this;
	}

	Matrix3 Matrix3::operator+(const Matrix3& anotherMatrix) const
	{
		Matrix3 newMatrix(*this);
		newMatrix *= anotherMatrix;

		return newMatrix;
	}

	Matrix3& Matrix3::operator-=(const Matrix3& anotherMatrix)
	{
		m_data -= anotherMatrix.m_data;

		return *this;
	}

	Matrix3 Matrix3::operator-(const Matrix3& anotherMatrix) const
	{
		Matrix3 newMatrix(*this);
		newMatrix *= anotherMatrix;

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
	// Matrix/scalar operations
	// ------------------------
	Matrix3& Matrix3::operator+=(const double scalar)
	{
		m_data += scalar;

		return *this;
	}

	Matrix3& Matrix3::operator-=(const double scalar)
	{
		m_data -= scalar;

		return *this;
	}

	Matrix3& Matrix3::operator*=(const double scalar)
	{
		m_data *= scalar;

		return *this;
	}

	Matrix3& Matrix3::operator/=(const double scalar)
	{
		m_data /= scalar;

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
	double& Matrix3::operator()(const unsigned int row, const unsigned int column)
	{
		return m_data[3 * row + column];
	}

	const double& Matrix3::operator()(const unsigned int row, const unsigned int column) const
	{
		return m_data[3 * row + column];
	}

	std::size_t Matrix3::getSize()
	{
		return 3;
	}
}