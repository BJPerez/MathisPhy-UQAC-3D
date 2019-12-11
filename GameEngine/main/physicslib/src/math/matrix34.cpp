#include "math/matrix34.hpp"

#include <algorithm>

namespace physicslib
{
	Matrix34::Matrix34()
		: m_data({
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0
		})
	{
	}
	

	Matrix34::Matrix34(double fillNumber)
	{
		m_data.fill(fillNumber);
	}

	Matrix34::Matrix34(const std::initializer_list<double>& initializerList)
	{
		std::copy(initializerList.begin(), initializerList.begin() + 9, m_data.begin());
	}

	Matrix34::Matrix34(const Quaternion& quaternion)
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
			quaternion.getI(),
			quaternion.getJ(),
			quaternion.getK()
		})
	{
	}

	Matrix34::Matrix34(const Matrix3& matrix3, const Vector3& vector)
		: m_data({
			matrix3(0, 0), matrix3(0, 1), matrix3(0, 2), vector.getX(),
			matrix3(1, 0), matrix3(1, 1), matrix3(1, 2), vector.getY(),
			matrix3(2, 0), matrix3(2, 1), matrix3(2, 2), vector.getZ()
		})
	{
	}

	double Matrix34::getDeterminant() const
	{
		return m_data[8] * m_data[5] * m_data[2] 
			+ m_data[4] * m_data[9] * m_data[2] 
			+ m_data[8] * m_data[1] * m_data[6]
			- m_data[0] * m_data[9] * m_data[6]
			- m_data[4] * m_data[1] * m_data[10]
			- m_data[0] * m_data[5] * m_data[10];
	}

	void Matrix34::reverse()
	{
		double determinant = getDeterminant();
		if (determinant == 0.)
		{
			return;
		}

		// Voir formule du cours
		*this =
		{
			// Ligne 1
			-m_data[9] * m_data[6] + m_data[5] * m_data[10],
			m_data[9] * m_data[2] - m_data[1] * m_data[10],
			-m_data[5] * m_data[2] + m_data[1] * m_data[6],
			// Rouge
			m_data[9] * m_data[6] * m_data[3]
			- m_data[5] * m_data[10] * m_data[3]
			- m_data[9] * m_data[2] * m_data[7]
			+ m_data[1] * m_data[10] * m_data[7]
			+ m_data[5] * m_data[2] * m_data[11]
			- m_data[1] * m_data[6] * m_data[11],

			// Ligne 2
			m_data[8] * m_data[6] - m_data[4] * m_data[10],
			-m_data[8] * m_data[2] + m_data[0] * m_data[10],
			m_data[4] * m_data[2] - m_data[0] * m_data[6],
			// Bleu
			-m_data[8]* m_data[6]* m_data[3]
			+ m_data[4] * m_data[10] * m_data[3]
			+ m_data[8] * m_data[2] * m_data[7]
			- m_data[0] * m_data[10] * m_data[7]
			- m_data[4] * m_data[2] * m_data[11]
			+ m_data[0] * m_data[6] * m_data[11],

			// Ligne 3
			-m_data[8]* m_data[5] + m_data[4] * m_data[9],
			m_data[8] * m_data[1] - m_data[0] * m_data[9],
			-m_data[4]* m_data[1] + m_data[0] * m_data[5],
			// Vert
			m_data[8]* m_data[5]* m_data[3]
			- m_data[4] * m_data[9] * m_data[3]
			- m_data[8] * m_data[1] * m_data[7]
			+ m_data[0] * m_data[9] * m_data[7]
			+ m_data[4] * m_data[1] * m_data[11]
			- m_data[0] * m_data[5] * m_data[11]
		};
		*this /= determinant;
	}

	Matrix34 Matrix34::getReverseMatrix() const
	{
		Matrix34 res(*this);
		res.reverse();
		return res;
	}

	// ------------------------------
	// Matrix mathematical operations
	// ------------------------------
	Matrix34& Matrix34::operator+=(const Matrix34& anotherMatrix)
	{
		// Term-term addition
		std::transform(m_data.begin(), m_data.end(), anotherMatrix.m_data.begin(), m_data.begin(), std::plus<double>());
		return *this;
	}

	Matrix34 Matrix34::operator+(const Matrix34& anotherMatrix)
	{
		Matrix34 res(*this);
		return res += anotherMatrix;
	}

	Matrix34& Matrix34::operator-=(const Matrix34& anotherMatrix)
	{
		// Term-term substraction
		std::transform(m_data.begin(), m_data.end(), anotherMatrix.m_data.begin(), m_data.begin(), std::minus<double>());
		return *this;
	}

	Matrix34 Matrix34::operator-(const Matrix34& anotherMatrix)
	{
		Matrix34 res(*this);
		return res -= anotherMatrix;
	}

	Matrix34& Matrix34::operator*=(const Matrix34& anotherMatrix)
	{
		*this =
		{
			m_data[0] * anotherMatrix.m_data[0] + m_data[1] * anotherMatrix.m_data[4] + m_data[2] * anotherMatrix.m_data[8],
			m_data[0] * anotherMatrix.m_data[1] + m_data[1] * anotherMatrix.m_data[5] + m_data[2] * anotherMatrix.m_data[9],
			m_data[0] * anotherMatrix.m_data[2] + m_data[1] * anotherMatrix.m_data[6] + m_data[2] * anotherMatrix.m_data[10],
			m_data[0] * anotherMatrix.m_data[3] + m_data[1] * anotherMatrix.m_data[7] + m_data[2] * anotherMatrix.m_data[11] + m_data[3],

			m_data[4] * anotherMatrix.m_data[0] + m_data[5] * anotherMatrix.m_data[4] + m_data[6] * anotherMatrix.m_data[8],
			m_data[4] * anotherMatrix.m_data[1] + m_data[5] * anotherMatrix.m_data[5] + m_data[6] * anotherMatrix.m_data[9],
			m_data[4] * anotherMatrix.m_data[2] + m_data[5] * anotherMatrix.m_data[6] + m_data[6] * anotherMatrix.m_data[10],
			m_data[4] * anotherMatrix.m_data[3] + m_data[5] * anotherMatrix.m_data[7] + m_data[6] * anotherMatrix.m_data[11] + m_data[7],

			m_data[8] * anotherMatrix.m_data[0] + m_data[9] * anotherMatrix.m_data[4] + m_data[10] * anotherMatrix.m_data[8],
			m_data[8] * anotherMatrix.m_data[1] + m_data[9] * anotherMatrix.m_data[5] + m_data[10] * anotherMatrix.m_data[9],
			m_data[8] * anotherMatrix.m_data[2] + m_data[9] * anotherMatrix.m_data[6] + m_data[10] * anotherMatrix.m_data[10],
			m_data[8] * anotherMatrix.m_data[3] + m_data[9] * anotherMatrix.m_data[7] + m_data[10] * anotherMatrix.m_data[11] + m_data[11]
		};
		return *this;
	}

	Matrix34 Matrix34::operator*(const Matrix34& anotherMatrix)
	{
		Matrix34 res(*this);
		return res *= anotherMatrix;
	}

	// ------------------------
	// Matrix/scalar operations
	// ------------------------
	Matrix34& Matrix34::operator+=(const double scalar)
	{
		// m_data[i] += scalar
		std::transform(m_data.begin(), m_data.end(), m_data.begin(), [scalar](const double n) { return n + scalar; });
		return *this;
	}

	Matrix34& Matrix34::operator-=(const double scalar)
	{
		// m_data[i] -= scalar
		std::transform(m_data.begin(), m_data.end(), m_data.begin(), [scalar](const double n) { return n - scalar; });
		return *this;
	}

	Matrix34& Matrix34::operator*=(const double scalar)
	{
		// m_data[i] *= scalar
		std::transform(m_data.begin(), m_data.end(), m_data.begin(), [scalar](const double n) { return n * scalar; });
		return *this;
	}

	Matrix34& Matrix34::operator/=(const double scalar)
	{
		// m_data[i] /= scalar
		std::transform(m_data.begin(), m_data.end(), m_data.begin(), [scalar](const double n) { return n / scalar; });
		return *this;
	}

	Matrix34 operator+(const Matrix34& matrix, const double scalar)
	{
		Matrix34 newMatrix(matrix);
		newMatrix += scalar;
		return newMatrix;
	}

	Matrix34 operator+(const double scalar, const Matrix34& matrix)
	{
		return matrix + scalar;
	}

	Matrix34 operator-(const Matrix34& matrix, const double scalar)
	{
		Matrix34 newMatrix(matrix);
		newMatrix -= scalar;
		return newMatrix;
	}

	Matrix34 operator-(const double scalar, const Matrix34& matrix)
	{
		return matrix - scalar;
	}

	Matrix34 operator*(const Matrix34& matrix, const double scalar)
	{
		Matrix34 newMatrix(matrix);
		newMatrix *= scalar;
		return newMatrix;
	}

	Matrix34 operator*(const double scalar, const Matrix34& matrix)
	{
		return matrix * scalar;
	}

	Matrix34 operator/(const Matrix34& matrix, const double scalar)
	{
		Matrix34 newMatrix(matrix);
		newMatrix /= scalar;
		return newMatrix;
	}

	Matrix34 operator/(const double scalar, const Matrix34& matrix)
	{
		return matrix / scalar;
	}
	
	// ---------------
	// Getters/Setters
	// ---------------
	double& Matrix34::operator()(const std::size_t row, const std::size_t column)
	{
		return m_data[4 * row + column];
	}

	const double& Matrix34::operator()(const std::size_t row, const std::size_t column) const
	{
		return m_data[4 * row + column];
	}
}