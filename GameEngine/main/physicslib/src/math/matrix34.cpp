#include "../../include/math/matrix34.hpp"

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
		: m_data(fillNumber, 12)
	{
	}

	Matrix34::Matrix34(const std::initializer_list<double>& initializerList)
		: m_data(initializerList)
	{
	}

	double Matrix34::getDeterminant() const
	{
		return (*this)(8) * (*this)(5)*(*this)(2) 
			+ (*this)(4) * (*this)(9) * (*this)(2) 
			+ (*this)(8) * (*this)(1) * (*this)(6)
			- (*this)(0) * (*this)(9) * (*this)(6)
			- (*this)(4) * (*this)(1) * (*this)(10)
			- (*this)(0) * (*this)(5) * (*this)(10);
	}

	void Matrix34::reverse()
	{
		double determinant = getDeterminant();
		if (determinant == 0.)
		{
			return;
		}

		/* Voir formule du cour */
		*this =
		{
			/* Ligne 1 */
			-(*this)(9) * (*this)(6) + (*this)(5) * (*this)(10),
			(*this)(9) * (*this)(2) - (*this)(1) * (*this)(10),
			-(*this)(5) * (*this)(2) + (*this)(1) * (*this)(6),
			/* Rouge */
			(*this)(9) * (*this)(6) * (*this)(3)
			- (*this)(5) * (*this)(10) * (*this)(3)
			- (*this)(9) * (*this)(2) * (*this)(7)
			+ (*this)(1) * (*this)(10) * (*this)(7)
			+ (*this)(5) * (*this)(2) * (*this)(11)
			- (*this)(1) * (*this)(6) * (*this)(11),

			/* Ligne 2 */
			(*this)(8) * (*this)(6) - (*this)(4) * (*this)(10),
			-(*this)(8) * (*this)(2) + (*this)(0) * (*this)(10),
			(*this)(4) * (*this)(2) - (*this)(0) * (*this)(6),
			/* Bleu */
			-(*this)(8)* (*this)(6)* (*this)(3)
			+ (*this)(4) * (*this)(10) * (*this)(3)
			+ (*this)(8) * (*this)(2) * (*this)(7)
			- (*this)(0) * (*this)(10) * (*this)(7)
			- (*this)(4) * (*this)(2) * (*this)(11)
			+ (*this)(0) * (*this)(6) * (*this)(11),

			/* Ligne 3 */
			-(*this)(8)* (*this)(5) + (*this)(4) * (*this)(9),
			(*this)(8) * (*this)(1) - (*this)(0) * (*this)(9),
			-(*this)(4)* (*this)(1) + (*this)(0) * (*this)(5),
			/* Vert */
			(*this)(8)* (*this)(5)* (*this)(3)
			- (*this)(4) * (*this)(9) * (*this)(3)
			- (*this)(8) * (*this)(1) * (*this)(7)
			+ (*this)(0) * (*this)(9) * (*this)(7)
			+ (*this)(4) * (*this)(1) * (*this)(11)
			- (*this)(0) * (*this)(5) * (*this)(11)
		};
		*this /= determinant;
	}

	Matrix34 Matrix34::getReverseMatrix() const
	{
		Matrix34 res(*this);
		res.reverse();
		return res;
	}

	// Matrix mathematical operations
	Matrix34& Matrix34::operator+=(const Matrix34& anotherMatrix)
	{
		m_data += anotherMatrix.m_data;
		return *this;
	}

	Matrix34 Matrix34::operator+(const Matrix34& anotherMatrix)
	{
		Matrix34 res(*this);
		return res += anotherMatrix;
	}

	Matrix34& Matrix34::operator-=(const Matrix34& anotherMatrix)
	{
		m_data -= anotherMatrix.m_data;
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
			(*this)(0) * anotherMatrix(0) + (*this)(1) * anotherMatrix(4) + (*this)(2) * anotherMatrix(8),
			(*this)(0) * anotherMatrix(1) + (*this)(1) * anotherMatrix(5) + (*this)(2) * anotherMatrix(9),
			(*this)(0) * anotherMatrix(2) + (*this)(1) * anotherMatrix(6) + (*this)(2) * anotherMatrix(10),
			(*this)(0) * anotherMatrix(3) + (*this)(1) * anotherMatrix(7) + (*this)(2) * anotherMatrix(11) + (*this)(3),

			(*this)(4) * anotherMatrix(0) + (*this)(5) * anotherMatrix(4) + (*this)(6) * anotherMatrix(8),
			(*this)(4) * anotherMatrix(1) + (*this)(5) * anotherMatrix(5) + (*this)(6) * anotherMatrix(9),
			(*this)(4) * anotherMatrix(2) + (*this)(5) * anotherMatrix(6) + (*this)(6) * anotherMatrix(10),
			(*this)(4) * anotherMatrix(3) + (*this)(5) * anotherMatrix(7) + (*this)(6) * anotherMatrix(11) + (*this)(7),

			(*this)(8) * anotherMatrix(0) + (*this)(9) * anotherMatrix(4) + (*this)(10) * anotherMatrix(8),
			(*this)(8) * anotherMatrix(1) + (*this)(9) * anotherMatrix(5) + (*this)(10) * anotherMatrix(9),
			(*this)(8) * anotherMatrix(2) + (*this)(9) * anotherMatrix(6) + (*this)(10) * anotherMatrix(10),
			(*this)(8) * anotherMatrix(3) + (*this)(9) * anotherMatrix(7) + (*this)(10) * anotherMatrix(11) + (*this)(11)
		};
		return *this;
	}

	Matrix34 Matrix34::operator*(const Matrix34& anotherMatrix)
	{
		Matrix34 res(*this);
		return res *= anotherMatrix;
	}

	// Matrix/scalar operations
	Matrix34& Matrix34::operator+=(const double scalar)
	{
		m_data += scalar;
		return *this;
	}
	Matrix34& Matrix34::operator-=(const double scalar)
	{
		m_data -= scalar;
		return *this;
	}
	Matrix34& Matrix34::operator*=(const double scalar)
	{
		m_data *= scalar;
		return *this;
	}
	Matrix34& Matrix34::operator/=(const double scalar)
	{
		m_data /= scalar;
		return *this;
	}
	
	// Getters/Setters
	double& Matrix34::operator()(const unsigned int row, const unsigned int column)
	{
		return m_data[4 * row + column];
	}
	const double& Matrix34::operator()(const unsigned int row, const unsigned int column) const
	{
		return m_data[4 * row + column];
	}
	double& Matrix34::operator()(const unsigned int index)
	{
		return m_data[index];
	}
	const double& Matrix34::operator()(const unsigned int index) const
	{
		return m_data[index];
	}
}