#include <valarray>

namespace physicslib
{
	class Matrix34
	{
	public :
		/* Default constructor, creating an identity matrix */
		Matrix34();

		/* Creates a matrix filled with fillNumber */
		explicit Matrix34(double fillNumber = 0.);

		/*
		 * Create a matrix 4x4 with an initializer list
		 * `Matrix3 mat { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 }`
		 */
		Matrix34(const std::initializer_list<double>& initializerList);


		Matrix34(const Matrix34& anotherMatrix) = default;
		virtual ~Matrix34() = default;

		Matrix34& operator=(const Matrix34& anotherMatrix) = default;

		// Matrix operations
		double getDeterminant() const;
		void reverse();
		Matrix34 getReverseMatrix() const;

		// Matrix mathematical operations
		Matrix34& operator+=(const Matrix34& anotherMatrix);
		Matrix34 operator+(const Matrix34& anotherMatrix);
		Matrix34& operator-=(const Matrix34& anotherMatrix);
		Matrix34 operator-(const Matrix34& anotherMatrix);
		Matrix34& operator*=(const Matrix34& anotherMatrix);
		Matrix34 operator*(const Matrix34& anotherMatrix);

		// Matrix/scalar operations
		Matrix34& operator+=(const double scalar);
		Matrix34& operator-=(const double scalar);
		Matrix34& operator*=(const double scalar);
		Matrix34& operator/=(const double scalar);

		// Getters/Setters
		double& operator()(const unsigned int row, const unsigned int column);
		const double& operator()(const unsigned int row, const unsigned int column) const;

		double& operator()(const unsigned int index);
		const double& operator()(const unsigned int index) const;

	private :
		std::valarray<double> m_data;
	};

	// Matrix/scalar operations
	Matrix34 operator+(const Matrix34& matrix, const double scalar);
	Matrix34 operator+(const double scalar, const Matrix34& matrix);
	Matrix34 operator-(const Matrix34& matrix, const double scalar);
	Matrix34 operator-(const double scalar, const Matrix34& matrix);
	Matrix34 operator*(const Matrix34& matrix, const double scalar);
	Matrix34 operator*(const double scalar, const Matrix34& matrix);
	Matrix34 operator/(const Matrix34& matrix, const double scalar);
	Matrix34 operator/(const double scalar, const Matrix34& matrix);
}