#pragma once

#include "math/vector3.hpp"

namespace physicslib
{
	class Contact
	{
	public:
		/**
		 * Constructor
		 */
		Contact(const Vector3& contactPoint, const Vector3& contactNormal, double penetration);

		/**
		 * Default copy constructor
		 */
		Contact(const Contact& anotherContact) = default;

		/**
		 * Virtual destructor
		 */
		virtual ~Contact() = default;

		/**
		 * Default assignment operator
		 */
		Contact& operator=(const Contact& anotherContact) = default;

	private:
		Vector3 m_contactPoint;
		Vector3 m_contactNormal;
		double m_penetration;
	};
}