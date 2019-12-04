#pragma once

#include "math/vector3.hpp"

namespace physicslib
{
	class Contact
	{
	public:
		Contact(const Vector3& contactPoint, const Vector3& contactNormal, double penetration);
		virtual ~Contact() = default;

	private:
		Vector3 m_contactPoint;
		Vector3 m_contactNormal;
		double m_penetration;
	};
}