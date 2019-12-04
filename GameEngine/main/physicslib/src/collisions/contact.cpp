#include "collisions/contact.hpp"

namespace physicslib
{
	Contact::Contact(const Vector3& contactPoint, const Vector3& contactNormal, double penetration)
		: m_contactPoint(contactPoint)
		, m_contactNormal(contactNormal)
		, m_penetration(penetration)
	{
	}

	std::string Contact::toString() const
	{
		return "Contact(\ncontactPoint = " + m_contactPoint.toString() + ";\n" +
			"contactNormal = " + m_contactNormal.toString() + ";\n" +
			"penetration = " + std::to_string(m_penetration) + "\n)";
	}
}