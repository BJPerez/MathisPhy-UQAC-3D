#include "collisions/contact.hpp"

namespace physicslib
{
	Contact::Contact(const Vector3& contactPoint, const Vector3& contactNormal, double penetration)
		: m_contactPoint(contactPoint)
		, m_contactNormal(contactNormal)
		, m_penetration(penetration)
	{
	}
}