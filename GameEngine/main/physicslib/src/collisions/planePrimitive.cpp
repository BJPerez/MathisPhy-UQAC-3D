#include "collisions/planePrimitive.hpp"

namespace physicslib
{
	PlanePrimitive::PlanePrimitive(const Vector3& normal, double offset)
		: Primitive(nullptr, Matrix34())
		, m_normal(normal)
		, m_offset(offset)
	{
	}
}