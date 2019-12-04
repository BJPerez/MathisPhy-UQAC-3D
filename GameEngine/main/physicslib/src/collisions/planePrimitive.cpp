#include "collisions/planePrimitive.hpp"

namespace physicslib
{
	PlanePrimitive::PlanePrimitive(const Vector3& normal, double offset)
		: Primitive(nullptr, Matrix34())
		, m_normal(normal)
		, m_offset(offset)
	{
	}

	std::vector<Vector3> PlanePrimitive::getVertices() const
	{
		return std::vector<Vector3>();
	}

	Vector3 PlanePrimitive::getNormal() const
	{
		return m_normal;
	}

	double PlanePrimitive::getOffset() const
	{
		return m_offset;
	}
}