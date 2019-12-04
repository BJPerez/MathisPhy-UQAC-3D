#pragma once

#include "collisions/primitive.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class PlanePrimitive : Primitive
	{
	public:
		PlanePrimitive(const Vector3& normal, double offset);
		virtual ~PlanePrimitive() = default;

	private:
		Vector3 m_normal;
		double m_offset;
	};
}