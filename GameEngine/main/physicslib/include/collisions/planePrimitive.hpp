#pragma once

#include "collisions/primitive.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class PlanePrimitive : public Primitive
	{
	public:
		PlanePrimitive(const Vector3& normal, double offset);
		virtual ~PlanePrimitive() = default;

		virtual std::vector<Vector3> getVertices() const;

		Vector3 getNormal() const;
		double getOffset() const;

	private:
		Vector3 m_normal;
		double m_offset;
	};
}