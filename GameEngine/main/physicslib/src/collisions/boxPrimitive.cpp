#include "collisions/boxPrimitive.hpp"

#include <math.h>

namespace physicslib
{
	BoxPrimitive::BoxPrimitive(std::shared_ptr<RigidBody> rigidBody)
		: Primitive(rigidBody)
		, m_halfSizes(rigidBody->getBoxSize() / 2.)
	{
	}

	std::vector<Vector3> BoxPrimitive::getVertices() const
	{
		std::vector<Vector3> vertices {
			{ +m_halfSizes.getX(), +m_halfSizes.getY(), +m_halfSizes.getZ() },
			{ -m_halfSizes.getX(), +m_halfSizes.getY(), +m_halfSizes.getZ() },
			{ -m_halfSizes.getX(), -m_halfSizes.getY(), +m_halfSizes.getZ() },
			{ +m_halfSizes.getX(), -m_halfSizes.getY(), +m_halfSizes.getZ() },
			{ +m_halfSizes.getX(), +m_halfSizes.getY(), -m_halfSizes.getZ() },
			{ -m_halfSizes.getX(), +m_halfSizes.getY(), -m_halfSizes.getZ() },
			{ -m_halfSizes.getX(), -m_halfSizes.getY(), -m_halfSizes.getZ() },
			{ +m_halfSizes.getX(), -m_halfSizes.getY(), -m_halfSizes.getZ() }
		};

		Matrix3 transform({
			m_transformMatrix(0, 0), m_transformMatrix(0, 1), m_transformMatrix(0, 2),
			m_transformMatrix(1, 0), m_transformMatrix(1, 1), m_transformMatrix(1, 2),
			m_transformMatrix(2, 0), m_transformMatrix(2, 1), m_transformMatrix(2, 2)
		});

		Vector3 position = m_rigidBody->getPosition();

		std::transform(vertices.begin(), vertices.end(), vertices.begin(),
			[&transform, &position](const Vector3& vertex)
			{
				return vertex.localToWorld(transform) + position;
			});

		return vertices;
	}
}