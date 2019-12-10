#include "collisions/boxPrimitive.hpp"

#include <math.h>

namespace physicslib
{
	BoxPrimitive::BoxPrimitive(std::shared_ptr<RigidBody> rigidBody, const Matrix34& transformMatrix, const Vector3& halfSizes)
		: Primitive(rigidBody, transformMatrix)
		, m_halfSizes(halfSizes)
	{
	}

	std::vector<Vector3> BoxPrimitive::getVertices() const
	{
		std::vector<Vector3> vertices;

		std::vector<double> rigidBodyVertices = m_rigidBody->getBoxVertices();
		for (int i = 0; i < rigidBodyVertices.size() - 2; i += 3)
		{
			vertices.push_back(Vector3(rigidBodyVertices[i], rigidBodyVertices[i + 1], rigidBodyVertices[i + 2]));
		}

		return vertices;
	}
}