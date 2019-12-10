#include "collisions/boxPrimitive.hpp"

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
		for (std::size_t i = 0; i < rigidBodyVertices.size() - 2; i += 3)
		{
			vertices.push_back(Vector3(rigidBodyVertices.at(i), rigidBodyVertices.at(i + 1), rigidBodyVertices.at(i + 2)));
		}

		return vertices;
	}
}