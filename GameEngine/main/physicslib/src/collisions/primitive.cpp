#include "collisions/primitive.hpp"

namespace physicslib
{
	Primitive::Primitive(std::shared_ptr<RigidBody> rigidBody, const Matrix34& transformMatrix)
		: m_rigidBody(rigidBody)
		, m_transformMatrix(transformMatrix)
	{
	}
}