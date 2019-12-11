#include "collisions/primitive.hpp"

namespace physicslib
{
	Primitive::Primitive(std::shared_ptr<RigidBody> rigidBody)
		: m_rigidBody(rigidBody)
	{
		if (rigidBody != nullptr)
		{
			m_transformMatrix = rigidBody->getTransformMatrix(), rigidBody->getPosition();
		}
	}
}