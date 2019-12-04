#include "collisions/boxPrimitive.hpp"

namespace physicslib
{
	BoxPrimitive::BoxPrimitive(std::shared_ptr<RigidBody> rigidBody, const Matrix34& transformMatrix, const Vector3& halfSizes)
		: Primitive(rigidBody, transformMatrix)
		, m_halfSizes(halfSizes)
	{
	}
}