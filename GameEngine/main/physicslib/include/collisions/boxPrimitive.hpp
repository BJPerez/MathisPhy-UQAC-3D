#pragma once

#include "collisions/primitive.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class BoxPrimitive : Primitive
	{
	public:
		BoxPrimitive(std::shared_ptr<RigidBody> rigidBody, const Matrix34& transformMatrix, const Vector3& halfSizes);
		virtual ~BoxPrimitive() = default;

	private:
		Vector3 m_halfSizes;
	};
}