#pragma once

#include <memory>
#include "rigidBody.hpp"
#include "math/matrix34.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class Primitive
	{
	public:
		Primitive(std::shared_ptr<RigidBody> rigidBody, const Matrix34& transformMatrix);
		virtual ~Primitive() = default;

		virtual std::vector<Vector3> getVertices() const = 0;

	protected:
		std::shared_ptr<RigidBody> m_rigidBody;
		Matrix34 m_transformMatrix;
	};
}