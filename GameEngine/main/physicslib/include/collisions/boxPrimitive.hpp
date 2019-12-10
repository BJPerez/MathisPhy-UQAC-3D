#pragma once

#include "collisions/primitive.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class BoxPrimitive : public Primitive
	{
	public:
		/**
		 * Contructor
		 */
		BoxPrimitive(std::shared_ptr<RigidBody> rigidBody, const Matrix34& transformMatrix, const Vector3& halfSizes);

		/**
		 * Default copy constructor
		 */
		BoxPrimitive(const BoxPrimitive& anotherBoxPrimitive) = default;

		/**
		 * Virtual destructor
		 */
		virtual ~BoxPrimitive() = default;

		/**
		 * Default assignment operator
		 */
		BoxPrimitive& operator=(const BoxPrimitive& anotherBoxPrimitive) = default;

		/**
		 * Get the vertices of the corresponding box rigidBody
		 */
		virtual std::vector<Vector3> getVertices() const;

	private:
		Vector3 m_halfSizes;
	};
}