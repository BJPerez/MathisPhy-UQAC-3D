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
		/**
		 * Contructor
		 */
		Primitive(std::shared_ptr<RigidBody> rigidBody);

		/**
		 * Default copy constructor
		 */
		Primitive(const Primitive& anotherPrimitive) = default;

		/**
		 * Virtual destructor
		 */
		virtual ~Primitive() = default;

		/**
		 * Default assignment operator
		 */
		Primitive& operator=(const Primitive& anotherPrimitive) = default;

		/**
		 * Abstract method to get the vertices of the primitive
		 */
		virtual std::vector<Vector3> getVertices() const = 0;

	protected:
		std::shared_ptr<RigidBody> m_rigidBody;
		Matrix34 m_transformMatrix;
	};
}