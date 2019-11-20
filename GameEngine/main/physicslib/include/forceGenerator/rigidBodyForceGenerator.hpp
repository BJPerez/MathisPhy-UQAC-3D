#pragma once

#include <memory>

#include "rigidBody.hpp"

namespace physicslib
{
	class RigidBodyForceGenerator
	{
	public:
		RigidBodyForceGenerator() = default;
		virtual ~RigidBodyForceGenerator() = default;

		virtual void updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const = 0;
	};
}