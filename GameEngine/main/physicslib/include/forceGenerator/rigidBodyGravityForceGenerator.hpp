#pragma once

#include "rigidBodyForceGenerator.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class GravityForceGenerator : public ParticleForceGenerator
	{
		public:
			GravityForceGenerator(Vector3 gravity);

			void updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const override;

		private:
			const Vector3 m_gravity;
	};
}