#pragma once

#include "particleForceGenerator.hpp"
#include "vector3.hpp"

namespace physicslib
{
	class GravityForceGenerator : public ParticleForceGenerator
	{
		public:
			GravityForceGenerator(Vector3 gravity);

			void updateForce(std::shared_ptr<Particle> particle, const double duration) const override;

		private:
			const Vector3 m_gravity;
	};
}