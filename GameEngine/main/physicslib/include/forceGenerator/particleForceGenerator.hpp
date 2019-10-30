#pragma once

#include <memory>

#include "particle.hpp"

namespace physicslib
{
	class ParticleForceGenerator
	{
	public:
		ParticleForceGenerator() = default;
		virtual ~ParticleForceGenerator() = default;

		virtual void updateForce(std::shared_ptr<Particle> particle, const double duration) const = 0;
	};
}