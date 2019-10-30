#pragma once

#include "particleForceGenerator.hpp"
#include "vector3.hpp"

namespace physicslib
{
	class BungeeSpringForceGenerator : public ParticleForceGenerator
	{
	public:
		BungeeSpringForceGenerator(const std::shared_ptr<const Particle> otherParticle, double elasticity, double restingLength);

		void updateForce(std::shared_ptr<Particle> particle, const double duration) const override;
	private:
		const std::shared_ptr<const Particle> m_otherParticle;
		const double m_elasticity;
		const double m_restingLength;
	};
}