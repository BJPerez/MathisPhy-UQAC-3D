#include "forceGenerator/particleSpringForceGenerator.hpp"

namespace physicslib 
{
	ParticleSpringForceGenerator::ParticleSpringForceGenerator(const std::shared_ptr<const Particle> otherParticle,
		double elasticity, double restingLength)
		: m_otherParticle(otherParticle), m_elasticity(elasticity), m_restingLength(restingLength)
	{}

	void ParticleSpringForceGenerator::updateForce(std::shared_ptr<Particle> particle, const double duration) const
	{
		Vector3 d = particle->getPosition() - m_otherParticle->getPosition();
		particle->addForce(d.getNormalizedVector() * (-m_elasticity) * (d.getNorm() - m_restingLength));
	}
}