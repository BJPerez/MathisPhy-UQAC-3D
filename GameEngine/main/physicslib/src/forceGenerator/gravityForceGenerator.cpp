#include "forceGenerator/gravityForceGenerator.hpp"

namespace physicslib
{
	GravityForceGenerator::GravityForceGenerator(Vector3 gravity)
		: m_gravity(gravity)
	{
	}

	void GravityForceGenerator::updateForce(std::shared_ptr<Particle> particle, const double duration) const
	{
		if (particle->getInverseMass() != 0)
		{
			particle->addForce(m_gravity);
		}
	}
}