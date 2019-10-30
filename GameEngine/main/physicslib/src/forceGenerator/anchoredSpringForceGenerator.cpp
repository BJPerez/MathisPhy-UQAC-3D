#include "forceGenerator/anchoredSpringForceGenerator.hpp"

namespace physicslib
{
	AnchoredSpringForceGenerator::AnchoredSpringForceGenerator
		(Vector3 anchorPosition, double elasticity, double restingLength) :
		m_anchorPosition(anchorPosition), m_elasticity(elasticity), m_restingLength(restingLength)
	{}

	void AnchoredSpringForceGenerator::updateForce(std::shared_ptr<Particle> particle, const double duration) const
	{
		Vector3 d = particle->getPosition() - m_anchorPosition;
		particle->addForce(d.getNormalizedVector() * (-m_elasticity) * (d.getNorm() - m_restingLength));
	}
}