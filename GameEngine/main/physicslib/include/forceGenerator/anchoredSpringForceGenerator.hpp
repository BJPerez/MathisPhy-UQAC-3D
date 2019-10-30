#pragma once

#include "particleForceGenerator.hpp"
#include "vector3.hpp"

namespace physicslib
{
	class AnchoredSpringForceGenerator : public ParticleForceGenerator
	{
	public:
		AnchoredSpringForceGenerator(Vector3 anchorPosition, double elasticity, double restingLength);

		void updateForce(std::shared_ptr<Particle> particle, const double duration) const override;
	private:
		const Vector3 m_anchorPosition;
		const double m_elasticity;
		const double m_restingLength;
	};
}