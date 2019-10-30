#pragma once

#include "particleForceGenerator.hpp"
#include "vector3.hpp"

namespace physicslib
{
	class ParticleStiffSpringForceGenerator : public ParticleForceGenerator
	{
	public:
		ParticleStiffSpringForceGenerator(Vector3 anchorPosition, double elasticity, double damping);
		virtual ~ParticleStiffSpringForceGenerator();

		void updateForce(std::shared_ptr<Particle> particle, const double duration) const override;

	private:
		const Vector3 m_anchorPosition;
		const double m_elasticity;
		const double m_damping;
	};
}