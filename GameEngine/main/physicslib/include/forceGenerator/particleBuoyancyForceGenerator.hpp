#pragma once

#include "particleForceGenerator.hpp"
#include "vector3.hpp"

namespace physicslib 
{
	class ParticleBuoyancyForceGenerator : public ParticleForceGenerator
	{
	public :
		ParticleBuoyancyForceGenerator(double maxDepth, double objectVolume, double liquidHeight, double liquidDensity);

		void updateForce(std::shared_ptr<Particle> particle, const double duration) const override;

	private:
		const double m_maxDepth;
		const double m_objectVolume;
		const double m_liquidHeight;
		const double m_liquidDensity;
	};
}