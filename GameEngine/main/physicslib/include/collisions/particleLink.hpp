#pragma once

#include "particleContactGenerator.hpp"

namespace physicslib
{
	class ParticleLink : ParticleContactGenerator
	{
	public:
		ParticleLink(Particle* particle1, Particle* particle2);
		virtual ~ParticleLink();

		virtual double getCurrentLength();

	protected:
		Particle* m_particles[2];
	};
}