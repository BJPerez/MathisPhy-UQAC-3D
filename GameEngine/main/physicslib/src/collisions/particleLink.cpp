#include "collisions/particleLink.hpp"

namespace physicslib
{
	ParticleLink::ParticleLink(Particle* particle1, Particle* particle2) :
		ParticleContactGenerator()
	{
		m_particles[0] = particle1;
		m_particles[1] = particle2;
	}

	ParticleLink::~ParticleLink()
	{
	}

	double ParticleLink::getCurrentLength()
	{
		return m_particles[0]->getDistance(*m_particles[1]);
	}
}