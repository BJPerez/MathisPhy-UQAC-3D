#include "collisions/particleRod.hpp"

#include "particle.hpp"

namespace physicslib
{
	ParticleRod::ParticleRod(Particle* particle1, Particle* particle2, double length) :
		ParticleLink(particle1, particle2), m_length(length)
	{
	}

	ParticleRod::~ParticleRod()
	{
	}

	void ParticleRod::addContact(ContactRegister& contactRegister)
	{
		int direction = 0;
		double currentLength = getCurrentLength();
		if (currentLength > m_length)
		{
			direction = -1;
		}
		else if (currentLength < m_length)
		{
			direction = 1;
		}
		Vector3 contactNormal = (m_particles[0]->getPosition() - m_particles[1]->getPosition()).getNormalizedVector() * direction;
		double vs = contactNormal * (m_particles[0]->getSpeed() - m_particles[1]->getSpeed());
		//contactRegister.add(ParticleContact(m_particles[0], m_particles[1], 1., vs, 0., contactNormal));
		ParticleContact particleContact(m_particles[0], m_particles[1], 1., vs, 0., contactNormal);
		contactRegister.add(particleContact);
	}
}