#include "collisions/particleCable.hpp"

#include "collisions/particleContact.hpp"

namespace physicslib
{
	ParticleCable::ParticleCable(Particle* particle1, Particle* particle2, double maxLength, double restitutionCoef) :
		ParticleLink(particle1, particle2), m_maxLength(maxLength), m_restitutionCoef(restitutionCoef)
	{
	}

	ParticleCable::~ParticleCable()
	{
	}

	void ParticleCable::addContact(ContactRegister& contactRegister)
	{
		if (getCurrentLength() >= m_maxLength)
		{
			Vector3 contactNormal = -(m_particles[0]->getPosition() - m_particles[1]->getPosition()).getNormalizedVector();
			double vs = contactNormal* (m_particles[0]->getSpeed() - m_particles[1]->getSpeed());
			ParticleContact particleContact(m_particles[0], m_particles[1], m_restitutionCoef, vs, 0., contactNormal);
			contactRegister.add(particleContact);
		}
	}
}