#pragma once

#include "particleLink.hpp"

namespace physicslib
{
	class ParticleCable : ParticleLink
	{
	public:
		ParticleCable(Particle* particle1, Particle* particle2, double maxLength, double restitutionCoef);
		virtual ~ParticleCable();
		void addContact(ContactRegister& contactRegister);

	private:
		double m_maxLength;
		double m_restitutionCoef;
	};
}