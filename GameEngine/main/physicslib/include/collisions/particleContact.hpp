#pragma once
#include <vector3.hpp>
#include <string>
#include "particle.hpp"

namespace physicslib
{
	class ParticleContact
	{
	public:
		ParticleContact(Particle* particle1, Particle* particle2, double restitution, double vs, double penetratition, Vector3 normal);
		void resolve(double frametime);
		//void calculateVariables();
		void resolveInterpenetration();
		void resolveVelocity(double frametime);
		virtual ~ParticleContact();



	private:

		Particle* m_particles[2];
		double m_restitution;
		double m_vs;
		double m_penetration;
		Vector3 m_contactNormal;
	};

}
