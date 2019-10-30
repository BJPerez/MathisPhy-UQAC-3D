#pragma once
#include <vector3.hpp>
#include <string>
#include "particle.hpp"

namespace physicslib
{
	class ParticleContactResolver
	{
		friend class ParticleContact;

	public:
		ParticleContactResolver();
		void setIteration();
		void resolveContact();
		virtual ~ParticleContactResolver();

	private:
		int maxIterationsNb;
		int currentIteration;
	};

}
