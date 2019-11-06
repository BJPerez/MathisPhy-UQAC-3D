#pragma once
#include <string>
#include "math/vector3.hpp"
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
