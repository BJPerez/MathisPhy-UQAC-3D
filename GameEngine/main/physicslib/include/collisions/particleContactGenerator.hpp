#pragma once

#include "contactRegister.hpp"

namespace physicslib
{
	class ParticleContactGenerator
	{

	public:
		ParticleContactGenerator() = default;
		virtual ~ParticleContactGenerator() = default;

		virtual void addContact(ContactRegister& contactRegister) = 0;
	};
}
