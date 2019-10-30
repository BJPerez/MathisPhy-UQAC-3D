#pragma once

#include "particleContact.hpp"

#include <vector>
#include <memory>

namespace physicslib
{
	class ContactRegister
	{
	public:
		void add(ParticleContact& contact);
		void clear();

		void resolveContacts(double frametime);
	private:
		std::vector<ParticleContact> m_register;
	};
}