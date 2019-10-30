#include "collisions/contactRegister.hpp"

#include <algorithm>

namespace physicslib
{
	void ContactRegister::add(ParticleContact& contact)
	{
		m_register.push_back(contact);
	}

	void ContactRegister::clear()
	{
		m_register = std::vector<ParticleContact>();
	}
	
	void ContactRegister::resolveContacts(double frametime)
	{
		auto contact = std::begin(m_register);
		while (contact != std::end(m_register))
		{
			(*contact).resolve(frametime);
			contact++;
		}
	}
}