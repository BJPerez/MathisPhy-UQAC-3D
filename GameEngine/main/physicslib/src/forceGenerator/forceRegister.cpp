#include "forceGenerator/forceRegister.hpp"

#include <algorithm>

namespace physicslib
{
	ForceRegister::ForceRecord::ForceRecord(const std::shared_ptr<Particle> particle,
		const std::shared_ptr<const ParticleForceGenerator> forceGenerator): 
		particle(particle), forceGenerator(forceGenerator)
	{
	}

	void ForceRegister::add(const ForceRecord& record)
	{
		m_register.push_back(record);
	}

	void ForceRegister::clear()
	{
		m_register.clear();
	}

	void ForceRegister::updateAllForces(double duration)
	{
		std::for_each(m_register.begin(), m_register.end(), 
			[duration](ForceRecord& record)
			{
				record.forceGenerator->updateForce(record.particle, duration);
			});
	}
}