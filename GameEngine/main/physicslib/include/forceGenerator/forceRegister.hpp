#pragma once

#include "particleForceGenerator.hpp"

#include <vector>
#include <memory>

namespace physicslib
{
	class ForceRegister
	{
	public:
		struct ForceRecord
		{
			ForceRecord(const std::shared_ptr<Particle> particle, 
				const std::shared_ptr<const ParticleForceGenerator> forceGenerator);

			const std::shared_ptr<Particle> particle;
			const std::shared_ptr<const ParticleForceGenerator> forceGenerator;
		};
		void add(const ForceRecord& record);
		void clear();

		void updateAllForces(double duration);
	private:
		std::vector<ForceRecord> m_register;
	};
}