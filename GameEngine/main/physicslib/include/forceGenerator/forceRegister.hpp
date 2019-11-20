#pragma once

#include "rigidBodyForceGenerator.hpp"

#include <vector>
#include <memory>

namespace physicslib
{
	class ForceRegister
	{
	public:
		struct ForceRecord
		{
			ForceRecord(const std::shared_ptr<RigidBody> rigidBody,
				const std::shared_ptr<const RigidBodyForceGenerator> forceGenerator);

			const std::shared_ptr<RigidBody> rigidBody;
			const std::shared_ptr<const RigidBodyForceGenerator> forceGenerator;
		};
		void add(const ForceRecord& record);
		void clear();

		void updateAllForces(double duration);
	private:
		std::vector<ForceRecord> m_register;
	};
}