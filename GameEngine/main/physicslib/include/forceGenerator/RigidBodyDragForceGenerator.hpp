#pragma once

#include "rigidBodyForceGenerator.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class RigidBodyDragForceGenerator : public RigidBodyForceGenerator
	{
	public:
		RigidBodyDragForceGenerator(double k1, double k2);

		void updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const override;

	private:
		const double m_k1;
		const double m_k2;
	};
}