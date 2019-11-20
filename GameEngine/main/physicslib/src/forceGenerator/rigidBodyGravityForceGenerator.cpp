#include "forceGenerator/rigidBodygravityForceGenerator.hpp"

namespace physicslib
{
	RigidBodyGravityForceGenerator::RigidBodyGravityForceGenerator(Vector3 gravity)
		: m_gravity(gravity)
	{
	}

	/* Apply gravity at the rigidBody's center */
	void RigidBodyGravityForceGenerator::updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const
	{
		if (rigidBody->getInverseMass() != 0)
		{
			rigidBody->addForceAtPoint(m_gravity,rigidBody->getPosition());
		}
	}
}