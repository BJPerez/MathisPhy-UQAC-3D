#include "forceGenerator/rigidBodygravityForceGenerator.hpp"

namespace physicslib
{
	GravityForceGenerator::GravityForceGenerator(Vector3 gravity)
		: m_gravity(gravity)
	{
	}

	void GravityForceGenerator::updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const
	{
		if (rigidBody->getInverseMass() != 0)
		{
			rigidBody->addForceAtPoint(m_gravity,rigidBody->getPosition());
		}
	}
}