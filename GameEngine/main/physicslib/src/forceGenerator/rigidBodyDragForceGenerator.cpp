#include "forceGenerator/rigidBodyDragForceGenerator.hpp"

namespace physicslib
{
	RigidBodyDragForceGenerator::RigidBodyDragForceGenerator(double k1, double k2)
		: m_k1(k1)
		, m_k2(k2)
	{
	}

	/* Add drag forces to the center of the rigidBody */
	void RigidBodyDragForceGenerator::updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const
	{
		double speedNorm = rigidBody->getVelocity().getNorm();
		double squaredSpeedNorm = rigidBody->getVelocity().getSquaredNorm();
		Vector3 normalizedSpeed = rigidBody->getVelocity().getNormalizedVector();

		Vector3 dragForce = -normalizedSpeed * (m_k1 * speedNorm + m_k2 * squaredSpeedNorm);

		rigidBody->addForceAtPoint(dragForce, rigidBody->getPosition());
	}
}