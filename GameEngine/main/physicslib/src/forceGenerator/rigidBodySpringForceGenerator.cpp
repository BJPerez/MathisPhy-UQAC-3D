#include "forceGenerator/rigidBodySpringForceGenerator.hpp"

namespace physicslib
{
	RigidBodySpringForceGenerator::RigidBodySpringForceGenerator(Vector3 extremity1, Vector3 extremity2, 
		const std::shared_ptr<const RigidBody> otherRigidBody,
		double elasticity, double restingLength)
		: m_extremity1(extremity1), m_extremity2(extremity2), m_otherRigidBody(otherRigidBody), m_elasticity(elasticity), m_restingLength(restingLength)
	{}

	/* Apply spring forces to rigidBody : the spring is attached to extremity1 on rigidBody and extremity2 on otherRigidBody */
	void RigidBodySpringForceGenerator::updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const
	{
		// Compute spring length
		Vector3 d = m_extremity1 - m_extremity2;

		// Apply Force
		rigidBody->addForceAtPoint(d.getNormalizedVector() * (-m_elasticity) * (d.getNorm() - m_restingLength), m_extremity1);
	}
}