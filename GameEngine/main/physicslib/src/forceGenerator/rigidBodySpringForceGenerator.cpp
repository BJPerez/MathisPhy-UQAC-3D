#include "forceGenerator/rigidBodySpringForceGenerator.hpp"

namespace physicslib
{
	RigidBodySpringForceGenerator::RigidBodySpringForceGenerator(Vector3 extremity1, Vector3 extremity2, 
		const std::shared_ptr<const RigidBody> otherRigidBody,
		double elasticity, double restingLength)
		: m_extremity1(extremity1), m_extremity2(extremity2), m_otherRigidBody(otherRigidBody), m_elasticity(elasticity), m_restingLength(restingLength)
	{}

	void RigidBodySpringForceGenerator::updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const
	{
		// Convert both extrimity coordinates to worldSpace
		Vector3 worldExtremity1(/*m_transformMatrix.getReverseMatrix() * m_extremity1 */  Vector3(0, 0, 0));
		Vector3 worldExtremity2(/*m_transformMatrix.getReverseMatrix() * m_extremity2*/ Vector3(0,0,0));

		// Compute spring length
		Vector3 d = worldExtremity1 - worldExtremity2;

		// Apply Force
		rigidBody->addForceAtPoint(d.getNormalizedVector() * (-m_elasticity) * (d.getNorm() - m_restingLength), m_extremity1);
	}
}