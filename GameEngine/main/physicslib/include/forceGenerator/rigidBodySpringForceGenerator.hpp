#pragma once

#include "rigidBodyForceGenerator.hpp"
#include "rigidBody.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class RigidBodySpringForceGenerator : public RigidBodyForceGenerator
	{
	public:
		RigidBodySpringForceGenerator(Vector3 extremity1, Vector3 extremity2, const std::shared_ptr<const RigidBody> otherRigidBody, double elasticity, double restingLength);

		void updateForce(std::shared_ptr<RigidBody> rigidBody, const double duration) const override;

	private:
		const Vector3 m_extremity1; //Coordinates where the spring is attached, in localSpace
		const Vector3 m_extremity2; //Coordinates where the spring is attached on otherRigidBody, in localSpace
		const std::shared_ptr<const RigidBody> m_otherRigidBody;
		const double m_elasticity;
		const double m_restingLength;
	};
}