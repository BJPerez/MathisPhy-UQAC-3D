#include "../include/physicEngine.hpp"

#include "math/vector3.hpp"

PhysicEngine::PhysicEngine()
{
}

void PhysicEngine::update(std::vector<std::shared_ptr<physicslib::RigidBody>>& rigidBodies, const double frametime)
{
	// Generates all forces and add them in the force register
	generateAllForces(rigidBodies);

	// applies the forces inside the force register
	m_forceRegister.updateAllForces(frametime);

	// integrate all rigid bodies
	for (auto& rigidBody : rigidBodies)
	{
		rigidBody->integrate(frametime);
	}

	// look for collisions and resolve them
	detectContacts(rigidBodies);
	m_contactRegister.resolveContacts(frametime);

	// clean registers
	m_contactRegister.clear();
	m_forceRegister.clear();
}

void PhysicEngine::generateAllForces(std::vector<std::shared_ptr<physicslib::RigidBody>>& rigidBodies)
{
	for (auto& rigidBody : rigidBodies)
	{
		m_forceRegister.add(physicslib::ForceRegister::ForceRecord(rigidBody, gravityGenerator));
		m_forceRegister.add(physicslib::ForceRegister::ForceRecord(rigidBody, dragGenerator));
	}
}

void PhysicEngine::detectContacts(std::vector<std::shared_ptr<physicslib::RigidBody>>& rigidBodies)
{
	if (rigidBodies.size() >= 2)
	{
		if (abs(rigidBodies[0]->getPosition().getX() - rigidBodies[1]->getPosition().getX()) < 10)
		{
			rigidBodies[0]->setVelocity(-rigidBodies[0]->getVelocity());
			rigidBodies[1]->setVelocity(-rigidBodies[1]->getVelocity());
			rigidBodies[0]->addForceAtBodyPoint(physicslib::Vector3(-500, 0, 500), physicslib::Vector3(15, 0, 15));
			rigidBodies[1]->addForceAtBodyPoint(physicslib::Vector3(500, 0, -500), physicslib::Vector3(-15, 0, -15));
		}
	}
}
