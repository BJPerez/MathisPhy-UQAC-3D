#include "../include/physicEngine.hpp"

PhysicEngine::PhysicEngine()
{

}

void PhysicEngine::update(std::vector<physicslib::RigidBody>& rigidBodies, const double frametime)
{
	// Generates all forces and add them in the force register
	generateAllForces();

	// applies the forces inside the force register
	m_forceRegister.updateAllForces(frametime);

	// integrate all rigid bodies
	for (physicslib::RigidBody& rigidBody : rigidBodies)
	{
		rigidBody.integrate(frametime);
	}

	// look for collisions and resolve them
	detectContacts();
	m_contactRegister.resolveContacts(frametime);

	// clean registers
	m_contactRegister.clear();
	m_forceRegister.clear();
}

void PhysicEngine::generateAllForces()
{
}

void PhysicEngine::detectContacts()
{
}
