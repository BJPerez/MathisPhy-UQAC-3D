#include "../include/physicEngine.hpp"

PhysicEngine::PhysicEngine()
{
	physicslib::ForceRegister::ForceRecord gravityRecord();
	physicslib::RigidBody rigidBody(10,0.9,100,physicslib::Vector3(0,0,0), physicslib::Vector3(0, 0, 0), physicslib::Vector3(0, 0, 0),physicslib::Quaternion(),
		physicslib::Vector3(0, 0, 0), physicslib::Vector3(0, 0, 0), physicslib::Vector3(0, 0, 0), physicslib::Vector3(0, 0, 0));

	m_forceRegister.add(gravityRecord);
}

void PhysicEngine::update(const double frametime)
{
	// Generates all forces and add them in the force register
	generateAllForces();

	// applies the forces inside the force register
	m_forceRegister.updateAllForces(frametime);

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
