#pragma once

#include "forceGenerator/forceRegister.hpp"
#include "collisions/contactRegister.hpp"

/*
 * This class represents the physic engine and implements all the functions 
 * needed to run the physic.
 */
class PhysicEngine
{
public:
	/*
	 * Constructor
	 */
	PhysicEngine();

	/*
	 * Realizes a whole loop of the physic engine. 
	 * Generates all forces, applies them to objects and detects and resolves collisions.
	 */
	void update(const double deltaTime);

private:
	physicslib::ForceRegister m_forceRegister; // The register containing all forces associated with the object they're applied to.
	physicslib::ContactRegister m_contactRegister; // The register containing all contacts between 2 objects.

	/*
	 * Function that generates all the forces and add them in the force register.
	 */
	void generateAllForces();

	/*
	 * Function that detects all the contacts between objects and add them in the contacts register.
	 */
	void detectContacts();
};