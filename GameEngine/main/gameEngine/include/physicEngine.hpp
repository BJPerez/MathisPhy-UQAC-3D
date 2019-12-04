#pragma once

#include "forceGenerator/forceRegister.hpp"
#include "collisions/contactRegister.hpp"
#include "rigidBody.hpp"
#include "forceGenerator/rigidBodyGravityForceGenerator.hpp"
#include "forceGenerator/rigidBodyDragForceGenerator.hpp"
#include "collisions/primitive.hpp"
#include "collisions/planePrimitive.hpp"
#include "collisions/boxPrimitive.hpp"
#include "collisions/contact.hpp"

/**
 * This class represents the physic engine and implements all the functions 
 * needed to run the physic.
 */
class PhysicEngine
{
public:
	/**
	 * Constructor
	 */
	PhysicEngine();

	/**
	 * Realizes a whole loop of the physic engine. 
	 * Generates all forces, applies them to objects and detects and resolves collisions.
	 */
	void update(std::vector<std::shared_ptr<physicslib::RigidBody>>& rigidBodies, const double deltaTime);

private:
	physicslib::ForceRegister m_forceRegister; // The register containing all forces associated with the object they're applied to.
	physicslib::ContactRegister m_contactRegister; // The register containing all contacts between 2 objects.

	std::shared_ptr<physicslib::RigidBodyGravityForceGenerator> gravityGenerator = std::make_shared<physicslib::RigidBodyGravityForceGenerator>(physicslib::Vector3(0, -20, 0));
	std::shared_ptr<physicslib::RigidBodyDragForceGenerator> dragGenerator = std::make_shared<physicslib::RigidBodyDragForceGenerator>(0.03, 0);

	/**
	 * Function that generates all the forces and add them in the force register.
	 */
	void generateAllForces(std::vector<std::shared_ptr<physicslib::RigidBody>>& rigidBodies);

	/**
	 * Function that detects all the contacts between objects and add them in the contacts register.
	 */
	void detectContacts(std::vector<std::shared_ptr<physicslib::RigidBody>>& rigidBodies);

	/**
	 * Function that generates collision data between two primitives
	 */
	std::vector<physicslib::Contact> generateContacts(physicslib::Primitive* primitive1, physicslib::Primitive* primitive2) const;

	/**
	 * Function that generates collision data between a plane primitive and a box primitive
	 */
	std::vector<physicslib::Contact> generateContactsVertexFace(physicslib::PlanePrimitive* planePrimitive, physicslib::BoxPrimitive* boxPrimitive) const;
};