#include "../include/physicEngine.hpp"

#include <iostream>
#include "math/vector3.hpp"
#include "collisions/planePrimitive.hpp"
#include "collisions/boxPrimitive.hpp"

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
		if (rigidBodies[0]->getPosition().getX() > 10)
		{
			physicslib::BoxPrimitive primitive1(rigidBodies.at(0));
			physicslib::PlanePrimitive primitive2(physicslib::Vector3(-1, 0, 0), 10);
			std::vector<physicslib::Contact> contacts = generateContacts(primitive1, primitive2);

			for (const auto& contact : contacts)
			{
				std::cout << contact.toString() << std::endl;
			}
		}
	}
}

std::vector<physicslib::Contact> PhysicEngine::generateContacts(const physicslib::Primitive& primitive1, const physicslib::Primitive& primitive2) const
{
	// Check if primitive1 is a plane primitive
	if (primitive1.getVertices().empty())
	{
		return generateContactsDerived<physicslib::PlanePrimitive, physicslib::BoxPrimitive>(&PhysicEngine::generateContactsVertexFace, primitive1, primitive2);
	}

	// Check if primitive2 is a plane primitive
	if (primitive2.getVertices().empty())
	{
		return generateContactsDerived<physicslib::PlanePrimitive, physicslib::BoxPrimitive>(&PhysicEngine::generateContactsVertexFace, primitive2, primitive1);
	}

	return std::vector<physicslib::Contact>();
}

std::vector<physicslib::Contact> PhysicEngine::generateContactsVertexFace(const physicslib::PlanePrimitive& planePrimitive, const physicslib::BoxPrimitive& boxPrimitive)
{
	std::vector<physicslib::Contact> collisionData;

	for (physicslib::Vector3 vertex : boxPrimitive.getVertices())
	{
		if (planePrimitive.getNormal() * vertex <= -planePrimitive.getOffset())
		{
			// Contact point at half way between point and plane
			physicslib::Vector3 contactPoint((vertex + planePrimitive.getNormal() * planePrimitive.getOffset()) / 2.);

			collisionData.push_back(physicslib::Contact(contactPoint, planePrimitive.getNormal(), planePrimitive.getOffset()));
		}
	}

	return collisionData;
}