#include "../include/physicEngine.hpp"

#include <iostream>
#include "math/vector3.hpp"
#include "collisions/planePrimitive.hpp"
#include "collisions/boxPrimitive.hpp"
#include "collisions/octree.hpp"

PhysicEngine::PhysicEngine()
	: m_leftPlane(physicslib::Vector3(1, 0, 0), 50)
	, m_rightPlane(physicslib::Vector3(-1, 0, 0), -50)
	, m_topPlane(physicslib::Vector3(0, -1, 0), -40)
	, m_bottomPlane(physicslib::Vector3(0, 1, 0), 40)
{
	physicslib::Octree::setBottomPlane(&m_bottomPlane);
	physicslib::Octree::setTopPlane(&m_topPlane);
	physicslib::Octree::setRightPlane(&m_rightPlane);
	physicslib::Octree::setLeftPlane(&m_leftPlane);

	/*physicslib::BoundingBox octreeBox;
	octreeBox.x = 0;
	octreeBox.y = 0;
	octreeBox.z = 0;
	octreeBox.width = 800;
	octreeBox.height = 600;
	octreeBox.depth = 1000;
	physicslib::Octree octree(0, octreeBox);

	physicslib::Vector3 v1(100, 0, 0);
	octree.insert(v1);
	std::cout << octree.getIndex(v1) << std::endl;
	physicslib::Vector3 v2(101, 0, 0);
	octree.insert(v2);
	std::cout << octree.getIndex(v2) << std::endl;
	physicslib::Vector3 v3(0, 500, 0);
	octree.insert(v3);
	std::cout << octree.getIndex(v3) << std::endl;
	physicslib::Vector3 v4(0, 0, 900);
	octree.insert(v4);
	std::cout << octree.getIndex(v4) << std::endl;

	std::vector<std::pair<physicslib::Vector3, const physicslib::PlanePrimitive*>> result;
	octree.retrieve(result, true, true, true, true);
	std::cout << result.size() << std::endl;*/
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
	std::vector<std::pair<physicslib::Vector3, const physicslib::PlanePrimitive*>> result;
	broadPhase(rigidBodies, result);
	std::vector<physicslib::Contact> collisionData = narrowPhase(result);

	for (const physicslib::Contact& contact : collisionData)
	{
		std::cout << contact.toString() << std::endl;
	}

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

void PhysicEngine::broadPhase(std::vector<std::shared_ptr<physicslib::RigidBody>>& rigidBodies, std::vector<std::pair<physicslib::Vector3, const physicslib::PlanePrimitive*>>& result)
{
	physicslib::BoundingBox octreeBox;
	octreeBox.x = 0;
	octreeBox.y = 0;
	octreeBox.z = 0;
	octreeBox.width = 100;
	octreeBox.height = 80;
	octreeBox.depth = 1000;
	physicslib::Octree octree(0, octreeBox);

	std::for_each(rigidBodies.begin(), rigidBodies.end(),
		[&octree](std::shared_ptr<physicslib::RigidBody> rigidBody)
		{
			std::shared_ptr<physicslib::BoxPrimitive> boxPrimitive = std::make_shared<physicslib::BoxPrimitive>(rigidBody);
			octree.insert(boxPrimitive);
		});

	octree.retrieve(result, true, true, true, true);
}

std::vector<physicslib::Contact> PhysicEngine::narrowPhase(std::vector<std::pair<physicslib::Vector3, const physicslib::PlanePrimitive*>>& possibleCollisions)
{
	std::vector<physicslib::Contact> collisionData;

	for (std::pair<physicslib::Vector3, const physicslib::PlanePrimitive*> collisionPair : possibleCollisions)
	{
		double penetration = collisionPair.second->getNormal() * collisionPair.first + abs(collisionPair.second->getOffset());
		if (penetration < 0)
		{
			// Contact point at half way between point and plane
			physicslib::Vector3 contactPoint(collisionPair.first);

			collisionData.push_back(physicslib::Contact(contactPoint, collisionPair.second->getNormal(), -penetration));
		}
	}

	return collisionData;
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

			collisionData.push_back(physicslib::Contact(contactPoint, planePrimitive.getNormal(), planePrimitive.getNormal() * vertex + planePrimitive.getOffset()));
		}
	}

	return collisionData;
}