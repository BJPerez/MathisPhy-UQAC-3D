#include "../include/physicEngine.hpp"

#include <iostream>
#include "math/vector3.hpp"
#include "collisions/planePrimitive.hpp"
#include "collisions/boxPrimitive.hpp"
#include "collisions/octree.hpp"
#include <chrono>
#include <thread>


PhysicEngine::PhysicEngine()
	: m_leftPlane(physicslib::Vector3(1, 0, 0), 55)
	, m_rightPlane(physicslib::Vector3(-1, 0, 0), -55)
	, m_topPlane(physicslib::Vector3(0, -1, 0), -41)
	, m_bottomPlane(physicslib::Vector3(0, 1, 0), 41)
{
	physicslib::Octree::setBottomPlane(&m_bottomPlane);
	physicslib::Octree::setTopPlane(&m_topPlane);
	physicslib::Octree::setRightPlane(&m_rightPlane);
	physicslib::Octree::setLeftPlane(&m_leftPlane);
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

	if (!collisionData.empty())
	{
		using namespace std::chrono_literals;
		std::this_thread::sleep_for(1000000s);
		exit(EXIT_SUCCESS);
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
	octreeBox.width = 110;
	octreeBox.height = 82;
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