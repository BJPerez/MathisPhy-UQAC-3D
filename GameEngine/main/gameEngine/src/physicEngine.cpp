#include "../include/physicEngine.hpp"

#include "math/vector3.hpp"
#include "collisions/planePrimitive.hpp"
#include "collisions/boxPrimitive.hpp"
#include "collisions/octree.hpp"

PhysicEngine::PhysicEngine():m_leftPlane(physicslib::Vector3(1, 0, 0), 0), m_rightPlane(physicslib::Vector3(-1, 0, 0), 800),
m_topPlane(physicslib::Vector3(0, -1, 0), 600), m_bottomPlane(physicslib::Vector3(0, 1, 0), 0)
{
	/*physicslib::Octree::setBottomPlane(&m_bottomPlane);
	physicslib::Octree::setTopPlane(&m_topPlane);
	physicslib::Octree::setRightPlane(&m_rightPlane);
	physicslib::Octree::setLeftPlane(&m_leftPlane);*/
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

	// MA FONCTION
	/*physicslib::BoundingBox octreeBox;
	octreeBox.x = 0;
	octreeBox.y = 0;
	octreeBox.z = 0;
	octreeBox.width = 800;
	octreeBox.height = 600;
	octreeBox.depth = 1000;
	physicslib::Octree octree(0, octreeBox);*/
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