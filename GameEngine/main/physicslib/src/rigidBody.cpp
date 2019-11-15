#include "rigidBody.hpp"

namespace physicslib
{
	RigidBody::RigidBody(
		const double mass,
		const Vector3 initialPosition, const Vector3 initialSpeed, const Vector3 initialAcceleration,
		const Quaternion initialOrientation, const Vector3 initialRotation
	)
		: m_inverseMass(1 / mass)
		, m_position(initialPosition)
		, m_speed(initialSpeed)
		, m_acceleration(initialAcceleration)
		, m_orientation(initialOrientation)
		, m_rotation(initialRotation)
	{
	}

	void RigidBody::computeDerivedData()
	{
		// TO VERIFY
		m_transformMatrix = Matrix3(Quaternion(0, m_position) * m_orientation);

		m_inverseInertiaTensor = m_transformMatrix * m_inverseInertiaTensor * m_transformMatrix.getReverseMatrix();
	}

	void RigidBody::addForceAtPoint(const Vector3& force, const Vector3& point)
	{
		Vector3 localPoint(m_transformMatrix * point);

		m_forceAccumulator += force;
		m_torqueAccumulator += localPoint.CrossProduct(force);
	}

	void RigidBody::addForceAtBodyPoint(const Vector3& force, const Vector3& point)
	{
		Vector3 worldPoint(m_transformMatrix.getReverseMatrix() * point);

		addForceAtPoint(force, worldPoint);
	}

	void RigidBody::clearAccumulators()
	{
		m_forceAccumulator = physicslib::Vector3();
		m_torqueAccumulator = physicslib::Vector3();
	}

	#pragma region Getters/Setters

	double RigidBody::getInverseMass() const
	{
		return m_inverseMass;
	}

	physicslib::Vector3 RigidBody::getPosition() const
	{
		return m_position;
	}

	physicslib::Vector3 RigidBody::getSpeed() const
	{
		return m_speed;
	}

	physicslib::Vector3 RigidBody::getAcceleration() const
	{
		return m_acceleration;
	}

	physicslib::Quaternion RigidBody::getOrientation() const
	{
		return m_orientation;
	}

	physicslib::Vector3 RigidBody::getRotation() const
	{
		return m_rotation;
	}

	void RigidBody::setPosition(physicslib::Vector3 position)
	{
		m_position = position;
	}

	void RigidBody::setSpeed(physicslib::Vector3 speed)
	{
		m_speed = speed;
	}

	void RigidBody::setAcceleration(physicslib::Vector3 acceleration)
	{
		m_acceleration = acceleration;
	}

	void RigidBody::setOrientation(physicslib::Quaternion orientation)
	{
		m_orientation = orientation;
	}

	void RigidBody::setRotation(physicslib::Vector3 rotation)
	{
		m_rotation = rotation;
	}

	#pragma endregion
}