#include "rigidBody.hpp"

namespace physicslib
{
	RigidBody::RigidBody(
		const double mass, const double angularDamping,
		const Vector3 initialPosition, const Vector3 initialVelocity, const Vector3 initialAcceleration,
		const Quaternion initialOrientation, const Vector3 initialRotation, const Vector3 initialAngularAcceleration
	)
		: m_inverseMass(1. / mass)
		, m_angularDamping(angularDamping)
		, m_position(initialPosition)
		, m_velocity(initialVelocity)
		, m_acceleration(initialAcceleration)
		, m_orientation(initialOrientation)
		, m_rotation(initialRotation)
		, m_angularAcceleration(initialAngularAcceleration)
	{
		// Hardcoded 1x1x1 box inertia tensor
		m_inverseInertiaTensor = Matrix3({
			mass / 6., 0, 0,
			0, mass / 6., 0,
			0, 0, mass / 6.
		}).getReverseMatrix();
	}

	void RigidBody::integrate(double frameTime)
	{
		// Position update
		m_acceleration = m_forceAccumulator;
		m_velocity = m_velocity + m_acceleration * frameTime;
		m_position = m_position + m_velocity * frameTime;
		

		// Orientation update
		m_acceleration = m_inverseInertiaTensor * m_torqueAccumulator;
		m_rotation = m_rotation * pow(m_angularDamping, frameTime) + m_angularAcceleration * frameTime;
		m_orientation.updateAngularVelocity(m_rotation, frameTime);
		
		computeDerivedData();
		clearAccumulators();
	}

	void RigidBody::computeDerivedData()
	{
		// TO VERIFY
		m_transformMatrix = Matrix3(Quaternion(0, m_position) * m_orientation);

		m_inverseInertiaTensor = m_transformMatrix * m_inverseInertiaTensor * m_transformMatrix.getReverseMatrix();
	}

	void RigidBody::addForceAtPoint(const Vector3& force, const Vector3& point)
	{
		// Convert point to coordinates relative to the center-of-mass
		Vector3 localPoint(m_transformMatrix * point);

		m_forceAccumulator += force;
		m_torqueAccumulator += localPoint.CrossProduct(force);
	}

	void RigidBody::addForceAtBodyPoint(const Vector3& force, const Vector3& point)
	{
		// Convert point to coordinates relative to the world
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

	physicslib::Vector3 RigidBody::getVelocity() const
	{
		return m_velocity;
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

	void RigidBody::setVelocity(physicslib::Vector3 velocity)
	{
		m_velocity = velocity;
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