#include "rigidBody.hpp"


#include <iostream>

namespace physicslib
{
	RigidBody::RigidBody(
		const double mass, const double angularDamping, const Vector3 boxSize,
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
		, m_boxSize(boxSize)
	{
		// Hardcoded box inertia tensor
		m_inverseInertiaTensor = Matrix3({
			(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ()) * mass / 12., 0, 0,
			0, (boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ()) * mass / 12., 0,
			0, 0, (boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ()) * mass / 12.
		}).getReverseMatrix();
	}

	RigidBody::RigidBody(
		const double mass, const double angularDamping, const double radius,
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
		// Hardcoded spheric inertia tensor
		m_inverseInertiaTensor = Matrix3({
			2. * mass * radius * radius / 5., 0, 0,
			0, 2. * mass * radius * radius / 5., 0,
			0, 0, 2. * mass * radius * radius / 5.
		}).getReverseMatrix();
	}

	RigidBody::RigidBody(
		const double mass, const double angularDamping, const std::vector<Vector3>& points,
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
		Vector3 xAxis = Vector3(1, 0, 0);
		Vector3 yAxis = Vector3(0, 1, 0);
		Vector3 zAxis = Vector3(0, 0, 1);

		double xInertia = 0.;
		double yInertia = 0.;
		double zInertia = 0.;

		double xyInertia = 0.;
		double yzInertia = 0.;
		double xzInertia = 0.;
		for (Vector3 point : points)
		{
			double xScalarProduct = (point - m_position).ScalarProduct(xAxis);
			double yScalarProduct = (point - m_position).ScalarProduct(yAxis);
			double zScalarProduct = (point - m_position).ScalarProduct(zAxis);

			// Compute moment of inertia
			xInertia += mass * (yScalarProduct * yScalarProduct + zScalarProduct * zScalarProduct);
			yInertia += mass * (xScalarProduct * xScalarProduct + zScalarProduct * zScalarProduct);
			zInertia += mass * (xScalarProduct * xScalarProduct + yScalarProduct * yScalarProduct);

			// Compute inertia product
			xyInertia += mass * xScalarProduct * yScalarProduct;
			xzInertia += mass * xScalarProduct * zScalarProduct;
			yzInertia += mass * yScalarProduct * zScalarProduct;
		}

		m_inverseInertiaTensor = Matrix3({
			xInertia, -xyInertia, -xzInertia,
			-xyInertia, yInertia, -yzInertia,
			-xzInertia, -yzInertia, zInertia
		});
		m_inverseInertiaTensor /= points.size();
		m_inverseInertiaTensor.reverse();
	}

	void RigidBody::integrate(double frameTime)
	{
		// Position update
		m_acceleration = m_forceAccumulator;
		m_velocity = m_velocity + m_acceleration * frameTime;
		m_position = m_position + m_velocity * frameTime;

		// Orientation update
		m_angularAcceleration = m_inverseInertiaTensor * m_torqueAccumulator;
		m_rotation = m_rotation * pow(m_angularDamping, frameTime) + m_angularAcceleration * frameTime;
		m_orientation.updateOrientation(m_rotation, frameTime);
		
		computeDerivedData();
		clearAccumulators();
	}

	void RigidBody::computeDerivedData()
	{
		// TO VERIFY
		m_transformMatrix = Matrix3((Quaternion(0, m_position) * m_orientation).getNormalizedQuaternion());

		m_inverseInertiaTensor = m_transformMatrix * m_inverseInertiaTensor * m_transformMatrix.getReverseMatrix();
	}

	void RigidBody::addForceAtPoint(const Vector3& force, const Vector3& point)
	{
		// Convert point to coordinates relative to the center-of-mass
		Vector3 localPoint = point.worldToLocal(m_transformMatrix);

		m_forceAccumulator += force;
		m_torqueAccumulator += localPoint.CrossProduct(force);
	}

	void RigidBody::addForceAtBodyPoint(const Vector3& force, const Vector3& point)
	{
		// Convert point to coordinates relative to the world
		Vector3 worldPoint = point.localToWorld(m_transformMatrix);

		addForceAtPoint(force, worldPoint);
	}

	void RigidBody::clearAccumulators()
	{
		m_forceAccumulator = physicslib::Vector3();
		m_torqueAccumulator = physicslib::Vector3();
	}

	std::vector<double> RigidBody::getBoxVertices() const
	{
		std::vector<double> vertices = getBoxLocalVertices();
		applyRotation(vertices);
		toWorldSpace(vertices);
		return vertices;
	}

	std::vector<double> RigidBody::getBoxLocalVertices() const
	{
		std::vector<double> vertices =
		{
			- m_boxSize.getX() / 2, - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2, - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,

			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,

			- m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,

			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,

			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  - m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,

			- m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			+ m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  + m_boxSize.getZ() / 2,
			- m_boxSize.getX() / 2,  + m_boxSize.getY() / 2,  - m_boxSize.getZ() / 2,
		};
		return vertices;
	}

	void RigidBody::toWorldSpace(std::vector<double>& vertices) const
	{
		for (unsigned int i = 0; i < vertices.size() - 2; i += 3)
		{
			vertices[i] += m_position.getX();
			vertices[i + 1] += m_position.getY();
			vertices[i + 2] += m_position.getZ();
		}
	}

	void RigidBody::applyRotation(std::vector<double>& vertices) const
	{
		Matrix3 orientationMatrix(m_orientation);
		for (unsigned int i = 0; i < vertices.size() - 2; i += 3)
		{
			physicslib::Vector3 vertex(vertices.at(i), vertices.at(i + 1), vertices.at(i + 2));
			vertex = orientationMatrix * vertex;
			vertices[i] = vertex.getX();
			vertices[i + 1] = vertex.getY();
			vertices[i + 2] = vertex.getZ();
		}
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