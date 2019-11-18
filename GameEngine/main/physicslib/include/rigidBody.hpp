#pragma once

#include "math/vector3.hpp"
#include "math/quaternion.hpp"
#include "math/matrix34.hpp"
#include "math/matrix3.hpp"

namespace physicslib
{
	class RigidBody
	{
	public:
		/**
		 * Constructors
		 */
		RigidBody(
			const double mass,
			const Vector3 initialPosition = Vector3(), const Vector3 initialSpeed = Vector3(), const Vector3 initialAcceleration = Vector3(),
			const Quaternion initialOrientation = Quaternion(), const Vector3 initialRotation = Vector3()
		);
		RigidBody(const RigidBody& anotherRigidBody) = default;

		/**
		 * Virtual destructor
		 */
		virtual ~RigidBody() = default;

		/**
		 * Compute transformation matrix and inertia tensor each frame
		 */
		void computeDerivedData();

		/**
		 * Apply a force to a point in the world
		 */
		void addForceAtPoint(const Vector3& force, const Vector3& point);

		/**
		 * Apply a force to a point in the rigid body
		 */
		void addForceAtBodyPoint(const Vector3& force, const Vector3& point);

		/**
		 * Clear force and torque accumulators
		 */
		void clearAccumulators();

		#pragma region Getters/Setters

		// Getters
		double getInverseMass() const;
		physicslib::Vector3 getPosition() const;
		physicslib::Vector3 getSpeed() const;
		physicslib::Vector3 getAcceleration() const;
		physicslib::Quaternion getOrientation() const;
		physicslib::Vector3 getRotation() const;

		// Setters
		void setPosition(physicslib::Vector3 position);
		void setSpeed(physicslib::Vector3 speed);
		void setAcceleration(physicslib::Vector3 acceleration);
		void setOrientation(physicslib::Quaternion orientation);
		void setRotation(physicslib::Vector3 rotation);

		#pragma endregion

	private:
		double m_inverseMass;
		physicslib::Vector3 m_position;
		physicslib::Vector3 m_speed;
		physicslib::Vector3 m_acceleration;
		physicslib::Vector3 m_forceAccumulator;
		physicslib::Quaternion m_orientation;
		physicslib::Vector3 m_rotation;
		physicslib::Vector3 m_torqueAccumulator;

		// Computed data
		physicslib::Matrix3 m_transformMatrix;
		physicslib::Matrix3 m_inverseInertiaTensor;
	};
}