#pragma once
#include <string>
#include "math/vector3.hpp"

namespace physicslib
{	
	class Particle
	{
	public:

		static const unsigned int PARTICLE_RADIUS = 10;
		Particle(double inverseMass = 0, physicslib::Vector3 position = physicslib::Vector3(),
			physicslib::Vector3 speed = physicslib::Vector3(), physicslib::Vector3 acceleration = physicslib::Vector3());
		Particle(Particle const& anotherParticle);
		virtual ~Particle();

		physicslib::Vector3 getPosition() const;
		physicslib::Vector3 getColor() const;
		physicslib::Vector3 getSpeed() const;
		physicslib::Vector3 getAcceleration() const;
		double getInverseMass() const;

		// Updates position, speed, acceleration using Newton laws
		void integrate(double frameTime = 0.0333333);
		void addForce(const physicslib::Vector3& force);
		void setSpeed(const physicslib::Vector3& newSpeed);
		void setPosition(const physicslib::Vector3& newPosition);
		void clearAccumulator();
		bool isVisible(unsigned int xMax, unsigned int yMax) const;
		bool isInContactWith(const Particle& particle) const;
		double getDistance(const Particle& particle) const;

		std::string toString() const;

	private:
		double m_inverseMass;
		physicslib::Vector3 m_position;
		physicslib::Vector3 m_speed;
		physicslib::Vector3 m_acceleration;
		physicslib::Vector3 m_forceAccumulator;
		physicslib::Vector3 m_color;
	};
}