#include "../include/particle.hpp"
#include <math.h>
#include <random>

namespace physicslib
{
	Particle::Particle(double inverseMass, physicslib::Vector3 position, physicslib::Vector3 speed, physicslib::Vector3 acceleration) :
		m_inverseMass(inverseMass), m_position(position), m_speed(speed), m_acceleration(acceleration), m_forceAccumulator()
	{
		std::random_device rd;  //Will be used to obtain a seed for the random number engine
		std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
		std::uniform_real_distribution<> dis(0.0, 1.0);
		double red = dis(gen);
		double green = dis(gen);
		double blue = dis(gen);
		m_color = Vector3(red, green, blue);
	}

	Particle::Particle(Particle const& anotherParticle) :
		m_inverseMass(anotherParticle.getInverseMass()), m_position(physicslib::Vector3(anotherParticle.getPosition())),
		m_speed(physicslib::Vector3(anotherParticle.getSpeed())), m_acceleration(physicslib::Vector3(anotherParticle.getAcceleration()))
	{
	}

	Particle::~Particle()
	{
	}

	//indicates whether or not a particle is visible or not on the screen
	bool Particle::isVisible(unsigned int xMax, unsigned int yMax) const
	{
		double x = getPosition().getX();
		double y = getPosition().getY();

		return (x - 5 < xMax && x + 5 > 0 && y - 5 < yMax && y + 5 > 0);
	}

	//indicates whether or not there is a contact between these two particles
	bool Particle::isInContactWith(const Particle& particle) const
	{
		return (getDistance(particle) <= 2. * PARTICLE_RADIUS);
	}

	double Particle::getDistance(const Particle& particle) const
	{
		return (m_position - particle.getPosition()).getNorm();
	}

	// Updates position, speed, acceleration using Newton laws
	void Particle::integrate(double frameTime)
	{
		setPosition(m_position + m_speed * frameTime);
		setSpeed(m_speed + m_acceleration * frameTime);
		m_acceleration = m_forceAccumulator;
		clearAccumulator();
	}

	void Particle::addForce(const physicslib::Vector3& force)
	{
		m_forceAccumulator += force;
	}

	physicslib::Vector3 Particle::getColor() const
	{
		return m_color;
	}

	void Particle::clearAccumulator()
	{
		m_forceAccumulator = physicslib::Vector3();
	}


	void Particle::setSpeed(const physicslib::Vector3& newSpeed) 
	{
		m_speed = newSpeed;
	}
	void Particle::setPosition(const physicslib::Vector3& newPosition)
	{
		m_position = newPosition;
	}

	double Particle::getInverseMass() const
	{
		return m_inverseMass;
	}

	physicslib::Vector3 Particle::getPosition() const
	{
		return m_position;
	}

	physicslib::Vector3 Particle::getSpeed() const
	{
		return m_speed;
	}

	physicslib::Vector3 Particle::getAcceleration() const
	{
		return m_acceleration;
	}

	std::string Particle::toString() const
	{
		std::string result = "inverseMass = " + std::to_string(m_inverseMass) + "\n";
		result += "position = (" + m_position.toString() + ")\n";
		result += "speed = (" + m_speed.toString() + ")\n";
		result += "acceleration = (" + m_acceleration.toString() + ")\n";
		return(result);
	}
}