#pragma once

#include "collisions/primitive.hpp"
#include "math/vector3.hpp"

namespace physicslib
{
	class PlanePrimitive : public Primitive
	{
	public:
		/**
		 * Contructor
		 */
		PlanePrimitive(const Vector3& normal, double offset);

		/**
		 * Default copy constructor
		 */
		PlanePrimitive(const PlanePrimitive& anotherPlanePrimitive) = default;

		/**
		 * Virtual destructor
		 */
		virtual ~PlanePrimitive() = default;

		/**
		 * Default assignment operator
		 */
		PlanePrimitive& operator=(const PlanePrimitive& anotherPlanePrimitive) = default;

		/**
		 * Get an empty array of vertices
		 * Planes don't have vertices
		 * Used to check if a primitive is a plane with polymorphism
		 */
		virtual std::vector<Vector3> getVertices() const;

		#pragma region Getters

		Vector3 getNormal() const;
		double getOffset() const;

		#pragma endregion

	private:
		Vector3 m_normal;
		double m_offset;
	};
}