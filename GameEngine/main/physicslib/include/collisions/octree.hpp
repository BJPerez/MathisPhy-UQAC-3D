#pragma once

#include <vector>
#include <memory>
#include <algorithm>

#include "planePrimitive.hpp"

namespace physicslib
{
	// Struc used to give bounds to an octree
	struct BoundingBox
	{
		// x, y, z represent the bottom left point of the octree
		double x;
		double y;
		double z;
		double width;
		double height;
		double depth;
	};

	class Octree
	{
	public:
		// Constructor
		Octree(const unsigned int pLevel, const BoundingBox& pBounds);

		// Function used to clear the octree.
		// Clean all the points and remove the subdivisions.
		void clear();

		// Insert an object in the octree.
		// It first cut the object in points and then add each point to the octree.
		void insert(std::shared_ptr<Primitive> body);

		// Get the list of all the points that are next to a plane associated with that plane.
		void retrieve(std::vector<std::pair<Vector3, const PlanePrimitive * >>& collisions, bool top, bool right, bool bottom, bool left) const;

		// Return if the octree has nodes.
		bool hasNodes() const;

		// Setters for the planes.
		static void setTopPlane(const PlanePrimitive * const topPlane);
		static void setBottomPlane(const PlanePrimitive * const bottomPlane);
		static void setRightPlane(const PlanePrimitive * const rightPlane);
		static void setLeftPlane(const PlanePrimitive * const leftPlane);

	private:
		static const unsigned int MAX_RIGIDBODY_BY_LEVEL = 1; // The maximum number of points by subdivision level
		static const unsigned int MAX_LEVELS = 8; // The maximum number of level
		static const PlanePrimitive * m_topPlane;
		static const PlanePrimitive * m_bottomPlane;
		static const PlanePrimitive * m_leftPlane;
		static const PlanePrimitive * m_rightPlane;

		const unsigned int m_level; // The level of this octree
		const BoundingBox m_bounds; // The bounds of the octree
		std::vector<Vector3> m_points; // The points contained by the octree
		std::vector<Octree> m_nodes; // The subdivision nodes of the octree

		// Split the octree in 8
		void split();

		// Return the index of the node in which the given point should be.
		int getIndex(Vector3 point) const;

		// Insert the point iin the octree.
		void insert(Vector3 point);
	};
}