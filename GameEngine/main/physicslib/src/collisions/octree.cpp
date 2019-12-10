#include "collisions/octree.hpp"

#include <utility>

namespace physicslib
{
	const PlanePrimitive* Octree::m_rightPlane = nullptr;
	const PlanePrimitive* Octree::m_leftPlane = nullptr;
	const PlanePrimitive* Octree::m_topPlane = nullptr;
	const PlanePrimitive* Octree::m_bottomPlane = nullptr;

	Octree::Octree(const unsigned int pLevel, const BoundingBox& pBounds): m_level(pLevel), m_bounds(pBounds)
	{
	}

	void Octree::clear()
	{
		m_points.clear();

		std::for_each(m_nodes.begin(), m_nodes.end(),
			[](Octree& node)
			{
				node.clear();
			});
		m_nodes.clear();
	}

	void Octree::split()
	{
		double subWidth = m_bounds.width / 2;
		double subHeight = m_bounds.height / 2;
		double subDepth = m_bounds.depth / 2;

		// We create the different new octrees
		BoundingBox node;
		node.x = m_bounds.x;
		node.y = m_bounds.y;
		node.z = m_bounds.z;
		node.width = subWidth;
		node.height = subHeight;
		node.depth = subDepth;
		m_nodes.push_back(Octree(m_level + 1, node));

		node.x = m_bounds.x + subWidth;
		node.y = m_bounds.y;
		node.z = m_bounds.z;
		node.width = subWidth;
		node.height = subHeight;
		node.depth = subDepth;
		m_nodes.push_back(Octree(m_level + 1, node));

		node.x = m_bounds.x;
		node.y = m_bounds.y + subHeight;
		node.z = m_bounds.z;
		node.width = subWidth;
		node.height = subHeight;
		node.depth = subDepth;
		m_nodes.push_back(Octree(m_level + 1, node));

		node.x = m_bounds.x;
		node.y = m_bounds.y;
		node.z = m_bounds.z + subDepth;
		node.width = subWidth;
		node.height = subHeight;
		node.depth = subDepth;
		m_nodes.push_back(Octree(m_level + 1, node));

		node.x = m_bounds.x + subDepth;
		node.y = m_bounds.y + subHeight;
		node.z = m_bounds.z;
		node.width = subWidth;
		node.height = subHeight;
		node.depth = subDepth;
		m_nodes.push_back(Octree(m_level + 1, node));

		node.x = m_bounds.x + subDepth;
		node.y = m_bounds.y;
		node.z = m_bounds.z + subDepth;
		node.width = subWidth;
		node.height = subHeight;
		node.depth = subDepth;
		m_nodes.push_back(Octree(m_level + 1, node));

		node.x = m_bounds.x;
		node.y = m_bounds.y + subHeight;
		node.z = m_bounds.z + subDepth;
		node.width = subWidth;
		node.height = subHeight;
		node.depth = subDepth;
		m_nodes.push_back(Octree(m_level + 1, node));

		node.x = m_bounds.x + subWidth;
		node.y = m_bounds.y + subHeight;
		node.z = m_bounds.z + subDepth;
		node.width = subWidth;
		node.height = subHeight;
		node.depth = subDepth;
		m_nodes.push_back(Octree(m_level + 1, node));
	}

	int Octree::getIndex(Vector3 point) const
	{
		int index;
		double verticalMidpoint = m_bounds.x + (m_bounds.width / 2);
		double horizontalMidpoint = m_bounds.y + (m_bounds.height / 2);
		double depthMidPoint = m_bounds.z + (m_bounds.depth / 2);

		// We look on the 3 different axes
		bool topQuadrant = (point.getY() > verticalMidpoint);
		bool rightQuadrant = (point.getX() > horizontalMidpoint);
		bool farQuadrant = (point.getZ() > depthMidPoint);

		// We return the index in functions of the position
		if (!topQuadrant && !rightQuadrant && !farQuadrant)
		{
			index = 0;
		}
		else if (!topQuadrant && rightQuadrant && !farQuadrant)
		{
			index = 1;
		}
		else if (topQuadrant && !rightQuadrant && !farQuadrant)
		{
			index = 2;
		}
		else if (!topQuadrant && !rightQuadrant && farQuadrant)
		{
			index = 3;
		}
		else if (topQuadrant && rightQuadrant && !farQuadrant)
		{
			index = 4;
		}
		else if (!topQuadrant && rightQuadrant && farQuadrant)
		{
			index = 5;
		}
		else if (topQuadrant && !rightQuadrant && farQuadrant)
		{
			index = 6;
		}
		else if (topQuadrant && rightQuadrant && farQuadrant)
		{
			index = 7;
		}

		return index;
	}

	void Octree::insert(std::shared_ptr<Primitive> body)
	{
		std::vector<Vector3> bodyPoints = body->getVertices();
		std::for_each(bodyPoints.begin(), bodyPoints.end(),
			[this](Vector3 point)
			{
				insert(point);
			});
	}

	void Octree::insert(Vector3 point)
	{
		// if we already has nodes we just insert in the good nodes
		if (hasNodes())
		{
			int index = getIndex(point);
			m_nodes.at(index).insert(point);
			return;
		}

		// if we don't we add inside this octree
		m_points.push_back(point);
		// if we have too many point we split the tree and insert the points in the new nodes
		if (m_points.size() > MAX_RIGIDBODY_BY_LEVEL && m_level < MAX_LEVELS)
		{
			split();
			std::for_each(m_points.begin(), m_points.end(),
				[this](Vector3 oldPoint)
				{
					int index = getIndex(oldPoint);
					m_nodes.at(index).insert(oldPoint);
				});
			m_points.clear();
		}
	}

	bool Octree::hasNodes() const
	{
		if (m_nodes.size() != 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	void Octree::retrieve(std::vector<std::pair<Vector3, const PlanePrimitive *>>& collisions, bool top, bool right, bool bottom, bool left) const
	{
		/* In this function, we pass 4 bool depending of the position of the octree in the space
		 * For example _________
		 *             |___|___|
		               |___|___|
		 * The bottom right square has right to true and bottom to true because it is on the bottom right and it cans
		 * touch the right plane and the bottom plane. 
		 */

		/*
		 * If this is a leave, we add the collision with the planes depending on the bool which are still set to true.
		 */
		if (!hasNodes())
		{
			for (unsigned int i = 0; i < m_points.size(); ++i)
			{
				if (top)
				{
					collisions.push_back(std::make_pair(m_points.at(i), m_topPlane));
				}
				if (left)
				{
					collisions.push_back(std::make_pair(m_points.at(i), m_leftPlane));
				}
				if (bottom)
				{
					collisions.push_back(std::make_pair(m_points.at(i), m_bottomPlane));
				}
				if (right)
				{
					collisions.push_back(std::make_pair(m_points.at(i), m_rightPlane));
				}
			}
		}
		/*
		 * If this is a node, we call retrieve on all the child nodes.
		 * For each node we update the 4 bools.
		 * For example, if the current node is on the top right and the child node
		 * on the bottom right. The only plane possible is on the right. 
		 */
		else
		{
			for (unsigned int i = 0; i < m_nodes.size(); ++i)
			{
				if (i == 0)
				{
					m_nodes.at(0).retrieve(collisions, false, false, true && bottom, true && left);
				}
				if (i == 1)
				{
					m_nodes.at(1).retrieve(collisions, false, true && right, true && bottom, false);
				}
				if (i == 2)
				{
					m_nodes.at(2).retrieve(collisions, true && top, false, false, true && left);
				}
				if (i == 3)
				{
					m_nodes.at(3).retrieve(collisions, false, false, true && bottom, true && left);
				}
				if (i == 4)
				{
					m_nodes.at(4).retrieve(collisions, true && top, true && right, false, false);
				}
				if (i == 5)
				{
					m_nodes.at(5).retrieve(collisions, false, true && right, true && bottom, false);
				}
				if (i == 6)
				{
					m_nodes.at(6).retrieve(collisions, true && top, false, false, true && left);
				}
				if (i == 7)
				{
					m_nodes.at(7).retrieve(collisions, true && top, true && right, false, false);
				}
			}
		}
	}

	void Octree::setTopPlane(const PlanePrimitive * const topPlane)
	{
		m_topPlane = topPlane;
	}

	void Octree::setBottomPlane(const PlanePrimitive * const bottomPlane)
	{
		m_bottomPlane = bottomPlane;
	}

	void Octree::setRightPlane(const PlanePrimitive * const rightPlane)
	{
		m_rightPlane = rightPlane;
	}

	void Octree::setLeftPlane(const PlanePrimitive * const leftPlane)
	{
		m_leftPlane = leftPlane;
	}
}