/*#include "collisions/octree.hpp"

namespace physicslib
{
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

		// Object can completely fit within the top quadrants
		bool topQuadrant = (point.y > verticalMidpoint);
		bool rightQuadrant = (point.x > horizontalMidpoint);
		bool farQuadrant = (point.z > depthMidPoint);

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
			[](Vector3 point)
			{
				insert(point);
			});
	}

	void Octree::insert(Vector3 point)
	{
		if (hasNodes())
		{
			int index = getIndex(point);
			m_nodes.at(index).insert(point);
			return;
		}

		m_points.push_back(point);

		if (m_points.size() > MAX_RIGIDBODY_BY_LEVEL && m_level < MAX_LEVELS)
		{
			if (!hasNodes())
			{
				split();
			}

			std::for_each(m_colliders.begin(), m_colliders.end(),
				[](Vector3 oldPoint)
				{
					int index = getIndex(oldPoint);
					m_nodes.at(index).insert(oldPoint);
				}
			m_colliders.clear();
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

	void Octree::retrieve(std::vector<std::pair<Vector3, const PlanePrimitive * const>& collisions, bool top, bool right, bool bottom, bool left) const
	{
		if (!hasNodes())
		{
			std::for_each(m_points.begin(), m_points.end(),
				[top, right, bottom, left](Vector3 point)
				{
					if (top)
					{
						collisions.insert(std::make_pair<Vector3, const PlanePrimitive* const>(point, m_topPlane));
					}
					if (left)
					{
						collisions.insert(std::make_pair<Vector3, const PlanePrimitive* const>(point, m_leftPlane));
					}
					if (bottom)
					{
						collisions.insert(std::make_pair<Vector3, const PlanePrimitive* const>(point, m_bottomPlane));
					}
					if (right)
					{
						collisions.insert(std::make_pair<Vector3, const PlanePrimitive* const>(point, m_rightPlane));
					}
				});
		}
		else
		{
			for (unsigned int i = 0; i < m_nodes.size(); ++i)
			{
				if (i == 0)
				{
					retrieve(collisions, false, false, true && bottom, true && left);
				}
				if (i == 1)
				{
					retrieve(collisions, false, true && right, true && bottom, false);
				}
				if (i == 2)
				{
					retrieve(collisions, true && top, false, false, true && left);
				}
				if (i == 3)
				{
					retrieve(collisions, false, false, true && bottom, true && left);
				}
				if (i == 4)
				{
					retrieve(collisions, true && top, true && right, false, false);
				}
				if (i == 5)
				{
					retrieve(collisions, false, true && right, true && bottom, false);
				}
				if (i == 6)
				{
					retrieve(collisions, true && top, false, false, true && left);
				}
				if (i == 7)
				{
					retrieve(collisions, true && top, true && right, false, false);
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
}*/