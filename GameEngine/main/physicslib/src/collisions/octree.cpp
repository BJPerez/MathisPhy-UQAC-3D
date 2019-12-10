#include "collisions/octree.hpp"

namespace physicslib
{
	Octree::Octree(const unsigned int pLevel, const BoundingBox& pBounds): m_level(pLevel), m_bounds(pBounds)
	{
	}

	void Octree::clear()
	{
		m_colliders.clear();

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

	int Octree::getIndex(BoundingBox bodyBounds) const
	{
		int index = -1;
		double verticalMidpoint = m_bounds.x + (m_bounds.width / 2);
		double horizontalMidpoint = m_bounds.y + (m_bounds.height / 2);
		double depthMidPoint = m_bounds.z + (m_bounds.depth / 2);

		// Object can completely fit within the top quadrants
		bool topQuadrant = (bodyBounds.y > verticalMidpoint);
		// Object can completely fit within the bottom quadrants
		bool bottomQuadrant = ((bodyBounds.y + bodyBounds.height) < verticalMidpoint);

		bool rightQuadrant = (bodyBounds.x > horizontalMidpoint);
		// Object can completely fit within the bottom quadrants
		bool leftQuadrant = ((bodyBounds.x + bodyBounds.width) < horizontalMidpoint);

		bool farQuadrant = (bodyBounds.z > depthMidPoint);
		// Object can completely fit within the bottom quadrants
		bool nearQuadrant = ((bodyBounds.z + bodyBounds.depth) < depthMidPoint);

		if (bottomQuadrant && leftQuadrant && nearQuadrant)
		{
			index = 0;
		}
		else if (bottomQuadrant && rightQuadrant && nearQuadrant)
		{
			index = 1;
		}
		else if (topQuadrant && leftQuadrant && nearQuadrant)
		{
			index = 2;
		}
		else if (bottomQuadrant && leftQuadrant && farQuadrant)
		{
			index = 3;
		}
		else if (topQuadrant && rightQuadrant && nearQuadrant)
		{
			index = 4;
		}
		else if (bottomQuadrant && rightQuadrant && farQuadrant)
		{
			index = 5;
		}
		else if (topQuadrant && leftQuadrant && farQuadrant)
		{
			index = 6;
		}
		else if (topQuadrant && rightQuadrant && farQuadrant)
		{
			index = 7;
		}

		return index;
	}

	void Octree::insert(std::shared_ptr<RigidBody> body)
	{
		Vector3 bodyPosition = body->getPosition();
		Vector3 bodyBoxSize = body->getBoxSize();

		BoundingBox bodyBounds;
		bodyBounds.x = bodyPosition.getX() - (bodyBoxSize.getX() / 2);
		bodyBounds.y = bodyPosition.getY() - (bodyBoxSize.getY() / 2);
		bodyBounds.z = bodyPosition.getZ() - (bodyBoxSize.getZ() / 2);
		bodyBounds.width = bodyBoxSize.getX();
		bodyBounds.height = bodyBoxSize.getY();
		bodyBounds.depth = bodyBoxSize.getZ();

		if (m_nodes.size() != 0)
		{
			int index = getIndex(bodyBounds);
			if (index != -1)
			{
				m_nodes.at(index).insert(body);
				return;
			}
		}

		m_colliders.push_back(body);

		if (m_colliders.size() > MAX_RIGIDBODY_BY_LEVEL && m_level < MAX_LEVELS)
		{
			if (m_nodes.size() == 0)
			{
				split();
			}

			int i = 0;
			while (i < m_colliders.size())
			{
				bodyPosition = body->getPosition();
				bodyBoxSize = body->getBoxSize();

				bodyBounds.x = bodyPosition.getX() - (bodyBoxSize.getX() / 2);
				bodyBounds.y = bodyPosition.getY() - (bodyBoxSize.getY() / 2);
				bodyBounds.z = bodyPosition.getZ() - (bodyBoxSize.getZ() / 2);
				bodyBounds.width = bodyBoxSize.getX();
				bodyBounds.height = bodyBoxSize.getY();
				bodyBounds.depth = bodyBoxSize.getZ();

				int index = getIndex(bodyBounds);
				if (index != -1)
				{
					m_nodes.at(index).insert(m_colliders.at(i));
					m_colliders.erase(m_colliders.begin() + i);
				}
				else
				{
					i++;
				}
			}
		}
	}

	std::vector<std::shared_ptr<RigidBody>> Octree::retrieve(std::shared_ptr<RigidBody> body) const
	{
		std::vector<std::shared_ptr<RigidBody>> result;

		Vector3 bodyPosition = body->getPosition();
		Vector3 bodyBoxSize = body->getBoxSize();

		BoundingBox bodyBounds;
		bodyBounds.x = bodyPosition.getX() - (bodyBoxSize.getX() / 2);
		bodyBounds.y = bodyPosition.getY() - (bodyBoxSize.getY() / 2);
		bodyBounds.z = bodyPosition.getZ() - (bodyBoxSize.getZ() / 2);
		bodyBounds.width = bodyBoxSize.getX();
		bodyBounds.height = bodyBoxSize.getY();
		bodyBounds.depth = bodyBoxSize.getZ();

		int index = getIndex(bodyBounds);
		if (index != -1 && m_nodes.size() != 0)
		{
			result = m_nodes.at(index).retrieve(body);
		}

		result.insert(result.end(), m_colliders.begin(), m_colliders.end());

		return result;
	}
}