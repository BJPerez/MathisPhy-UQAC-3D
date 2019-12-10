/*#pragma once

#include <vector>
#include <memory>
#include <algorithm>

#include "planePrimitive.hpp"

namespace physicslib
{
	struct BoundingBox
	{
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
		// M�thode pour cr�er un quad tree
		Octree(const unsigned int pLevel, const BoundingBox& pBounds);

		void clear();

		// Ins�re l'�l�ment dans le quad tree
		// Si apr�s l'insertion de l'�l�ment le quad tree contient trop d'�l�ment,
		// split le noeud actuel en 4
		void insert(std::shared_ptr<Primitive> body);


		// Retourne la liste de tous les �l�ments pouvant �tre en collision
		// avec l'�l�ment donn�
		void retrieve(std::vector < std::pair<Vector3, const PlanePrimitive* const> & collisions, bool top, bool right, bool bottom, bool left) const

		bool hasNodes() const;

		static void setTopPlane(const PlanePrimitive * const topPlane);
		static void setBottomPlane(const PlanePrimitive * const bottomPlane);
		static void setRightPlane(const PlanePrimitive * const rightPlane);
		static void setLeftPlane(const PlanePrimitive * const leftPlane);

	private:
		static const unsigned int MAX_RIGIDBODY_BY_LEVEL = 1;
		static const unsigned int MAX_LEVELS = 8;
		static const PlanePrimitive * const m_topPlane;
		static const PlanePrimitive * const m_bottomPlane;
		static const PlanePrimitive * const m_leftPlane;
		static const PlanePrimitive * const m_rightPlane;

		const unsigned int m_level;
		const BoundingBox m_bounds;
		std::vector<std::shared_ptr<RigidBody>> m_points;
		std::vector<Octree> m_nodes;

		// Split le noeud actuel en 4 noeuds fils
		// Les collider situ�s entre 2 noeuds sont stock�s dans ce noeud.
		void split();

		// Retourne l'index du noeud fils qui devrait contenir pRect
		// Retourne -1 si pRect est situ� entre 2 noeuds fils
		int getIndex(Vector3 point) const;

		void insert(Vector3 point);
	};
}*/