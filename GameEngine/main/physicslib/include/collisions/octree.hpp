#pragma once

#include <vector>
#include <memory>
#include <algorithm>

#include <rigidBody.hpp>

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
		void insert(std::shared_ptr<RigidBody> body);


		// Retourne la liste de tous les �l�ments pouvant �tre en collision
		// avec l'�l�ment donn�
		std::vector<std::shared_ptr<RigidBody>> retrieve(std::shared_ptr<RigidBody> body) const;

	private:
		static const unsigned int MAX_RIGIDBODY_BY_LEVEL = 3;
		static const unsigned int MAX_LEVELS = 5;

		const unsigned int m_level;
		const BoundingBox m_bounds;
		std::vector<std::shared_ptr<RigidBody>> m_colliders;
		std::vector<Octree> m_nodes;

		// Split le noeud actuel en 4 noeuds fils
		// Les collider situ�s entre 2 noeuds sont stock�s dans ce noeud.
		void split();

		// Retourne l'index du noeud fils qui devrait contenir pRect
		// Retourne -1 si pRect est situ� entre 2 noeuds fils
		int getIndex(BoundingBox body) const;
	};
}