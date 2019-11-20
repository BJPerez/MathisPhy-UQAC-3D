#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include "openGlWrapper.hpp"
#include "shader.hpp"
#include "particle.hpp"
#include "collisions/particleContact.hpp"
#include "../include/inputsManager.hpp"
#include "physicEngine.hpp"
#include "renderEngine.hpp"

/*
 * The GameWorld class
 */
class GameWorld
{
public:

	/*
	 * Constructor
	 */
	GameWorld();

	/*
	 * Starts and runs the game engine.
	 */
	void run();
private:
	InputsManager m_inputsManager; // The instance of the input manager
	PhysicEngine m_physicEngine; // The instance of the physic engine
	RenderEngine m_renderEngine; // The instance of the render engine
	GLFWwindow* const m_mainWindow; // The opengl id of the main window
	std::vector<std::shared_ptr<physicslib::RigidBody>> m_rigidBodies; // List of all rigid bodies in the world
	
	void initializeRigidBodies();

	/*
	 * Function to get the list of all the pending envent.
	 */
	std::vector<InputsManager::Intention> getPendingIntentions();

	/*
	 * Function to process the given pending intention
	 */
	void processInputs(const std::vector<InputsManager::Intention>& pendingIntentions);

	/*
	 * Function to process one intention
	 */
	void processIntention(InputsManager::Intention intention);
};