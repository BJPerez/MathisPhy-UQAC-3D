#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "openGlWrapper.hpp"
#include "shader.hpp"
#include "particle.hpp"
#include "forceGenerator/particleForceGenerator.hpp"
#include "forceGenerator/gravityForceGenerator.hpp"
#include "forceGenerator/dragForceGenerator.hpp"
#include "forceGenerator/forceRegister.hpp"
#include "collisions/contactRegister.hpp"
#include "collisions/particleContact.hpp"
#include "../include/inputsManager.hpp"

class GameWorld
{
public:
	GameWorld();

	void run();
private:

	enum ShaderProgramType
	{
		ST_DEFAULT
	};

	const unsigned int SCR_WIDTH = 800;
	const unsigned int SCR_HEIGHT = 600;
	const std::string WINDOW_TITLE = "Game Engine Demo";
	std::unordered_map<ShaderProgramType, opengl_wrapper::Shader> m_shaderPrograms;
	physicslib::ForceRegister m_forceRegister;
	physicslib::ContactRegister m_contactRegister;

	const opengl_wrapper::OpenGlWrapper m_openGlWrapper;
	GLFWwindow* const m_mainWindow;
	InputsManager m_inputsManager;

	void updateGame(const std::vector<InputsManager::Intention> pendingIntentions, const double frametime);
	std::vector<InputsManager::Intention> getPendingIntentions();
	void processInputs(const std::vector<InputsManager::Intention>& pendingIntentions);
	void processIntention(InputsManager::Intention intention);
	void updatePhysics(const double frametime);
	void generateAllForces();
	void detectCollision();

	void renderGame() const;
};