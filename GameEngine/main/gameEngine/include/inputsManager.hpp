#pragma once

#include <vector>

#include "OpenGlWrapper.hpp"

class InputsManager
{
public:
	enum Intention
	{
		CLOSE_MAIN_WINDOW,
	};

	InputsManager() {};

	std::vector<Intention> getPendingIntentions(GLFWwindow* window);
	void clearIntentions();
	void addIntention(Intention intention);

private:
	std::vector<Intention> m_pendingIntentions;
};