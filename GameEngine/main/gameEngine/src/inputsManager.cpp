#include "../include/inputsManager.hpp"

std::vector<InputsManager::Intention> InputsManager::getPendingIntentions(GLFWwindow* window)
{
	getFrameIntentions();
	return m_pendingIntentions;
}

void InputsManager::getFrameIntentions()
{
	/*if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
	{
		m_pendingIntentions.push_back(Intention::MOVE_BLOB_TOP);
	}*/
}

void InputsManager::clearIntentions()
{
	m_pendingIntentions.clear();
}

void InputsManager::addIntention(Intention intention)
{
	m_pendingIntentions.push_back(intention);
}

void getUniqueIntentions(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	InputsManager* const inputsManager = static_cast<InputsManager* const>(glfwGetWindowUserPointer(window));
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		inputsManager->addIntention(InputsManager::Intention::CLOSE_MAIN_WINDOW);
	}
	if (key == GLFW_KEY_A && action == GLFW_PRESS)
	{
		inputsManager->addIntention(InputsManager::Intention::CREATE_SINGLE_BOX);
	}
	if (key == GLFW_KEY_Z && action == GLFW_PRESS)
	{
		inputsManager->addIntention(InputsManager::Intention::CREATE_TWO_BOXES);
	}
}
