#include "../include/inputsManager.hpp"

std::vector<InputsManager::Intention> InputsManager::getPendingIntentions(GLFWwindow* window)
{
	/*if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
	{
		m_pendingIntentions.push_back(Intention::MOVE_BLOB_TOP);
	}*/
	return m_pendingIntentions;
}

void InputsManager::clearIntentions()
{
	m_pendingIntentions.clear();
}

void InputsManager::addIntention(Intention intention)
{
	m_pendingIntentions.push_back(intention);
}