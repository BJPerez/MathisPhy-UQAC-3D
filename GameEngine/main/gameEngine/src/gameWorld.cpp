#include "../include/gameWorld.hpp"

#include <chrono>
#include <algorithm>

GameWorld::GameWorld(): m_mainWindow(m_renderEngine.getMainWindow())
{
	const opengl_wrapper::OpenGlWrapper& openGlWrapper = m_renderEngine.getOpenGlWrapper();

	// Initialize graphics
	openGlWrapper.setKeyboardCallback(m_mainWindow, getUniqueIntentions);

	// Game variables
	glfwSetWindowUserPointer(m_mainWindow, &m_inputsManager); //save the manager's pointer to the window to be able to access it in the inputs callback function
}

void GameWorld::run()
{
	double frametime = 0.033333;//first frame considered at 30fps

	// game loops
	const opengl_wrapper::OpenGlWrapper& openGlWrapper = m_renderEngine.getOpenGlWrapper();
	while (!openGlWrapper.windowShouldClose(m_mainWindow))
	{
		// manage frame time
		auto start(std::chrono::system_clock::now());

		// input
		auto pendingIntentions = getPendingIntentions();

		// logic
		processInputs(pendingIntentions);
		m_physicEngine.update(frametime);

		// render
		m_renderEngine.render();

		// manage frame time
		auto end(std::chrono::system_clock::now());
		std::chrono::duration<double> elapsedSeconds = end - start;
		frametime = elapsedSeconds.count();//update frametime using last frame
	}
}

std::vector<InputsManager::Intention> GameWorld::getPendingIntentions()
{
	const opengl_wrapper::OpenGlWrapper& openGlWrapper = m_renderEngine.getOpenGlWrapper();
	openGlWrapper.pollEvent(); //triggers the event callbacks
	std::vector<InputsManager::Intention> pendingIntentions = m_inputsManager.getPendingIntentions(m_mainWindow);
	m_inputsManager.clearIntentions();
	return pendingIntentions;
}

void GameWorld::processInputs(const std::vector<InputsManager::Intention>& pendingIntentions)
{
	std::for_each(pendingIntentions.begin(), pendingIntentions.end(),
		[this](const InputsManager::Intention intention)
		{
			processIntention(intention);
		});
}

void GameWorld::processIntention(const InputsManager::Intention intention)
{
	if (intention == InputsManager::CLOSE_MAIN_WINDOW)
	{
		const opengl_wrapper::OpenGlWrapper& openGlWrapper = m_renderEngine.getOpenGlWrapper();
		openGlWrapper.closeMainWindow();
	}
}



