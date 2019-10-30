#pragma once
#include "OpenGlWrapper.hpp"
#include <vector>

/*
 * This class manages the player inputs.
 * It is based on "intentions". Indeed, the keys are not directly mapped 
 * to actions. Each key is mapped to an intention and then the GameWorld
 * knows what to do with it.
 */
class InputsManager
{
public:

	/*
	 * This enum contains the list of all the possible intention types.
	 */
	enum Intention
	{
		CLOSE_MAIN_WINDOW,
	};

	/*
	 * Constructor
	 */
	InputsManager() {};

	/*
	 * Generates the unique intentions and return m_pendingIntentions.
	 */
	std::vector<Intention> getPendingIntentions(GLFWwindow* window);

	/*
	 * Clear m_pendingIntentions attribute. 
	 */
	void clearIntentions();

	/*
	 * Add the given intention to m_pendingIntentions attribute.
	 */
	void addIntention(Intention intention);

private:
	std::vector<Intention> m_pendingIntentions; // The list of all the intention not processed yet. 

	 /*
	  * Each frame, checks if some keys are pressed and generate the corresponding intentions.
	  *
	  * The difference with getUniqueIntentions is that if you keep pressing a key, you will
	  * detect the pressure each frame and so add an intention each frame.
	  */
	void getFrameIntentions();
};

/*
 * When a key is pressed, opengl raises an event. In this function
 * we get those events and generate intentions from them.
 *
 * The difference with getFrameIntentions is that if you keep pressing a key,
 * only one event is raised so getUniqueIntention will add an intention only on
 * the first frame the key has been pressed.
 *
 * The function is not inside the InputManager class because it is OpenGl that calls it
 * and OpenGl doesn't want member functions as callbacks. OpenGl calls this function
 * each time a key is pressed.
 */
void getUniqueIntentions(GLFWwindow* window, int key, int scancode, int action, int mods);