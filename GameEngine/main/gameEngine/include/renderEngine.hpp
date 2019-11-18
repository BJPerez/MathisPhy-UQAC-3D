#pragma once

#include "openGlWrapper.hpp"
#include "shader.hpp"
#include <unordered_map>

/*
 * This class manages all the display.
 */
class RenderEngine
{
public:
	/*
	 * Constructor
	 */
	RenderEngine();

	/*
	 * The function to call to render and display the game to the screen
	 */
	void render();

	/*
	 * Getter for the m_openGlWrapper attribute
	 */
	const opengl_wrapper::OpenGlWrapper& getOpenGlWrapper() const;

	/*
	 * Getter for m_mainWindow attribute.
	 */
	GLFWwindow* const getMainWindow() const;
private:

	/*
	 * Enumerates the list of all the shader program.
	 *
	 * Each shaderprogram represents a combinaison of one fragment shader and a
	 * vertex shader linked in one shader program.
	 */
	enum ShaderProgramType
	{
		ST_DEFAULT
	};

	const unsigned int SCR_WIDTH = 800; // Width of the game window
	const unsigned int SCR_HEIGHT = 600; // Height of the game window
	const std::string WINDOW_TITLE = "Game Engine Demo"; // Title of the game window
	std::unordered_map<ShaderProgramType, opengl_wrapper::Shader> m_shaderPrograms; // The list of all the shader programm
	const opengl_wrapper::OpenGlWrapper m_openGlWrapper; // The instance of the opengl wrapper
	GLFWwindow* const m_mainWindow; // The opengl id of the main window.

	/*
	 * Function to effectivly draw the display.
	 */
	void draw();

	bool res = true;
};