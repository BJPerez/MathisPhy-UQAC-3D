#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <string_view>
#include <tuple>
#include <vector>

namespace opengl_wrapper
{
	/*
	 * Function called when the window is resized. 
	 * Update the size of the opengl inner window to match the new window size.
	 *
	 * The function is not inside the class because it is OpenGl that calls it
	 * and OpenGl doesn't want member functions as callbacks. 
	 */
	void framebuffer_size_callback(GLFWwindow* window, int width, int height);

	/*
	 * This class aims to offer a wrapper to opengl. 
	 * The aim is to be able to mask every old code from opengl
	 * in this class and allow some proper and modern c++ in the rest
	 * the project.
	 */
	class OpenGlWrapper
	{
	public:
		/*
		 * Constructor
		 */
		OpenGlWrapper(const unsigned int screenWidth = 800, const unsigned int screenHeight = 600, const std::string_view windowName = "Default Window Name");
		
		/*
		 * Destructor
		 */
		~OpenGlWrapper();

		/*
		 * Getter for m_mainWindow attribute.
		 */
		GLFWwindow* getMainWindow() const;

		/*
		 * Return if the given window should close.
		 */
		bool windowShouldClose(GLFWwindow * const window) const;

		/*
		 * Clear the window of every drawing and fill it with the given color.
		 */
		void clearCurrentWindow(float red = 0.0, float green = 0.0, float blue = 0.0, float opacity = 1.0) const;

		/*
		 * Clear the depth buffer.
		 */
		void clearDepthBuffer() const;

		/*
		 * Set the function callback for when a key is pressed.
		 */
		void setKeyboardCallback(GLFWwindow* window, GLFWkeyfun callbackFunction) const;

		/*
		 * Swap the two buffers of the rendering.
		 */
		void swapGraphicalBuffers(GLFWwindow * const window) const;

		/*
		 * Takes all the pending event and process them.
		 */
		void pollEvent() const;

		/*
		 * Return the given window's size.
		 */
		std::tuple<unsigned int, unsigned int> getWindowSize(GLFWwindow* const window) const;

		/*
		 * Create the opengl buffer from the given data.
		 * Bind the created buffer.
		 */
		std::tuple<unsigned int, unsigned int> createAndBindDataBuffer
			(const std::vector<double>& verticesBuffer) const;

		/*
		 * Draw the given shaped count times.
		 * The memory must have been set properly before.
		 */
		void draw(GLenum shape, unsigned int count) const;

		/*
		 * Clear and free the memory of the opengl buffers corresponding to the given ids.
		 */
		void cleanAndDeleteDataBuffers(std::tuple<unsigned int, unsigned int> buffers) const;

		/*
		 * Close the main window
		 */
		void closeMainWindow() const;

	private:
		GLFWwindow* m_mainWindow; // The opengl id of the main window
	};
}