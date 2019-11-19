#include "../include/openGlWrapper.hpp"

#include <cassert>
#include <iostream>

namespace opengl_wrapper
{
	// glfw: whenever the window size changed (by OS or user resize) this callback function executes
	// ---------------------------------------------------------------------------------------------
	void framebuffer_size_callback(GLFWwindow* window, int width, int height)
	{
		// make sure the viewport matches the new window dimensions; note that width and 
		// height will be significantly larger than specified on retina displays.
		glViewport(0, 0, width, height);
	}

	OpenGlWrapper::OpenGlWrapper(const unsigned int screenWidth, const unsigned int screenHeight, const std::string_view windowName)
	{
		// glfw: initialize and configure
		// ------------------------------
		glfwInit();
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif


		// glfw window creation
		// --------------------
		m_mainWindow = glfwCreateWindow(screenWidth, screenHeight, windowName.data(), NULL, NULL);
		assert(m_mainWindow != nullptr);

		glfwMakeContextCurrent(m_mainWindow);
		glfwSetFramebufferSizeCallback(m_mainWindow, framebuffer_size_callback);

		// glad: load all OpenGL function pointers
		// ---------------------------------------
		bool success = gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
		assert(success == true);
	}

	OpenGlWrapper::~OpenGlWrapper()
	{
		// glfw: terminate, clearing all previously allocated GLFW resources.
		// ------------------------------------------------------------------
		glfwTerminate();
	}

	GLFWwindow* OpenGlWrapper::getMainWindow() const
	{
		return m_mainWindow;
	}

	bool OpenGlWrapper::windowShouldClose(GLFWwindow * const window) const
	{
		return glfwWindowShouldClose(window);
	}

	void OpenGlWrapper::clearCurrentWindow(float red, float green, float blue, float opacity) const
	{
		glClearColor(red, green, blue, opacity);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	std::tuple<unsigned int, unsigned int> OpenGlWrapper::getWindowSize(GLFWwindow* const window) const
	{
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		return { width, height };
	}

	void OpenGlWrapper::draw(GLenum shape, unsigned int count) const
	{
		glDrawElements(shape, count, GL_UNSIGNED_INT, 0);
	}

	void OpenGlWrapper::cleanAndDeleteDataBuffers(std::tuple<unsigned int, unsigned int> buffers) const
	{
		glDeleteVertexArrays(1, &std::get<0>(buffers));
		glDeleteBuffers(1, &std::get<1>(buffers));
	}

	// create buffers to contains graphical data
	std::tuple<unsigned int, unsigned int> OpenGlWrapper::createAndBindDataBuffer
		(const std::vector<double>& verticesBuffer) const
	{
		const double * vertices = verticesBuffer.data();

		unsigned int VBO, VAO;
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, verticesBuffer.size() * sizeof(double), vertices, GL_STATIC_DRAW);

		// position attribute
		glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(double), (void*)0);
		glEnableVertexAttribArray(0);

		return { VAO, VBO };
	}

	void OpenGlWrapper::clearDepthBuffer() const
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	void OpenGlWrapper::swapGraphicalBuffers(GLFWwindow* const window) const
	{
		glfwSwapBuffers(window);
	}

	void OpenGlWrapper::pollEvent() const
	{
		// Processes all pending events.
		glfwPollEvents();
	}

	void OpenGlWrapper::setKeyboardCallback(GLFWwindow* window, GLFWkeyfun callbackFunction) const
	{
		// Set the callback function to handle keyboard events
		glfwSetKeyCallback(window, callbackFunction);
	}

	void OpenGlWrapper::closeMainWindow() const
	{
		glfwSetWindowShouldClose(m_mainWindow, true);
	}
}