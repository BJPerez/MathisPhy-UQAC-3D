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

	void OpenGlWrapper::cleanAndDeleteDataBuffers(std::tuple<unsigned int, unsigned int, unsigned int> buffers) const
	{
		glDeleteVertexArrays(1, &std::get<0>(buffers));
		glDeleteBuffers(1, &std::get<1>(buffers));
		glDeleteBuffers(1, &std::get<2>(buffers));
	}

	// create buffers to contains graphical data
	std::tuple<unsigned int, unsigned int, unsigned int> OpenGlWrapper::createAndBindBackgroundDataBuffers
		(const std::vector<double>& verticesBuffer, const std::vector<unsigned int>& indicesBuffer) const
	{
		const double * vertices = verticesBuffer.data();
		const unsigned int* indices = indicesBuffer.data();

		unsigned int VBO, VAO, EBO;
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
		glGenBuffers(1, &EBO);
		// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, verticesBuffer.size()*sizeof(double), vertices, GL_STREAM_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer.size()*sizeof(unsigned int), indices, GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 6 * sizeof(double), (void*)0);
		glEnableVertexAttribArray(0);

		glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 6 * sizeof(double), (void*)(3 * sizeof(double)));
		glEnableVertexAttribArray(1);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		return { VAO, VBO, EBO };
	}

	// create buffers to contains graphical data
	std::tuple<unsigned int, unsigned int, unsigned int> OpenGlWrapper::createAndBindParticlesDataBuffers
	(const std::vector<double>& verticesBuffer, const std::vector<unsigned int>& indicesBuffer) const
	{
		const double* vertices = verticesBuffer.data();
		const unsigned int* indices = indicesBuffer.data();

		unsigned int VBO, VAO, EBO;
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
		glGenBuffers(1, &EBO);
		// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, verticesBuffer.size() * sizeof(double), vertices, GL_STREAM_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesBuffer.size() * sizeof(unsigned int), indices, GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 9 * sizeof(double), (void*)0);
		glEnableVertexAttribArray(0);

		glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 9 * sizeof(double), (void*)(3 * sizeof(double)));
		glEnableVertexAttribArray(1);

		glVertexAttribPointer(2, 3, GL_DOUBLE, GL_FALSE, 9 * sizeof(double), (void*)(6 * sizeof(double)));
		glEnableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, 0);

		return { VAO, VBO, EBO };
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