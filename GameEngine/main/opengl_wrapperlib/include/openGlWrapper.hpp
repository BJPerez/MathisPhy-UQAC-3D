#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <string_view>
#include <tuple>
#include <vector>

namespace opengl_wrapper
{
	void framebuffer_size_callback(GLFWwindow* window, int width, int height);

	class OpenGlWrapper
	{
	public:
		OpenGlWrapper(const unsigned int screenWidth = 800, const unsigned int screenHeight = 600, const std::string_view windowName = "Default Window Name");
		~OpenGlWrapper();

		GLFWwindow* getMainWindow() const;

		bool windowShouldClose(GLFWwindow * const window) const;
		void clearCurrentWindow(float red = 0.0, float green = 0.0, float blue = 0.0, float opacity = 1.0) const;
		void setKeyboardCallback(GLFWwindow* window, GLFWkeyfun callbackFunction) const;
		void swapGraphicalBuffers(GLFWwindow * const window) const;
		void pollEvent() const;
		std::tuple<unsigned int, unsigned int> getWindowSize(GLFWwindow* const window) const;
		std::tuple<unsigned int, unsigned int, unsigned int> createAndBindParticlesDataBuffers
			(const std::vector<double>& verticesBuffer, const std::vector<unsigned int>& indicesBuffer) const;
		std::tuple<unsigned int, unsigned int, unsigned int> createAndBindBackgroundDataBuffers
			(const std::vector<double>& verticesBuffer, const std::vector<unsigned int>& indicesBuffer) const;
		void draw(GLenum shape, unsigned int count) const;
		void cleanAndDeleteDataBuffers(std::tuple<unsigned int, unsigned int, unsigned int> buffers) const;
		void closeMainWindow() const;

	private:
		GLFWwindow* m_mainWindow;
	};
}