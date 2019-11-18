#include "../include/renderEngine.hpp"

#include "math/matrix3.hpp"
#include "math/quaternion.hpp"
#include <math.h> // just for cos and sin functions

RenderEngine::RenderEngine() : m_openGlWrapper(SCR_WIDTH, SCR_HEIGHT, WINDOW_TITLE), m_mainWindow(m_openGlWrapper.getMainWindow())
{
	// Register particle shader
	opengl_wrapper::Shader defaultShader;
	defaultShader.loadFromFile("resources/shaders/particle.vs", "resources/shaders/particle.fs");
	m_shaderPrograms.insert(std::make_pair(ShaderProgramType::ST_DEFAULT, defaultShader));
}

void RenderEngine::render()
{
	// cleaning screen
	m_openGlWrapper.clearCurrentWindow();

	// drawings
	draw();

	// swapping the double buffers
	m_openGlWrapper.swapGraphicalBuffers(m_mainWindow);
}

const opengl_wrapper::OpenGlWrapper& RenderEngine::getOpenGlWrapper() const
{
	return m_openGlWrapper;
}

GLFWwindow* const RenderEngine::getMainWindow() const
{
	return m_mainWindow;
}

void RenderEngine::draw()
{
	if (res)
	{
		std::vector<double> vertices = {
			 0.5f,  0.5f, 0.0f,  // top right
			 0.5f, -0.5f, 0.0f,  // bottom right
			-0.5f, -0.5f, 0.0f,  // bottom left
			-0.5f,  0.5f, 0.0f   // top left 
		};
		std::vector<unsigned int>indices = {  // note that we start from 0!
			0, 1, 3,  // first Triangle
			1, 2, 3   // second Triangle
		};
		m_openGlWrapper.createAndBindDataBuffers(vertices, indices);


		// draw our first triangle
		m_shaderPrograms.at(ST_DEFAULT).use();
		res = false;
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	float time = (float)glfwGetTime();
	physicslib::Matrix3 mat(physicslib::Quaternion(cos(time/2), 0, 0, sin(time/2)));
	physicslib::Matrix3 scaling{ 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5 };
	mat = scaling * mat;
	float matrixData[] = {
		 mat(0,0), mat(1,0), mat(2,0), 0,
		 mat(0,1), mat(1,1), mat(2,1), 0,
		 mat(0,2), mat(2,1), mat(2,2), 0,
		 0.4,        -0.4,        0,        1
	};
	// get matrix's uniform location and set matrix
	unsigned int transformLoc = glGetUniformLocation(m_shaderPrograms.at(ST_DEFAULT).getId(), "transform");
 	glUniformMatrix4fv(transformLoc, 1, GL_FALSE, matrixData);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}



