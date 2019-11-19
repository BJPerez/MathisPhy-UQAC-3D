#include "../include/renderEngine.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

RenderEngine::RenderEngine() : m_openGlWrapper(SCR_WIDTH, SCR_HEIGHT, WINDOW_TITLE), m_mainWindow(m_openGlWrapper.getMainWindow())
{
	// Enable depth testing
	glEnable(GL_DEPTH_TEST);

	// Register shader
	opengl_wrapper::Shader defaultShader;
	defaultShader.loadFromFile("resources/shaders/particle.vs", "resources/shaders/particle.fs");
	m_shaderPrograms.insert(std::make_pair(ShaderProgramType::ST_DEFAULT, defaultShader));
}

void RenderEngine::render()
{
	// cleaning screen
	m_openGlWrapper.clearCurrentWindow();
	m_openGlWrapper.clearDepthBuffer();

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
	opengl_wrapper::Shader currentShader = m_shaderPrograms.at(ST_DEFAULT);
	currentShader.use();

	if (res)
	{
		std::vector<double> vertices = {
			-0.5f, -0.5f, -0.5f,  
			 0.5f, -0.5f, -0.5f, 
			 0.5f,  0.5f, -0.5f,  
			 0.5f,  0.5f, -0.5f, 
			-0.5f,  0.5f, -0.5f,  
			-0.5f, -0.5f, -0.5f, 

			-0.5f, -0.5f,  0.5f,  
			 0.5f, -0.5f,  0.5f,  
			 0.5f,  0.5f,  0.5f, 
			 0.5f,  0.5f,  0.5f, 
			-0.5f,  0.5f,  0.5f,  
			-0.5f, -0.5f,  0.5f,  

			-0.5f,  0.5f,  0.5f,  
			-0.5f,  0.5f, -0.5f,  
			-0.5f, -0.5f, -0.5f, 
			-0.5f, -0.5f, -0.5f, 
			-0.5f, -0.5f,  0.5f,  
			-0.5f,  0.5f,  0.5f,  

			 0.5f,  0.5f,  0.5f, 
			 0.5f,  0.5f, -0.5f, 
			 0.5f, -0.5f, -0.5f,  
			 0.5f, -0.5f, -0.5f,  
			 0.5f, -0.5f,  0.5f,  
			 0.5f,  0.5f,  0.5f,  

			-0.5f, -0.5f, -0.5f,  
			 0.5f, -0.5f, -0.5f,  
			 0.5f, -0.5f,  0.5f,  
			 0.5f, -0.5f,  0.5f,
			-0.5f, -0.5f,  0.5f, 
			-0.5f, -0.5f, -0.5f, 

			-0.5f,  0.5f, -0.5f, 
			 0.5f,  0.5f, -0.5f,  
			 0.5f,  0.5f,  0.5f,  
			 0.5f,  0.5f,  0.5f, 
			-0.5f,  0.5f,  0.5f, 
			-0.5f,  0.5f, -0.5f,  
		};
		m_openGlWrapper.createAndBindDataBuffer(vertices);
		res = false;
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Model matrix
	glm::mat4 model = glm::mat4(1.0f);
	currentShader.setUniform("model", glm::value_ptr(model));

	// View matrix
	glm::mat4 view = glm::mat4(1.0f);
	view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));
	currentShader.setUniform("view", glm::value_ptr(view));

	// Project matrix
	glm::mat4 projection;
	projection = glm::perspective(glm::radians(45.0f), static_cast<float>(SCR_WIDTH) / SCR_HEIGHT, 0.1f, 100.0f);
	currentShader.setUniform("projection", glm::value_ptr(projection));

	glDrawArrays(GL_TRIANGLES, 0, 36);
}



