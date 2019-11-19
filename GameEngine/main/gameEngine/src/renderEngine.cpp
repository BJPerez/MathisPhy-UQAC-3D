#include "../include/renderEngine.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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
	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	opengl_wrapper::Shader currentShader = m_shaderPrograms.at(ST_DEFAULT);


	if (res)
	{
		std::vector<float> vertices = {
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

		/*std::vector<unsigned int>indices = {  // note that we start from 0!
			0, 1, 3,  // first Triangle
			1, 2, 3   // second Triangle
		};
		m_openGlWrapper.createAndBindDataBuffers(vertices, indices);*/
		unsigned int VBO, VAO;
		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		// draw our first triangle
		currentShader.use();
		res = false;
	}

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Model matrix
	glm::mat4 model = glm::mat4(1.0f);
	model = glm::rotate(model, (float)glfwGetTime() * glm::radians(50.0f), glm::vec3(0.5f, 1.0f, 0.0f));
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



