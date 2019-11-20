#include "../include/renderEngine.hpp"

#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

RenderEngine::RenderEngine() : m_openGlWrapper(SCR_WIDTH, SCR_HEIGHT, WINDOW_TITLE), m_mainWindow(m_openGlWrapper.getMainWindow())
{
	const char* particleVertexShader =
		"#version 330 core\n"
		"layout(location = 0) in vec3 aPos;\n"
		"\n"
		"uniform mat4 model;\n"
		"uniform mat4 view;\n"
		"uniform mat4 projection;\n"
		"\n"
		"void main()\n"
		"{\n"
		"	gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
		"}\n\0";

	const char* particleFragmentShader =
		"#version 330 core\n"
		"out vec4 FragColor;\n"
		"void main()\n"
		"{\n"
		"	FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
		"}\n\0";

	// Enable depth testing
	glEnable(GL_DEPTH_TEST);

	// Register shader
	opengl_wrapper::Shader defaultShader;
	defaultShader.loadFromString(particleVertexShader, particleFragmentShader);
	m_shaderPrograms.insert(std::make_pair(ShaderProgramType::ST_DEFAULT, defaultShader));
}

void RenderEngine::render(const std::vector<std::shared_ptr<physicslib::RigidBody>>& bodies)
{
	// cleaning screen
	m_openGlWrapper.clearCurrentWindow();
	m_openGlWrapper.clearDepthBuffer();

	// drawings
	draw(bodies);

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

void RenderEngine::draw(const std::vector<std::shared_ptr<physicslib::RigidBody>>& bodies)
{
	opengl_wrapper::Shader currentShader = m_shaderPrograms.at(ShaderProgramType::ST_DEFAULT);
	currentShader.use();

	std::vector<double> vertices;
	for (auto body : bodies)
	{
		std::vector<double> bodyVertices = body->getBoxVertices();
		vertices.insert(vertices.end(), bodyVertices.begin(), bodyVertices.end());
	};
	std::tuple<unsigned int, unsigned int> openGlBuffers = m_openGlWrapper.createAndBindDataBuffer(vertices);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Model matrix
	glm::mat4 model = glm::mat4(1.0f);
	currentShader.setUniform("model", glm::value_ptr(model));

	// View matrix
	glm::mat4 view = glm::mat4(1.0f);
	view = glm::translate(view, glm::vec3(0.0f, 0.0f, -100.0f));
	currentShader.setUniform("view", glm::value_ptr(view));

	// Project matrix
	glm::mat4 projection;
	projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 150.0f);
	currentShader.setUniform("projection", glm::value_ptr(projection));

	glDrawArrays(GL_TRIANGLES, 0, vertices.size()/ 3);
	m_openGlWrapper.cleanAndDeleteDataBuffers(openGlBuffers);
}



