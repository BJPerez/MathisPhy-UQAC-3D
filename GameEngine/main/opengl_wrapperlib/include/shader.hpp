#pragma once

#include <glad/glad.h> // include glad to get all the required OpenGL headers

#include <string>

namespace opengl_wrapper
{
	using ShaderID = GLuint;

	/*
	 * This class represents a shader program. 
	 * The program is compiled and linked from a vertex shader and a fragment shader.
	 */
	class Shader
	{
	public:
		/*
		 * Constructor
		 */
		Shader();

		/*
		 * Function to initialize the shader from 2 strings containing the shader sources.
		 */
		bool loadFromString(const std::string& vertexShaderSource, const std::string& fragmentShaderSource);

		/*
		 * Function to initialize the shader from 2 files containing the shader sources.
		 */
		bool loadFromFile(const std::string& vertexShaderFilename, const std::string& fragmentShaderFilename);

		/*
		 * Function to set the opengl context to use this shader program.
		  */
		void use();

		/*
		 * Function to set a bool uniform value.
		 */
		void setUniform(const std::string& name, bool value) const;

		/*
		 * Function to set an int uniform value.
		 */
		void setUniform(const std::string& name, int value) const;

		/*
		 * Function to set a float uniform value.
		 */
		void setUniform(const std::string& name, float value) const;

		/*
		 * Function to set a float matrice 4 value.
		 */
		void setUniform(const std::string& name, float* data) const;

		/*
		 * Getter for m_shaderId attribute.
		 */
		ShaderID getId() const;

	private:
		ShaderID m_shaderId; // The shader's opengl id 
	};
}