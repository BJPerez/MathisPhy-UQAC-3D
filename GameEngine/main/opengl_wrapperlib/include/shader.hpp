#pragma once

#include <glad/glad.h> // include glad to get all the required OpenGL headers

#include <string>

namespace opengl_wrapper
{
	using ShaderID = GLuint;

	class Shader
	{
	public:
		Shader();

		bool loadFromString(const std::string& vertexShaderSource, const std::string& fragmentShaderSource);
		bool loadFromFile(const std::string& vertexShaderFilename, const std::string& fragmentShaderFilename);
		void use();

		void setUniform(const std::string& name, bool value) const;
		void setUniform(const std::string& name, int value) const;
		void setUniform(const std::string& name, float value) const;

	private:
		ShaderID m_shaderId;
	};
}