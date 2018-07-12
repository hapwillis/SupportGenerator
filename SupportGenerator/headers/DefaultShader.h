#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class DefaultShader
{
public:
	unsigned int ID;

	// TODO: update shader to pass through color
	DefaultShader(const char* vertexPath, const char* fragmentPath, const char* geometryPath = nullptr);

	~DefaultShader();

	void use();

	void setMat4(const std::string &name, const glm::mat4 &mat) const;

	void setVec3(const std::string &name, const glm::vec3 &value) const;

	void setVec4(const std::string &name, const glm::vec4 &value) const;

	void setFloat(const std::string &name, float value) const;

private:
	void checkCompileErrors(GLuint shader, std::string type);
};

