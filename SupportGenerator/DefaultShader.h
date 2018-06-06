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

	DefaultShader(const char* vertexPath, const char* fragmentPath, const char* geometryPath = nullptr);
	
	~DefaultShader();

	void use();

	void setMat4(const std::string &name, const glm::mat4 &mat) const;

private:
	void checkCompileErrors(GLuint shader, std::string type);
};

