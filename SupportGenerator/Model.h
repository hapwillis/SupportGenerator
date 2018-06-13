#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <DefaultShader.h>
#include <Octree.h>

#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include <fstream>
#include <sstream>
#include <iostream>

struct Vertex {
	// position
	glm::vec3 Position;
	// normal
	glm::vec3 Normal;

	Vertex();
	Vertex(glm::vec3 p, glm::vec3 n);
};

class Mesh {
public:
	/*  Mesh Data  */
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	unsigned int VAO;

	/*  Functions  */
	// constructor
	Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices);

	void Draw(DefaultShader shader);

private:
	/*  Render data  */
	unsigned int VBO, EBO;

	/*  Functions    */
	// initializes all the buffer objects/arrays
	void setupMesh();
};

class Model
{
public:
	float boundingRadius = 0.0;
	/*  Model Data  */
	std::vector<Mesh> meshes;
	int vertices;
	int faces;
	Octree octree;

	Model(std::string &path);
	
	~Model();

	void Draw(DefaultShader shader);

	float BoundingSphere(); //returns radius of bounding sphere

private:
	std::string directory;

	/*  Functions   */
	void loadModel(std::string &path);

	void processNode(aiNode *node, const aiScene *scene);

	Mesh processMesh(aiMesh *mesh, const aiScene *scene);
};

