#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <DefaultShader.h>

#include <memory>
#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

using std::unique_ptr;
using std::vector;

struct Vertex {
	glm::vec3 Position;
	glm::vec3 Normal;
	float Wireframe;

	Vertex();
	Vertex(glm::vec3 p, glm::vec3 n, float wireframe);
};

class Mesh {
public:
	vector<Vertex> vertices;
	vector<unsigned int> indices;
	unsigned int VAO;

	Mesh(vector<Vertex> vertices, vector<unsigned int> indices);

	void Draw(DefaultShader shader);

private:
	unsigned int VBO, EBO;

	void setupMesh();
};

class Model
{
public:
	float boundingRadius = 0.0f;
	float AABB = 0.0f;
	std::vector<Mesh> meshes;

	Model(std::string &path);
	~Model();

	void Draw(DefaultShader shader);
	float BoundingSphere(); 
	float AABBsize(); //AABB = Axis-Aligned Bounding Box
	void clean();

private:
	std::string directory;

	void loadModel(std::string &path);
	bool exportModel(std::string &path);

	void processNode(aiNode *node, const aiScene *scene);
	Mesh processMesh(aiMesh *mesh, const aiScene *scene);

	void concatenateModelMeshes();
	std::vector<int> constructUniqueVertices(int size);
	int checkMultimap(Vertex *v, std::unordered_multimap<float, int> &vertexMap);
	void constructUniqueFaces(int size, std::vector<int> &translate);
	bool pointsEqual(glm::vec3 p, glm::vec3 q);
};

