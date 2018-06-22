#pragma once

#include "Octree.h"

struct Edge {
	unsigned int indexA;
	unsigned int indexB;
	float length;

	Edge(int a, int b, float len);
};

struct EdgeContainer {
	unsigned int indexA;
	unsigned int indexB;
	glm::vec3 negVert;
	glm::vec3 posVert;
	float len;

	EdgeContainer(int a, int b, glm::vec3 e1, glm::vec3 e2);
};

struct EdgeSet {
	std::unordered_multimap<float, EdgeContainer*> edgeMap;
	std::vector<EdgeContainer*> edges;

	~EdgeSet();

	void insert(EdgeContainer *e1);
};

struct KeyFuncs
{
	size_t operator()(const EdgeContainer& e)const;

	bool operator()(const EdgeContainer& a, const EdgeContainer& b)const;
};

class NavigationMesh
{
public:
	unsigned int VAO, VBO, EBO;
	Model *model = NULL; 
	float displacement;
	Graph *modelGraph = NULL;
	std::priority_queue<Edge> edgeHeap;
	Octree *octree = NULL;
	Graph *navGraph = NULL;
	Mesh *navMesh = NULL;

	NavigationMesh();
	~NavigationMesh();

	bool loadModel(Model &model, float dist);
	void Draw(DefaultShader shader);
	Mesh* convertToMesh(Graph *graph);

private:
	void initializeHeap();
	void getUniqueEdges(std::priority_queue<Edge> &edgeHeap);
	Graph* decimateMesh();
	void windFaces(std::vector<Node*> &nodes, std::vector<unsigned int> &indices);
	bool edgeValid(Edge edge, Graph *graph);
};
