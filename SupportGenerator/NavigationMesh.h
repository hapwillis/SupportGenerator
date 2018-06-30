#pragma once

#include "Graph.h"

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
	Graph *graph = NULL; 
	Mesh *mesh = NULL; 

	NavigationMesh(Model &newModel); 
	~NavigationMesh();

	// TODO: return smart pointers for mesh and graph
	// suggested value of minDist = 0.5 * (support width) + offset
	Graph* getSimpleGraph(float minLength);
	Mesh* convertToMesh(Graph *graph, float offset);
	void PruneSubBedVertices(glm::mat4 model); 
	void Draw(DefaultShader shader);

private:
	void initializeHeap(std::priority_queue<Edge> &edgeHeap);
	Graph* decimateMesh(float minLength);
	void facesToIndices(Graph *graph, std::vector<unsigned int> &indices);
	bool edgeValid(Edge edge, Graph *graph);
};

