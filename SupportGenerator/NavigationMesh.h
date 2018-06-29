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
	Graph *graph = NULL; //TODO: rename to graph
	Mesh *mesh = NULL; //TODO: rename to mesh
	float displacement; //TODO: remove
	float supportWidth; //TODO: remove
	Graph *navGraph = NULL; //TODO: remove

	NavigationMesh(Model &newModel, float offset, float width);
	NavigationMesh(Model &model);
	~NavigationMesh();

	Graph* getSimpleGraph(float offset, float width);
	Mesh* convertToMesh(Graph *graph, float offset);
	void PruneSubBedVertices(glm::mat4 model); // TODO: pruning
	void Draw(DefaultShader shader);

private:
	void initializeHeap(std::priority_queue<Edge> &edgeHeap);
	Graph* decimateMesh();
	void facesToIndices(Graph *graph, std::vector<unsigned int> &indices);
	bool edgeValid(Edge edge, Graph *graph);
};

