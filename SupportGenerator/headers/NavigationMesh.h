#pragma once

#include <Graph.h>
  
#include <random>

using std::vector;
using std::tuple;
using std::shared_ptr;
using std::unordered_multimap;
using glm::vec3;

struct Edge {
	unsigned int indexA_;
	unsigned int indexB_;
	float length_;

	Edge(int a, int b, float len);
};

struct EdgeContainer {
	unsigned int indexA_;
	unsigned int indexB_;
	vec3 negVert_;
	vec3 posVert_;
	float length_;

	EdgeContainer(int a, int b, vec3 e1, vec3 e2);
};

class EdgeHeap {
public:
	const unsigned int defaultSize_ = 20;
	vector<Edge> heap_;

	EdgeHeap();
	EdgeHeap(vector<shared_ptr<Face>> &faces);
	~EdgeHeap();

	bool empty();
	Edge* pop();
	void push(Edge *e);
	bool heapTest();
};

struct EdgeSet {
	unordered_multimap<float, EdgeContainer*> edgeMap_;
	vector<EdgeContainer*> edges_;

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
  shared_ptr<Model> model_;
  shared_ptr<Graph> graph_;
  shared_ptr<Mesh> mesh_;


	NavigationMesh(Model &newModel); 
	~NavigationMesh();

	// TODO: return smart pointers for mesh and graph
	// suggested value of minDist = 0.5 * (support width) + offset
	Graph* getSimpleGraph(float minLength);
	Mesh* convertToMesh(Graph *g, float offset);
	void PruneSubBedVertices(glm::mat4 model); 
	void Draw(DefaultShader shader);

private:
	void decimateMesh(Graph *g, float minLength);
	void facesToIndices(Graph *g, std::vector<unsigned int> &indices);
	bool edgeValid(Edge *edge, Graph *g);
	
	void heapTest();
};

