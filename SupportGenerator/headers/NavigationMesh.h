#pragma once

#include <Graph.h>
  
#include <random>

using std::vector;
using std::tuple;
using std::shared_ptr;
using std::unordered_multimap;
using glm::vec3;

struct Edge {
  weak_ptr<Node> a_;
  weak_ptr<Node> b_;
	float length_;

	Edge(weak_ptr<Node> a, weak_ptr<Node> b, float length);
};

struct EdgeContainer {
  weak_ptr<Node> a_;
  weak_ptr<Node> b_;
	vec3 negVert_;
	vec3 posVert_;
	float length_;

	EdgeContainer(weak_ptr<Node> a, weak_ptr<Node> b, vec3 e1, vec3 e2);
};

class EdgeHeap {
public:
	const unsigned int defaultSize = 20;
	vector<Edge> heap_;

	EdgeHeap();
	EdgeHeap(vector<shared_ptr<Node>> &nodes, vector<shared_ptr<Face>> &faces);
	~EdgeHeap();

	bool empty();
	Edge pop();
	void push(Edge e);
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
  vector<shared_ptr<Vertex>> vertices_;
  vector<int> indices_;
  shared_ptr<Graph> graph_;
  shared_ptr<Mesh> mesh_;

	NavigationMesh(vector<shared_ptr<Vertex>> &vertices, vector<int> &indices);
	~NavigationMesh();

	// TODO: return smart pointers for mesh and graph
	// suggested value of minDist = 0.5 * (support width) + offset
  shared_ptr<Graph> getSimpleGraph(float minLength);
  shared_ptr<Mesh> convertToMesh(shared_ptr<Graph> graph, float offset);
	void PruneSubBedVertices(glm::mat4 model); 
	void Draw(DefaultShader shader);

private:
	void decimateMesh(shared_ptr<Graph> graph, float minLength);
  void reduceFootprint(shared_ptr<Graph> graph);
	void facesToIndices(shared_ptr<Graph> graph, std::vector<unsigned int> &indices);
	bool edgeValid(Edge edge, shared_ptr<Graph> graph);
	
	void heapTest();
};

