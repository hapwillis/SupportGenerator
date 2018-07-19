#pragma once
#include <Octree.h> 

#include <thread>

using std::vector;
using std::unordered_set;
using std::shared_ptr;
using glm::vec3;

class Node {
public:
  Vertex vertex;
  vector<shared_ptr<Vertex>> connections; // TODO: change to unordered set

  Node(int index, Vertex v);
  Node(int index, vec3 pos, vec3 norm, bool wireframe);

  void addConnection(shared_ptr<Vertex> vertex);
};

class Graph
{
public: 
	vector<shared_ptr<Node>> nodes_; //addressible by ID
	vector<shared_ptr<Face>> faceVector_;
	float Range = 0.0;

	Graph(Model &model);
	Graph(vector<shared_ptr<Node>> nodes, vector<shared_ptr<Face>> faces);
	~Graph();

	float getRange();
	void scale(float offset);
	void recalculateNormalsFromFaces();
	int CombineNodes(int n1, int n2);
	void ReduceFootprint();

	bool verifyFacesFromConnections(int node); // Test function.

private:
	std::thread updateThread;

	void populateConnections();
	Node* nodeFromAverage(Node* n1, Node* n2);
	void CombineConnections(int n1, int n2, Node *node);

	void cleanConnections(vector<int> &translate);
};
