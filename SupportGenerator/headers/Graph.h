#pragma once
#include <Octree.h> 

#include <thread>

using std::vector;
using std::unordered_set;
using std::shared_ptr;
using std::weak_ptr;
using glm::vec3;

class Node {
public:
  bool removed = false; // set to true on removal
  weak_ptr<Node> parentA;
  weak_ptr<Node> parentB;
  shared_ptr<Vertex> vertex_;
  unordered_set<weak_ptr<Node>> connections_; // TODO: change to unordered set

  Node(int index, Vertex v);
  Node(int index, vec3 pos, vec3 norm, bool wireframe);

  void addConnection(shared_ptr<Node> node);
  void addConnection(weak_ptr<Node> node);
};

class Graph
{
public: 
	vector<shared_ptr<Node>> nodes_;
  vector<shared_ptr<Face>> faces_;
	float range_ = 0.0f;

	Graph(vector<shared_ptr<Vertex>> &vertices, vector<int> &indices);
	Graph(vector<shared_ptr<Node>> nodes);
	~Graph();

	float getRange();
	void scale(float offset);
	void recalculateNormalsFromFaces();
	shared_ptr<Node> CombineNodes(shared_ptr<Node> n1, shared_ptr<Node> n2);

private:
	std::thread updateThread;

	void populateConnections();
  shared_ptr<Node> nodeFromAverage(shared_ptr<Node> n1, shared_ptr<Node> n2);

	void cleanConnections(vector<int> &translate);
};
