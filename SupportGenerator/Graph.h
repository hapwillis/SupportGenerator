#pragma once
#include <Octree.h> 

#include <unordered_map>
#include <unordered_set>

class Graph
{
public:
	Octree * octree;
	std::vector<Node*> nodes; //addressible by ID
	std::vector<Face*> faceVector;
	float Range = 0.0;

	Graph(const Model &model);
	Graph(std::vector<Node*> newNodes, std::vector<Face*> faces);
	~Graph();

	void buildOctree();
	float getRange();
	void scale(float displacement);
	void recalculateNormalsFromFaces();
	int CombineNodes(int n1, int n2);
	Graph* ReduceFootprint();
	bool verifyFacesFromConnections(int node);

private:
	void ConcatenateModelMeshes(const Model &model);
	std::vector<int> constructUniqueVertices(int size, std::vector<Mesh> &meshes);
	void constructUniqueFaces(int size, std::vector<int> &translate, std::vector<Mesh> &meshes);
	void populateConnections();
	bool pointsEqual(glm::vec3 p, glm::vec3 q);

	Node* nodeFromAverage(Node* n1, Node* n2);
	Node* nodeFromIntercept(Node* n1, Node* n2);
	void CombineConnections(int n1, int n2, Node *node);
	void CombineFaces(int n1, int n2, Node *node);

	void cleanConnections(std::vector<Node*> &nodeList, std::vector<int> &translate);
	std::vector<Face*> cleanFaces(std::vector<Node*> &nodeList, std::vector<int> &translate);
};
