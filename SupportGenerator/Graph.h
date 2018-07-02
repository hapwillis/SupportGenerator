#pragma once
#include <Octree.h> 

class Graph
{
public: 
	std::vector<Node*> nodes; //addressible by ID
	std::vector<Face*> faceVector;
	float Range = 0.0;

	Graph(Model &model);
	Graph(std::vector<Node*> &newNodes, std::vector<Face*> &faces);
	~Graph();

	Octree* getOctree();
	float getRange();
	void scale(float displacement);
	void recalculateNormalsFromFaces();
	int CombineNodes(int n1, int n2);
	void ReduceFootprint();

	bool verifyFacesFromConnections(int node); // Test function.

private:
	Octree * octree;

	void populateConnections();
	Node* nodeFromAverage(Node* n1, Node* n2);
	Node* nodeFromIntercept(Node* n1, Node* n2);
	void CombineConnections(int n1, int n2, Node *node);
	void CombineFaces(int n1, int n2, Node *node);

	void cleanConnections(std::vector<int> &translate);
	void cleanFaces(std::vector<int> &translate);

	bool test();
	bool testCleanFaces();
	bool testCleanConnections();
	bool testPopulateConnections();
	bool testCombineNodes();
	bool testReduceFootprint();
	bool validateNodeCombination(Node *n1, Node *n2, Node *combined);
	bool validateNode(Node *node);
};
