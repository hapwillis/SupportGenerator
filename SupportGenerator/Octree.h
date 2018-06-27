#pragma once
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/intersect.hpp>

#include <model.h> 

#include <unordered_map>
#include <unordered_set>

struct ordered_set {
	std::unordered_set<int> set;
	std::vector<int> vector;

	void insert(int);
	int size();
	int popFromSet();
	int pop();
};

class Node;

struct Face {
	const float EPSILON = 0.0000001;
	glm::vec3 edgeOne;
	glm::vec3 edgeTwo;
	glm::vec3 vertex;
	glm::vec3 normal;
	int v1 = -1; 
	int v2 = -1;
	int v3 = -1;

	Face(glm::vec3 e1, glm::vec3 e2, glm::vec3 v);
	Face(int v1, int v2, int v3, std::vector<Vertex> &vertices);

	void update(const std::vector<Node*> &nodes);
	bool MollerTrumbore(glm::vec3 rayOrigin, glm::vec3 rayEnd);
};

struct FaceCell {
	glm::vec3 center;
	float range;
	std::vector<Face*> faces;
	std::vector<FaceCell*> children;

	FaceCell(glm::vec3 c, float r);

	~FaceCell();

	bool intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd, float doubleSize);

	int findQuadrant(glm::vec3 p);

	void add(Face *f, glm::vec3 center, float doubleSize);

	FaceCell* populateChild(int i);
};

struct Cell {
	glm::vec3 center;
	float range;
	std::vector<Cell*> children;
	std::vector<int> points; //indices of points or faces (first of three indices)
	bool filled;
	const unsigned int max = 8;

	Cell(glm::vec3 c, float r);

	~Cell();

	std::vector<int> getPoints(glm::vec3 point, float radius, float minD, std::vector<Vertex> &vertices);

	int findOctant(glm::vec3);

	void add(glm::vec3 p, int index, std::vector<Vertex> &vertices);

	void split(std::vector<Vertex> &vertices);

	bool pointsNotEqual(glm::vec3 p, glm::vec3 q);

	void PopulateChildren(std::vector<Cell*> &children, glm::vec3 center, float r);

	Cell find(glm::vec3 p);
};

class Octree
{
public:
	std::vector<Vertex> vertices;
	std::vector<unsigned int> faces;
	float range;
	Cell *root;
	FaceCell *faceRoot;
	int max; 

	Octree(std::vector<Mesh> &meshes);

	~Octree();

	void removeDuplicateVertices(std::vector<Mesh> &meshes);
	std::vector<int> constructUniqueVertices(int size, std::vector<Mesh> &meshes);
	bool pointsEqual(glm::vec3 p, glm::vec3 q);
	void removeDuplicateFaces();

	void getRange();
	void oldGetRange();

	void addVertex(int index);

	void addFace(int index);

	glm::vec3 getNearest(glm::vec3 p);

	std::vector<int> findInRadius(glm::vec3 point, float radius, float minD);

	// tests an octree of faces for intersection with a line segment
	// faces stored by lowest index first
	bool intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd);
};

class Node
{
public:
	int ID; //serial for each vertex added
	glm::vec3 position;
	glm::vec3 normal;
	std::vector<int> connections;
	std::unordered_set<int> faces;

	Node(int index, glm::vec3 pos, glm::vec3 norm);

	void addConnection(int index);
	void addFace(int face);
	void updateNormal(glm::vec3 n);
};

class Graph
{
public:
	Octree *octree;
	Model *modelRef;
	std::vector<Node*> nodes; //addressible by ID
	std::vector<Face*> faceVector;

	Graph(Model model);
	Graph(std::vector<Node*> newNodes, std::vector<Face*> faces);

	~Graph();

	void populateFaces();
	void populateConnections(std::vector<ordered_set> &connections);
	void addConnections(std::vector<ordered_set> connections);
	void orderConnections(std::vector<ordered_set> connections);
	void recalculateNormals();
	void recalculateNormalsFromFaces();
	void recalculateNormalsFaceWeight();
	void scale(float displacement);
	int CombineNodes(int n1, int n2);
	void CombineConnections(int n1, int n2, Node *node);
	void CombineFaces(int n1, int n2, Node *node);
	Node* nodeFromAverage(Node* n1, Node* n2);
	Node* nodeFromIntercept(Node* n1, Node* n2);
	Node* nodeFromPreservedEdges(Node* n1, Node* n2);
	Graph* ReduceFootprint();
	void cleanConnections(std::vector<Node*> &nodeList, std::vector<int> &translate);
	std::vector<Face*> cleanFaces(std::vector<Node*> &nodeList, std::vector<int> &translate);
	bool verifyFacesFromConnections(int node);
};
