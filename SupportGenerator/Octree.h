#pragma once
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/intersect.hpp>

#include <model.h> //not including this?

struct Face {
	glm::vec3 edgeOne;
	glm::vec3 edgeTwo;
	glm::vec3 vertex;

	Face(glm::vec3 e1, glm::vec3 e2, glm::vec3 v);

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
	int index; // index of a point or a face (first of three indices)
	std::vector<Cell*> children;
	bool filled;

	Cell(glm::vec3 c, float r);

	~Cell();

	std::vector<int> getPoints(glm::vec3 point, float radius, float minD, std::vector<Vertex*> &vertices);

	int findQuadrant(glm::vec3);

	void add(glm::vec3 p, int index, std::vector<Vertex*> &vertices);

	void PopulateChildren(std::vector<Cell*> &children, glm::vec3 center, float r);

	Cell find(glm::vec3 p);
};

class Octree
{
public:
	std::vector<Vertex*> vertices;
	std::vector<unsigned int> faces;
	float range;
	Cell *root;
	FaceCell *faceRoot;
	int max; 

	Octree(std::vector<Mesh> meshes, int vertNum, int faceNum);

	~Octree();

	void add(int index);

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

	Node(int index, glm::vec3 pos, glm::vec3 norm);

	void addConnection(int index);
};

class Graph
{
public:
	Octree *octree;
	Model *modelRef;
	float displacement;
	std::vector<Node*> nodes; //addressible by ID

	Graph(Model model, float dist);
	Graph(std::vector<Node*> newNodes, float dist);

	~Graph();

	// Add new node to nodes, return index
	// Delete and null pointers at n1 and n2
	int CombineNodes(int n1, int n2);
	Graph* ReduceFootprint();
	void cleanConnections(std::vector<Node*>, std::vector<int> &translate);
};
