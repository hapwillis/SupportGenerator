#pragma once
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/intersect.hpp>

#include <model.h> 


struct ordered_set {
	std::unordered_set<int> set;
	std::vector<int> vector;

	void insert(int);
	int size();
	int popFromSet();
	int pop();
};

class Node
{
public:
	int ID; //serial for each vertex added
	Vertex vertex;
	std::vector<int> connections;
	std::unordered_set<int> faces;

	Node(int index, Vertex v);
	Node(int index, glm::vec3 pos, glm::vec3 norm, bool wireframe);

	void addConnection(int index);
	void addFace(int face);
};

struct Face {
	const float EPSILON = 0.0000001f;
	glm::vec3 edgeOne;
	glm::vec3 edgeTwo;
	glm::vec3 vertex;
	glm::vec3 normal;
	int v1 = -1; 
	int v2 = -1;
	int v3 = -1;

	Face(glm::vec3 e1, glm::vec3 e2, glm::vec3 v);
	Face(int v1, int v2, int v3, const std::vector<Node*> &nodes);

	void update(const std::vector<Node*> &nodes);
	bool MollerTrumbore(glm::vec3 rayOrigin, glm::vec3 rayEnd);
};

struct FaceCell {
	glm::vec3 center;
	float range;
	std::vector<FaceCell*> children;
	std::vector<Face*> faces;

	FaceCell(glm::vec3 c, float r);
	~FaceCell();

	void add(Face *f, glm::vec3 center, float doubleSize);
	int findQuadrant(glm::vec3 p);
	FaceCell* populateChild(int i);
	bool intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd, float doubleSize);
};

struct Octant {
	const unsigned int maxChildren = 8;
	glm::vec3 center;
	float range;
	std::vector<Octant*> children;
	std::vector<int> points; //indices of points
	bool filled;

	Octant(glm::vec3 c, float r);
	~Octant();

	std::vector<int> getPoints(glm::vec3 point, float radius, float minD, std::vector<Vertex*> &vertices);
	int findOctant(glm::vec3);
	void add(glm::vec3 p, int index, std::vector<Vertex*> &vertices);
	void split(std::vector<Vertex*> &vertices);
	bool pointsNotEqual(glm::vec3 p, glm::vec3 q);
	void PopulateChildren(std::vector<Octant*> &children, glm::vec3 center, float r);

	Octant find(glm::vec3 p);
};

class Octree
{
public:
	std::vector<Vertex*> vertices;
	std::vector<unsigned int> *faces;
	float Range;
	Octant *root;
	FaceCell *faceRoot;

	Octree(const std::vector<Node*> *nodes, std::vector<Face*> *aFaces, float range);
	~Octree();

	void addVertex(int index);
	void addFace(int index);
	Node* getNearestPoint(glm::vec3 p);
	Face* getNearestFace(glm::vec3 p);
	std::vector<int> findInRadius(glm::vec3 point, float radius, float minD);
	// tests an octree of faces for intersection with a line segment
	// faces stored by lowest index first
	bool intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd);
	glm::vec3 rayCast(glm::vec3 rayOrigin, glm::vec3 direction); //takes an origin and direction and returns first intersection
};
