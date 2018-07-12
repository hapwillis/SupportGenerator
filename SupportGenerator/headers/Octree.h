#pragma once
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/intersect.hpp>

#include <model.h> 

#include <algorithm>
#include <tuple>
#include <math.h>


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

	glm::vec3 edge21, edge31, edge32;
	glm::vec3 vertex1, vertex2, vertex3;
	glm::vec3 edge21Normal, edge32Normal, edge13Normal;
	glm::vec3 normal;
	int index1 = -1; 
	int index2 = -1;
	int index3 = -1;

	Face(int v1, int v2, int v3, const std::vector<Node*> &nodes);

	void size(float &doubleSize, glm::vec3 &center);
	void update(const std::vector<Node*> &nodes);
	std::tuple<bool, glm::vec3> MollerTrumbore(glm::vec3 rayOrigin, glm::vec3 rayEnd);
	float faceDistanceSquared(glm::vec3 point);
	float clamp(float n);
};

struct Octant {
	const unsigned int maxChildren = 8; 

	glm::vec3 center;
	float range;
	std::vector<Octant*> children;
	std::vector<int> points; //indices of points
	std::vector<Face*> faces;
	bool filled;

	Octant(glm::vec3 center, float width);
	~Octant();

	std::vector<int> getPoints(glm::vec3 point, float radius, float minD, std::vector<glm::vec3> &vertices);
	int findOctant(glm::vec3 point);
	bool add(glm::vec3 p, int index, std::vector<glm::vec3> &vertices);
	void add(Face *f, glm::vec3 center, float doubleSize);
	void split(std::vector<glm::vec3> &vertices);
	bool pointsNotEqual(glm::vec3 p, glm::vec3 q);
	Octant* populateChild(int i);
	std::tuple<bool, glm::vec3, float> intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd, float doubleSize);
	float closestFace(glm::vec3 point, Face **resultSquared);

	Octant* findLeaf(glm::vec3 p);
};

class Octree
{
public:
	std::vector<glm::vec3> vertices; 
	std::vector<Face*> *faces;
	float Range;
	Octant *root;
	std::vector<int> destroyList;
	void operator()();

	Octree(const std::vector<Node*> *nodes, std::vector<Face*> *aFaces, float range);
	Octree();
	~Octree();

	void update(const std::vector<Node*> *nodes, std::vector<Face*> *aFaces, float range);
	void addPoint(glm::vec3 p);
	void addVertex(int index);
	void addFace(Face *face);
	void enlargeRoot();
	glm::vec3 getNearestPoint(glm::vec3 p);
	int getNearestNodeIndex(glm::vec3 p);
	Face* getNearestFace(glm::vec3 p);
	std::vector<int> findInRadius(glm::vec3 point, float radius, float minD);
	// tests an octree of faces for intersection with a line segment
	// faces stored by lowest index first
	bool intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd);
	glm::vec3 rayCast(glm::vec3 rayOrigin, glm::vec3 direction); //takes an origin and direction and returns first intersection
};
