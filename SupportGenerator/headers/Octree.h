#pragma once
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/intersect.hpp>

#include <model.h> 

#include <algorithm>
#include <tuple>
#include <memory>
#include <math.h>

using std::vector;
using std::tuple;
using std::shared_ptr;
using glm::vec3;

struct ordered_set {
	std::unordered_set<int> set;
	std::vector<int> vector;

	void insert(int);
	int size();
	int popFromSet();
	int pop();
};

struct Face {
	const float EPSILON = 0.0000001f;

	vec3 edge21, edge31, edge32;
  shared_ptr<Vertex> vertex1, vertex2, vertex3;
	vec3 edge21Normal, edge32Normal, edge13Normal;
	vec3 normal;
	int index1 = -1; 
	int index2 = -1;
	int index3 = -1;

  Face();
	Face(int i1, int i2, int i3, const vector<shared_ptr<Vertex>> &vertices);
  Face(const Face& that);
  Face(Face&& that);

  Face& operator=(const Face& that);
  Face& operator=(Face&& that);

	void size(float &doubleSize, vec3 &center);
	void update(const vector<shared_ptr<Vertex>> &vertices);
	std::tuple<bool, vec3> MollerTrumbore(vec3 rayOrigin, vec3 rayEnd);
	float faceDistanceSquared(vec3 point);
	float clamp(float n);
};

struct Octant {
	const unsigned int maxChildren = 8; 

	vec3 center_;
	float range_;
	vector<unique_ptr<Octant>> children_;
	vector<shared_ptr<Vertex>> points_; //indices of points
  vector<shared_ptr<Face>> faces_;
	bool filled_;

	Octant(vec3 center, float width);
  Octant(const Octant& that); // copy
  Octant(Octant&& that); // move
	~Octant();

  Octant& operator=(const Octant& that); // copy assignment
  Octant& operator=(Octant&& that); // move assignment

	vector<shared_ptr<Vertex>> getPoints(vec3 point, float radius, float minD, vector<shared_ptr<Vertex>> &vertices);
	int findOctant(vec3 point);
	bool add(shared_ptr<Vertex> vertex);
	void add(shared_ptr<Face> face, vec3 center, float doubleSize);
	void split();
	bool pointsNotEqual(vec3 p, vec3 q);
  unique_ptr<Octant> populateChild(int i);
	tuple<bool, vec3, float> intersects(vec3 rayOrigin, vec3 rayEnd, float doubleSize);
  tuple<bool, shared_ptr<Face>, float> closestFace(vec3 point);
  tuple<bool, shared_ptr<Vertex>, float> closestVertex(vec3 p);
};

class Octree
{
public:
  vector<shared_ptr<Vertex>> vertices_;
	vector<shared_ptr<Face>> faces_;
	float range_;
	Octant root_;
	void operator()();

	Octree(const vector<shared_ptr<Vertex>> vertices, vector<shared_ptr<Face>> faces, float range);
	Octree();
	~Octree();

	void update(const vector<shared_ptr<Vertex>> vertices, vector<shared_ptr<Face>> faces, float range);
	void addPoint(vec3 p);
	void addVertex(shared_ptr<Vertex> vertex);
	void addFace(shared_ptr<Face> face);
	void enlargeRoot();
	vec3 getNearestPoint(vec3 p);
  tuple<bool, shared_ptr<Vertex>, float> getNearestVertex(vec3 p);
  shared_ptr<Face> getNearestFace(vec3 p);
	vector<shared_ptr<Vertex>> findInRadius(vec3 point, float radius, float minD);
	// tests an octree of faces for intersection with a line segment
	// faces stored by lowest index first
	bool intersects(vec3 rayOrigin, vec3 rayEnd);
	vec3 rayCast(vec3 rayOrigin, vec3 direction); //takes an origin and direction and returns first intersection
};
