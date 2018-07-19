#pragma once

#define _USE_MATH_DEFINES
#include <math.h> 
#include "Graph.h"
#include <unordered_map>

#include <ConnectionPoints.h>

using std::unordered_map;
using std::pair;

class Cylinder
{
public: 
	int faces_;
	float width_;
	std::vector<glm::vec3> vertices_;
	std::vector<int> indices_;
	glm::vec3 start_;
	glm::vec3 end_;
	const glm::vec3 cosVec = glm::vec3(1.0f, 0.0f, 0.0f);
	const float pi2 = 2 * M_PI;

	Cylinder(int numSides, float supportWidth);
	// TODO: rename to createGeometry 
	void updateGeometry(glm::vec3 startP, glm::vec3 endP);
	void genVerts(glm::vec3 sinVec, glm::vec3 center);
	void genIndices();
};

class PFNode
{
public:
  shared_ptr<Node> node_;
  vector<weak_ptr<PFNode>> connections_;
	// TODO: make costs private, put them behind getters/setters that update pathCost
	float costToStart_; //g cost
	float costToEnd_; // h cost
	float pathCost_; //f cost
  shared_ptr<PFNode> parent_;
	bool seesBed_;
	glm::vec3 bedConnection_;

	PFNode(shared_ptr<Node> node);
	~PFNode();

};

struct PFNodeComparator
{
	bool operator()(const shared_ptr<PFNode> n1, const shared_ptr<PFNode> n2) const {
		return n1->pathCost_ > n2->pathCost_;
	}
};

class Path
{
public:
	std::vector<shared_ptr<PFNode>> path_;
	float maxOverhang_;
	float width_;
	int faces_ = 6;
	glm::mat4 transform_;
  shared_ptr<Mesh> pathGeometry_;
	std::priority_queue<shared_ptr<PFNode>, std::vector<shared_ptr<PFNode>>, PFNodeComparator> open_;
	std::unordered_set<shared_ptr<PFNode>> seen_; // is it faster for seen to be a property of PFNode?
	std::unordered_set<shared_ptr<PFNode>> closed_; // is it faster for closed to be a property of PFNode?
	unsigned int VAO, VBO;
	std::vector<Vertex> renderpoints_;
	int pathStep_;

	Path(float overhang, glm::mat4 model);
	~Path();

	void addNode(shared_ptr<Node> node);
  void addNode(shared_ptr<PFNode> node);
	void aStar(bool pathFound);
	void Geometry();
	void Draw(DefaultShader shader);

private:
	float getCost(shared_ptr<PFNode> node); // todo: move into PFNode
	void restartAStar();
	void retracePath(shared_ptr<PFNode> end);
};

//SupportPaths comprises pathfinding, the paths themselves, relaxation, and exporting geometry.
class SupportPaths
{
public:
	const bool OVERLAP_PATHS = true;
  unordered_map<shared_ptr<Vertex>, shared_ptr<PFNode>> PFnodeMap_;
	shared_ptr<Graph> modelGraph_;
  shared_ptr<Octree> modelOctree_;
  shared_ptr<Graph> navGraph_;
  shared_ptr<Octree> navOctree_;
	std::vector<glm::vec3> points_;
	glm::mat4 transform_; // TODO: implement transformation
	float maxOverhang_;
	std::vector<shared_ptr<Path>> paths_;
	// TODO: add mesh for concatenated paths

	std::vector<shared_ptr<Mesh>> supportGeometry_;

	SupportPaths(shared_ptr<Graph> modelGraph, shared_ptr<Graph> navGraph, shared_ptr<Octree> modelOctree, shared_ptr<Octree> navOctree, float max, float offset);
	~SupportPaths();


	void FindPaths(std::vector<std::tuple<glm::vec3, shared_ptr<Face>>> points);
	void Relax(float degree);
	void Geometry(int faces, float tipD);
	void Draw(DefaultShader shader);

	void DeleteSupport();

private:
  shared_ptr<Path> findStartNode(glm::vec3 point);
	void regroupPaths(); //convert overlapping paths into multiple distinct paths
	void descendPaths();
	void intersectionGeometry();
	void tip();
	void base();
	void cylinderSegment();
};

