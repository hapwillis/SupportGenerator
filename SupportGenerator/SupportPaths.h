#pragma once

#include "Octree.h"

class PFNode
{
public:
	Node * node;
	int cost;

	PFNode();
	~PFNode();

};

class PathFindingGraph
{
public:
	Graph *graph;
	std::vector<PFNode*> nodes;

	PathFindingGraph();
	~PathFindingGraph();

};

class Path
{
public:
	std::vector<glm::vec3> path;
	int maxOverhang;
	Mesh *pathGeometry;
	Mesh *supportGeometry;
	std::priority_queue<PFNode*> open;
	std::unordered_set<int> closed;

	Path();
	~Path();

	void Geometry();
	void Draw();

private:
	int getCost();
	void intersection();
	void cylinder();
};

//SupportPaths comprises pathfinding, the paths themselves, relaxation, and exporting geometry.
class SupportPaths
{
public:
	Graph *navGraph;
	std::vector<glm::vec3> points;
	float maxOverhang;
	std::vector<Path> paths;

	SupportPaths(Graph *g, std::vector<glm::vec3> p, float max);
	~SupportPaths();

	void FindPaths();
	void Relax(int degree);
	void Geometry(int faces, float tipD);
	void Draw();

	void DeleteSupport();

private:
	void findStartNode(glm::vec3 point);
};

