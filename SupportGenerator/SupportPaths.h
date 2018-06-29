#pragma once

#define _USE_MATH_DEFINES
#include <math.h> 
#include "Graph.h"

class Cylinder
{
public: 
	int faces;
	float width;
	std::vector<glm::vec3> vertices;
	std::vector<int> indices;
	glm::vec3 start;
	glm::vec3 end;
	const glm::vec3 cosVec = glm::vec3(0.0f, 0.0f, 1.0f);
	const float pi2 = 2 * M_PI;

	Cylinder(int numSides, float supportWidth);
	void updateGeometry(glm::vec3 startP, glm::vec3 endP);
	void genVerts(glm::vec3 sinVec, glm::vec3 center);
	void genIndices();
};

class PFNode
{
public:
	Node *node;
	float costToStart;
	float costToEnd;
	float pathCost;
	PFNode *parent = NULL;
	bool seesBed;
	glm::vec3 bedConnection;

	PFNode(Node *aNode);
	~PFNode();

};

class Path
{
public:
	std::vector<PFNode*> path;
	float maxOverhang;
	float width;
	int faces = 6;
	glm::mat4 transform;
	Mesh *pathGeometry;
	std::vector<PFNode> *nav;
	std::priority_queue<PFNode*> open;
	std::unordered_set<int> seen;
	std::unordered_set<int> closed;

	Path(std::vector<PFNode> *nodes, float overhang, glm::mat4 model);
	~Path();

	void addNode(Node *node);
	void aStar(bool pathFound);
	void Geometry();
	void Draw(DefaultShader shader);

private:
	float getCost(PFNode *node);
	void restartAStar();
	void retracePath(PFNode* end);
};

//SupportPaths comprises pathfinding, the paths themselves, relaxation, and exporting geometry.
class SupportPaths
{
public:
	Graph *modelGraph;
	Graph *navGraph;
	std::vector<PFNode> nodes;
	std::vector<glm::vec3> points;
	glm::mat4 transform; // TODO: implement transformation
	float maxOverhang;
	std::vector<Path*> paths;
	// TODO: add mesh for concatenated paths

	std::vector<Mesh*> supportGeometry;

	SupportPaths(Graph *model, Graph *nav, std::vector<glm::vec3> p, float max, float offset);
	~SupportPaths();

	void FindPaths();
	void Relax(float degree);
	void Geometry(int faces, float tipD);
	void Draw(DefaultShader shader);

	void DeleteSupport();

private:
	Path* findStartNode(glm::vec3 point);
	void regroupPaths(); //convert overlapping paths into multiple distinct paths
	void intersectionGeometry();
	void tip();
	void base();
	void cylinderSegment();
};

