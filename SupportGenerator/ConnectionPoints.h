#pragma once

#include "Graph.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <tuple>

class ConnectionPoints
{
public:
	Graph *model;
	float overhang;
	Octree octree;
	std::vector<std::tuple<glm::vec3, Face*>> points;
	std::vector<Vertex> vertices;
	std::vector<Face*> faces;
	unsigned int VAO, VBO;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> contDistribution;
	std::uniform_int_distribution<int> intDistribution;
	const float minDist = 5.0f;

	// Llyod's algorithm gives slightly better results at a much lower speed
	ConnectionPoints();
	ConnectionPoints(Graph *graph, float maxAngle);
	~ConnectionPoints();

	void setUpRendering();
	// Llyod's algorithm gives slightly better results at a much lower speed
	void MitchellsBestCandidates(int seeds, int maxCandidates);
	// Option to start at model vertices (ie lowest point):
	void vertexMitchells(int seeds, int maxCandidates);
	void PoissonDiscDistribution();
	void seedPoints(int seeds); //used for Poisson
	void Draw(DefaultShader shader);
	bool createConnection(glm::vec3 rayOrigin, glm::vec3 rayDirection);
	bool deleteConnection(glm::vec3 rayOrigin, glm::vec3 rayDirection);
	void undoConnection();

	void populateOctree();
	Face* randomFace();
	// Uses barycentric coordinates to generate a random point on the triangle
	glm::vec3 randomPoint(Face*);
	float prominence(glm::vec3 point);
	// prominenceManhattan returns the Manhattan distance (relative to the bed) to the nearest neighbor
	// using this means more sloped faces don't get extra supports (less support overlap)
	float prominenceManhattan(glm::vec3 point);
	glm::vec3 lowestVertex(Face *face);
};



