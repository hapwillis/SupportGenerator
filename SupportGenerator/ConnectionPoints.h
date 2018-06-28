#pragma once

#include "Graph.h"

#include <random>

class ConnectionPoints
{
public:
	Graph *model;
	std::vector<glm::vec3> points;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> contDistribution;
	std::uniform_int_distribution<int> intDistribution;
	const float minDist = 5.0f;

	// Llyod's algorithm gives slightly better results at a much lower speed
	ConnectionPoints(Graph *graph);
	~ConnectionPoints();

	// Llyod's algorithm gives slightly better results at a much lower speed
	void MitchellsBestCandidates(int seeds, int maxCandidates);
	// Option to start at model vertices (ie lowest point):
	void vertexMitchells(int seeds, int maxCandidates);
	void PoissonDiscDistribution();
	void seedPoints(int seeds); //used for Poisson
	void Draw(DefaultShader shader);

	Face* randomFace();
	// Uses barycentric coordinates to generate a random point on the triangle
	glm::vec3 randomPoint();
	float prominence(glm::vec3 point, Octree octree);
	// prominenceManhattan returns the Manhattan distance (relative to the bed) to the nearest neighbor
	// using this means more sloped faces don't get extra supports (less support overlap)
	float prominenceManhattan(glm::vec3 point, Octree octree);
	glm::vec3 lowestVertex(Face *face);
};



