#pragma once

#include "Graph.h"

class ConnectionPoints
{
public:
	std::vector<glm::vec3> points;

	ConnectionPoints();
	~ConnectionPoints();

	// Uses barycentric coordinates to generate a random point on the triangle
	glm::vec3 randomPoint(Face *face);
};



