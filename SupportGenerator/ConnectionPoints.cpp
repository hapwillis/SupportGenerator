#include "ConnectionPoints.h"



ConnectionPoints::ConnectionPoints(Graph *graph) : model(graph)
{
	contDistribution = std::uniform_real_distribution<double>(0.0f, 1.0f);
	intDistribution = std::uniform_int_distribution<int>(0, graph->faceVector.size() - 1);

	//For poisson, must be ># of disconnected support areas
	// For Mitchell's, seeds = max number of support points
	const int seeds = 500; 
	const int maxCandidates = 10;
	const float maxRadius = 10.0f;
	
	points.reserve(seeds);
	MitchellsBestCandidates(seeds, maxCandidates);
}


ConnectionPoints::~ConnectionPoints()
{

}

void ConnectionPoints::MitchellsBestCandidates(int seeds, int maxCandidates)
{
	glm::vec3 bestCandidate;
	float bestDistance;
	Octree octree;
	octree.addPoint(randomPoint());

	for (int seed = 0; seed < seeds; seed++) {
		bestCandidate = randomPoint();
		bestDistance = prominence(bestCandidate, octree);
		for (int c = 1; c < maxCandidates; c++) {
			glm::vec3 candidate = randomPoint();
			float distance = prominence(candidate, octree);

			if (bestDistance < minDist || distance < bestDistance) {
				bestCandidate = candidate;
				bestDistance = distance;
			}
		}

		if (bestDistance < minDist)
			points.push_back(bestCandidate);
	}
}

void ConnectionPoints::Draw(DefaultShader shader)
{
}

Face * ConnectionPoints::randomFace()
{
	return model->faceVector[intDistribution(generator)];
}

glm::vec3 ConnectionPoints::randomPoint()
{
	Face *face = randomFace();
	glm::vec3 v1, e1, tVert, e2, randPoint;
	float r1 = contDistribution(generator);
	float r2 = contDistribution(generator);

	v1 = model->nodes[face->v1]->vertex.Position;
	e1 = model->nodes[face->v2]->vertex.Position - v1;
	tVert = r1 * e1 + v1;

	e2 = model->nodes[face->v3]->vertex.Position - tVert;
	randPoint = r2 * e2 + tVert;

	return randPoint;
}

float ConnectionPoints::prominence(glm::vec3 point, Octree octree)
{
	return glm::distance(octree.getNearestPoint(point), point);
}

glm::vec3 ConnectionPoints::lowestVertex(Face * face)
{
	glm::vec3 v1 = model->nodes[face->v1]->vertex.Position;
	glm::vec3 v2 = model->nodes[face->v2]->vertex.Position;
	glm::vec3 v3 = model->nodes[face->v3]->vertex.Position;
	float lowest = std::min(v1.x, v2.x);

	if (v3.x < lowest)
		return v3;
	if (v2.x < v1.x)
		return v2;
	return v1;
}
