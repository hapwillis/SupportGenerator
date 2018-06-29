#include "ConnectionPoints.h"



ConnectionPoints::ConnectionPoints(Graph *graph, float maxAngle) : 
	model(graph), overhang((M_PI / 2.0) - maxAngle)
{
	//For poisson, must be ># of disconnected support areas
	// For Mitchell's, seeds = max number of support points
	const int seeds = 500;
	const int maxCandidates = 10;
	const float maxRadius = 10.0f;

	intDistribution = std::uniform_int_distribution<int>(0, graph->faceVector.size() - 1);
	contDistribution = std::uniform_real_distribution<double>(0.0f, 1.0f);
	points.reserve(seeds);

	populateOctree();
	MitchellsBestCandidates(seeds, maxCandidates);

	for (glm::vec3 p : points) {
		vertices.emplace_back(p, glm::vec3(0,0,0), true);
		//vertices.push_back(Vertex(p, glm::vec3(0, 0, 0), true));
	}

	setUpRendering();
}


ConnectionPoints::~ConnectionPoints()
{
}

void ConnectionPoints::populateOctree()
{
	glm::vec3 down(-1.0f, 0.0f, 0.0f);

	octree.vertices.reserve(model->nodes.size());
	for (Node *n : model->nodes) {
		octree.vertices.push_back(&n->vertex);
	}

	for (Face *f : model->faceVector) {
		// TODO: this appears to be wildly broken, as points are being added that are nowhere near
		// it may be because the face normals are flipped?
		if (glm::acos(glm::dot(f->normal, down)) < overhang)
			octree.addFace(f);
	}
}

void ConnectionPoints::setUpRendering()
{
	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(10.0f);

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// A great thing about structs is that their memory layout is sequential for all its items.
	// The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
	// again translates to 3/2 floats which translates to a byte array.
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Wireframe));
	glBindVertexArray(0);
}

void ConnectionPoints::MitchellsBestCandidates(int seeds, int maxCandidates)
{
	glm::vec3 bestCandidate;
	float bestDistance;
	octree.addPoint(randomPoint());

	for (int seed = 0; seed < seeds; seed++) {
		bestCandidate = randomPoint();
		bestDistance = prominence(bestCandidate);

		for (int c = 1; c < maxCandidates; c++) {
			glm::vec3 candidate = randomPoint();
			float distance = prominence(candidate);

			if (bestDistance < minDist || distance < bestDistance) {
				bestCandidate = candidate;
				bestDistance = distance;
			}
		}

		if (bestDistance > minDist)
			points.push_back(bestCandidate);
	}
}

void ConnectionPoints::Draw(DefaultShader shader)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS);
	glBindVertexArray(VAO);
	glDrawArrays(GL_POINTS, 0, 3 * (vertices.size() - 1));
	glBindVertexArray(0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
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

	// TODO: remove this nested dereferencing
	v1 = model->nodes[face->index1]->vertex.Position;
	e1 = model->nodes[face->index2]->vertex.Position - v1;
	tVert = r1 * e1 + v1;

	e2 = model->nodes[face->index3]->vertex.Position - tVert;
	randPoint = r2 * e2 + tVert;

	return randPoint;
}

float ConnectionPoints::prominence(glm::vec3 point)
{
	return glm::distance(octree.getNearestPoint(point), point);
}

glm::vec3 ConnectionPoints::lowestVertex(Face * face)
{
	// TODO: remove this nested dereferencing
	glm::vec3 v1 = model->nodes[face->index1]->vertex.Position;
	glm::vec3 v2 = model->nodes[face->index2]->vertex.Position;
	glm::vec3 v3 = model->nodes[face->index3]->vertex.Position;
	float lowest = std::min(v1.x, v2.x);

	if (v3.x < lowest)
		return v3;
	if (v2.x < v1.x)
		return v2;
	return v1;
}
