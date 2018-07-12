#include "ConnectionPoints.h"


ConnectionPoints::ConnectionPoints() 
{

}

ConnectionPoints::ConnectionPoints(Graph *graph, float maxAngle) : 
	model(graph), overhang((M_PI / 2.0f) - maxAngle)
{
	//For poisson, must be ># of disconnected support areas
	// For Mitchell's, seeds = max number of support points
	const int seeds = 200;
	const int maxCandidates = 50;
	const float maxRadius = 10.0f;

	populateOctree();
	intDistribution = std::uniform_int_distribution<int>(0, faces.size() - 1);
	contDistribution = std::uniform_real_distribution<double>(0.0f, 1.0f);
	points.reserve(seeds);

	MitchellsBestCandidates(seeds, maxCandidates);

	struct {
		bool operator()(std::tuple<glm::vec3, Face*> a, std::tuple<glm::vec3, Face*> b) const
		{
			return std::get<0>(a).z <std::get<0>(b).z;
		}
	} compPoints;

	std::sort(points.begin(), points.end(), compPoints);

	for (std::tuple<glm::vec3, Face*> p : points) {
		vertices.emplace_back(std::get<0>(p), glm::vec3(0,0,0), true);
	}

	setUpRendering();
}

ConnectionPoints::~ConnectionPoints()
{
	for (Face *f : faces) {
		delete (f);
	}

}

void ConnectionPoints::populateOctree()
{
	glm::vec3 down(0.0f, 0.0f, -1.0f);

	for (Face *f : model->faceVector) {
		float angle = glm::acos(glm::dot(f->normal, down));
		if (angle < overhang && angle > 0.001f) {
			faces.push_back(new Face(f->index1, f->index2, f->index3, model->nodes));
			octree.addFace(faces.back());
		}
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
	Face* bestFace;
	Face* face = randomFace();
	octree.addPoint(randomPoint(face)); 

	for (int seed = 0; seed < seeds; seed++) {
		face = randomFace();
		bestFace = face;
		bestCandidate = randomPoint(face);
		bestDistance = prominence(bestCandidate);

		for (int c = 1; c < maxCandidates; c++) {
			face = randomFace();
			glm::vec3 candidate = randomPoint(face);
			float distance = prominence(candidate);

			if (distance > bestDistance) {
				bestFace = face;
				bestCandidate = candidate;
				bestDistance = distance;
			}
		}

		if (bestDistance > minDist) {
			auto pTuple = std::make_tuple(bestCandidate, bestFace);
			points.push_back(pTuple);
			octree.addPoint(bestCandidate);
		}
	}
}

void ConnectionPoints::Draw(DefaultShader shader)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS);
	glBindVertexArray(VAO);
	shader.setVec4("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
	glDrawArrays(GL_POINTS, 0, vertices.size());
	glBindVertexArray(0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

bool ConnectionPoints::createConnection(glm::vec3 rayOrigin, glm::vec3 rayDirection)
{
	return false;
}

bool ConnectionPoints::deleteConnection(glm::vec3 rayOrigin, glm::vec3 rayDirection)
{
	return false;
}

void ConnectionPoints::undoConnection()
{
}

Face * ConnectionPoints::randomFace()
{
	return faces[intDistribution(generator)];
}

glm::vec3 ConnectionPoints::randomPoint(Face* face)
{
	float r1 = contDistribution(generator);
	float r2 = contDistribution(generator);

	// TODO: This is nonuniform.  Points are biased towards v2 and v1.
	glm::vec3 tVert = r1 * face->edge21 + face->vertex1;

	return r2 * (face->vertex3 - tVert) + tVert;
}

float ConnectionPoints::prominence(glm::vec3 point)
{
	glm::vec3 neighbor = octree.getNearestPoint(point);
	return glm::distance(neighbor, point);
}

glm::vec3 ConnectionPoints::lowestVertex(Face * face)
{
	float x1 = face->vertex1.x;
	float x2 = face->vertex2.x;
	float x3 = face->vertex3.x;

	// TODO: finish lowestVertex

	return face->vertex1;
}
