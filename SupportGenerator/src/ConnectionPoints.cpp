#include "ConnectionPoints.h"


ConnectionPoints::ConnectionPoints() 
{

}


ConnectionPoints::ConnectionPoints(Graph *graph, float maxAngle) : 
	model_(graph), overhang_((M_PI / 2.0f) - maxAngle)
{
	//For poisson, must be ># of disconnected support areas
	// For Mitchell's, seeds = max number of support points
	const int seeds = 200;
	const int maxCandidates = 50;
	const float maxRadius = 10.0f;

	populateOctree();
	intDistribution_ = std::uniform_int_distribution<int>(0, faces_.size() - 1);
	contDistribution_ = std::uniform_real_distribution<double>(0.0f, 1.0f);
	points_.reserve(seeds);

	MitchellsBestCandidates(seeds, maxCandidates);

	struct {
		bool operator()(std::tuple<glm::vec3, shared_ptr<Face>> a, std::tuple<glm::vec3, shared_ptr<Face>> b) const
		{
			return std::get<0>(a).z <std::get<0>(b).z;
		}
	} compPoints;

	std::sort(points_.begin(), points_.end(), compPoints);

	for (std::tuple<glm::vec3, shared_ptr<Face>> p : points_) {
		vertices_.emplace_back(std::get<0>(p), glm::vec3(0,0,0), true);
	}

	setUpRendering();
}


ConnectionPoints::~ConnectionPoints() {
}

// Adds only supportable faces to the faces vector and octree.
void ConnectionPoints::populateOctree()
{
	glm::vec3 down(0.0f, 0.0f, -1.0f);

	for (shared_ptr<Face> f : model_->faces_) {
		float angle = glm::acos(glm::dot(f->normal, down));
		if (angle < overhang_ && angle > 0.001f) {
			faces_.push_back(f);
			octree_.addFace(faces_.back());
		}
	}
}

// Uploads Vertex information to the GPU 
void ConnectionPoints::setUpRendering()
{
	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(10.0f);

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
  // TODO: Vertex objects are overkill.
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// A great thing about structs is that their memory layout is sequential for all its items.
	// The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
	// again translates to 3/2 floats which translates to a byte array.
	glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex), &vertices_[0], GL_STATIC_DRAW);

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

// Places at most seeds points which are spread out evenly in 3d space.
void ConnectionPoints::MitchellsBestCandidates(int seeds, int maxCandidates)
{
	glm::vec3 bestCandidate;
	float bestDistance;
  shared_ptr<Face> bestFace;
  shared_ptr<Face> face = randomFace();
	octree_.addPoint(randomPoint(face)); 

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
			points_.push_back(pTuple);
			octree_.addPoint(bestCandidate);
		}
	}
}

// Draws each of the connection points 
void ConnectionPoints::Draw(DefaultShader shader)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS);
	glBindVertexArray(VAO);
	shader.setVec4("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
	glDrawArrays(GL_POINTS, 0, vertices_.size());
	glBindVertexArray(0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

// Given a click vector from the camera, attempts to add a connection 
// where the ray hits the model.
bool ConnectionPoints::createConnection(glm::vec3 rayOrigin, glm::vec3 rayDirection)
{
	return false;
}

// Given a click vector from the camera, attempts to delete a point in the octree
// This may require scissoring or something.
bool ConnectionPoints::deleteConnection(glm::vec3 rayOrigin, glm::vec3 rayDirection)
{
	return false;
}

// Deletes the last connection from the octree
void ConnectionPoints::undoConnection()
{

}

// Chooses a random face
shared_ptr<Face> ConnectionPoints::randomFace()
{
	return faces_[intDistribution_(generator_)];
}

// Creates a random vertex on a given face using barycentric coordinates
glm::vec3 ConnectionPoints::randomPoint(shared_ptr<Face> face)
{
	float r1 = contDistribution_(generator_);
	float r2 = contDistribution_(generator_);

	// TODO: This is nonuniform.  Points are biased towards v2 and v1.
	glm::vec3 tVert = r1 * face->edge21 + face->vertex1->Position;

	return r2 * (face->vertex3->Position - tVert) + tVert;
}

// Returns the distance of the closest vertex to point.
float ConnectionPoints::prominence(glm::vec3 point)
{
	glm::vec3 neighbor = octree_.getNearestPoint(point);
	return glm::distance(neighbor, point);
}

// Returns the lowest vertex of a face.
glm::vec3 ConnectionPoints::lowestVertex(shared_ptr<Face> face)
{
	float x1 = face->vertex1->Position.x;
	float x2 = face->vertex2->Position.x;
	float x3 = face->vertex3->Position.x;

	// TODO: finish lowestVertex

	return face->vertex1->Position;
}
